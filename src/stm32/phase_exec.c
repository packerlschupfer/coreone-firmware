// Phase-stepping executor — Phase 1a: high-rate XDIRECT drive on the MCU.
//
// Phase 0 (host-paced ~80 Hz XDIRECT writes) proved the electrical foundation
// but stalled — the host can't stream a smooth high-rate rotating field. This
// module puts a fixed-rate timer on the F427 that advances an electrical-angle
// accumulator and streams the corresponding coil currents to the TMC2130's
// XDIRECT register, 100x+ the host rate. Constant-velocity sweep for now (the
// minimum to confirm high rate => smooth motion); trajectory integration and
// per-angle cogging correction come later.
//
// SAFE PATTERN (from Argo's Snapmaker-U1 executor): the timer never blocks on
// SPI. It advances the phase + pushes it to a ring buffer and wakes a task; the
// task drains the ring and does the (blocking) spidev_transfer. Phase 2 replaces
// that with a timer-triggered SPI3 TX-DMA burst (reuse src/display_dma.c).
//
// HOST CONTRACT before phase_exec_start (all learned in Phase 0, extras/phase_test.py):
//   - the TMC driver must be ENABLED (SET_STEPPER_ENABLE) — direct_mode only sets
//     the current target; a disabled driver freewheels with no torque.
//   - IHOLD (not IRUN) scales the XDIRECT current: SET_TMC_CURRENT ... HOLDCURRENT=.
//   - GCONF.direct_mode = 1.
// This module only streams the rotating current vector; it does not touch GCONF/CS.

#include <string.h>
#include "basecmd.h"   // oid_alloc, oid_lookup, oid_next
#include "command.h"   // DECL_COMMAND, DECL_INIT
#include "sched.h"     // DECL_TASK, timers, task_wake
#include "spicmds.h"   // spidev_transfer, spidev_oid_lookup
#include "stepper.h"   // stepper_oid_lookup, stepper_get_subphase
#include "board/irq.h" // irq_disable, irq_enable (phase_exec_stop quiesce)
#include "internal.h"          // SPI/DMA CMSIS (SPI3, DMA1, DMA1_Stream5)
#include "gpio.h"              // spi_setup, spi_prepare, gpio_out_setup
#include "board/armcm_boot.h"  // armcm_enable_irq
#include "board/misc.h"        // timer_read_time (DWT CYCCNT, atomic)

#define TMC_XDIRECT_WR  0xAD    // 0x2D | 0x80 (SPI write to XDIRECT)
#define SINE_LEN        1024    // phase units per electrical period
#define SINE_MASK       (SINE_LEN - 1)
#define PE_MAX_HARM     4       // cogging correction: harmonics 1..4 (2 & 4 dominant)
#define RING_SIZE       64      // power of two
#define SEG_RING        1024    // analytic move-segment ring (power of two). 16 was fine
                                // for plain live-trapq streaming (1 seg/move) but input shaping bursts ~6
                                // segs per move boundary and the host queues a whole
                                // flush-horizon at once -> 16 overflowed. 1024 (Fix 1,
                                // 2026-06-28) gives deep burst headroom so a SLOW section
                                // pre-builds a cushion the fast travels drain before the
                                // ring goes dry (the dry=27126 first-layer drift). head/tail
                                // are now u16 (capacity 1023 via leave-one-slot full-detect;
                                // depth reported as %hu). A dense print can still outrun any
                                // fixed ring -> extrapolate-on-dry (Fix 2) makes a brief
                                // starvation harmless + host-side flow control bounds it.
#define PE_MAX_DRY_SEC  0.005f  // Fix 2: cap the on-dry velocity coast at 5ms (~1.75mm @
                                // 350mm/s) so a real host stall can't fly the motor away.
#define PE_STEPS_PER_PERIOD 64  // step-units per electrical period (=4 full steps @16ustep)
#define PE_ANGLE_PER_STEP   (SINE_LEN / PE_STEPS_PER_PERIOD)   // = 16 angle-units/step
#define PE_DRY_MAX_STEPS  24.0f // Fix 3: clamp the dry COAST distance to < 1/2 electrical
                                // period (=32 step-units). Past 1/2 period a stale-velocity
                                // coast pushes the rotor where the re-anchor snap can't pull
                                // it back -> a permanent full-period slip = the per-layer Y
                                // staircase. 24 (~0.37 period) keeps every snap recoverable
                                // while preserving Fix 2's coast for genuine short underruns.

// sine_table[i] ~= 16383 * sin(2*pi*i/SINE_LEN); coil = (amp * sine) >> 14
static int16_t sine_table[SINE_LEN];

// 1/clock_freq (seconds per timer tick) — for analytic epoch from a tick delta.
static float pe_inv_clock;

// reinterpret a 32-bit command arg as an IEEE754 float (host bit-casts the value)
static inline float
u32_as_float(uint32_t u)
{
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

struct phase_dma;             // DMA play-out (defined below)

// One analytic constant-accel move segment (Prusa MoveTarget analogue). Position
// at tick `now` = start_pos + (start_v + half_accel*epoch)*epoch, epoch in seconds.
// Positions are in STEP-UNITS (same frame as stepper_get_position): 64 step-units =
// one electrical period, so electrical angle = (pos * PE_ANGLE_PER_STEP) & SINE_MASK.
struct phase_seg {
    uint32_t start_clock;     // segment epoch origin (MCU clock ticks)
    uint32_t duration;        // segment length (ticks); expires at start_clock+duration
    float    start_pos;       // step-units at start_clock
    float    start_v;         // step-units per second
    float    half_accel;      // 0.5*accel, step-units per second^2
    // NOTE: the host's reanchor flag is packed into duration's MSB (durations are << 2^31)
    // to avoid a per-seg byte that would pad the 1024x2 ring +8KB -> alloc_chunk fail.
};

struct phase_exec {
    struct timer timer;
    struct spidev_s *spi;
    struct phase_dma *dma;    // non-NULL = DMA play-out instead of the PIO task
    struct phase_exec *next;  // next motor in the shared-bus chain (NULL = end)
    struct stepper *stepper;  // trajectory source (NULL = oscillator only)
    uint8_t  traj;            // 1 = drive angle from the live stepper trajectory
    uint32_t interval;        // timer ticks between updates (= refresh period)
    uint32_t lead_ticks;      // velocity commutation lead: evaluate the trajectory this many
                              // ticks in the FUTURE so the coil field is set for where the rotor
                              // WILL be when the XDIRECT write lands (mirrors Prusa's
                              // now = ticks_us() + REFRESH_PERIOD_US, phase_stepping.cpp:796).
                              // Without it the field lags by vel*interval at speed -> under-torque
                              // / slip that grows with speed. Defaults to interval (one period).
    uint16_t phase_index;     // current electrical angle 0..1023
    uint16_t phase_offset;    // MSCNT - subphase at engage: aligns our LUT angle to
                              // the TMC's actual rotor commutation -> seamless handoff
    int16_t  phase_advance;   // units added per update (sign = direction)
    uint8_t  amp;             // current amplitude 0..255
    uint8_t  swap;            // swap coil A/B (TMC XDIRECT quirk; default on)
    uint8_t  active;
    uint8_t  run_tmr;         // 1 = this executor owns the chain-driving timer
    uint8_t  head, tail;
    uint16_t ring[RING_SIZE];
    uint16_t overflow;
    // cogging correction: pic = pi + sum_h(corr_mag[dir][h]*sin(h*pi + corr_pha[dir][h]))
    // Per-direction LUTs (fwd/bwd) mirror Prusa's forward/backward current tables, so the
    // direction-ANTIsymmetric cogging cancels (the load-sensitive single-axis drift source).
    int16_t  corr_mag[2][PE_MAX_HARM + 1];   // [dir][harmonic]; dir 0=fwd 1=bwd; 0 = off
    uint16_t corr_pha[2][PE_MAX_HARM + 1];   // [dir][harmonic] phase offset 0..1023
    // fwd<->bwd BLEND weight (Q8): 0 = pure fwd LUT, 256 = pure bwd LUT. Replaces the old
    // hard corr_dir switch: a binary flip jumps the applied phase delta at a reversal (and
    // CHATTERS when a motor's seg velocity hovers near 0, e.g. the CoreXY slave on a near-
    // pure-X move) -> torque transient that tips the marginal open-loop motor into slip at
    // LOADED reversals (the bidirectional-cogging drift). Weighting by velocity across a
    // +/-blend_vband window makes the correction continuous through zero; below the band it
    // settles to the ~symmetric average (the proven-stable state). Symmetric LUTs (fwd==bwd)
    // are unaffected for any weight. blend_vband==0 keeps the legacy hard-switch behavior.
    uint16_t corr_w;
    float    blend_vband;                     // |vel| (step-units/s) for full fwd/bwd; 0=hard
    // calibration sweep (Prusa-style): ramp ONE harmonic's correction linearly over
    // a time window so a single move covers the whole param range. prog 0..1024.
    uint8_t  sweep_active;
    uint8_t  sweep_harm;
    uint32_t sweep_start_clock;
    uint32_t sweep_dur_per_1024;          // timer ticks per 1/1024 of the sweep
    int16_t  sweep_pha_start, sweep_pha_diff;
    int16_t  sweep_mag_start, sweep_mag_diff;
    // resonance test (Prusa-style): hum the held angle in place (gantry doesn't
    // translate) -> phase_index += osc_amp*sin(osc_phase), osc_phase += osc_inc/tick.
    uint8_t  osc_active;
    uint16_t osc_amp;
    uint16_t osc_inc;
    uint16_t osc_phase;
    // analytic trajectory: evaluate position from constant-accel segments
    // instead of reading the live step count -> no X/Y step ISR dependency.
    uint8_t  analytic;        // 1 = analytic segment mode (supersedes traj/oscillator)
    uint16_t seg_head, seg_tail;
    uint8_t  cur_valid;       // a segment is currently active
    struct phase_seg seg_ring[SEG_RING];
    struct phase_seg cur_seg; // the segment being evaluated now
    float    last_pos;        // last evaluated position (anchor when the ring runs dry)
    float    last_v;          // Fix 2: end-velocity of the last retired segment (coast rate)
    uint32_t last_end_clock;  // Fix 2: clock at the end of the last retired segment
    uint16_t seg_overflow;    // host queued into a full ring
    uint16_t seg_dry;         // ticks evaluated with no segment (ran dry)
    // margin diagnostics (Fix-3 analysis): low-water mark of ring depth. seg_min_depth
    // is the running min (gated by seg_primed so the startup fill doesn't count);
    // seg_min_rep is snapshotted at every host feed, so the harmless end-of-print drain
    // (after the last feed) doesn't pull it to 0 -> seg_min_rep = true steady-state margin.
    uint16_t seg_min_depth;
    uint16_t seg_min_rep;
    uint8_t  seg_primed;
    // INSTRUMENTATION (localize the per-layer staircase): at each ring REFILL after a dry
    // span, snap = |fresh seg.start_pos - held last_pos| = the feed-position discontinuity
    // the drain introduced. Big sum -> the slip is the feed/refill path; ~0 -> commutation.
    uint8_t  in_dry;          // 1 = we ran dry since the last fresh segment (refill pending)
    uint16_t seg_refills;     // count of dry->refill transitions
    float    seg_snap_sum;    // sum of |snap| (step-units) over all refills
    float    seg_snap_max;    // max single |snap| (step-units)
    // OVER-SHOOT INSTRUMENTATION (the layer-change travel +Y/-Y drift): after a decel to
    // ~a stop, watch how far the COMMANDED field moves PAST the settle point. Big
    // seg_settle_exc -> the commanded field over-shoots the endpoint (a bug, fixable to
    // Prusa-parity fast travels); ~0 -> commanded is clean, over-shoot is physical/rotor.
    // seg_max_coast = max PRE-clamp dry coast distance (is the field flying on stale vel).
    uint8_t  settled;         // 1 = in a post-decel settle window, watching for over-shoot
    float    settle_pos;      // the endpoint the field settled at
    float    seg_settle_exc;  // max |pos - settle_pos| during settle windows (step-units)
    float    seg_max_coast;   // max pre-clamp |last_v*dts| dry coast (step-units)
    // FIELD-TELEPORT TRIPWIRE: a CONTINUOUS segment pull (ring never emptied, so
    // NOT a dry-refill that snapSUM sees) whose start_pos jumps > 1/4 electrical period from
    // the held endpoint = the host DROPPED trajectory (backpressure overflow) and the field snaps many
    // periods -> gross slip. Counts EVERY non-dry pull. seg_jump>0 = smoking gun for a feed hole.
    uint8_t  seg_jseen;       // 1 after the first pull (skip the spurious first-pull jump)
    uint16_t seg_jump;        // count of continuous pulls with |Δstart_pos| > 1/4 period
    float    seg_jump_max;    // largest such jump (step-units)
};

static struct task_wake phase_wake;

// forward decl — used as the oid key in tasks defined before it
void command_config_phase_exec(uint32_t *args);

// pack signed 9-bit coil currents: coil_A bits 8:0, coil_B bits 24:16
static inline uint32_t
pack_xdirect(int ca, int cb)
{
    return (((uint32_t)cb & 0x1ff) << 16) | ((uint32_t)ca & 0x1ff);
}

// Build the sine LUT with an incremental Q15 rotation (no libm at init).
void
phase_exec_init(void)
{
    int32_t c = 32767, s = 0;
    const int32_t cd = 32766, sd = 201;   // cos/sin(2*pi/1024) in Q15
    for (int i = 0; i < SINE_LEN; i++) {
        sine_table[i] = (int16_t)(s >> 1);
        int32_t c2 = (c * cd - s * sd) >> 15;
        int32_t s2 = (s * cd + c * sd) >> 15;
        c = c2; s = s2;
    }
    pe_inv_clock = 1.0f / (float)CONFIG_CLOCK_FREQ;
}
DECL_INIT(phase_exec_init);

// ---- DMA play-out (Phase 2: per-tick SPI3 TX-DMA from the phase_event timer) -
// One 5-byte XDIRECT frame per tick via SPI3 TX-DMA, kicked by the Klipper
// phase_event timer (on SysTick — works WITH Klipper's cooperative scheduler; a
// dedicated foreign timer fights the SysTick dispatch loop and hangs on moves).
// Completion + CS handled by the DMA1 Stream5 TC IRQ. The 8kHz main-loop pressure
// is relieved host-side by suspending the HX717 loadcell during a run (the thing
// that starves) — see extras/phase_exec.py _set_bulk_sensors. SPI3_TX = DMA1 S5 Ch0.
#define PE_DMA_STREAM DMA1_Stream5

struct phase_dma {
    SPI_TypeDef *spi;
    struct gpio_out cs;
    struct phase_exec *exec;        // back-ref (TC IRQ chains via exec->next)
    uint8_t frame[5];               // the buffer being TRANSMITTED this tick (DMA reads it)
    uint8_t pending[5];             // NEXT tick's frame, computed ahead during the DMA shift
    volatile uint8_t busy;
    // --- instrumentation (read via PHASE_DMA_STATS) -------------------------
    volatile uint32_t tx_count;     // frames kicked this run
    volatile uint32_t ovr_count;    // SPI OVR seen at TC-IRQ cleanup
    volatile uint32_t dmaerr_count; // DMA transfer/FIFO/direct-mode errors
    volatile uint32_t maxrx;        // max RX bytes drained in one cleanup
    volatile uint32_t skips;        // frames skipped: prev DMA still busy
    volatile uint32_t skips_same;   // writes skipped because XDIRECT unchanged (holds)
    uint32_t saved_br;              // original SPI CR1 BR (prescaler) to restore on stop
    uint8_t  br_set;                // 1 = we overrode the prescaler (see spi_div)
};
static struct phase_dma *pe_dma_active;   // any configured dma (shutdown safety)
static struct phase_dma *pe_inflight;     // the dma currently on the bus (chain cursor)
static uint8_t pe_rr_parity;              // ROUND-ROBIN cursor: which CoreXY motor this tick
// CS-latch slack: busy-loop iterations inserted AFTER the SPI shift completes (BSY=0)
// and BEFORE deasserting CS. The TMC2130 latches XDIRECT on the CS rising edge and needs
// hold time after the last SCK edge. At the native 2.625MHz the IRQ/BSY overhead supplies
// that for free, but an overclocked SPI (for >26kHz refresh) finishes so fast that CS
// deasserts too soon -> the write never latches ("burp then no move"). Prusa solves the
// same problem by delaying CS-high. 0 = no slack (fine at 2.625MHz). Set via cs_slack cmd.
static volatile uint32_t pe_cs_slack;

// Cogging correction: phase-modulate the electrical angle by the per-harmonic
// spectrum -> pic = pi + sum_h( mag_h * sin(h*pi + pha_h) ). Mirrors Prusa's
// MotorPhaseCorrection (mag in phase units, pha 0..1023). All-zero mags = identity.
// sine_table is ~Q14 (+/-16383), so (mag*sine)>>14 yields +/-mag phase units.
static inline uint16_t
pe_correct_angle(struct phase_exec *e, uint16_t pi, uint16_t w)
{
    // w (Q8, 0..256): 0 = pure fwd LUT, 256 = pure bwd LUT, in between = linear blend of
    // the two directions' additive phase corrections. Blending the SCALAR delta (not mag/pha)
    // is exact (sum of the two dirs' sinusoids) and continuous through a reversal. Symmetric
    // LUTs -> d0==d1 -> delta is weight-independent (identical to the pre-blend behavior).
    if (w > 256)
        w = 256;
    int32_t d0 = 0, d1 = 0;
    for (uint8_t h = 1; h <= PE_MAX_HARM; h++) {
        int16_t m0 = e->corr_mag[0][h];
        if (m0)
            d0 += ((int32_t)m0 * sine_table[((uint32_t)h * pi
                    + e->corr_pha[0][h]) & SINE_MASK]) >> 14;
        int16_t m1 = e->corr_mag[1][h];
        if (m1)
            d1 += ((int32_t)m1 * sine_table[((uint32_t)h * pi
                    + e->corr_pha[1][h]) & SINE_MASK]) >> 14;
    }
    int32_t delta = (d0 * (int32_t)(256 - w) + d1 * (int32_t)w) >> 8;
    return (uint16_t)((int32_t)pi + delta) & SINE_MASK;
}

// Build the 5-byte XDIRECT write frame for an electrical angle (mirrors the PIO
// task's computation in phase_exec_task).
static void
pe_compute_frame(struct phase_exec *e, uint16_t pi, uint16_t w, uint8_t *buf)
{
    uint16_t pic = pe_correct_angle(e, pi, w);
    int sa = sine_table[(pic + 256) & SINE_MASK];  // cos
    int sb = sine_table[pic];                      // sin
    int ca = (e->amp * sa) >> 14;
    int cb = (e->amp * sb) >> 14;
    if (e->swap) { int tmp = ca; ca = cb; cb = tmp; }
    uint32_t v = pack_xdirect(ca, cb);
    buf[0] = TMC_XDIRECT_WR;
    buf[1] = (uint8_t)(v >> 24); buf[2] = (uint8_t)(v >> 16);
    buf[3] = (uint8_t)(v >> 8);  buf[4] = (uint8_t)(v);
}

// Assert CS and DMA the frame out (non-blocking; the TC IRQ deasserts CS).
static void
pe_dma_kick(struct phase_dma *d)
{
    DMA_Stream_TypeDef *s = PE_DMA_STREAM;
    d->spi->CR1 |= SPI_CR1_SPE;            // ensure the shared SPI is enabled
    // The stream is ALREADY disabled here (the TC IRQ disables+spins it after every
    // transfer; config_phase_dma leaves it off) -> the disable+spin is redundant on the
    // critical path. Skip it (NDTR is writable since EN==0). Saves ~0.5us/motor.
    DMA1->HIFCR = (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5
                   | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
    gpio_out_write(d->cs, 0);             // assert CS (TMC is active-low)
    s->M0AR = (uint32_t)d->frame;
    s->NDTR = 5;
    d->spi->CR2 |= SPI_CR2_TXDMAEN;
    d->busy = 1;
    d->tx_count++;
    s->CR |= DMA_SxCR_EN;
}

void
DMA1_Stream5_IRQHandler(void)
{
    struct phase_dma *d = pe_inflight;
    DMA_Stream_TypeDef *s = PE_DMA_STREAM;
    if (!d) {
        // No frame in flight (chain finished or run stopped) -> a stray/late TC.
        // Clear every flag, make sure the stream is off, and bail. NEVER deref a
        // NULL pe_inflight (that hardfaults -> "Lost communication" MCU reset).
        DMA1->HIFCR = (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5
                       | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
        s->CR &= ~DMA_SxCR_EN;
        return;
    }
    if (DMA1->HISR & DMA_HISR_TCIF5) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
        while (d->spi->SR & SPI_SR_BSY)       // shift complete before CS change (BSY
            ;                                 // subsumes the TXE wait -> one spin, not two)
        (void)d->spi->DR;                     // clear RXNE (write-only: ignore RX data)
        (void)d->spi->SR;                     // DR-then-SR read clears any OVR
        for (uint32_t _s = pe_cs_slack; _s; _s--)  // CS-latch hold (see pe_cs_slack)
            asm volatile("nop");
        gpio_out_write(d->cs, 1);             // deassert CS (latch THIS motor's word)
        s->CR &= ~DMA_SxCR_EN;
        while (s->CR & DMA_SxCR_EN)
            ;
        d->spi->CR2 &= ~SPI_CR2_TXDMAEN;
        d->busy = 0;
        pe_inflight = 0;                      // ROUND-ROBIN: no chain; bus idle until next tick
    } else {
        d->dmaerr_count++;                    // TEIF/DMEIF/FEIF — DMA fault
        DMA1->HIFCR = (DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
        s->CR &= ~DMA_SxCR_EN;
        gpio_out_write(d->cs, 1);
        d->spi->CR2 &= ~SPI_CR2_TXDMAEN;
        d->busy = 0;
        pe_inflight = 0;
    }
}

// Calibration sweep: set the swept harmonic's correction to its linearly-ramped
// value for time `now` (prog 0..1024 across [start_clock, start_clock+dur]). The
// host then maps each accel sample's time -> prog -> the param that was active.
static inline void
pe_sweep_update(struct phase_exec *e, uint32_t now)
{
    int32_t elapsed = (int32_t)(now - e->sweep_start_clock);
    uint32_t prog = (elapsed <= 0) ? 0 : (uint32_t)elapsed / e->sweep_dur_per_1024;
    if (prog > 1024)
        prog = 1024;
    int32_t pha = (int32_t)e->sweep_pha_start
                  + (int32_t)e->sweep_pha_diff * (int32_t)prog / 1024;
    int32_t mag = (int32_t)e->sweep_mag_start
                  + (int32_t)e->sweep_mag_diff * (int32_t)prog / 1024;
    // Stage-A calibration sweep writes both directions symmetrically (a per-direction
    // sweep is Stage B). prog drives one harmonic's {pha,mag} across the move.
    e->corr_pha[0][e->sweep_harm] = e->corr_pha[1][e->sweep_harm]
        = (uint16_t)pha & SINE_MASK;
    e->corr_mag[0][e->sweep_harm] = e->corr_mag[1][e->sweep_harm]
        = (int16_t)mag;
}

// Analytic mode: retire the current segment once `now` passes its end and pull the
// next one from the ring. Segments are time-contiguous (host sets each segment's
// start_clock = prev start_clock + prev duration), so absolute time drives the walk.
// When the ring is empty the motor holds last_pos (a safe stop, not a lurch).
static void
pe_seg_advance(struct phase_exec *m, uint32_t now)
{
    while (1) {
        if (m->cur_valid) {
            int32_t into = (int32_t)(now - m->cur_seg.start_clock);
            if (into < (int32_t)m->cur_seg.duration)
                return;                       // still inside the current segment
            // segment done: snap last_pos to its exact endpoint, then drop it
            float ep = (float)m->cur_seg.duration * pe_inv_clock;
            m->last_pos = m->cur_seg.start_pos
                + (m->cur_seg.start_v + m->cur_seg.half_accel * ep) * ep;
            // Fix 2: remember the endpoint velocity (v = v0 + a*t = start_v +
            // 2*half_accel*ep) + clock so a subsequent dry tick can COAST on the
            // last trajectory instead of freezing. End-velocity self-regulates:
            // ~0 at a decel-to-corner (safe hold), high mid-fast-move (tracks).
            m->last_v = m->cur_seg.start_v + 2.0f * m->cur_seg.half_accel * ep;
            m->last_end_clock = m->cur_seg.start_clock + m->cur_seg.duration;
            if (m->last_v < 500.f && m->last_v > -500.f) {  // decel to ~stop (<2.5mm/s):
                m->settle_pos = m->last_pos;                // start watching for over-shoot
                m->settled = 1;
            }
            m->cur_valid = 0;
        }
        if (m->seg_head == m->seg_tail)
            return;                           // ring empty -> hold last_pos
        m->cur_seg = m->seg_ring[m->seg_tail];
        m->seg_tail = (m->seg_tail + 1) & (SEG_RING - 1);
        m->cur_valid = 1;
        if (m->in_dry) {       // refill after a dry span: record the feed discontinuity
            float snap = m->cur_seg.start_pos - m->last_pos;
            if (snap < 0)
                snap = -snap;
            m->seg_snap_sum += snap;
            if (snap > m->seg_snap_max)
                m->seg_snap_max = snap;
            m->seg_refills++;
            m->in_dry = 0;
        } else if (m->seg_jseen) {   // field-teleport: continuous pull (ring never emptied) -> a big jump
            float jd = m->cur_seg.start_pos - m->last_pos;   // here = a DROPPED segment
            if (jd < 0.f)
                jd = -jd;
            if (jd > 16.f) {         // 1/4 electrical period = 64/4 = 16 step-units
                m->seg_jump++;
                if (jd > m->seg_jump_max)
                    m->seg_jump_max = jd;
            }
        }
        m->seg_jseen = 1;            // first pull primes last_pos; subsequent ones are checked
        uint8_t reanc = m->cur_seg.duration >> 31;   // unpack the halt-resume flag
        m->cur_seg.duration &= 0x7fffffffu;          // strip it -> duration logic unaffected
        if (reanc) {
            // Prusa reset_from_halt / set_phase_origin: this is the first move resuming
            // AFTER a queue-drain halt (a layer change). Re-seed the phase origin so the
            // resume position maps to the HELD phase (last_phase = phase_index, the angle
            // the commutation last drove) -> ties pos<->rotor EXACTLY at the restart,
            // every halt, without re-reading MSCNT (which would re-inject the +-1/2-period
            // homing scatter = the 512/0.16mm staircase quantum). phase_index here is still
            // last tick's held value (pe_update_angle recomputes it after this call). Both
            // CoreXY motors re-anchor independently with their own held phase + start_pos,
            // so the projection (X<-x+y, Y<-x-y) is inherent and symmetric -> no
            // differential (Y-biased) residual.
            int32_t start_ang = (int32_t)(m->cur_seg.start_pos
                                          * (float)PE_ANGLE_PER_STEP);
            m->phase_offset = ((int32_t)m->phase_index - start_ang) & SINE_MASK;
        }
        // loop again in case this freshly-pulled segment already expired (catch up)
    }
}

// Compute this motor's electrical angle for tick `now` and store it in phase_index.
// Three position sources, in priority: analytic segments, the live step trajectory,
// or the free-running phase_advance accumulator. Then the optional resonance hum.
static inline void
pe_update_angle(struct phase_exec *m, uint32_t now)
{
    if (m->sweep_active)
        pe_sweep_update(m, now);
    if (m->analytic) {
        // Velocity commutation lead (Prusa phase_stepping.cpp:796 now = ticks_us() +
        // REFRESH_PERIOD_US): evaluate the trajectory lead_ticks in the future so the field
        // leads the rotor instead of lagging by vel*interval. lead_ticks=0 = legacy no-lead.
        uint32_t tnow = now + m->lead_ticks;
        pe_seg_advance(m, tnow);
        // margin diagnostic: track the ring depth low-water mark (Fix-3 analysis)
        uint16_t _depth = (m->seg_head - m->seg_tail) & (SEG_RING - 1);
        if (_depth >= (SEG_RING / 2))
            m->seg_primed = 1;
        if (m->seg_primed && _depth < m->seg_min_depth)
            m->seg_min_depth = _depth;
        float pos, vel;
        if (m->cur_valid) {
            int32_t de = (int32_t)(tnow - m->cur_seg.start_clock);
            if (de < 0)
                de = 0;
            float epoch = (float)de * pe_inv_clock;
            pos = m->cur_seg.start_pos
                + (m->cur_seg.start_v + m->cur_seg.half_accel * epoch) * epoch;
            m->last_pos = pos;
            vel = m->cur_seg.start_v + 2.0f * m->cur_seg.half_accel * epoch;
        } else {
            // Fix 2: dry (host fell behind feeding the ring). COAST on the last
            // segment's end-velocity instead of freezing, so the next re-anchored
            // segment snaps only a tiny amount -> no slipped electrical periods.
            // Capped at PE_MAX_DRY_SEC so a real host stall can't run away.
            int32_t dt = (int32_t)(tnow - m->last_end_clock);
            if (dt < 0)
                dt = 0;
            float dts = (float)dt * pe_inv_clock;
            if (dts > PE_MAX_DRY_SEC)
                dts = PE_MAX_DRY_SEC;
            // Fix 3: bound the coast DISTANCE to < 1/2 electrical period (the time cap
            // alone allows 1.75mm @350mm/s = many periods of slip). Beyond a recoverable
            // snap the rotor slips a full period and never recovers (the Y staircase).
            float coast = m->last_v * dts;
            float ac = coast < 0.f ? -coast : coast;   // INSTRUMENT: pre-clamp coast distance
            if (ac > m->seg_max_coast)
                m->seg_max_coast = ac;
            if (coast > PE_DRY_MAX_STEPS)
                coast = PE_DRY_MAX_STEPS;
            else if (coast < -PE_DRY_MAX_STEPS)
                coast = -PE_DRY_MAX_STEPS;
            pos = m->last_pos + coast;
            vel = m->last_v;
            m->seg_dry++;
            m->in_dry = 1;     // mark: the next fresh segment is a refill (measure its snap)
        }
        // INSTRUMENT: how far does the COMMANDED field move PAST a post-decel settle point?
        // (the layer-change over-shoot). Big -> commanded field over-shoots (a bug);
        // ~0 -> commanded clean -> over-shoot is physical/rotor.
        if (m->settled) {
            float exc = pos - m->settle_pos;
            if (exc < 0.f)
                exc = -exc;
            if (exc > m->seg_settle_exc)
                m->seg_settle_exc = exc;
            if (exc > 400.f)          // >2mm = a genuine next move started; close the window
                m->settled = 0;
        }
        // Travel direction (Prusa physical_speed sign) for the per-direction cogging LUT,
        // BLENDED through zero (see corr_w decl). corr_w: 0 = pure fwd at vel >= +band,
        // 256 = pure bwd at vel <= -band, linear between -> continuous, no chatter when vel
        // hovers near zero. band==0 reverts to the legacy hard switch (HOLD through the apex,
        // Prusa's `!= 0` guard, phase_stepping.cpp:734).
        if (m->blend_vband > 0.f) {
            float r = vel / m->blend_vband;        // +1 at +band (fwd) .. -1 at -band (bwd)
            if (r > 1.f) r = 1.f; else if (r < -1.f) r = -1.f;
            m->corr_w = (uint16_t)((1.f - r) * 128.f + 0.5f);   // r=+1 ->0  r=-1 ->256
        } else if (vel > 0.f) {
            m->corr_w = 0;
        } else if (vel < 0.f) {
            m->corr_w = 256;
        }
        // vel == 0 (hard-switch mode): hold the last weight (don't flip at the apex)
        int32_t ang = (int32_t)(pos * (float)PE_ANGLE_PER_STEP);
        m->phase_index = ((uint32_t)ang + m->phase_offset) & SINE_MASK;
    } else if (m->traj && m->stepper) {
        m->phase_index = (stepper_get_subphase(m->stepper, now)
                          + m->phase_offset) & SINE_MASK;
    } else {
        m->phase_index = (m->phase_index + m->phase_advance) & SINE_MASK;
    }
    if (m->osc_active) {                       // hum in place around the held angle
        m->osc_phase += m->osc_inc;
        int32_t osc = ((int32_t)m->osc_amp
                       * sine_table[m->osc_phase & SINE_MASK]) >> 14;
        m->phase_index = (uint16_t)((int32_t)m->phase_index + osc) & SINE_MASK;
    }
}

// ---- Dedicated TIM8 hardware-timer drive (Prusa TIM8-ISR model) ------------
// The Klipper SysTick sched timer (prio 2) can't sustain a >~24kHz phase refresh,
// and the DMA TC IRQ at the SAME prio 2 couldn't preempt SysTick -> the chain
// stalled -> ~5kHz effective (skip-saturation). TIM8 gives a guaranteed refresh
// tick at PRIORITY 1 (above SysTick): its ISR computes the chain's angles+frames
// and kicks the master DMA; the prio-1 DMA TC IRQ chains the slave + CS within the
// tick. Both touch only phase_exec/SPI3/DMA1 state, so no Klipper contention. TIM8
// and the CYCCNT clock both run at 168MHz, so ARR(ticks) == interval(clock ticks).
static struct phase_exec *pe_tim8_master;

// PIPELINED DMA refresh. To give the ~23us 2-motor chain the FULL tick, the DMA kick
// happens FIRST (from `frame`, computed last tick), then the NEXT tick's frame is
// computed into `pending` DURING the DMA shift (off the critical path -> the kick lands
// at the tick start instead of after the compute, so the chain finishes before the next
// tick = no 50% skip @40kHz). `pending`->`frame` promotion runs ONLY when the prior chain
// is done (pe_inflight clear) so an in-flight DMA is never overwritten. `now` is already
// one interval ahead (the computed frame is applied on the NEXT tick); pe_update_angle
// adds the commutation lead on top. REQUIRES BASEPRI + prio-0 phase IRQs.
static void
pe_dma_refresh(struct phase_exec *e, uint32_t now)
{
    // ROUND-ROBIN (Prusa parity): refresh exactly ONE motor per tick, alternating the two
    // CoreXY motors. Each motor's hold is therefore 2 ticks (50us @40kHz), for which a
    // one-refresh-period (25us) lead is the ZOH-center -> the field lands AHEAD of the rotor
    // and anticipates decels (no over-shoot). No SPI chaining: each motor is kicked on its
    // own tick and its DMA (~7.6us) completes well within the 25us tick, so the two writes
    // never contend and the slave is NOT applied stale (kills the differential-Y asymmetry).
    // Compute straight into `frame` and kick same-tick: the previous SAME-motor DMA finished
    // two ticks (50us) ago, so `frame` is free. (Replaces the both-motors-per-tick chained
    // design whose 0.5-tick lead + ~11us slave stagger caused the layer-registration drift.)
    struct phase_exec *m = (e->next && (pe_rr_parity++ & 1)) ? e->next : e;
    pe_update_angle(m, now);
    if (!m->dma)
        return;
    pe_compute_frame(m, m->phase_index, m->corr_w, m->dma->frame);
    // XDIRECT-skip (Prusa's new_currents != last_currents guard): skip the SPI write when the XDIRECT
    // value is UNCHANGED -> the bus is SILENT through holds/dwells (no bus-hammering / CS
    // mis-latch through the per-layer Z-hop dwell). `pending` is dead since round-robin computes
    // into `frame` directly, so reuse it as the last-KICKED cache.
    if (memcmp(m->dma->frame, m->dma->pending, 5) == 0) {
        m->dma->skips_same++;
        return;                            // nothing changed -> nothing to send (TMC holds it)
    }
    memcpy(m->dma->pending, m->dma->frame, 5);
    if (!pe_inflight) {
        pe_inflight = m->dma;
        pe_dma_kick(m->dma);
    } else {
        m->dma->skips++;                   // bus still busy (should not happen: 7.6us << 25us)
    }
}

// TIM8 update ISR (prio 0, BASEPRI-unmaskable): the refresh tick. Compute `pending` for
// one interval ahead (it's applied next tick); timer_read_time()=CYCCNT is an atomic DWT
// read safe from any priority. NOTE: the commutation lead is added inside pe_update_angle
// (m->lead_ticks) - the caller must NOT add it (the old `+lead_ticks` here was a double-lead).
void
TIM8_UP_TIM13_IRQHandler(void)
{
    TIM8->SR &= ~TIM_SR_UIF;               // clear the update flag (rc_w0)
    struct phase_exec *e = pe_tim8_master;
    if (e)
        pe_dma_refresh(e, timer_read_time());  // lead is now-relative (lead_ticks in pe_update_angle)
}

// Start TIM8 at the refresh given by `interval` clock ticks (ARR == interval since
// TIM8 and CYCCNT both clock at 168MHz). Idempotent across re-engages.
static void
pe_tim8_start(struct phase_exec *e, uint32_t interval)
{
    pe_tim8_master = e;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB2ENR;                          // ensure the clock is up
    TIM8->CR1 = 0;                         // stop, edge-aligned up-counter
    TIM8->PSC = 0;
    TIM8->ARR = interval - 1;              // period in 168MHz ticks
    TIM8->EGR = TIM_EGR_UG;                // latch PSC/ARR (generates an update)
    TIM8->SR &= ~TIM_SR_UIF;               // clear that spurious UG update flag
    TIM8->DIER = TIM_DIER_UIE;             // enable the update interrupt
    // PRIME the pipeline: compute the first `pending` frame (engage-aligned: analytic
    // pos=0 -> angle=phase_offset=MSCNT) so the first tick's promote+kick has valid data.
    struct phase_exec *m;
    uint32_t now = timer_read_time() + interval;
    for (m = e; m; m = m->next) {
        pe_update_angle(m, now);
        if (m->dma)
            pe_compute_frame(m, m->phase_index, m->corr_w, m->dma->pending);
    }
    // priority 0 = the reserved BASEPRI-unmaskable phase tier (serial/CAN bumped 0->1)
    armcm_enable_irq(TIM8_UP_TIM13_IRQHandler, TIM8_UP_TIM13_IRQn, 0);
    TIM8->CR1 = TIM_CR1_CEN;               // start counting
}

static void
pe_tim8_stop(void)
{
    TIM8->CR1 = 0;                         // stop the counter
    TIM8->DIER = 0;                        // disable the update interrupt source
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    pe_tim8_master = 0;
}

// Timer: PIO path (single motor) via the Klipper sched timer; for the DMA chain it
// is only a sched-timer FALLBACK (the TIM8 ISR is the normal DMA driver).
static uint_fast8_t
phase_event(struct timer *t)
{
    struct phase_exec *e = container_of(t, struct phase_exec, timer);
    if (e->dma) {
        pe_dma_refresh(e, t->waketime); // sched fallback (lead is now-relative via lead_ticks)
    } else {
        // PIO path (single motor, non-DMA): queue the angle for the blocking task.
        pe_update_angle(e, t->waketime);
        uint8_t next = (e->head + 1) & (RING_SIZE - 1);
        if (next != e->tail) {
            // pack the blend weight (Q8 0..256) quantized to 6 bits in [10:15] above the
            // 10-bit phase_index; unpacked + rescaled in phase_exec_task.
            e->ring[e->head] = e->phase_index | ((uint16_t)(e->corr_w >> 3) << 10);
            e->head = next;
        } else {
            e->overflow++;        // SPI task fell behind (rate too high for PIO)
        }
        sched_wake_task(&phase_wake);
    }
    t->waketime += e->interval;
    return SF_RESCHEDULE;
}

// Task: drain the ring and push coil currents to XDIRECT over SPI (blocking here).
void
phase_exec_task(void)
{
    if (!sched_check_wake(&phase_wake))
        return;
    uint8_t oid = 0xff;
    struct phase_exec *e;
    while ((e = oid_next(&oid, command_config_phase_exec))) {
        while (e->tail != e->head) {
            uint16_t raw = e->ring[e->tail];
            e->tail = (e->tail + 1) & (RING_SIZE - 1);
            uint16_t pi = raw & SINE_MASK;
            uint16_t pic = pe_correct_angle(e, pi, ((raw >> 10) & 0x3f) << 3);
            int sa = sine_table[(pic + 256) & SINE_MASK];  // cos
            int sb = sine_table[pic];                      // sin
            int ca = (e->amp * sa) >> 14;
            int cb = (e->amp * sb) >> 14;
            if (e->swap) { int tmp = ca; ca = cb; cb = tmp; }
            uint32_t v = pack_xdirect(ca, cb);
            uint8_t buf[5] = { TMC_XDIRECT_WR,
                               (uint8_t)(v >> 24), (uint8_t)(v >> 16),
                               (uint8_t)(v >> 8),  (uint8_t)(v) };
            spidev_transfer(e->spi, 0, sizeof(buf), buf);
        }
    }
}
DECL_TASK(phase_exec_task);

void
command_config_phase_exec(uint32_t *args)
{
    struct phase_exec *e = oid_alloc(args[0], command_config_phase_exec,
                                     sizeof(*e));
    e->spi = spidev_oid_lookup(args[1]);
    e->stepper = stepper_oid_lookup(args[2]);
    e->dma = 0;
    e->next = 0;
    e->traj = 0;
    e->phase_offset = 0;
    e->active = 0;
    e->sweep_active = 0;
    e->osc_active = 0;
    e->analytic = 0;
    e->seg_head = e->seg_tail = 0;
    e->cur_valid = 0;
    e->last_pos = 0;
    e->last_v = 0;
    e->last_end_clock = 0;
    e->seg_overflow = e->seg_dry = 0;
    e->seg_min_depth = e->seg_min_rep = SEG_RING;
    e->seg_primed = 0;
    e->head = e->tail = 0;
    for (uint8_t h = 0; h <= PE_MAX_HARM; h++) {
        e->corr_mag[0][h] = e->corr_mag[1][h] = 0;
        e->corr_pha[0][h] = e->corr_pha[1][h] = 0;
    }
    e->corr_w = 0;
    // blend_vband stays as last set (host sends phase_exec_blend on connect / live);
    // default 0 = legacy hard switch until the host configures a band.
}
DECL_COMMAND(command_config_phase_exec,
             "config_phase_exec oid=%c spi_oid=%c stepper_oid=%c");

// Set the fwd<->bwd cogging-blend velocity band (step-units/s) live. vband=0 reverts to
// the legacy hard fwd/bwd switch. The host derives the count from a mm/s knob x steps/mm.
void
command_phase_exec_blend(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->blend_vband = u32_as_float(args[1]);
}
DECL_COMMAND(command_phase_exec_blend, "phase_exec_blend oid=%c vband=%u");

// Set the velocity commutation lead (timer ticks) live. ticks=interval = Prusa-faithful one
// refresh period; 0 = legacy no-lead (field lags the rotor at speed). For A/B + tuning.
void
command_phase_exec_lead(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->lead_ticks = args[1];
}
DECL_COMMAND(command_phase_exec_lead, "phase_exec_lead oid=%c ticks=%u");

// Set one cogging-correction harmonic: mag (phase units, 0=off), pha (0..1023).
// Persists in the executor struct until changed or MCU reset (host re-sends on
// connect); applied live in pe_compute_frame / the PIO task. Safe to send mid-run
// (it touches the executor oid, not the suppressed TMC SPI path).
void
command_phase_exec_set_corr(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    uint8_t dir = args[1] & 1;
    uint8_t h = args[2];
    if (h >= 1 && h <= PE_MAX_HARM) {
        e->corr_mag[dir][h] = (int16_t)args[3];
        e->corr_pha[dir][h] = args[4] & SINE_MASK;
    }
}
DECL_COMMAND(command_phase_exec_set_corr,
             "phase_exec_set_corr oid=%c dir=%c harm=%c mag=%hi pha=%hu");

// Arm/disarm a calibration sweep on one harmonic. While armed, phase_event ramps
// that harmonic's {pha,mag} from (start) to (start+diff) across the window
// [start_clock, start_clock + 1024*dur]. active=0 disarms AND clears the harmonic.
void
command_phase_exec_sweep(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    uint8_t harm = args[2];
    if (harm < 1 || harm > PE_MAX_HARM) {
        e->sweep_active = 0;
        return;
    }
    if (args[1]) {
        e->sweep_harm = harm;
        e->sweep_start_clock = args[3];
        e->sweep_dur_per_1024 = args[4] ? args[4] : 1;
        e->sweep_pha_start = (int16_t)args[5];
        e->sweep_pha_diff = (int16_t)args[6];
        e->sweep_mag_start = (int16_t)args[7];
        e->sweep_mag_diff = (int16_t)args[8];
        e->sweep_active = 1;
    } else {
        e->sweep_active = 0;
        e->corr_mag[0][harm] = e->corr_mag[1][harm] = 0;
        e->corr_pha[0][harm] = e->corr_pha[1][harm] = 0;
    }
}
DECL_COMMAND(command_phase_exec_sweep,
    "phase_exec_sweep oid=%c active=%c harm=%c start_clock=%u dur=%u"
    " pha_start=%hi pha_diff=%hi mag_start=%hi mag_diff=%hi");

// Resonance test: hum the held electrical angle in place at a fixed frequency (the
// gantry doesn't translate). inc = osc_phase units per tick = freq * SINE_LEN / rate.
void
command_phase_exec_osc(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->osc_active = args[1];
    if (args[1]) {
        e->osc_amp = args[2];
        e->osc_inc = args[3];
        e->osc_phase = 0;
    }
}
DECL_COMMAND(command_phase_exec_osc,
             "phase_exec_osc oid=%c active=%c amp=%hu inc=%hu");

// Link a motor into the shared-bus chain after `oid` (next_oid=0 clears the link).
// The chain MASTER's timer refreshes every linked motor each tick, serialized on
// the bus by the TC IRQ. Used for coordinated multi-motor (CoreXY) phase-stepping.
void
command_phase_exec_chain(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->next = args[1] ? oid_lookup(args[1], command_config_phase_exec) : 0;
}
DECL_COMMAND(command_phase_exec_chain,
             "phase_exec_chain oid=%c next_oid=%c");

// Attach a DMA play-out to an executor: from here its timer streams XDIRECT via
// SPI3 TX-DMA instead of the blocking PIO task. spi_bus/mode/rate must match the
// TMC's bus; cs_pin is the TMC's CS (raw pin number -> no host pin-enum clash).
void
command_config_phase_dma(uint32_t *args)
{
    struct phase_dma *d = oid_alloc(args[0], command_config_phase_dma,
                                    sizeof(*d));
    struct phase_exec *e = oid_lookup(args[1], command_config_phase_exec);
    // reuse the executor's existing TMC spidev: same SPI peripheral + CS, so no
    // re-config of the shared bus and no duplicate CS setup.
    d->spi = (SPI_TypeDef *)spidev_get_spi(e->spi);
    d->cs = spidev_get_cs_pin(e->spi);
    d->exec = e;
    d->busy = 0;
    d->br_set = 0;
    e->dma = d;
    pe_dma_active = d;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR;                          // ensure the clock is up
    DMA_Stream_TypeDef *s = PE_DMA_STREAM;
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN)
        ;
    // SPI3_TX = DMA1 S5 Ch0: mem->periph, byte, MINC, TCIE (Ch0 => no CHSEL bits)
    s->CR = (DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PL_0 | DMA_SxCR_TCIE);
    s->FCR = 0;                            // direct mode
    s->PAR = (uint32_t)&d->spi->DR;
    armcm_enable_irq(DMA1_Stream5_IRQHandler, DMA1_Stream5_IRQn, 0);  // phase tier (BASEPRI-unmaskable)
}
DECL_COMMAND(command_config_phase_dma,
    "config_phase_dma oid=%c exec_oid=%c");

// Override the shared TMC SPI prescaler for the engaged run. div = CR1 BR field
// (SPI clk = PCLK1[42MHz] / 2^(div+1)): 0=21MHz 1=10.5 2=5.25 3=2.625(Klipper default
// for a 4MHz request). Both CoreXY motors share this one SPI3, so one call covers both.
// SAFE because phase-stepping suppresses TMC polling (we own the bus); the original
// prescaler is saved and restored by phase_exec_stop. Prusa runs this SPI at 21MHz
// (div 0) on Buddy boards — far past the TMC2130 4MHz datasheet spec, but XDIRECT is a
// short write-only frame refreshed every tick, so it tolerates the high clock.
void
command_phase_exec_spi_div(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    struct phase_dma *d = e->dma;
    if (!d)
        return;
    SPI_TypeDef *spi = d->spi;
    if (!d->br_set) {                          // remember the original prescaler once
        d->saved_br = spi->CR1 & SPI_CR1_BR_Msk;
        d->br_set = 1;
    }
    uint32_t br = ((uint32_t)args[1] << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk;
    uint32_t cr1 = spi->CR1 & ~SPI_CR1_SPE;    // BR is writable only with SPE=0
    spi->CR1 = cr1;
    spi->CR1 = (cr1 & ~SPI_CR1_BR_Msk) | br;   // SPE stays off; the DMA kick re-enables
}
DECL_COMMAND(command_phase_exec_spi_div, "phase_exec_spi_div oid=%c div=%c");

// Set the CS-latch slack (busy-loop iterations after BSY=0, before CS-high). Needed when
// the SPI is overclocked (spi_div<3) so the TMC reliably latches the XDIRECT write.
void
command_phase_exec_cs_slack(uint32_t *args)
{
    pe_cs_slack = args[0];
}
DECL_COMMAND(command_phase_exec_cs_slack, "phase_exec_cs_slack value=%u");

// Bump a pin's GPIO output slew rate to "very high" (OSPEEDR=0x03). Klipper's
// spi_setup configures SPI SCK/MOSI with GPIO_FUNCTION() WITHOUT the high-speed
// flag -> F4 OSPEEDR=0x02 ("high", ~50MHz@40pF). On the Buddy's TMC SPI traces
// that rolls off the edges above ~5-10MHz and corrupts the XDIRECT write at the
// overclocked rate (5.25MHz clean, 10.5MHz+ garbled). Re-applying the SAME AF
// with GPIO_HIGH_SPEED forces OSPEEDR=0x03 ("very high"), matching Prusa's
// GPIO_SPEED_FREQ_VERY_HIGH, so 21MHz is clean on the identical board.
void
command_phase_exec_pin_hs(uint32_t *args)
{
    gpio_peripheral(args[0], GPIO_FUNCTION(args[1]) | GPIO_HIGH_SPEED, 0);
}
DECL_COMMAND(command_phase_exec_pin_hs, "phase_exec_pin_hs pin=%u af=%c");

// Report the per-run DMA diagnostics (read AFTER stop, bus idle). High ovr/maxrx
// => Candidate A (RX/OVR); high skips/dmaerr => Candidate B (timer/IRQ latency).
void
command_phase_dma_query(uint32_t *args)
{
    struct phase_dma *d = oid_lookup(args[0], command_config_phase_dma);
    sendf("phase_dma_stats oid=%c tx=%u ovr=%u dmaerr=%u maxrx=%u skips=%u",
          args[0], d->tx_count, d->ovr_count, d->dmaerr_count,
          d->maxrx, d->skips);
}
DECL_COMMAND(command_phase_dma_query, "phase_dma_query oid=%c");

void
command_phase_exec_start(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    if (e->run_tmr)
        sched_del_timer(&e->timer);
    e->interval = args[1];
    e->lead_ticks = args[1];               // default lead = one refresh period (Prusa-faithful)
    e->phase_index = args[2] & SINE_MASK;
    e->phase_advance = (int16_t)args[3];
    e->amp = args[4];
    e->swap = args[5];
    // Phase-align the handoff: args[8]=MSCNT (the TMC's actual rotor commutation,
    // read by the host while still in step/dir). Offset our LUT angle so the first
    // streamed vector == the rotor's current vector -> no engage lurch. The motor is
    // stationary at engage so subphase(start_clock) is stable.
    if (e->analytic)
        // analytic positions are referenced to pos=0 at engage, so angle(pos=0)=0;
        // the offset is simply MSCNT (the rotor commutation the host read at engage).
        e->phase_offset = (uint16_t)args[8] & SINE_MASK;
    else if (e->stepper)
        e->phase_offset = ((uint16_t)args[8]
            - (uint16_t)(stepper_get_subphase(e->stepper, args[6]) & SINE_MASK))
            & SINE_MASK;
    else
        e->phase_offset = 0;
    e->head = e->tail = 0;
    e->overflow = 0;
    // belt-and-braces: an engage always starts with NO calibration sweep in flight
    // (a sweep is only armed mid-run, after start). Force-clear sweep_active so a sweep the
    // host left armed on a prior aborted CALIBRATE_PHASE_COGGING cannot survive to this run
    // and let pe_sweep_update force-overwrite this harmonic's live cogging correction with a
    // stale sweep-endpoint magnitude. Kills the whole leak class; the host try/finally is the
    // primary fix, this is the last line of defense (survives even a klippy crash mid-sweep).
    e->sweep_active = 0;
    e->active = 1;
    if (e->dma) {                          // fresh diagnostics each run
        struct phase_dma *d = e->dma;
        d->tx_count = d->ovr_count = d->dmaerr_count = d->maxrx = d->skips = 0;
        d->busy = 0;
    }
    // run_timer (args[7]): 1 = chain MASTER (owns the timer that drives the whole
    // chain); 0 = SLAVE (params set + active, but refreshed by the master's tick).
    if (args[7]) {
        e->run_tmr = 1;
        if (e->dma) {
            pe_tim8_start(e, e->interval); // TIM8 drives the DMA chain (prio 1)
        } else {
            e->timer.func = phase_event;
            e->timer.waketime = args[6];   // start_clock
            sched_add_timer(&e->timer);
        }
    } else {
        e->run_tmr = 0;
    }
}
DECL_COMMAND(command_phase_exec_start,
             "phase_exec_start oid=%c interval=%u phase_index=%hu"
             " phase_advance=%hi amp=%c swap=%c start_clock=%u run_timer=%c"
             " mscnt=%hu");

void
command_phase_exec_stop(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    if (!e->active)
        return;
    if (e->run_tmr) {                      // only the master drives the refresh timer
        if (e->dma)
            pe_tim8_stop();
        else
            sched_del_timer(&e->timer);
    }
    e->run_tmr = 0;
    e->active = 0;
    e->next = 0;                           // drop the chain link
    if (e->dma) {
        // quiesce the bus: kill any in-flight transfer + clear the chain cursor so
        // a late TC can't chain into a stopped run (the IRQ NULL-guards pe_inflight).
        irq_disable();
        PE_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        pe_inflight = 0;
        e->dma->busy = 0;
        irq_enable();
        if (e->dma->br_set) {                  // restore the original SPI prescaler
            SPI_TypeDef *spi = e->dma->spi;
            uint32_t cr1 = spi->CR1 & ~SPI_CR1_SPE;
            spi->CR1 = cr1;
            spi->CR1 = (cr1 & ~SPI_CR1_BR_Msk) | e->dma->saved_br;
            e->dma->br_set = 0;
        }
    }
    // leave the last current vector applied; host zeroes/restores via XDIRECT
}
DECL_COMMAND(command_phase_exec_stop, "phase_exec_stop oid=%c");

void
command_phase_exec_trajectory(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->traj = args[1];   // 1 = follow stepper trajectory, 0 = oscillator
}
DECL_COMMAND(command_phase_exec_trajectory,
             "phase_exec_trajectory oid=%c enable=%c");

// Enable/disable analytic segment mode. Enabling clears the segment ring + holds
// position at 0 (the engage reference). Send BEFORE phase_exec_start so the start
// command picks the analytic phase_offset (= MSCNT) branch.
void
command_phase_exec_analytic(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    e->analytic = args[1];
    if (args[1]) {
        e->seg_head = e->seg_tail = 0;
        e->cur_valid = 0;
        e->last_pos = 0;
        e->last_v = 0;
        e->last_end_clock = 0;
        e->seg_overflow = e->seg_dry = 0;
        e->seg_min_depth = e->seg_min_rep = SEG_RING;
        e->seg_primed = 0;
        e->in_dry = e->seg_refills = 0;
        e->seg_snap_sum = e->seg_snap_max = 0.f;
        e->settled = 0;
        e->seg_settle_exc = e->seg_max_coast = 0.f;
        e->seg_jseen = 0; e->seg_jump = 0; e->seg_jump_max = 0.f;   // field-teleport tripwire reset
        if (e->dma) {
            e->dma->skips_same = 0;
            e->dma->pending[0] = 0xff;     // XDIRECT-skip: invalidate last-kicked cache -> first frame kicks
        }
    }
}
DECL_COMMAND(command_phase_exec_analytic,
             "phase_exec_analytic oid=%c enable=%c");

// Queue one analytic move segment. pos/v/ha are IEEE754 floats bit-cast to u32 by
// the host (step-units, step-units/s, step-units/s^2). start_clock/duration are MCU
// clock ticks; the host keeps segments time-contiguous. Dropped if the ring is full.
void
command_phase_exec_seg(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    uint16_t nh = (e->seg_head + 1) & (SEG_RING - 1);   // u16: ring is 1024 now
    if (nh == e->seg_tail) {                   // ring full -> drop (host backs off)
        e->seg_overflow++;
        return;
    }
    // snapshot the low-water mark at each host feed: excludes the end-of-print drain
    // (after the last feed) so seg_min_rep reflects true steady-state margin.
    e->seg_min_rep = e->seg_min_depth;
    struct phase_seg *sg = &e->seg_ring[e->seg_head];
    sg->start_clock = args[1];
    // pack the reanchor flag into duration's MSB (no extra struct byte -> no ring padding).
    sg->duration = args[2] | (args[6] ? 0x80000000u : 0u);
    sg->start_pos = u32_as_float(args[3]);
    sg->start_v = u32_as_float(args[4]);
    sg->half_accel = u32_as_float(args[5]);
    e->seg_head = nh;
}
DECL_COMMAND(command_phase_exec_seg,
             "phase_exec_seg oid=%c start_clock=%u duration=%u pos=%u v=%u ha=%u"
             " reanchor=%c");

// Report analytic-mode diagnostics (queue depth, overflow, dry ticks). Read anytime.
void
command_phase_exec_seg_query(uint32_t *args)
{
    struct phase_exec *e = oid_lookup(args[0], command_config_phase_exec);
    uint16_t depth = (e->seg_head - e->seg_tail) & (SEG_RING - 1);
    // snap/settle/coast reported in milli-step-units (su*1000); /200000 = mm (200 su/mm).
    uint32_t skips_same = e->dma ? e->dma->skips_same : 0;
    sendf("phase_exec_seg_state oid=%c depth=%hu overflow=%hu dry=%hu cur_valid=%c"
          " minrep=%hu refills=%hu snapsum=%u snapmax=%u settle=%u coast=%u"
          " jump=%hu jumpmax=%u skipsame=%u",
          args[0], depth, e->seg_overflow, e->seg_dry, e->cur_valid, e->seg_min_rep,
          e->seg_refills, (uint32_t)(e->seg_snap_sum * 1000.f),
          (uint32_t)(e->seg_snap_max * 1000.f),
          (uint32_t)(e->seg_settle_exc * 1000.f), (uint32_t)(e->seg_max_coast * 1000.f),
          e->seg_jump, (uint32_t)(e->seg_jump_max * 1000.f), skips_same);
}
DECL_COMMAND(command_phase_exec_seg_query, "phase_exec_seg_query oid=%c");

void
phase_exec_shutdown(void)
{
    if (pe_dma_active) {
        PE_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        pe_dma_active->spi->CR2 &= ~SPI_CR2_TXDMAEN;
    }
    pe_inflight = 0;
    uint8_t oid = 0xff;
    struct phase_exec *e;
    while ((e = oid_next(&oid, command_config_phase_exec))) {
        if (e->dma) {
            gpio_out_write(e->dma->cs, 1);   // deselect every motor on the bus
            e->dma->busy = 0;
        }
        if (e->run_tmr) {
            if (e->dma)
                pe_tim8_stop();
            else
                sched_del_timer(&e->timer);
        }
        e->run_tmr = 0;
        e->active = 0;
        e->next = 0;
    }
}
DECL_SHUTDOWN(phase_exec_shutdown);
