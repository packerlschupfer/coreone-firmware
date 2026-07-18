// Input-shaping convolution for the phase-stepping executor — axis-space evaluator.
//
// PHASE A of MCU-side input shaping (docs/mcu-input-shaping-phaseA.md). This is a
// DEPENDENCY-FREE translation unit by design: <stdint.h> and nothing else. No sched.h,
// no command.h, no CMSIS, no internal.h, no floating-point library calls. That is what
// lets test/shaper_golden/ compile THIS EXACT SOURCE with plain gcc and assert it against
// the Python oracle (extras/phase_exec.py `_shaped_motor` / `_eval_axis`). If you add an
// MCU header here you break the validation gate — put the MCU-facing glue in phase_exec.c
// instead.
//
// WHY AXIS SPACE. Input shaping is a convolution with a short impulse train and the CoreXY
// projection is linear, so they commute — but only PER AXIS. X uses EI@55.4, Y uses MZV@43.4:
// different shapers, so the convolution cannot collapse into a motor's (X±Y) trajectory.
// The MCU therefore convolves each AXIS with its own pulse table and projects to the motors
// afterwards:  posA = cx*Xs + cy*Ys - ref  (see phase_exec.c).
//
// WHAT THIS FILE IS NOT. pe_axis_eval is a PURE evaluator: it reproduces the host oracle
// exactly, including freezing at the last segment's endpoint when asked for a time past the
// ring head. It deliberately does NOT implement the dry-coast safety policy (Fix 2/Fix 3 in
// phase_exec.c) — that policy has to be applied to the PROJECTED motor position so a
// simultaneous X and Y coast cannot sum to 2x the half-electrical-period clamp. The eval
// reports "past head" in its status bits and the caller decides. Keeping the policy out of
// here is also what keeps the golden compare meaningful.

#ifndef __PHASE_SHAPER_H
#define __PHASE_SHAPER_H

#include <stdint.h>

// SATURATING 16-bit diagnostic increment. Use this for EVERY counter in this file and in
// phase_exec.c's stats block -- never a bare `++`.
//
// 🚨 WHY (2026-10-12). All of these count 40 kHz ISR TICKS, so a plain `++` wraps every
// 65536/40000 = 1.638 s and passes back THROUGH ZERO. For `dry`/`hold`/`past` that merely
// makes a long run's numbers meaningless (a 4 h 10 m print reported dry=51286, which is a
// residue, not a count). For `pe_shape_late_before` it is UNSAFE: PHASE_XY_CHECK grants
// `xy_trusted` on `lateBEF == 0` and PRINT_END then drives an ABSOLUTE park to X242 Y211 on
// that verdict, so a counter that returns to its own "healthy" value every 1.6 s can
// authorise a park on a position the machine does not have -- the 2026-10-09 rail strike.
//
// Saturating is strictly safer than wrapping, with the same asymmetry the XY guard already
// chose deliberately: it can over-report (costs a re-home) but can never under-report (costs
// the frame). The cost is that 0xFFFF reads as "at least 65535" -- a CEILING, not a count.
// Widening to uint32 was the alternative and was rejected: these live in `struct pe_axis` and
// `struct phase_exec`, the latter holding a 1024-entry seg_ring, and the ring already sits
// against an alloc_chunk cliff.
#define PE_CNT_MAX 0xFFFFu

static inline void
pe_cnt_inc(uint16_t *c)
{
    if (*c != (uint16_t)PE_CNT_MAX)
        (*c)++;
}

static inline void
pe_cnt_add(uint16_t *c, uint16_t n)
{
    uint32_t v = (uint32_t)*c + (uint32_t)n;
    *c = (v > PE_CNT_MAX) ? (uint16_t)PE_CNT_MAX : (uint16_t)v;
}

// One analytic constant-accel move segment (Prusa MoveTarget analogue). Position at tick
// `now` = start_pos + (start_v + half_accel*epoch)*epoch, epoch in seconds.
//
// For MOTOR-space rings (the legacy un-shaped path) positions are in step-units: 64
// step-units = one electrical period. For AXIS-space rings (the shaped path) they are in
// the same step-unit scale but pre-projection — the host multiplies by steps/mm and folds
// the axis direction sign in, so the MCU only has to apply the CoreXY coefficients.
//
// The host packs a reanchor flag into duration's MSB (durations are always << 2^31) rather
// than spending a per-segment byte, which would pad the 1024-entry ring by 8 KB and fail
// alloc_chunk. Use PE_SEG_DUR() to read the duration; never the raw field.
struct phase_seg {
    uint32_t start_clock;     // segment epoch origin (MCU clock ticks)
    uint32_t duration;        // length in ticks | PE_SEG_REANCHOR
    float    start_pos;       // position at start_clock
    float    start_v;         // position units per second
    float    half_accel;      // 0.5*accel, position units per second^2
};

#define PE_SEG_REANCHOR  0x80000000u
#define PE_SEG_DUR_MASK  0x7fffffffu
#define PE_SEG_DUR(sg)   ((sg)->duration & PE_SEG_DUR_MASK)

// Impulse count ceiling. EI and MZV are 3 each; 2-hump EI is 4. 5 leaves headroom without
// making struct pe_axis interesting to the allocator.
#define PE_MAX_PULSE     5

// Bounded backward walk when a cursor has over-advanced (see pe_axis_eval).
#define PE_BACKSCAN_MAX  4

// Bound on the forward cursor walk PER IMPULSE PER TICK. In steady state a tick advances
// `now` by one refresh interval (125 us @ 8 kHz) against segments floored at min_seg_dt
// (0.8 ms), so this is 0 or 1. It only goes large after an engage or after the host skips a
// long idle gap. Unlike the retire loop in phase_exec.c we cannot simply defer the work --
// a lagging cursor evaluates a STALE segment, i.e. commands a wrong angle -- so saturation
// raises PE_EVAL_LAGGED and the caller must hold rather than trust the value. The cursor
// persists across ticks, so it converges within a few ticks either way. 64 iterations of a
// compare-and-increment is ~400 cycles, tolerable at prio 0; 1023 (the ring depth) is not.
#define PE_ADVANCE_MAX   64

// STALENESS LATCH threshold, SECONDS. See the latch in pe_axis_eval.
//
// Everything in this file computes positions from SIGNED 32-bit clock differences, which
// are only meaningful within +-2^31 ticks = 12.783 s @168MHz. If the host stops refreshing
// the ring the newest segment AGES, and once it is older than that the differences change
// sign: the BEFORE branch commands a segment's start_pos, the bounded backward walk retreats
// into pre-park history, and after a full 2^32 wrap (25.567 s) trel lands back in range and
// the stale segment REPLAYS AT SPEED. On 2026-10-11 that drove this machine's gantry into
// the left rail once per 25.567 s with the host completely silent -- see
// extras/phase_exec.py _keepalive_event.
//
// WHAT IS MEASURED is the length of the current UNCOVERED episode: how long `now` has sat
// continuously outside every segment the impulses landed on, i.e. how long the evaluator has
// been extrapolating from windows that have already elapsed. That is the quantity the wrap
// threatens, and it is deliberately NOT "the age of the newest segment" -- segments are
// routinely stamped far in the FUTURE (a bridged 3-minute pause is 23 segments spanning 180
// s, and `now - start_clock` for a 180 s offset overflows 2^32 seven times over, so it
// measures nothing). While `now` falls INSIDE a segment the ring is doing its job no matter
// how long that segment is, so a legitimate slow 30 s move never trips this.
//
// CHOOSING 8 s. The threshold has to sit strictly between the longest LEGITIMATE dry episode
// and the 12.783 s ceiling:
//   lower bound -- the host's keep-alive refreshes the ring every _KEEPALIVE_S = 4.0 s and
//     stamps each hold segment to END at the present, so while idle `now` runs past the
//     newest segment and a healthy dry episode peaks at ~4 s plus reactor jitter. 8 s leaves
//     ~3.7 s of jitter margin before a FALSE latch.
//   upper bound -- 12.783 s. 8 s leaves a 4.78 s detection window, which at the 40 kHz
//     refresh rate is ~191,000 evaluations. The latch cannot be stepped over.
// Note this is deliberately NOT derived from _BRIDGE_MAX (8.0 s, the longest single hold
// segment): they are equal by coincidence of both being margins on the same 12.783 s
// ceiling, and tying them together would couple two unrelated tunings.
#define PE_STALE_S       8.0f

// One shaper impulse. `a` is the normalized weight (the weights sum to 1); `dt` is the time
// offset in MCU CLOCK TICKS, signed, pre-converted by the host so the hot path never does a
// seconds<->ticks conversion.
//
// Sign convention, and the thing most likely to be got wrong: _init_shaper_pulses (the
// verbatim kin_shaper.c replica) reverses the traditional (A,T) pairs, normalizes, and then
// shifts so sum(a*t) == 0. The offsets therefore STRADDLE ZERO — for MZV@43.4 they span
// roughly -11.5 ms .. +11.5 ms, NOT 0 .. 23 ms. Positive dt looks into the FUTURE (covered
// host-side, the flush horizon lags by _pulse_margin); negative dt looks into the PAST, so
// the ring must retain roughly 11.5 ms of already-played history. Retirement is driven by
// the OLDEST cursor — pe_axis_retire().
struct pe_pulse {
    float   a;
    int32_t dt;
};

// Per-axis evaluator state. The ring is BORROWED, not owned: motor A's existing seg_ring
// serves as the X axis ring and motor B's as Y. That is worth ~40 KB of not-allocating a
// second pair of 1024-entry rings, which the dynmem pool does not have to spare. `head` is
// likewise borrowed — it points at the owning executor's live seg_head, which the host's
// phase_exec_seg command advances.
struct pe_axis {
    const struct phase_seg *ring;
    const volatile uint16_t *head;   // borrowed producer index
    const volatile uint16_t *tail;   // borrowed consumer index (oldest retained segment)
    uint16_t mask;                   // ring capacity - 1 (capacity is a power of two)
    uint8_t  npulse;
    struct pe_pulse p[PE_MAX_PULSE];
    uint16_t cur[PE_MAX_PULSE];      // forward cursor per impulse
    float    inv_clock;              // seconds per clock tick
    // diagnostics (see pe_axis_eval status bits)
    uint16_t n_before;               // lookback underrun: needed history already retired
    uint16_t n_past;                 // lookahead underrun: ring dry ahead of the evaluated time
    uint16_t n_empty;                // evaluated with no segments at all
    uint16_t n_backscan;             // cursor had over-advanced and needed a backward walk
    uint16_t n_lagged;               // forward walk hit PE_ADVANCE_MAX (value not trustworthy)
    // STALENESS LATCH (see pe_axis_eval). stale_ticks is PE_STALE_S converted with
    // inv_clock, so it follows the configured clock; set it via pe_axis_set_stale.
    uint32_t stale_ticks;            // dry duration past which the ring is not trusted
    uint32_t dry_since;              // clock at which the current UNCOVERED episode began
    uint16_t stale_head;             // head value when the latch was set (clears on change)
    uint16_t n_stale;                // times the latch ENGAGED (events, not held ticks)
    uint8_t  dry_valid;              // 1 = dry_since holds a live episode start
    uint8_t  stale_latch;            // 1 = holding; sticky until a new segment arrives
};

// pe_axis_eval status bits. OK == 0.
#define PE_EVAL_OK         0
#define PE_EVAL_EMPTY      (1 << 0)   // ring held no segments; pos/vel left UNMODIFIED
#define PE_EVAL_PAST_HEAD  (1 << 1)   // some impulse fell past the newest segment (froze)
#define PE_EVAL_BEFORE     (1 << 2)   // some impulse fell before the oldest retained segment
#define PE_EVAL_LAGGED     (1 << 3)   // forward walk saturated; pos/vel are STALE, do not use
#define PE_EVAL_STALE      (1 << 5)   // ring uncovered for longer than stale_ticks; pos/vel
                                      // left UNMODIFIED and the caller MUST hold. Sticky --
                                      // only a newly arrived segment clears it.
#define PE_EVAL_REANCHOR   (1 << 4)   // a sampled segment carries the host's halt-resume flag.
                                      // Stays set for as long as that segment is inside the
                                      // shaper window (~18ms, many ticks) -- the caller must
                                      // edge-detect it, not treat it as a per-tick event.

// ---- electrical-angle mapping ----------------------------------------------
// Shared with phase_exec.c so the two cannot drift. 64 step-units = one electrical period;
// the sine LUT has 1024 entries per period, hence 16 angle-units per step-unit.
#define SINE_LEN             1024
#define SINE_MASK            (SINE_LEN - 1)
#define PE_STEPS_PER_PERIOD  64
#define PE_ANGLE_PER_STEP    (SINE_LEN / PE_STEPS_PER_PERIOD)

static inline int32_t
pe_pos_to_angle(float pos)
{
    return (int32_t)(pos * (float)PE_ANGLE_PER_STEP);
}

// ---- per-motor projection --------------------------------------------------
// The shape-specific part of one motor's state. Deliberately a separate struct embedded in
// phase_exec's executor rather than loose fields: it keeps the projection + hold + reanchor
// arithmetic in THIS translation unit, where test/shaper_golden/ can reach it. That
// arithmetic used to live in phase_exec.c where it could not be tested at all -- and the
// reanchor angle bookkeeping in particular is the least verifiable thing in the project.
//
// last_pos and phase_offset stay in the executor and are passed by pointer: the LEGACY
// un-shaped path shares them, and moving them would churn a proven code path for nothing.
struct pe_motor {
    float   cx, cy;           // CoreXY projection coefficients (host folds direction signs in)
    float   pos_ref;          // engage reference, subtracted AFTER the projection
    uint8_t reanchor_pend;    // a halt-resume edge is waiting to re-seed this motor's origin
};

#define PE_PROJ_HELD        (1 << 0)   // eval unusable; *pos is the held value, do not trust vel
#define PE_PROJ_DRY         (1 << 1)   // ring starved (empty / stale / past the head)
#define PE_PROJ_REANCHORED  (1 << 2)   // phase_offset was re-seeded on this call

// One motor's per-tick computation: project the shaped axis values, apply the hold policy,
// and consume a pending reanchor. `st` is the OR of both axes' pe_axis_eval status.
// `phase_index_held` is the angle the commutation last drove -- the reanchor ties the new
// position to it, so the commanded angle does NOT move on the anchoring tick.
uint8_t pe_motor_step(struct pe_motor *mm, uint8_t st,
                      float xp, float xv, float yp, float yv,
                      uint16_t phase_index_held,
                      float *last_pos, uint16_t *phase_offset,
                      float *pos, float *vel);

// Latch a halt-resume edge onto both motors. PE_EVAL_REANCHOR is a LEVEL -- it stays
// asserted for as long as the flagged segment is inside the shaper window (~18 ms, >140
// ticks at 8 kHz) -- so this detects the 0->1 transition and arms each motor exactly once.
// `*prev` is the caller's edge-state store. Returns 1 if an edge fired. Either motor may be
// NULL. Call it BEFORE pe_motor_step so the motor being refreshed this tick anchors
// immediately rather than one tick late.
uint8_t pe_reanchor_edge(uint8_t *prev, uint8_t st,
                         struct pe_motor *m0, struct pe_motor *m1);

// Bind an axis to a borrowed ring. Clears cursors and diagnostics; leaves pulses alone.
void pe_axis_init(struct pe_axis *ax, const struct phase_seg *ring, uint16_t mask,
                  const volatile uint16_t *head, const volatile uint16_t *tail,
                  float inv_clock);

// Install the shaper pulse table. n == 0 (or a NULL table) installs the identity pulse
// ({a=1, dt=0}), which makes pe_axis_eval reduce exactly to un-shaped segment evaluation.
void pe_axis_set_pulses(struct pe_axis *ax, uint8_t n, const struct pe_pulse *p);

// Set the staleness threshold from a duration in SECONDS, using the axis's inv_clock.
// pe_axis_init applies PE_STALE_S; call this afterwards only to override it (the golden
// harness does, so a test can cross the threshold without simulating 8 real seconds).
// Changing inv_clock directly does NOT rescale the threshold -- re-apply it if you do.
void pe_axis_set_stale(struct pe_axis *ax, float seconds);

// Park every cursor at `tail`. Call whenever the ring is emptied or re-seeded.
void pe_axis_reset(struct pe_axis *ax, uint16_t tail);

// Shaped axis position (and velocity) at absolute clock `now`:
//   pos = sum_i a_i * axis(now + dt_i),  vel = sum_i a_i * axis'(now + dt_i)
// Returns a bitwise-OR of the PE_EVAL_* status bits. pos/vel are written on every return
// EXCEPT PE_EVAL_EMPTY and PE_EVAL_STALE, which leave them untouched so the caller can keep
// its held value. On PE_EVAL_LAGGED the written values are stale and must not be commanded.
uint8_t pe_axis_eval(struct pe_axis *ax, uint32_t now, float *pos, float *vel);

// New ring tail: the oldest slot any cursor still needs. Segments strictly before it can be
// overwritten by the producer. The segment a cursor currently sits ON is retained -- it is
// still being evaluated.
uint16_t pe_axis_retire(struct pe_axis *ax);

#endif // __PHASE_SHAPER_H
