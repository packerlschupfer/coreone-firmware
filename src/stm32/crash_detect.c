// StallGuard crash detection (TMC2130) — the Prusa "Crash Detection" equivalent.
//
// A plain G1 print move has NO crash protection in our port (the loadcell only
// guards homing/probing). Prusa's Buddy fw flags a motor stall (= a nozzle/gantry
// crash) via StallGuard during moves and halts. This module does the same: while
// ARMED (for the duration of a print), it reads the TMC2130 sg_result over SPI and,
// when it stays at/near 0 (an absolute stall, the way Prusa's DIAG1 comparator fires
// at SG_RESULT==0), calls shutdown() — an immediate ISR-speed halt of all steppers/
// heaters. The nozzle stops grinding.
//
// This is DETECTION + STOP only (no Prusa-style re-home + g-code replay recovery:
// Klipper has no replay infra). After a crash: FIRMWARE_RESTART + re-print.
//
// Detection matches Prusa's production design (Prusa-Firmware-Buddy CRASH_STALL_GUARD
// / CRASH_FILTER): trigger on an ABSOLUTE sg_result <= sg_floor (==0, with the SGT
// register biasing where 0 lands), NOT a relative fall from a baseline. The earlier
// adaptive-baseline/% -drop design false-tripped because sg_result is intrinsically
// low just above the velocity gate and climbs with speed, so a % drop reads fast moves
// as stalls; an absolute near-0 is velocity-robust. Two notes:
//  1) the trigger is shutdown() (whole-print safety) instead of a homing trsync;
//  2) a VELOCITY GATE: StallGuard's sg_result is only valid above the coolStep
//     velocity (TSTEP <= TCOOLTHRS). Below it (slow moves, accel/decel), sg_result
//     is invalid-low; we skip those samples so they can't false-trigger a crash.
//     (Same velocity limitation Prusa's crash detection has.)
// SGT=+2 and sfilt=off are applied by the host (extras/crash_detect.py) on arm.
//
// The SPI read runs in a task (woken by the timer) because spidev_transfer is not
// IRQ-safe.

#include "basecmd.h"   // oid_alloc, oid_lookup, oid_next
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h"   // DECL_COMMAND, sendf, shutdown
#include "sched.h"     // struct timer, sched_add_timer, DECL_TASK
#include "spicmds.h"   // spidev_transfer, spidev_oid_lookup

#define TMC_TSTEP       0x12   // read address: TSTEP (velocity, 20-bit; large = slow)
#define TMC_DRV_STATUS  0x6f   // read address: DRV_STATUS (sg_result in bits 9:0)

struct crash_detect {
    struct timer time;
    uint32_t rest_time;
    uint32_t gate_tstep;      // velocity gate: skip a sample when TSTEP > this (slower
                              // than the coolStep gate). 0 = gate disabled.
    struct spidev_s *spi;
    uint16_t sg_floor;        // Prusa SG==0 trigger: stall when sg_result <= this
                              // absolute floor (SGT biases where 0 lands). NOT a % drop.
    uint8_t flags, sample_count, trigger_count;
};

enum { CDF_ARMED = 1 << 0 };

static struct task_wake cd_wake;

// forward decl so the task can use it as the oid key
void command_config_crash_detect(uint32_t *args);

// Read TSTEP and DRV_STATUS(sg_result) from the TMC2130. TMC SPI reads are
// "delayed": a transfer returns the value of the register addressed in the
// PREVIOUS transfer. Three self-priming transfers therefore yield both fresh:
//   T1 addr=TSTEP       (returns stale -- ignored)
//   T2 addr=DRV_STATUS  (returns TSTEP, addressed in T1)
//   T3 addr=DRV_STATUS  (returns DRV_STATUS, addressed in T2)
static void
cd_read(struct crash_detect *e, uint32_t *tstep, uint16_t *sg)
{
    uint8_t buf[5];
    buf[0] = TMC_TSTEP; buf[1] = buf[2] = buf[3] = buf[4] = 0;
    spidev_transfer(e->spi, 1, sizeof(buf), buf);          // addr TSTEP (return stale)
    buf[0] = TMC_DRV_STATUS; buf[1] = buf[2] = buf[3] = buf[4] = 0;
    spidev_transfer(e->spi, 1, sizeof(buf), buf);          // return = TSTEP
    *tstep = (((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16)
              | ((uint32_t)buf[3] << 8) | buf[4]) & 0xfffff;
    buf[0] = TMC_DRV_STATUS; buf[1] = buf[2] = buf[3] = buf[4] = 0;
    spidev_transfer(e->spi, 1, sizeof(buf), buf);          // return = DRV_STATUS
    *sg = (((uint16_t)buf[3] & 0x03) << 8) | buf[4];        // sg_result = bits 9:0
}

// Timer: wake the task at the sample rate (the SPI read must run in task context).
static uint_fast8_t
crash_detect_event(struct timer *t)
{
    struct crash_detect *e = container_of(t, struct crash_detect, time);
    if (!(e->flags & CDF_ARMED))
        return SF_DONE;
    sched_wake_task(&cd_wake);
    e->time.waketime += e->rest_time;
    return SF_RESCHEDULE;
}

// Task: for every armed detector, read sg_result, apply the velocity gate + the
// adaptive sustained-stall filter, and shutdown() on a sustained stall.
void
crash_detect_task(void)
{
    if (!sched_check_wake(&cd_wake))
        return;
    uint8_t oid = 0xff;
    struct crash_detect *e;
    while ((e = oid_next(&oid, command_config_crash_detect))) {
        if (!(e->flags & CDF_ARMED))
            continue;
        uint32_t tstep;
        uint16_t sg;
        cd_read(e, &tstep, &sg);
        // Velocity gate: sg_result is only valid above the coolStep velocity
        // (TSTEP <= TCOOLTHRS). Below it -- slow moves, accel/decel, and the
        // fast->slow transition that would otherwise read as a sudden drop --
        // skip and reset, so they can't false-trigger and the baseline isn't
        // poisoned with invalid-low values.
        if (e->gate_tstep && tstep > e->gate_tstep) {
            e->trigger_count = e->sample_count;
            continue;
        }
        // Prusa "Crash Detection" trigger: an ABSOLUTE sg_result <= sg_floor (==0 with
        // SGT biasing where 0 lands), NOT a relative fall from a baseline. The TMC2130
        // DIAG1 comparator Prusa uses fires at SG_RESULT==0; reading the same sg_result
        // over SPI and testing <= sg_floor is the software equivalent and is velocity-
        // ROBUST -- unlike a % drop, an absolute near-0 means a real stall at any speed
        // above the gate, so faster moves / higher accel don't false-trip. The small
        // sample_count debounces a stray single-sample 0 (DIAG/line noise).
        if (sg <= e->sg_floor) {
            uint8_t count = e->trigger_count - 1;
            if (!count)
                shutdown("Crash detected: StallGuard stall");   // __noreturn ISR-speed halt
            e->trigger_count = count;
        } else {
            e->trigger_count = e->sample_count;   // reset on any non-stall
        }
    }
}
DECL_TASK(crash_detect_task);

void
command_config_crash_detect(uint32_t *args)
{
    struct crash_detect *e = oid_alloc(args[0], command_config_crash_detect,
                                       sizeof(*e));
    e->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_crash_detect, "config_crash_detect oid=%c spi_oid=%c");

// Arm (sample_count>0) or disarm (sample_count=0) continuous crash detection.
void
command_crash_detect_arm(uint32_t *args)
{
    struct crash_detect *e = oid_lookup(args[0], command_config_crash_detect);
    sched_del_timer(&e->time);
    e->time.waketime = args[1];
    e->rest_time = args[2];
    e->sample_count = args[3];
    if (!e->sample_count) {              // disarm
        e->flags = 0;
        return;
    }
    e->sg_floor = args[4];
    e->gate_tstep = args[5];
    e->trigger_count = e->sample_count;
    e->flags = CDF_ARMED;
    e->time.func = crash_detect_event;
    sched_add_timer(&e->time);
}
DECL_COMMAND(command_crash_detect_arm,
             "crash_detect_arm oid=%c clock=%u rest_ticks=%u sample_count=%c"
             " sg_floor=%hu gate_tstep=%u");

void
command_crash_detect_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct crash_detect *e = oid_lookup(oid, command_config_crash_detect);
    sendf("crash_detect_state oid=%c armed=%c", oid, !!(e->flags & CDF_ARMED));
}
DECL_COMMAND(command_crash_detect_query_state, "crash_detect_query_state oid=%c");
