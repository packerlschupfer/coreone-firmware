// StallGuard-filtered sensorless endstop (TMC2130).
//
// Klipper's native sensorless homing triggers on the TMC diag1 pin, which
// fires on a SINGLE StallGuard dip. On a noisy axis (e.g. a per-belt-tooth
// load ripple) sg_result momentarily punches to 0 mid-move, so a long homing
// sweep false-triggers and "homes" short.
//
// This endstop instead reads the TMC's sg_result (DRV_STATUS bits 9:0) over
// SPI during the homing move and triggers the homing trsync only after
// `sample_count` CONSECUTIVE samples below `threshold` -- i.e. a SUSTAINED
// stall. Brief noise dips (shorter than sample_count*rest_ticks) are filtered;
// the real frame stall (held indefinitely) triggers cleanly. Mirrors
// src/endstop.c, but the SPI read runs in a task (woken by the timer) because
// spidev_transfer must not run in the timer IRQ context.

#include "basecmd.h"   // oid_alloc, oid_lookup, oid_next
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h"   // DECL_COMMAND
#include "sched.h"     // struct timer, sched_add_timer, DECL_TASK
#include "trsync.h"    // trsync_do_trigger, trsync_oid_lookup
#include "spicmds.h"   // spidev_transfer, spidev_oid_lookup

#define TMC_DRV_STATUS  0x6f   // read address (no write bit) for sg_result

struct sg_endstop {
    struct timer time;
    uint32_t rest_time;
    uint32_t trigger_clock;   // captured at trigger; reported as next_clock
    uint32_t baseline_x16;    // decaying-max of sg_result (x16 fixed point): the
                              // free-motion reference the RELATIVE trigger falls from
    struct spidev_s *spi;
    struct trsync *ts;
    uint16_t threshold, drop_permille, warmup;
    uint8_t flags, sample_count, trigger_count, trigger_reason, prime;
};

enum { SGF_HOMING = 1 << 0 };

static struct task_wake sg_wake;

// forward decl so the task can use it as the oid key
void command_config_sg_endstop(uint32_t *args);

// Read sg_result from the TMC2130 DRV_STATUS register. TMC SPI reads are
// "delayed": the value returned is for the PREVIOUSLY addressed register. We
// always re-address DRV_STATUS, so after one priming read every read returns
// sg_result. buf comes back as [status, d31:24, d23:16, d15:8, d7:0].
static uint16_t
sg_read_result(struct sg_endstop *e)
{
    uint8_t buf[5] = { TMC_DRV_STATUS, 0, 0, 0, 0 };
    spidev_transfer(e->spi, 1, sizeof(buf), buf);
    return (((uint16_t)buf[3] & 0x03) << 8) | buf[4];   // sg_result = bits 9:0
}

// Timer: wake the SG task at the sample rate. The SPI read itself must run in
// task context (spidev_transfer is not IRQ-safe), so the timer only wakes it.
static uint_fast8_t
sg_endstop_event(struct timer *t)
{
    struct sg_endstop *e = container_of(t, struct sg_endstop, time);
    if (!(e->flags & SGF_HOMING))
        return SF_DONE;
    sched_wake_task(&sg_wake);
    e->time.waketime += e->rest_time;
    return SF_RESCHEDULE;
}

// Task: read sg_result for every armed endstop, apply the sustained-stall
// filter, and trigger the trsync on `sample_count` consecutive sub-threshold
// reads.
void
sg_endstop_task(void)
{
    if (!sched_check_wake(&sg_wake))
        return;
    uint8_t oid = 0xff;
    struct sg_endstop *e;
    while ((e = oid_next(&oid, command_config_sg_endstop))) {
        if (!(e->flags & SGF_HOMING))
            continue;
        if (e->prime) {                 // discard the TMC delayed-read priming
            sg_read_result(e);
            e->prime = 0;
            continue;
        }
        uint16_t sg = sg_read_result(e);
        // Adaptive baseline = a decaying running MAX of sg_result. It jumps up to a
        // new free-motion peak and decays slowly toward lower peaks, so it RIDES the
        // temperature drift (sg_result falls as the motor warms). The stall is then a
        // relative fall below THIS, not a fixed number -- temperature-robust.
        uint32_t sg16 = (uint32_t)sg << 4;
        if (sg16 > e->baseline_x16)
            e->baseline_x16 = sg16;                 // track up to a new peak
        else
            e->baseline_x16 -= (e->baseline_x16 >> 8);   // ~1/256-per-sample decay
        if (e->warmup) {                  // still learning the baseline -- don't trigger
            e->warmup--;
            e->trigger_count = e->sample_count;
            continue;
        }
        // Stall = sg a big RELATIVE fall below the baseline, or below a low absolute
        // floor (degenerate-baseline backstop). sample_count still filters the brief
        // per-belt-tooth dips (which also momentarily punch sg toward 0).
        uint16_t base = e->baseline_x16 >> 4;
        uint16_t rel = (uint16_t)(((uint32_t)base * e->drop_permille) / 1000);
        if (sg < rel || sg < e->threshold) {
            uint8_t count = e->trigger_count - 1;
            if (!count) {
                e->trigger_clock = timer_read_time();
                trsync_do_trigger(e->ts, e->trigger_reason);
                e->flags &= ~SGF_HOMING;   // timer self-removes next fire
                continue;
            }
            e->trigger_count = count;
        } else {
            e->trigger_count = e->sample_count;   // reset on any non-stall
        }
    }
}
DECL_TASK(sg_endstop_task);

void
command_config_sg_endstop(uint32_t *args)
{
    struct sg_endstop *e = oid_alloc(args[0], command_config_sg_endstop,
                                     sizeof(*e));
    e->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_sg_endstop, "config_sg_endstop oid=%c spi_oid=%c");

// Arm (or, with sample_count=0, disarm) the SG endstop for a homing move.
void
command_sg_endstop_home(uint32_t *args)
{
    struct sg_endstop *e = oid_lookup(args[0], command_config_sg_endstop);
    sched_del_timer(&e->time);
    e->time.waketime = args[1];
    e->rest_time = args[2];
    e->sample_count = args[3];
    if (!e->sample_count) {
        e->flags = 0;
        e->ts = NULL;
        return;
    }
    e->threshold = args[4];
    e->trigger_count = e->sample_count;
    e->prime = 1;
    e->flags = SGF_HOMING;
    e->ts = trsync_oid_lookup(args[5]);
    e->trigger_reason = args[6];
    e->drop_permille = args[7];
    e->warmup = args[8];
    e->baseline_x16 = 0;             // seeded from the first sample read in the task
    e->time.func = sg_endstop_event;
    sched_add_timer(&e->time);
}
DECL_COMMAND(command_sg_endstop_home,
             "sg_endstop_home oid=%c clock=%u rest_ticks=%u sample_count=%c"
             " threshold=%hu trsync_oid=%c trigger_reason=%c drop_permille=%hu"
             " warmup=%hu");

void
command_sg_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct sg_endstop *e = oid_lookup(oid, command_config_sg_endstop);
    sendf("sg_endstop_state oid=%c homing=%c next_clock=%u",
          oid, !!(e->flags & SGF_HOMING), e->trigger_clock);
}
DECL_COMMAND(command_sg_endstop_query_state, "sg_endstop_query_state oid=%c");
