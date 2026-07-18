// Golden-compare harness for the MCU-side input-shaping evaluator.
//
// Compiles THE SHIPPED SOURCE (../../../src/stm32/phase_shaper.c) with plain gcc and replays a
// scripted axis-segment feed against it, so golden.py can assert the C output against the
// Python oracle in extras/phase_exec.py (_shaped_motor / _eval_axis). If phase_shaper.c ever
// grows an MCU dependency this stops building -- which is the point.
//
// Protocol (stdin, whitespace-separated tokens):
//   CLOCK  <ticks_per_second>
//   PULSE  <axis> <n> <a0> <dt0> ... <a[n-1]> <dt[n-1]>    axis: 0=X 1=Y, dt in ticks
//   SEG    <axis> <start_clock> <duration> <start_pos> <start_v> <half_accel> <reanchor>
//   PROJ   <cxA> <cyA> <refA> <cxB> <cyB> <refB>
//   RETIRE <0|1>       1 = advance the ring tail from the cursors after every tick
//   TICK   <now>
//   END
//
// Output, one line per TICK:
//   T <now> <xs> <ys> <vxs> <vys> <posA> <posB> <statusX> <statusY> <tailX> <tailY>
//     <angleA> <angleB> <projA> <projB>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../src/stm32/phase_shaper.c"

#define CAP   4096                 // host-side ring; power of two, mask = CAP-1
#define MASK  (CAP - 1)

static struct phase_seg ring[2][CAP];
static uint16_t head[2], tail[2];
static struct pe_axis axis[2];
static int retire_mode;
static double cxA = 1., cyA = 1., refA = 0., cxB = 1., cyB = -1., refB = 0.;

// Two motors driven through the REAL pe_motor_step -- the projection, hold policy and
// reanchor arithmetic that used to sit untestable in phase_exec.c. `held` mirrors the
// executor's phase_index (last tick's commanded angle), which is what the reanchor ties to.
static struct pe_motor mot[2];
static float mot_last_pos[2];
static uint16_t mot_phase_off[2];
static uint16_t mot_held[2];
static uint8_t reanchor_prev;

static void
push_seg(int a, uint32_t sc, uint32_t dur, float pos, float v, float ha)
{
    uint16_t nh = (head[a] + 1) & MASK;
    if (nh == tail[a]) {
        fprintf(stderr, "harness: ring %d overflow\n", a);
        exit(2);
    }
    struct phase_seg *sg = &ring[a][head[a]];
    sg->start_clock = sc;
    sg->duration = dur;
    sg->start_pos = pos;
    sg->start_v = v;
    sg->half_accel = ha;
    head[a] = nh;
}

int
main(void)
{
    char tok[64];
    double clock_freq = 168e6;

    for (int a = 0; a < 2; a++) {
        head[a] = tail[a] = 0;
        axis[a].npulse = 0;
        pe_axis_init(&axis[a], ring[a], MASK, &head[a], &tail[a],
                     (float)(1.0 / clock_freq));
    }

    while (scanf("%63s", tok) == 1) {
        if (!strcmp(tok, "CLOCK")) {
            if (scanf("%lf", &clock_freq) != 1) return 2;
            for (int a = 0; a < 2; a++)
                axis[a].inv_clock = (float)(1.0 / clock_freq);
        } else if (!strcmp(tok, "PULSE")) {
            int a, n;
            if (scanf("%d %d", &a, &n) != 2) return 2;
            struct pe_pulse p[PE_MAX_PULSE];
            for (int i = 0; i < n; i++) {
                double av; long dt;
                if (scanf("%lf %ld", &av, &dt) != 2) return 2;
                if (i < PE_MAX_PULSE) { p[i].a = (float)av; p[i].dt = (int32_t)dt; }
            }
            pe_axis_set_pulses(&axis[a], (uint8_t)n, p);
        } else if (!strcmp(tok, "SEG")) {
            int a, ra; long sc, dur; double pos, v, ha;
            if (scanf("%d %ld %ld %lf %lf %lf %d", &a, &sc, &dur, &pos, &v, &ha, &ra) != 7)
                return 2;
            // `ra` sets the host's reanchor flag in duration's MSB. The evaluator must mask
            // it off (PE_SEG_DUR); if it ever reads the raw field the duration becomes
            // ~2^31 ticks and the frozen-endpoint branch stops firing.
            push_seg(a, (uint32_t)sc, (uint32_t)dur | (ra ? PE_SEG_REANCHOR : 0u),
                     (float)pos, (float)v, (float)ha);
        } else if (!strcmp(tok, "PROJ")) {
            if (scanf("%lf %lf %lf %lf %lf %lf",
                      &cxA, &cyA, &refA, &cxB, &cyB, &refB) != 6) return 2;
        } else if (!strcmp(tok, "RETIRE")) {
            if (scanf("%d", &retire_mode) != 1) return 2;
        } else if (!strcmp(tok, "TICK")) {
            long now;
            if (scanf("%ld", &now) != 1) return 2;
            float xs = 0.f, ys = 0.f, vxs = 0.f, vys = 0.f;
            uint8_t stx = pe_axis_eval(&axis[0], (uint32_t)now, &xs, &vxs);
            uint8_t sty = pe_axis_eval(&axis[1], (uint32_t)now, &ys, &vys);
            uint8_t st = stx | sty;
            // Same order as pe_shaped_pos: edge-latch, then step.
            pe_reanchor_edge(&reanchor_prev, st, &mot[0], &mot[1]);
            mot[0].cx = (float)cxA; mot[0].cy = (float)cyA; mot[0].pos_ref = (float)refA;
            mot[1].cx = (float)cxB; mot[1].cy = (float)cyB; mot[1].pos_ref = (float)refB;
            float posA, velA, posB, velB;
            uint8_t pa = pe_motor_step(&mot[0], st, xs, vxs, ys, vys, mot_held[0],
                                       &mot_last_pos[0], &mot_phase_off[0], &posA, &velA);
            uint8_t pb = pe_motor_step(&mot[1], st, xs, vxs, ys, vys, mot_held[1],
                                       &mot_last_pos[1], &mot_phase_off[1], &posB, &velB);
            // The commanded electrical angle, computed exactly as pe_update_angle does.
            uint16_t angA = (uint16_t)((pe_pos_to_angle(posA) + mot_phase_off[0]) & SINE_MASK);
            uint16_t angB = (uint16_t)((pe_pos_to_angle(posB) + mot_phase_off[1]) & SINE_MASK);
            printf("T %ld %.9g %.9g %.9g %.9g %.9g %.9g %u %u %u %u %u %u %u %u\n",
                   now, (double)xs, (double)ys, (double)vxs, (double)vys,
                   (double)posA, (double)posB,
                   (unsigned)stx, (unsigned)sty,
                   (unsigned)tail[0], (unsigned)tail[1],
                   (unsigned)angA, (unsigned)angB, (unsigned)pa, (unsigned)pb);
            mot_held[0] = angA;
            mot_held[1] = angB;
            if (retire_mode) {
                tail[0] = pe_axis_retire(&axis[0]);
                tail[1] = pe_axis_retire(&axis[1]);
            }
        } else if (!strcmp(tok, "END")) {
            break;
        } else {
            fprintf(stderr, "harness: bad token '%s'\n", tok);
            return 2;
        }
    }
    // diagnostics go to stderr so they never pollute the compared stream
    for (int a = 0; a < 2; a++)
        fprintf(stderr, "axis%d before=%u past=%u empty=%u backscan=%u lagged=%u\n",
                a, axis[a].n_before, axis[a].n_past, axis[a].n_empty,
                axis[a].n_backscan, axis[a].n_lagged);
    return 0;
}
