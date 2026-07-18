// Input-shaping convolution for the phase-stepping executor — axis-space evaluator.
// See phase_shaper.h for the design contract. DEPENDENCY-FREE: <stdint.h> only.
//
// This is a transcription of `_build_st_forward` (extras/phase_exec.py:869), which is
// itself the forward-cursor optimization of `_shaped_motor` / `_eval_axis` — the Python
// replica of chelper/kin_shaper.c. test/shaper_golden/ asserts this file against those.
//
// The one structural difference from the Python is deliberate and documented at the
// containment walk below.

#include "phase_shaper.h"

void
pe_axis_init(struct pe_axis *ax, const struct phase_seg *ring, uint16_t mask,
             const volatile uint16_t *head, const volatile uint16_t *tail,
             float inv_clock)
{
    ax->ring = ring;
    ax->head = head;
    ax->tail = tail;
    ax->mask = mask;
    ax->inv_clock = inv_clock;
    ax->n_before = ax->n_past = ax->n_empty = ax->n_backscan = ax->n_lagged = 0;
    for (uint8_t i = 0; i < PE_MAX_PULSE; i++)
        ax->cur[i] = tail ? *tail : 0;
    if (!ax->npulse)
        pe_axis_set_pulses(ax, 0, 0);
}

void
pe_axis_set_pulses(struct pe_axis *ax, uint8_t n, const struct pe_pulse *p)
{
    if (!n || !p) {                      // identity: reduces to un-shaped evaluation
        ax->p[0].a = 1.0f;
        ax->p[0].dt = 0;
        ax->npulse = 1;
        return;
    }
    if (n > PE_MAX_PULSE)
        n = PE_MAX_PULSE;
    for (uint8_t i = 0; i < n; i++)
        ax->p[i] = p[i];
    ax->npulse = n;
}

void
pe_axis_reset(struct pe_axis *ax, uint16_t tail)
{
    for (uint8_t i = 0; i < PE_MAX_PULSE; i++)
        ax->cur[i] = tail;
}

uint8_t
pe_axis_eval(struct pe_axis *ax, uint32_t now, float *pos, float *vel)
{
    const struct phase_seg *ring = ax->ring;
    uint16_t mask = ax->mask;
    uint16_t head = *ax->head;
    uint16_t tail = *ax->tail;
    uint16_t depth = (head - tail) & mask;
    if (!depth) {                        // no segments at all -> caller holds its own value
        ax->n_empty++;
        return PE_EVAL_EMPTY;
    }

    float p_acc = 0.0f, v_acc = 0.0f;
    uint8_t status = PE_EVAL_OK;
    uint8_t npulse = ax->npulse;

    for (uint8_t i = 0; i < npulse; i++) {
        // Absolute time this impulse samples. dt straddles zero (see phase_shaper.h), so
        // tau may be behind OR ahead of `now`. Wrapping u32 arithmetic throughout; all
        // comparisons are on signed differences.
        uint32_t tau = now + (uint32_t)ax->p[i].dt;

        uint16_t c = ax->cur[i];
        uint16_t rel = (c - tail) & mask;
        if (rel >= depth) {              // stale cursor (ring flushed/re-seeded under us)
            c = tail;
            rel = 0;
        }

        // FORWARD walk: advance while the NEXT segment has already begun at tau. This is
        // the whole point of the cursor -- pts are sampled in increasing `now` and each dt
        // is fixed, so tau is monotonic per impulse and this is O(1) amortized instead of
        // the O(log n) bisection _eval_axis does.
        uint16_t steps = 0;
        while (rel + 1 < depth) {
            uint16_t nxt = (c + 1) & mask;
            if ((int32_t)(ring[nxt].start_clock - tau) > 0)
                break;                   // next segment starts after tau -> `c` is the one
            c = nxt;
            rel++;
            if (++steps >= PE_ADVANCE_MAX) {
                status |= PE_EVAL_LAGGED;
                ax->n_lagged++;
                break;
            }
        }

        // BACKWARD walk (bounded). The Python keeps a containment back-scan to survive
        // trapq shadow/overlap entries; MCU segments are time-contiguous by construction
        // (the telescoping-clock invariant in _trapq_flush_cb) so that case cannot arise
        // here. This walk exists for a different reason: the axis cursors are SHARED by
        // both motors while the ISR refreshes them round-robin (one motor per tick), so if
        // the two motors are ever given different lead_ticks, `now` oscillates by their
        // difference and a forward-only cursor would over-advance and evaluate the wrong
        // segment -- a silently wrong commanded angle, the worst failure mode on this path.
        // Cheap insurance: never taken when the leads match.
        uint8_t back = 0;
        while (rel > 0 && back < PE_BACKSCAN_MAX
               && (int32_t)(tau - ring[c].start_clock) < 0) {
            c = (c - 1) & mask;
            rel--;
            back++;
        }
        if (back)
            ax->n_backscan++;
        ax->cur[i] = c;

        const struct phase_seg *sg = &ring[c];
        int32_t trel = (int32_t)(tau - sg->start_clock);
        uint32_t dur = PE_SEG_DUR(sg);
        float a = ax->p[i].a;
        if (sg->duration & PE_SEG_REANCHOR)
            status |= PE_EVAL_REANCHOR;   // caller edge-detects; see phase_shaper.h

        if (trel < 0) {
            // tau precedes the oldest retained segment: the history this impulse needs has
            // already been retired. Python's equivalent (tau before moves[0]) holds the
            // first move's start statically, so match that -- but flag it, because on the
            // MCU it means the lookback window was too short, which is a real bug class.
            status |= PE_EVAL_BEFORE;
            ax->n_before++;
            p_acc += a * sg->start_pos;
        } else if ((uint32_t)trel > dur) {
            // Past this segment's end. Contiguous segments make this the newest-segment
            // case in practice (ring dry ahead of tau); Python freezes at the endpoint with
            // vel = accel = 0, which is what the idle/dwell path wants. Matched exactly.
            // NOTE: no dry-COAST here on purpose -- that policy belongs to the caller, on
            // the PROJECTED position, so a simultaneous X and Y coast cannot sum past the
            // half-electrical-period clamp (Fix 3).
            float d = (float)dur * ax->inv_clock;
            p_acc += a * (sg->start_pos + (sg->start_v + sg->half_accel * d) * d);
            if (rel + 1 >= depth) {
                status |= PE_EVAL_PAST_HEAD;
                ax->n_past++;
            }
        } else {
            float e = (float)trel * ax->inv_clock;
            p_acc += a * (sg->start_pos + (sg->start_v + sg->half_accel * e) * e);
            v_acc += a * (sg->start_v + 2.0f * sg->half_accel * e);
        }
    }

    *pos = p_acc;
    *vel = v_acc;
    return status;
}

uint8_t
pe_reanchor_edge(uint8_t *prev, uint8_t st, struct pe_motor *m0, struct pe_motor *m1)
{
    uint8_t now = (st & PE_EVAL_REANCHOR) ? 1 : 0;
    uint8_t edge = now && !*prev;
    if (edge) {
        if (m0)
            m0->reanchor_pend = 1;
        if (m1)
            m1->reanchor_pend = 1;
    }
    *prev = now;
    return edge;
}

uint8_t
pe_motor_step(struct pe_motor *mm, uint8_t st,
              float xp, float xv, float yp, float yv,
              uint16_t phase_index_held,
              float *last_pos, uint16_t *phase_offset,
              float *pos, float *vel)
{
    // EMPTY (no segments) or LAGGED (the forward cursor walk saturated, so the value belongs
    // to a stale segment) both mean the evaluation cannot be commanded. Hold the last
    // position instead: commanding a stale angle is the one failure mode that can slip a
    // pole, and holding is what the host's explicit idle HOLD segment already relies on.
    if (st & (PE_EVAL_EMPTY | PE_EVAL_LAGGED)) {
        *pos = *last_pos;
        *vel = 0.f;
        return PE_PROJ_HELD | PE_PROJ_DRY;
    }
    float p = mm->cx * xp + mm->cy * yp - mm->pos_ref;
    *pos = p;
    *vel = mm->cx * xv + mm->cy * yv;
    *last_pos = p;
    uint8_t flags = (st & PE_EVAL_PAST_HEAD) ? PE_PROJ_DRY : 0;

    // RE-ANCHOR (Prusa reset_from_halt / set_phase_origin). The legacy pe_seg_advance path
    // ties a resume to the segment's start_pos; under convolution the position at that
    // instant is the sum over the whole shaper window and no single segment's start_pos
    // means anything, so anchor on the PROJECTED value.
    //
    // The invariant this establishes, and what the golden test asserts: with
    //     phase_offset = phase_index_held - angle(p)
    // the commanded angle
    //     (angle(p) + phase_offset) & SINE_MASK == phase_index_held
    // so the field does NOT move on the anchoring tick, and everything after is relative to
    // the angle the commutation was actually driving. No MSCNT re-read -- that would
    // re-inject the +-1/2-period homing scatter.
    if (mm->reanchor_pend) {
        *phase_offset = (uint16_t)(((int32_t)phase_index_held - pe_pos_to_angle(p))
                                   & SINE_MASK);
        mm->reanchor_pend = 0;
        flags |= PE_PROJ_REANCHORED;
    }
    return flags;
}

uint16_t
pe_axis_retire(struct pe_axis *ax)
{
    uint16_t mask = ax->mask;
    uint16_t head = *ax->head;
    uint16_t tail = *ax->tail;
    uint16_t depth = (head - tail) & mask;
    if (!depth)
        return tail;
    uint16_t minrel = depth;
    for (uint8_t i = 0; i < ax->npulse; i++) {
        uint16_t rel = (ax->cur[i] - tail) & mask;
        if (rel >= depth)                // stale cursor -> retain everything, re-sync later
            return tail;
        if (rel < minrel)
            minrel = rel;
    }
    // Hold back PE_BACKSCAN_MAX segments BEHIND the oldest cursor. Retiring right up to it
    // races the bounded backward walk in pe_axis_eval: if `now` then steps backwards, the
    // cursor needs a segment retirement has already freed for reuse, and the walk stops at
    // the tail and evaluates the wrong one. The two mechanisms have to agree on how far back
    // "back" can go, and PE_BACKSCAN_MAX is exactly that number.
    //
    // Found by the golden gate's retire-equivalence check once it was retargeted at the
    // machine's LIVE shapers (mzv 63.6 / 49.4). The previous EI@55.4 / MZV@43.4 vectors
    // masked it: their longer lookback put the oldest cursor further from the head, so a
    // backward step did not reach the retired tail. Costs 4 retained segments.
    if (minrel <= PE_BACKSCAN_MAX)
        return tail;
    return (tail + (uint16_t)(minrel - PE_BACKSCAN_MAX)) & mask;
}
