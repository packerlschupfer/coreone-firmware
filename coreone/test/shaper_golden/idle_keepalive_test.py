"""Reproduction for the IDLE-PAUSE self-driving fault, and proof the host keep-alive fixes it.

    python3 idle_keepalive_test.py

WHY THIS EXISTS. 2026-10-11: an estall PAUSE left phase-stepping engaged (deliberately --
resumability is a design goal). The park completed, the toolhead went FULLY idle (klippy
`print_time` frozen, `buffer_time=0.000`) and ~13 s later the machine started driving Y into
the left end of the rail, repeating every ~25 s for as long as it was left alone, with FIVE
lines in klippy.log for the whole episode. No gcode, no host motion -- the MCU was doing it
alone, off whatever segments were still in the ring when the host stopped talking.

This differs from stale_arrival_test.py, which pushes a segment that arrives LATE. Here
NOTHING arrives at all. That is the realistic shape of a pause: every segment emission in
extras/phase_exec.py is flush-driven, and a quiet toolhead produces no flushes, so the ring
simply AGES. `_BRIDGE_MAX = 8.0` exists for exactly this ceiling but can only act inside a
flush, which is why the fix is a TIMER (_keepalive_event) and not a bigger bridge.

ARMS -- identical history, identical ticks, identical measurement points:
  * IDLE        after the park's trailing hold, emit nothing (shipped behaviour)
  * KEEPALIVE   emit a contiguous zero-velocity HOLD every KA_S at the parked position,
                which is exactly what _keepalive_event/_emit_keepalive do
So any difference is attributable to the keep-alive alone.

⚠ ON THE MECHANISM -- do not restate it from the BEFORE branch alone. `trel` wrapping negative
at 2^31 makes the evaluator read "tau precedes this segment", but the newest segment after a
park is a zero-velocity HOLD whose start_pos EQUALS its end_pos, so the BEFORE branch on its
own would command the same value and move nothing. The displacement comes from the BOUNDED
BACKWARD WALK (phase_shaper.c, PE_BACKSCAN_MAX=4): every older segment has an even more
negative wrapped delta, so the walk retreats the full 4 entries into the pre-park MOVES and
commands one of THEIR start positions. Same root cause (the signed-32 wrap), different
consumer -- which is the same trap recorded in stale_arrival_test.py, where guarding the walk
merely handed the loss to BEFORE. The columns below report both so the claim stays checkable.

The keep-alive fixes both consumers for one reason: it keeps the newest segment's start within
2*KA_S of the present, so `trel` is small and POSITIVE and neither the wrap nor the walk is
ever reachable. It does not patch the evaluator -- the MCU-side staleness latch is a separate,
still-needed defence for the case where the host stops for its own reasons.

Tick rate is decimated to TICK_HZ (the ISR is 40 kHz). Valid here because which branch fires
depends only on `tau - start_clock`, not on tick density, and with a handful of segments the
forward walk never approaches PE_ADVANCE_MAX. It keeps a 60 s window to ~60k tokens.
"""
import sys, os; sys.path.insert(0, '.')
import golden
golden.build_harness()
CLOCK = golden.CLOCK; SPM = 200.0; M32 = 0xffffffff
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)

IDLE_S   = 60.0     # how long the host stays quiet; spans 2^31 (12.783 s) and 2^32 (25.567 s)
KA_S     = 4.0      # _KEEPALIVE_S: timer period AND hold-chunk cap
TICK_HZ  = 1000.0
PARK_MM  = 10.0     # where the park leaves the gantry

EV_BEFORE = 1 << 2
EV_PAST   = 1 << 1
EV_EMPTY  = 1 << 0
EV_LAGGED = 1 << 3


def mk(t, dur, pos_mm, v_mms):
    """One segment pair (x carries the motion, y is held)."""
    return ((int(round(t)) & M32, int(round(dur)), pos_mm * SPM, v_mms * SPM, 0.0, 0),
            (int(round(t)) & M32, int(round(dur)), 0.0, 0.0, 0.0, 0))


def history():
    """Pre-pause print moves, the park, and the park's trailing hold.

    Several real moves matter: the backward walk can retreat PE_BACKSCAN_MAX=4 entries, so a
    one-segment history would give it nowhere to go and would hide the fault.
    """
    S = 1.0 * CLOCK
    segs, t, pos = [], S, 0.0
    for v in (40.0, -30.0, 25.0, -35.0, 60.0):      # five short print moves
        segs.append(mk(t, 0.20 * CLOCK, pos, v))
        pos += v * 0.20
        t += 0.20 * CLOCK
    segs.append(mk(t, 0.50 * CLOCK, pos, (PARK_MM - pos) / 0.50))   # the PARK move
    t += 0.50 * CLOCK
    segs.append(mk(t, 0.05 * CLOCK, PARK_MM, 0.0))                  # trailing idle hold
    t += 0.05 * CLOCK
    return segs, S, t                                # t = the moment the host goes quiet


def run(keepalive):
    segs, S, quiet = history()
    events = []
    for sx, sy in segs:
        events.append(('SEG', 0, sx)); events.append(('SEG', 1, sy))
    # The keep-alive holds, stamped exactly as _emit_keepalive would: contiguous from where
    # the ring head stopped, chunked at KA_S, zero velocity, ra=0 (NEVER re-anchor).
    ka = []
    if keepalive:
        t = quiet
        while t < quiet + IDLE_S * CLOCK:
            n = min(quiet + IDLE_S * CLOCK, t + KA_S * CLOCK)
            ka.append((n, mk(t, n - t, PARK_MM, 0.0)))   # (arrival_time, segment pair)
            t = n
    step = int(CLOCK / TICK_HZ)
    end = int(quiet + IDLE_S * CLOCK)
    ticks = list(range(int(S + 0.05 * CLOCK), end, step))
    ki = 0
    for tk in ticks:
        # A keep-alive hold ENTERS THE STREAM when its timer would have fired: at the end of
        # the span it covers, i.e. emitted into the recent past, never future-dated. That is
        # the property _emit_keepalive relies on and the one this arm has to honour.
        while ki < len(ka) and tk >= ka[ki][0]:
            events.append(('SEG', 0, ka[ki][1][0])); events.append(('SEG', 1, ka[ki][1][1]))
            ki += 1
        events.append(('TICK', tk & M32))
    # Pin the C STALENESS LATCH out of the way (PE_STALE_S = 8 s) on BOTH arms, so the only
    # difference between them stays the host keep-alive. With the latch live the IDLE arm
    # would hold at ~8 s and this file would silently stop testing what it claims to -- the
    # keep-alive would look unnecessary because a DIFFERENT fix was doing the work. The latch
    # is proved separately, with the keep-alive taken out of ITS way: stale_latch_test.py.
    got, err = golden.run_harness_timeline(px_dt, px_q[0], py_dt, py_q[0],
                                           events, proj, retire=True, stale=600.0)
    # Reference: the parked position, sampled just after the trailing hold ends.
    ref_t = quiet + 0.01 * CLOCK
    base = None
    worst = 0.0; worst_at = None; first_at = None
    for g in got:
        tnow = g['now']
        # ticks are emitted in increasing real time; recover elapsed-since-quiet in seconds
        if base is None:
            if tnow >= (int(ref_t) & M32):
                base = g['posA']
            continue
        d = (g['posA'] - base) / SPM
        if abs(d) > abs(worst):
            worst = d
        if abs(d) > 0.05 and first_at is None:
            first_at = g
    # Cluster the transitions: a "cycle" is a burst of movement separated from the next by
    # more than a second of stillness. The gap between cluster STARTS is the signature.
    clusters = []
    prev_p = None; prev_t = None
    for i, g in enumerate(got):
        el = (ticks[i] - quiet) / CLOCK
        if el < 0:
            prev_p = g['posA'] / SPM
            continue
        pm = g['posA'] / SPM
        if prev_p is not None and abs(pm - prev_p) > 0.05:
            if prev_t is None or el - prev_t > 1.0:
                clusters.append(el)
            prev_t = el
        prev_p = pm
    st = lambda bit: sum(1 for g in got if (g['stx'] | g['sty']) & bit)
    # elapsed of the first deviation, reconstructed from its index in the tick list
    first_s = None
    if first_at is not None:
        idx = got.index(first_at)
        first_s = (ticks[idx] - quiet) / CLOCK
    bs = 0
    for ln in err.strip().splitlines():
        for f in ln.split():
            if f.startswith('backscan='):
                bs += int(f.split('=')[1])
    return {'worst': worst, 'first_s': first_s, 'clusters': clusters,
            'before': st(EV_BEFORE),
            'past': st(EV_PAST), 'empty': st(EV_EMPTY), 'lagged': st(EV_LAGGED),
            'backscan': bs, 'nka': len(ka), 'nt': len(ticks)}


print("%-12s %-14s %-13s %-9s %-9s %-8s %-7s"
      % ("arm", "worst drift", "first at", "backscan", "BEFORE", "PAST", "holds"))
print("-" * 82)
rows = {}
for name, ka in (("IDLE", False), ("KEEPALIVE", True)):
    r = rows[name] = run(ka)
    print("%-12s %+.3f mm%6s %-13s %-9d %-9d %-8d %-7d"
          % (name, r['worst'], "",
             ("%.3f s" % r['first_s']) if r['first_s'] is not None else "never",
             r['backscan'], r['before'], r['past'], r['nka']))

print()
bad, good = rows['IDLE'], rows['KEEPALIVE']

# The PERIOD is the part that ties this bench repro to what was heard at the machine: bursts
# every 2^32 ticks, each burst being a hard SNAP followed ~2^31 ticks later by a REPLAY of the
# last second of print moves (the ramp back and forth), then a snap home. Two audible events
# per cycle ~11.7 s and ~13.9 s apart, which is why it was reported as "every 20 seconds".
WRAP = 4294967296.0 / CLOCK                      # 2^32 ticks = 25.567 s at 168 MHz
# Stride 2, not 1: each cycle contributes TWO bursts (snap, then replay), so consecutive
# gaps are the ~11.7/~13.9 s intra-cycle halves and it is every OTHER gap that is the wrap.
cyc = [b - a for a, b in zip(bad['clusters'], bad['clusters'][2:])]
print("IDLE      bursts at %s s" % ", ".join("%.3f" % c for c in bad['clusters']))
print("          snap-to-snap %s s vs 2^32/CLOCK = %.3f s"
      % (", ".join("%.3f" % c for c in cyc), WRAP))
print("IDLE      worst drift %+.3f mm with the host silent -- unCOMMANDED motion" % bad['worst'])
print("KEEPALIVE worst drift %+.3f mm over the same %.0f s, %d bursts"
      % (good['worst'], IDLE_S, len(good['clusters'])))
print()
ok = (abs(bad['worst']) > 1.0 and abs(good['worst']) <= 0.05
      and not good['clusters'] and cyc
      and all(abs(c - WRAP) < 0.5 for c in cyc))
print("PASS -- the keep-alive holds position where the shipped path drives off,"
      " and the fault's period is the 2^32 clock wrap"
      if ok else
      "FAIL -- expected IDLE to drift >1mm in bursts one 2^32 wrap apart,"
      " and KEEPALIVE to stay within 0.05mm with no bursts")
sys.exit(0 if ok else 1)
