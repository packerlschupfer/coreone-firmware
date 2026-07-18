"""Gate for the MCU-side STALENESS LATCH in src/phase_shaper.c.

    python3 stale_latch_test.py

WHY A SECOND GATE. idle_keepalive_test.py proves the HOST fix: extras/phase_exec.py's
_keepalive_event refreshes the segment ring every 4 s so it never ages into the signed-32
wrap. That defence is necessary but not sufficient -- it lives in the process that can
itself be the thing that fails. If klippy dies, USB drops, or the reactor starves past the
wrap, the MCU is alone with a ring that is no longer being fed, and before this latch it
would drive the gantry off whatever was left in it (2026-10-11, the left rail, once per
25.567 s, host completely silent).

So the two gates are deliberately complementary, and each one takes the OTHER defence out of
the way:
  * idle_keepalive_test.py  host keep-alive on/off, latch disabled via `stale=`
  * this file               latch on/off, host keep-alive on/off
Neither fix is allowed to mask the other's regression.

ARMS -- identical history, identical ticks, identical measurement points:
  * NO-LATCH   host silent, latch pushed beyond the timeline -> the shipped pre-latch
               evaluator, which must still reproduce the fault
  * LATCH      host silent, latch at PE_STALE_S -> must HOLD, and must hold STICKILY through
               the 2^32 wrap rather than merely delaying the first burst
  * LATCH+KA   host keep-alive running AND the latch armed -> must NOT fire. This is the
               arm that matters most in daily use: a false latch during a legitimate pause
               would hold a machine that was about to be resumed, so the threshold has to
               clear the keep-alive's own 4 s dry sawtooth with margin.

ON WHAT THE LATCH MEASURES. Not "the age of the newest segment" -- segments are routinely
stamped far in the FUTURE (a bridged 3-minute pause is 23 segments spanning 180 s, and
`now - start_clock` for a 180 s offset overflows 2^32 seven times over). It times the current
UNCOVERED episode: how long `now` has sat outside every segment the impulses landed on. The
NO-LATCH arm's drift and the LATCH+KA arm's silence are the two ends of that distinction.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

golden.build_harness()
CLOCK = golden.CLOCK; SPM = 200.0; M32 = 0xffffffff
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)

IDLE_S   = 60.0     # host-silent span; spans 2^31 (12.783 s) and 2^32 (25.567 s)
KA_S     = 4.0      # _KEEPALIVE_S: the host timer period AND its hold-chunk cap
STALE_S  = 8.0      # PE_STALE_S, the shipped threshold under test
OFF_S    = 600.0    # "latch disabled": beyond any timeline here
TICK_HZ  = 1000.0   # decimated from the 40 kHz ISR; branch choice is tick-density free
PARK_MM  = 10.0

EV_EMPTY  = 1 << 0
EV_PAST   = 1 << 1
EV_BEFORE = 1 << 2
EV_LAGGED = 1 << 3
EV_STALE  = 1 << 5


def mk(t, dur, pos_mm, v_mms):
    """One segment pair (x carries the motion, y is held)."""
    return ((int(round(t)) & M32, int(round(dur)), pos_mm * SPM, v_mms * SPM, 0.0, 0),
            (int(round(t)) & M32, int(round(dur)), 0.0, 0.0, 0.0, 0))


def history():
    """Pre-pause print moves, the park, and the park's trailing hold.

    Five real moves, not one: the bounded backward walk can retreat PE_BACKSCAN_MAX=4
    entries, and a short history would give it nowhere to go and hide the fault.
    """
    S = 1.0 * CLOCK
    segs, t, pos = [], S, 0.0
    for v in (40.0, -30.0, 25.0, -35.0, 60.0):
        segs.append(mk(t, 0.20 * CLOCK, pos, v))
        pos += v * 0.20
        t += 0.20 * CLOCK
    segs.append(mk(t, 0.50 * CLOCK, pos, (PARK_MM - pos) / 0.50))   # the PARK move
    t += 0.50 * CLOCK
    segs.append(mk(t, 0.05 * CLOCK, PARK_MM, 0.0))                  # trailing idle hold
    t += 0.05 * CLOCK
    return segs, S, t                                # t = the moment the host goes quiet


def run(keepalive, stale_s):
    segs, S, quiet = history()
    events = []
    for sx, sy in segs:
        events.append(('SEG', 0, sx)); events.append(('SEG', 1, sy))
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
        # A keep-alive hold enters the stream when its timer would have fired: at the END of
        # the span it covers, i.e. emitted into the recent past, never future-dated.
        while ki < len(ka) and tk >= ka[ki][0]:
            events.append(('SEG', 0, ka[ki][1][0])); events.append(('SEG', 1, ka[ki][1][1]))
            ki += 1
        events.append(('TICK', tk & M32))
    got, err = golden.run_harness_timeline(px_dt, px_q[0], py_dt, py_q[0],
                                           events, proj, retire=True, stale=stale_s)

    ref_t = int(quiet + 0.01 * CLOCK) & M32
    base = None
    worst = 0.0
    first_s = None
    stale_from = None
    for i, g in enumerate(got):
        el = (ticks[i] - quiet) / CLOCK
        if (g['stx'] | g['sty']) & EV_STALE and stale_from is None:
            stale_from = el
        if base is None:
            if g['now'] >= ref_t:
                base = g['posA']
            continue
        d = (g['posA'] - base) / SPM
        if abs(d) > abs(worst):
            worst = d
        if abs(d) > 0.05 and first_s is None:
            first_s = el

    st = lambda bit: sum(1 for g in got if (g['stx'] | g['sty']) & bit)
    # The harness prints one diagnostic line PER AXIS. Keep them separate rather than summing:
    # the latch is per-axis state, so "X latched once and Y never did" and "each latched once"
    # are different outcomes that a sum cannot tell apart -- and a half-latched pair would be a
    # real bug (the two motors are driven from both axes, so one stale axis poisons both).
    per_axis = []
    for ln in err.strip().splitlines():
        if not ln.startswith('axis'):
            continue
        d = {}
        for f in ln.split():
            if '=' in f:
                k, v = f.split('=', 1)
                try:
                    d[k] = int(v)
                except ValueError:
                    pass
        per_axis.append(d)
    return {'worst': worst, 'first_s': first_s, 'stale_from': stale_from,
            'before': st(EV_BEFORE), 'past': st(EV_PAST), 'stale': st(EV_STALE),
            'backscan': sum(d.get('backscan', 0) for d in per_axis),
            'events': [d.get('stale', 0) for d in per_axis],
            'latched': [d.get('latched', 0) for d in per_axis],
            'nka': len(ka)}


print("%-11s %-13s %-11s %-10s %-9s %-8s %-7s %-8s"
      % ("arm", "worst drift", "first at", "latch at", "backscan", "BEFORE", "events", "latched"))
print("-" * 88)
rows = {}
for name, ka, stale_s in (("NO-LATCH", False, OFF_S),
                          ("LATCH",    False, STALE_S),
                          ("LATCH+KA", True,  STALE_S)):
    r = rows[name] = run(ka, stale_s)
    print("%-11s %+.3f mm%4s %-11s %-10s %-9d %-8d %-7s %-8s"
          % (name, r['worst'], "",
             ("%.3f s" % r['first_s']) if r['first_s'] is not None else "never",
             ("%.3f s" % r['stale_from']) if r['stale_from'] is not None else "never",
             r['backscan'], r['before'],
             ",".join(str(v) for v in r['events']),
             ",".join(str(v) for v in r['latched'])))

raw, latched, both = rows['NO-LATCH'], rows['LATCH'], rows['LATCH+KA']
WRAP = 4294967296.0 / CLOCK                      # 2^32 ticks = 25.567 s at 168 MHz
print()
print("NO-LATCH  drifts %+.3f mm, first at %s -- the shipped pre-latch evaluator"
      % (raw['worst'], ("%.3f s" % raw['first_s']) if raw['first_s'] else "never"))
print("LATCH     holds  %+.3f mm, latched from %.3f s (threshold %.1f s) and STILL latched"
      " at %.0f s -- sticky through the %.3f s wrap"
      % (latched['worst'], latched['stale_from'] or -1, STALE_S, IDLE_S, WRAP))
print("LATCH+KA  holds  %+.3f mm with %d keep-alive segments and %s latch events"
      " -- no false positive against the keep-alive's %.0f s dry sawtooth"
      % (both['worst'], both['nka'],
         ",".join(str(v) for v in both['events']), KA_S))
print()

checks = [
    # the repro must still work, or the arms below prove nothing
    ("NO-LATCH reproduces the fault (>1mm)", abs(raw['worst']) > 1.0),
    ("NO-LATCH never latches", raw['events'] == [0, 0]),
    # the latch must hold, and hold STICKILY -- not merely postpone the first burst
    ("LATCH holds position (<=0.05mm)", abs(latched['worst']) <= 0.05),
    # Once per axis, not once in total: BOTH axes feed BOTH motors through the CoreXY
    # projection, so a latch on one axis and not the other would be the broken outcome.
    ("LATCH engages exactly once on each axis", latched['events'] == [1, 1]),
    ("LATCH engages near the threshold",
     latched['stale_from'] is not None and STALE_S <= latched['stale_from'] < STALE_S + 1.0),
    ("LATCH still held at the end (sticky past the wrap)", latched['latched'] == [1, 1]),
    # the wrap's two consumers must both be unreachable, not just one of them
    ("LATCH silences the backward walk", latched['backscan'] == 0),
    ("LATCH silences BEFORE", latched['before'] == 0),
    # and the whole thing must stay out of the way of a healthy host
    ("LATCH+KA never fires", both['events'] == [0, 0] and both['stale'] == 0),
    ("LATCH+KA holds position (<=0.05mm)", abs(both['worst']) <= 0.05),
]
bad = [n for n, ok in checks if not ok]
for n, ok in checks:
    print("  %-52s %s" % (n, "ok" if ok else "FAIL"))
print()
print("STALE-LATCH GATE: PASS" if not bad else "STALE-LATCH GATE: FAIL -- " + "; ".join(bad))
sys.exit(0 if not bad else 1)
