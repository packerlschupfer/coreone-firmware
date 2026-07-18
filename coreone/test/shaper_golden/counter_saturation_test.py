"""Gate for SATURATING 16-bit diagnostic counters in src/phase_shaper.c + src/phase_exec.c.

    python3 counter_saturation_test.py

WHY THIS GATE EXISTS. Every diagnostic counter on the shaping path is `uint16_t` and counts
40 kHz ISR TICKS, so a bare `++` wraps every 65536/40000 = 1.638 s and passes back THROUGH
ZERO. That is not merely cosmetic for one of them: `PHASE_XY_CHECK` grants `xy_trusted` on
`lateBEF == 0` (extras/phase_exec.py), and PRINT_END then drives an ABSOLUTE park to
X242 Y211 on that verdict. A counter that returns to its own "healthy" value every 1.6 s can
therefore authorise a park on a position the machine does not have -- the 2026-10-09 rail
strike, reached by a route the 2026-10-11 latch does not cover.

FOUND IN PRODUCTION, 2026-10-12. A completed 4 h 10 m print reported
`PHASE_XY_CHECK: lateBEF=0 ... -> xy_trusted=1` and then, seconds later and from the SAME
MCU fields, `lateBEF=11631` with `MCU-STALE events=2`. Both readings were correct; the field
had wrapped between them.

!! WHY THE EXISTING SUITE WAS BLIND TO IT !!
gap_bridge_test.py measures its BEFORE storm from the per-tick STATUS BITS in the compared
stream (`(g['stx']|g['sty']) & 4`), not from `ax->n_before`. It reports 3,151,364 BEFORE
events -- a vector that wraps the 16-bit field 48 times -- and passes either way. So the
repo already contained a wrapping vector and could not see the wrap. This file asserts the
COUNTER FIELDS, which is the only place the defect is observable. (See the memory note
"test the PRODUCER not just the consumer".)

ARMS. Each measures the same counter at three magnitudes against a self-calibrated per-tick
rate, so the test never hard-codes an impulse count:
  * CAL     a short run, to measure increments-per-tick for this pulse table
  * LINEAR  below the ceiling -> must still be EXACT (saturation must not cost resolution
            where it matters, which is every short engage session)
  * SATURATE past the ceiling -> must read exactly 65535, and must NOT read the wrapped
            value. The wrapped value is computed and asserted against explicitly, so a
            revert to `++` fails loudly instead of quietly matching "nonzero".
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

golden.build_harness()
CLOCK = golden.CLOCK
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)

CNT_MAX  = 65535     # PE_CNT_MAX in src/phase_shaper.h
TICK_HZ  = 40000.0   # the real ISR rate; the counters are defined per tick
STALE_OFF = 600.0    # keep the staleness latch out of the way -- it returns EARLY and would
                     # stop the counters we are measuring (see defences-tested-independently)

fails = []


def diag(stderr, axis):
    """Pull one axis' counter fields out of the harness' stderr diagnostic line."""
    for ln in stderr.splitlines():
        if ln.startswith('axis%d ' % axis):
            return {k: int(v) for k, v in
                    (p.split('=') for p in ln.split()[1:])}
    raise SystemExit('saturation: no axis%d diag line in harness stderr' % axis)


def run_before(nticks):
    """Tick BEFORE a single future segment -> PE_EVAL_BEFORE on every impulse, every tick.

    One segment, stamped to start AFTER the whole tick window. `trel = tau - start_clock` is
    negative for every tick, which is exactly the branch `lateBEF` counts.
    """
    t0 = 1 << 20
    future = t0 + int(nticks * (CLOCK / TICK_HZ)) + int(CLOCK)   # starts 1 s past the last tick
    seg = (future, int(0.1 * CLOCK), 0.0, 0.0, 0.0, 0)
    ticks = [t0 + int(k * (CLOCK / TICK_HZ)) for k in range(nticks)]
    _, err = golden.run_harness(px_dt, px_q[0], py_dt, py_q[0],
                                ([seg], [seg]), ticks, proj,
                                retire=False, stale=STALE_OFF)
    return diag(err, 0)['before']


def run_empty(nticks):
    """Tick an EMPTY ring -> PE_EVAL_EMPTY every tick. Covers a second pe_cnt_inc call site."""
    t0 = 1 << 20
    ticks = [t0 + int(k * (CLOCK / TICK_HZ)) for k in range(nticks)]
    _, err = golden.run_harness(px_dt, px_q[0], py_dt, py_q[0],
                                ([], []), ticks, proj,
                                retire=False, stale=STALE_OFF)
    return diag(err, 0)['empty']


print("%-10s %-9s %-10s %-12s %-12s %s"
      % ("counter", "ticks", "observed", "if-exact", "if-wrapped", "verdict"))

for name, fn in (("lateBEF", run_before), ("empty", run_empty)):
    # CAL: measure this counter's increments-per-tick instead of assuming the impulse count.
    CAL = 64
    cal = fn(CAL)
    if cal == 0:
        fails.append("%s: CAL arm produced 0 -- the vector no longer reaches this branch"
                     % name)
        continue
    rate = cal / CAL
    if rate != int(rate):
        fails.append("%s: non-integral per-tick rate %r -- vector is not uniform"
                     % (name, rate))
        continue
    rate = int(rate)

    for nticks in (CAL, 1024, int(CNT_MAX // rate) + 4096):
        exact = nticks * rate
        got = fn(nticks)
        wrapped = exact & 0xffff
        if exact <= CNT_MAX:
            ok = (got == exact)
            if not ok:
                fails.append("%s: %d ticks below the ceiling must report EXACTLY %d, got %d"
                             % (name, nticks, exact, got))
        else:
            ok = (got == CNT_MAX)
            if not ok:
                fails.append("%s: %d ticks (%d increments) must SATURATE at %d, got %d"
                             % (name, nticks, exact, CNT_MAX, got))
            if got == wrapped and wrapped != CNT_MAX:
                fails.append("%s: counter WRAPPED to %d -- pe_cnt_inc reverted to `++`"
                             % (name, wrapped))
            if got == 0:
                fails.append("%s: counter read ZERO after %d increments -- this is the"
                             " xy_trusted hazard, a park would be authorised here"
                             % (name, exact))
        print("%-10s %-9d %-10d %-12d %-12d %s"
              % (name, nticks, got, exact, wrapped, "ok" if ok else "FAIL"))

print()
if fails:
    for f in fails:
        print("FAIL: %s" % f)
    raise SystemExit("COUNTER-SATURATION GATE: FAIL (%d)" % len(fails))
print("COUNTER-SATURATION GATE: PASS"
      " -- counters are exact below %d and saturate there instead of wrapping" % CNT_MAX)
