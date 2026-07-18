"""Regression gate for the GAP-SKIP ring hole (2026-09-23).

    python3 gap_bridge_test.py

Asserts that a multi-minute idle while phase-stepping is ENGAGED does not leave a hole in the
segment ring. phase_shaper.c:122 compares clocks with a SIGNED 32-bit delta
(`int32_t trel = (int32_t)(tau - sg->start_clock)`), meaningful only within +-2^31 ticks =
12.783 s at 168 MHz, and the forward walk is documented to rely on the ring being
time-contiguous (phase_shaper.c:102, "the telescoping-clock invariant"). A hole longer than
that window makes every comparison against the stale cursor garbage and the evaluator raises
PE_EVAL_BEFORE continuously -- seen on hardware as lateBEF=65045, a field whose own doc says
"Must be 0".

MEASURED HERE: a 180 s hole gives 3,151,364 BEFORE events; the bridged ring gives 0.

!! SCOPE, READ THIS BEFORE TRUSTING IT !!
This gate covers the BEFORE storm, which is a real defect and is fixed. It does NOT reproduce
the motion loss seen on the printer -- the travel completes in BOTH arms here. The harness
feeds the whole ring up front and ticks monotonically, so it cannot model segments ARRIVING
after their clock window has already elapsed, which is the separate condition under which the
travel is genuinely skipped (reproduced in /tmp but not portable). Do not read a PASS here as
"pause-during-phase-stepping is safe".

Original docstring:
Does bridging the skipped span with bounded hold segments restore the travel?

Scenario = the real one: a pre-pause ring, a MULTI-MINUTE hole (what GAP_MAX leaves behind),
then the resumed batch. Ticked continuously at 40 kHz across the resume, correct normalised
pulses, retire on. Compared against the SAME scenario with the hole bridged by a chain of
hold segments each well inside the signed-32 window (phase_shaper.c:122).
"""
import sys, os; sys.path.insert(0, '.')
import golden
golden.build_harness()
CLOCK = golden.CLOCK; SPM = 200.0; M32 = 0xffffffff
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)
PAUSE = 180.0
BRIDGE_MAX = 8.0          # seconds per bridge segment (limit is 12.783)

def build(bridge):
    S = int(1.0 * CLOCK)
    segX, segY = [], []
    def add(t_s, dur_s, pos_mm, v_mms, ra=0):
        segX.append((int(round(t_s)) & M32, int(round(dur_s)), pos_mm*SPM, v_mms*SPM, 0.0, ra))
        segY.append((int(round(t_s)) & M32, int(round(dur_s)), 0.0, 0.0, 0.0, ra))
    # pre-pause: a real move then the trailing hold the emitter appends
    add(S, 0.25*CLOCK, 0.0, 40.0)
    add(S + 0.25*CLOCK, 0.05*CLOCK, 10.0, 0.0)
    t = S + 0.30*CLOCK
    resume = S + (0.30 + PAUSE) * CLOCK
    if bridge:
        while t < resume - 1:
            n = min(resume, t + BRIDGE_MAX*CLOCK)
            add(t, n - t, 10.0, 0.0)
            t = n
    add(resume, 0.10*CLOCK, 10.0, 0.0, ra=1)      # lead-in hold, carries the reanchor
    add(resume + 0.10*CLOCK, 0.10*CLOCK, 10.0, 400.0)   # THE TRAVEL 10 -> 50 mm
    add(resume + 0.20*CLOCK, 0.30*CLOCK, 50.0, 0.0)
    return segX, segY, S, resume

def run(bridge):
    segX, segY, S, resume = build(bridge)
    ticks, t = [], int(S + 0.10*CLOCK)
    end = int(resume + 0.45*CLOCK)
    step = int(CLOCK/40000)
    while t < end:
        ticks.append(t & M32); t += step
    got, _ = golden.run_harness(px_dt, px_q[0], py_dt, py_q[0], (segX, segY),
                                ticks, proj, retire=True)
    pre = [g for g in got if g['now'] == (int(resume + 0.05*CLOCK) & M32)]
    a_pre = pre[0]['posA'] if pre else got[0]['posA']
    a_end = got[-1]['posA']
    before = sum(1 for g in got if (g['stx']|g['sty']) & 4)
    held   = sum(1 for g in got if (g['pa']|g['pb']) & 1)
    return (a_end - a_pre)/SPM, before, held, len(segX), len(ticks)

print("%-26s %-15s %-9s %-7s %-6s" % ("ring","motorA travel","BEFORE","HELD","segs"))
print("-"*70)
for bridge, name in ((False, "HOLE (today)"), (True, "BRIDGED (candidate fix)")):
    mv, before, held, nseg, nt = run(bridge)
    ok = "OK" if abs(mv - 40.0) < 0.5 else "*** SHORT ***"
    print("%-26s %+-15.3f %-9d %-7d %-6d %s" % (name, mv, before, held, nseg, ok))
print("\n(expected motorA travel = +40.000 mm; ticks per run = %d)" % nt)


# ---- gate -------------------------------------------------------------------------------
_hole = run(False)
_brid = run(True)
print()
fails = []
if _brid[1] != 0:
    fails.append("bridged ring still raised %d PE_EVAL_BEFORE (must be 0)" % _brid[1])
if _hole[1] <= 0:
    fails.append("hole arm raised no BEFORE -- the vector no longer reproduces the defect")
for nm, r in (("hole", _hole), ("bridged", _brid)):
    if abs(r[0] - 40.0) >= 0.5:
        fails.append("%s arm: motorA travelled %.3f mm, expected 40.000" % (nm, r[0]))
if fails:
    for f in fails:
        print("FAIL:", f)
    raise SystemExit(1)
print("GAP-BRIDGE GATE: PASS  (hole BEFORE=%d -> bridged BEFORE=%d)" % (_hole[1], _brid[1]))
