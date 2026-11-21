"""Gate: no emitted segment may reach the MCU's 2^31 duration ceiling.

    python3 seg_duration_cap_test.py

🚨 WHY THIS EXISTS. The MCU packs the re-anchor flag into BIT 31 of the duration field:

    src/phase_exec.c:1486   sg->duration = args[2] | (args[6] ? PE_SEG_REANCHOR : 0u)
    src/phase_shaper.h      PE_SEG_REANCHOR 0x80000000   PE_SEG_DUR_MASK 0x7fffffff

and the host encodes args[2] as a plain VLQ uint32 with NO range check. So a segment of 2^31
ticks or more -- 12.783 s at 168 MHz -- arrives TRUNCATED **and with the re-anchor flag
spuriously set**.

That shipped. PRINT_END's presentation lift (`G1 Z{zpres} F900`, zpres = max(axis_max.z-50,
z+20) = 220 mm, F900 binding at 15 mm/s under max_z_velocity 30) is Z-ONLY, so it emits ONE XY
HOLD for the whole lift. On a 7 mm part that is 213/15 = 14.20 s = 2.386e9 ticks, so the MCU saw
a **1.42 s hold with a bogus re-anchor**, stopped being covered, latched stale 8 s later, and
never evaluated the park segments. The gantry did not park -- the head was found mid-bed instead
of at X242 Y211, operator-confirmed 2026-11-01. Introduced by f4aa0db35 (2026-10-13) raising the
lift floor 168 -> zmax-50; at 168 the longest possible lift was 11.20 s and could never reach the
ceiling. Any part under 28.3 mm tall trips it.

🔑 ARM A IS THE ONE THAT MATTERS, and it is the check nobody ran on 2026-10-13: drive a Z-only
move long enough to overflow and assert EVERY emitted duration is below the ceiling. It fails on
the pre-fix emitter. golden.py does NOT cover this -- its longest scenario is ~24k ticks against
a 2^30 cap, so it proves "no regression" and says nothing about the overflow path.

🔑 ARM C pins that splitting is EXACT. A split that merely avoided the ceiling while shifting the
trajectory would trade a stall for a position error, which is strictly worse. Each piece is
recomputed from the move's own constant-accel parameters, so piece boundaries must land on the
un-split trajectory to floating-point noise.

⚠ ARM D pins RE-ANCHOR ON THE FIRST PIECE ONLY. Repeating it per piece rewrites the MCU's
position origin once per piece, which is the first rail strike (2026-10-09). A split that
duplicated `ra` would be a new way to drive the gantry into the frame.
"""
import sys, os, importlib.util

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

PHASE_EXEC_PY = golden.PHASE_EXEC_PY
CLK = 168e6                     # F427 MCU clock
SPM = 100.0                     # steps/mm, arbitrary but fixed
CEILING = 0x80000000            # PE_SEG_REANCHOR -- a duration at/above this collides with it
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))


spec = importlib.util.spec_from_file_location('pe_cap', PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
cls = None
for n in dir(mod):
    o = getattr(mod, n)
    if isinstance(o, type) and hasattr(o, '_raw_axis_segments'):
        cls = o
        break
CAP = getattr(mod, 'PE_SEG_DUR_CAP', None)


class Move(object):
    """Minimal stand-in carrying only the attributes _raw_axis_segments reads."""

    def __init__(self, print_time, move_t, start_x=0., start_y=0.,
                 x_r=0., y_r=0., start_v=0., accel=0.):
        self.print_time = print_time
        self.move_t = move_t
        self.start_x = start_x
        self.start_y = start_y
        self.x_r = x_r
        self.y_r = y_r
        self.start_v = start_v
        self.accel = accel


def clock_of(t):
    return int(round(t * CLK))


def emit(moves, t0, t1, reanchor=False):
    return cls._raw_axis_segments(moves, t0, t1, SPM, clock_of, (0., 0.), reanchor)


def arm_a():
    # THE REAL CASE: a 14.20 s Z-only move. Z-only means x_r = y_r = 0, so the XY side is a
    # hold spanning the whole lift -- exactly what PRINT_END's presentation lift produces.
    LIFT_S = 14.20
    # t1 = the move's own end: the emitter also appends a TRAILING hold out to t1,
    # and counting that in the move's span is what made an earlier draft of this arm
    # report '1.000 s of invented time'. The hold is legitimate; the test was wrong.
    steps, _ept, _exy = emit([Move(1.0, LIFT_S)], 1.0, 1.0 + LIFT_S)
    check(bool(steps), "a 14.20 s Z-only move emits segments (got %d)" % len(steps))
    worst = max(d for (_c, d, _p, _x, _xt, _yt, _r) in steps)
    print("     %d segment(s), longest %d ticks = %.3f s   (ceiling %d = %.3f s)"
          % (len(steps), worst, worst / CLK, CEILING, CEILING / CLK))
    check(worst < CEILING,
          "EVERY emitted duration is below the 2^31 ceiling (worst %d < %d)" % (worst, CEILING))
    check(CAP is not None and worst <= CAP,
          "and below PE_SEG_DUR_CAP (%r), so there is margin rather than a tie" % (CAP,))
    # the pre-fix emitter produced ONE segment for the whole span -- prove we now split
    check(len(steps) > 1,
          "the span was SPLIT rather than emitted whole (%d pieces)" % len(steps))
    # and the total must still cover the full span exactly
    total = sum(d for (_c, d, _p, _x, _xt, _yt, _r) in steps)
    span = clock_of(1.0 + LIFT_S) - clock_of(1.0)
    check(total == span,
          "the pieces sum to the ORIGINAL span exactly (%d == %d) -- no time lost or invented"
          % (total, span))


def arm_b():
    # Clocks must TELESCOPE: each piece starts exactly where the last ended. The evaluator's
    # forward cursor depends on a gapless ring; a hole re-introduces the dry-episode class.
    steps, _e, _x = emit([Move(1.0, 14.20)], 1.0, 20.0)
    gaps = []
    for i in range(1, len(steps)):
        prev_end = steps[i - 1][0] + steps[i - 1][1]
        if steps[i][0] != prev_end:
            gaps.append((i, steps[i][0] - prev_end))
    check(not gaps, "split pieces TELESCOPE with no gap or overlap (offenders %r)" % (gaps,))
    # a long MOVE must split too, not just a hold
    mv, _e2, _x2 = emit([Move(1.0, 14.20, x_r=1.0, start_v=10.0)], 1.0, 20.0)
    worst_mv = max(d for (_c, d, _p, _x, _xt, _yt, _r) in mv)
    check(worst_mv < CEILING,
          "a long MOVE (not just a hold) is also capped (worst %d ticks = %.2f s)"
          % (worst_mv, worst_mv / CLK))
    check(len(mv) > 1, "and is split into %d pieces" % len(mv))
    # THE TRAILING HOLD -- the third append site, uncapped in the first patch. No moves at
    # all, so the emitter holds from t0 straight out to the horizon.
    th, _e3, _x3 = emit([], 1.0, 1.0 + 30.0)
    check(bool(th), "a bare 30 s horizon with no moves still emits a trailing hold")
    worst_th = max(d for (_c, d, _p, _x, _xt, _yt, _r) in th)
    check(worst_th < CEILING,
          "and THAT site is capped too (worst %d ticks = %.2f s) -- enumerating every"
          " steps.append is how this one was caught" % (worst_th, worst_th / CLK))


def arm_c():
    # EXACTNESS. Each piece is recomputed from the move's own parameters, so a piece boundary
    # must sit on the un-split trajectory. Check position AND velocity at every boundary.
    START_V, ACCEL, T = 8.0, 3.0, 14.20
    m = Move(1.0, T, start_x=5.0, start_y=-2.0, x_r=0.6, y_r=0.8,
             start_v=START_V, accel=ACCEL)
    steps, _e, _x = emit([m], 1.0, 1.0 + T)   # no trailing hold: see ARM A
    c0 = steps[0][0]
    worst_p = worst_v = 0.0
    for (sc, _d, _pt, _xy, xt, yt, _r) in steps:
        tau = (sc - c0) / CLK                       # offset into the move, seconds
        dist = START_V * tau + 0.5 * ACCEL * tau * tau
        vel = START_V + ACCEL * tau
        for (got, rate, pos0, ratio) in ((xt, m.x_r, m.start_x, m.x_r),
                                         (yt, m.y_r, m.start_y, m.y_r)):
            want_p = (pos0 + ratio * dist) * SPM
            want_v = rate * vel * SPM
            worst_p = max(worst_p, abs(got[0] - want_p))
            worst_v = max(worst_v, abs(got[1] - want_v))
    print("     worst boundary error: pos %.3e steps, vel %.3e steps/s over %d pieces"
          % (worst_p, worst_v, len(steps)))
    check(worst_p < 1e-6,
          "every piece's START POSITION sits on the un-split trajectory (%.3e)" % worst_p)
    check(worst_v < 1e-6,
          "every piece's START VELOCITY sits on the un-split trajectory (%.3e)" % worst_v)
    # acceleration is constant across the whole move, so every piece must carry the same one
    accs = set(round(xt[2], 9) for (_c, _d, _p, _x, xt, _y, _r) in steps)
    check(len(accs) == 1,
          "and the half-accel term is identical in every piece (%r) -- constant accel" % (accs,))


def arm_d():
    # RE-ANCHOR ON THE FIRST PIECE ONLY. Repeating it per piece rewrites the MCU's origin once
    # per piece: the 2026-10-09 first rail strike.
    steps, _e, _x = emit([Move(1.0, 14.20)], 1.0, 20.0, reanchor=True)
    ras = [r for (_c, _d, _p, _x, _xt, _yt, r) in steps]
    check(len(steps) > 1, "the reanchor case really did split (%d pieces)" % len(steps))
    check(ras[0] == 1, "the FIRST piece carries the re-anchor")
    check(all(r == 0 for r in ras[1:]),
          "and NO later piece does (got %r) -- repeating it is the first-rail-strike bug" % (ras,))


def arm_e():
    # The cap must be a real margin below the ceiling, and mirror the MCU's own constants.
    check(CAP is not None, "PE_SEG_DUR_CAP is exported (got %r)" % (CAP,))
    if CAP is None:
        raise RuntimeError("no cap to check")
    check(CAP < CEILING, "cap (%#x) is below the ceiling (%#x)" % (CAP, CEILING))
    check(CAP <= CEILING // 2,
          "and by at least a factor of two (%.2f s vs %.2f s), not a tie"
          % (CAP / CLK, CEILING / CLK))
    src = open(PHASE_EXEC_PY).read()
    check('PE_SEG_REANCHOR' in src and '0x7fffffff' in src.lower(),
          "the host documents the MCU's bit-31 packing that makes this necessary")


arm("ARM A -- THE REAL CASE: a 14.20 s Z-only lift must not reach 2^31:", arm_a)
arm("ARM B -- pieces telescope, and long MOVES split too:", arm_b)
arm("ARM C -- the split is EXACT on the un-split trajectory:", arm_c)
arm("ARM D -- re-anchor on the FIRST piece only:", arm_d)
arm("ARM E -- the cap mirrors the MCU ceiling with margin:", arm_e)

print()
if fails:
    print("SEG-DURATION-CAP GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("SEG-DURATION-CAP GATE: PASS -- no emitted segment can reach the MCU's bit-31 duration"
      " ceiling, long holds AND long moves are split, the pieces telescope and sum to the"
      " original span, every boundary sits on the un-split trajectory, and the re-anchor flag"
      " appears on the first piece only")
