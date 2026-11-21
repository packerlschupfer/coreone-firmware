"""Gate: a DEAD-WINDOW segment is NEUTRALISED to a hold, never acted on, and re-anchors.

    python3 stale_neutralise_test.py

WHY THIS EXISTS. `stale_arrival_test.py` reproduces the stale-arrival loss against the MCU
evaluator and concludes, in its own docstring, that the fix does NOT belong there: *"this is NOT
an evaluator bug to patch. A segment whose window has already elapsed must never be ACTED ON at
all -- the host should not emit it."* The host already DETECTED the condition (`_stale_segs`) and
emitted it anyway. This gate pins the host-side fix.

THE TWO OUTCOMES OF ACTING ON A DEAD WINDOW, both bad, from the repro's measurements:
    inside the loss band  -> the move is LOST    (silent XY desync)
    outside it            -> the move is JUMPED  (a lurch; the class of event that rams a rail)
The motion itself is unrecoverable -- its time has passed. What the fix buys is removing the
LURCH and making the loss visible (`xy_trusted=0`), not recovering the move.

🔑 WHY NEUTRALISE AND NOT SKIP. Not sending the segment would punch a HOLE in the ring. The
emitter's contract is that clocks TELESCOPE ("each step's start_clock is the previous step's exact
end clock, so the ring is gapless -- the invariant pe_axis_eval's forward cursor relies on"), and
a hole is exactly what produced the BEFORE storm (3,151,364 PE_EVAL_BEFORE; `phase_shaper.c`
compares clocks with a signed 32-bit delta that goes garbage past 12.783 s @168 MHz). So the
segment is still SENT, with identical start_clock and duration, carrying only its own start
position and ZERO velocity/accel. A late HOLD is explicitly harmless -- which is why the
staleness gate only ever counted segments carrying velocity or accel.

🔑 WHY THE RE-ANCHOR IS PART OF THE FIX, not a nicety. After a neutralised run the MCU sits at the
held position while the host's belief has advanced. Without `reanchor=1` on the next LIVE segment
the MCU would traverse that difference in one segment interval -- reintroducing the very lurch
the neutralise just removed.

Real method under test: PhaseExec._trapq_flush_raw_cb, loaded from the real file. The segment
GENERATOR is stubbed so the test controls arrival times exactly; everything from the staleness
compare to the `seg_cmd.send` calls is the shipped code.
"""
import sys, os, importlib.util, logging
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

spec = importlib.util.spec_from_file_location('pe_stale', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec

CLOCK = 168000000.0
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def u2f(u):
    import struct
    return struct.unpack('<f', struct.pack('<I', u))[0]


class Rec(object):
    """Captures seg_cmd sends as decoded (oid, clock, dur, pos, vel, ha, ra)."""

    def __init__(self):
        self.sent = []

    def send(self, args):
        oid, m32, dur, p, v, ha, ra = args
        self.sent.append((oid, m32, dur, u2f(p), u2f(v), u2f(ha), ra))

    def xs(self):
        return [s for s in self.sent if s[0] == 10]


class FakeFFILib(object):
    def trapq_extract_old(self, *a):
        return 0            # no real moves; the generator is stubbed


class FakeFFIMain(object):
    def new(self, spec):
        return [None] * 64


class FakeMCU(object):
    def __init__(self, now_pt):
        self.now_pt = now_pt

    def print_time_to_clock(self, pt):
        return int(pt * CLOCK)

    def estimated_print_time(self, eventtime):
        return self.now_pt


class FakeReactor(object):
    def monotonic(self):
        return 1000.0


class FakePrinter(object):
    def get_reactor(self):
        return FakeReactor()


class Dut(PhaseExec):
    def __init__(self, steps, now_pt, t0=0.75):
        self.stepper_name = 'stepper_x'
        self.printer = FakePrinter()
        self.mcu = FakeMCU(now_pt)
        self.seg_cmd = Rec()
        self._trapq_engaged = True
        self._trapq = object()
        self._raw_last_xy = (0., 0.)
        self._trapq_last_pt = t0
        self._extract_cap = 64
        self._extract_buf = None
        self._ffi_main = FakeFFIMain()
        self._ffi_lib = FakeFFILib()
        self._inflight = []
        self._max_inflight = 1000
        self._raw_axis_oids = (10, 11)
        self._steps_per_mm = 200.0
        self._inv_clock_s = 1.0 / CLOCK
        self._stale_slack = int(0.025 * CLOCK)      # 25 ms, as engage sets
        self._stale_segs = 0
        self._stale_motion = 0.0
        self._stale_warned = False
        self._stale_reanchor = False
        self._reanchor_next = False
        self._gap_skips = 0
        self._gap_moves_kept = 0
        self._trapq_nseg = 0
        self._trapq_debug = False
        self._last_motion_wall = 0.0
        self._steps = steps

    # stub ONLY the generator; the emit path under test is untouched
    def _raw_axis_segments(self, moves, t0, t1, spm, clock_of, last_xy, reanchor=False):
        ra = 1 if reanchor else 0
        out = []
        for (pt, dur_s, pos, vel) in self._steps:
            out.append((int(pt * CLOCK), int(dur_s * CLOCK), pt + dur_s,
                        (pos + 1., pos + 1.), (pos, vel, 0.), (pos, vel, 0.), ra))
            ra = 0
        end_pt = self._steps[-1][0] + self._steps[-1][1] if self._steps else t0
        return out, end_pt, (99., 99.)


def run(steps, now_pt, t0=0.75, flush_to=1.10):
    # Keep (flush_to - t0) UNDER GAP_MAX=1.0s on purpose: a wider window takes the
    # long-idle gap-bridge branch, which prepends its own hold segments and would
    # double every count here. That is a different fix with its own gate
    # (gap_bridge_test.py); this gate is only about the emit path.
    d = Dut(steps, now_pt, t0)
    cap = []
    h = logging.Handler()
    h.emit = lambda r: cap.append(r.getMessage())
    root = logging.getLogger()
    root.addHandler(h)
    lvl = root.level
    root.setLevel(logging.DEBUG)
    try:
        d._trapq_flush_raw_cb(flush_to, 0.)
    finally:
        root.removeHandler(h)
        root.setLevel(lvl)
    return d, cap


# A segment 200 ms in the PAST carrying 50 mm/s, then a live one. now_pt = 1.0.
PAST = [(0.80, 0.02, 5.0, 50.0), (1.05, 0.02, 7.0, 50.0)]

print("ARM A -- the dead segment is SENT (no ring hole) but commands NOTHING:")
d, cap = run(PAST, 1.0)
xs = d.seg_cmd.xs()
check(len(xs) == 2, "both segments were sent -- no hole punched in the ring (got %d)" % len(xs))
if len(xs) == 2:
    dead, live = xs
    check(dead[1] == int(0.80 * CLOCK) and dead[2] == int(0.02 * CLOCK),
          "the dead segment kept its EXACT start_clock and duration (clocks still telescope)")
    check(dead[4] == 0.0 and dead[5] == 0.0,
          "its velocity AND accel were stripped to zero (vel=%r ha=%r)" % (dead[4], dead[5]))
    check(dead[3] == 5.0,
          "it holds its OWN start position, not the endpoint it would have jumped to (pos=%r)"
          % (dead[3],))
    check(live[4] == 50.0, "the LIVE segment still carries its real velocity (%r)" % (live[4],))

print("\nARM B -- the first LIVE segment after a dead run re-anchors:")
if len(xs) == 2:
    check(xs[0][6] == 0, "the neutralised segment itself does NOT re-anchor")
    check(xs[1][6] == 1,
          "the next live segment carries reanchor=1, so the MCU re-seeds instead of"
          " traversing the gap in one interval (the lurch)")
check(d._stale_reanchor is False, "the pending-reanchor flag was consumed, not left armed")

print("\nARM C -- the loss is COUNTED and SURFACED, not silent:")
check(d._stale_segs == 1, "_stale_segs incremented (got %d)" % d._stale_segs)
check(1.9 < d._stale_motion < 2.1,
      "_stale_motion recorded the lost travel: 50mm/s x 20ms = 1.0mm per axis, summed over"
      " BOTH axes = 2.0mm (got %.3f)" % d._stale_motion)
check(any("INTO THE PAST" in m for m in cap), "it logged the dead window")
check(any("LOST" in m and "RE-HOME" in m for m in cap),
      "and the log says the motion is LOST and to RE-HOME -- not 'silently offset'")

print("\nARM D -- a segment inside the slack window is left completely alone:")
# 10 ms late, under the 25 ms slack
d2, _ = run([(0.99, 0.02, 5.0, 50.0)], 1.0, t0=0.95, flush_to=1.05)
x2 = d2.seg_cmd.xs()
check(len(x2) == 1 and x2[0][4] == 50.0,
      "a 10ms-late segment keeps its velocity (slack is 25ms) (vel=%r)" % (x2[0][4],))
check(d2._stale_segs == 0, "and is not counted stale")
check(d2._stale_reanchor is False, "and arms no re-anchor")

print("\nARM E -- a LATE HOLD is harmless and must not trigger any of this:")
d3, cap3 = run([(0.80, 0.02, 5.0, 0.0), (1.05, 0.02, 7.0, 50.0)], 1.0)
x3 = d3.seg_cmd.xs()
check(d3._stale_segs == 0, "a zero-velocity segment in the past is NOT counted stale")
check(len(x3) == 2 and x3[1][6] == 0,
      "and it arms no re-anchor, so the next live segment is untouched")
check(not any("INTO THE PAST" in m for m in cap3), "and nothing is logged")

print("\nARM F -- several consecutive dead segments: ONE re-anchor, on the first live one:")
d4, _ = run([(0.80, 0.02, 1.0, 50.0), (0.85, 0.02, 2.0, 50.0),
             (0.90, 0.02, 3.0, 50.0), (1.05, 0.02, 9.0, 50.0)], 1.0)
x4 = d4.seg_cmd.xs()
check(len(x4) == 4, "all four sent (got %d)" % len(x4))
if len(x4) == 4:
    check(all(s[4] == 0.0 for s in x4[:3]), "all three dead segments neutralised")
    check([s[6] for s in x4] == [0, 0, 0, 1],
          "exactly ONE re-anchor and it is on the first live segment (got %r)"
          % ([s[6] for s in x4],))
check(d4._stale_segs == 3, "all three counted (got %d)" % d4._stale_segs)

print("\nARM G -- the detector is INERT when slack is 0 (never engaged):")
d5 = Dut(PAST, 1.0)
d5._stale_slack = 0
d5._trapq_flush_raw_cb(1.10, 0.)
x5 = d5.seg_cmd.xs()
check(d5._stale_segs == 0 and x5[0][4] == 50.0,
      "slack 0 leaves everything untouched -- no accidental neutralising pre-engage")

print("\nARM H -- the OPERATOR-FACING stop-stats summary must describe the NEW behaviour:")
# Why this arm exists: the fix changed behaviour in TWO places that talk to the operator -- the
# logging.warning at the emit site AND the teardown stop-stats summary. I updated the first and
# missed the second, so the end-of-run banner still read "WERE EMITTED INTO THE PAST ... XY IS
# SILENTLY WRONG" -- both halves false after the fix (they are neutralised, not emitted; and
# xy_trusted=0, so not silent). Caught on re-verification, not by a gate. Now gated.
src = open(golden.PHASE_EXEC_PY).read()
i = src.find('stalewarn = (')
seg = src[i:i + 900] if i >= 0 else ''
check(i >= 0, "found the stalewarn stop-stats message")
check('WERE EMITTED INTO THE PAST' not in seg,
      "it no longer claims the segments were EMITTED (they are neutralised to holds)")
check('SILENTLY' not in seg.upper(),
      "it no longer calls the loss SILENT -- xy_trusted goes 0, so it is detected")
check('NEUTRALISED' in seg.upper(), "it says the segments were NEUTRALISED")
check('xy_trusted' in seg, "it names xy_trusted as the signal the operator can check")
check('LOST' in seg, "it still says the travel is LOST -- the fix does not recover it")

print()
if fails:
    print("STALE-NEUTRALISE GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("STALE-NEUTRALISE GATE: PASS -- a dead-window segment is sent as a HOLD (ring stays"
      " contiguous, nothing commanded), the next live segment re-anchors so the loss cannot"
      " become a lurch, and the loss is counted and logged instead of silent")
