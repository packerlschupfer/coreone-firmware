"""Unit checks for the idle keep-alive EMITTER in extras/phase_exec.py.

    python3 keepalive_unit_test.py

idle_keepalive_test.py proves that a contiguous hold chain stops the 2^32-wrap fault, but it
hand-builds that chain. This file checks the shipped code actually PRODUCES it -- contiguous,
chunked under _KEEPALIVE_S, at the last commanded value, ra=0 -- and that it stays dormant
while flushes are doing the job. Fakes only the MCU/reactor seams; the methods under test are
the real ones, loaded from the real file.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# Take the path from golden.py rather than hardcoding one. The overlay keeps extras at
# klipper-port/extras while the release fork keeps it at <root>/klippy/extras, and the fold
# already rewrites golden.PHASE_EXEC_PY for that. Borrowing it means this file needs no edit
# when it is folded -- hardcoding '../../extras' here would resolve to <root>/coreone/extras
# in the fork layout and break silently. golden.py is import-safe (main() is __main__-guarded).
import golden
spec = importlib.util.spec_from_file_location('pe_ka', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec
CLOCK = 168000000.0
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class FakeCmd(object):
    def __init__(self):
        self.sent = []
    def send(self, args):
        self.sent.append(list(args))


class FakeMcu(object):
    def print_time_to_clock(self, t):
        return int(t * CLOCK)
    def estimated_print_time(self, et):
        return et


class FakeReactor(object):
    NEVER = 9e99
    def __init__(self):
        self.armed = None
    def monotonic(self):
        return 100.0
    def register_timer(self, cb, when=None):
        return 'tmr'
    def update_timer(self, tmr, when):
        self.armed = when


class FakePrinter(object):
    def __init__(self, r):
        self._r = r
    def get_reactor(self):
        return self._r


class Dut(PhaseExec):
    """Skip the config-driven __init__; wire only what the keep-alive touches."""
    def __init__(self, raw=True):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(self.reactor)
        self.mcu = FakeMcu()
        self.seg_cmd = FakeCmd()
        self.stepper_name = 'stepper_x'
        self.oid = 7
        self._trapq_slave_oid = 8
        self._raw_axis_oids = (7, 8)
        self._steps_per_mm = 200.0
        self._raw_last_xy = (120.0, 45.0)
        self._last_end_posA = 1234.0
        self._last_end_posB = -567.0
        self._inflight = []
        self._max_inflight = 768
        self._trapq_last_pt = 0.0
        self._trapq_engaged = True
        self._trapq_debug = False
        self._ka_segs = 0
        self._ka_fires = 0
        self._ka_maxlag = None
        # The period is now an INSTANCE attribute (config: keepalive_period, runtime:
        # SET_PHASE_KEEPALIVE) rather than the class constant, so a DUT that skips __init__ has
        # to supply it. Default to the shipped value so every existing arm keeps its meaning.
        self._ka_period = PhaseExec._KEEPALIVE_S
        self._mcu_shape = raw
        self._flush_cb = (self._trapq_flush_raw_cb if raw else self._trapq_flush_cb)


def seg_fields(args):
    # seg_cmd.send([oid, start_clock, duration, pos, vel, half_accel, reanchor])
    return {'oid': args[0], 'sc': args[1], 'dur': args[2], 'pos': args[3],
            'vel': args[4], 'ha': args[5], 'ra': args[6]}


KA = PhaseExec._KEEPALIVE_S
print("_KEEPALIVE_S = %.1f s   (worst-case segment age 2x = %.1f s, ceiling 12.783 s)"
      % (KA, 2 * KA))

# ---------------------------------------------------------------- raw / MCU-shape path
print("\nraw (SHAPE=MCU) path, 30 s of silence:")
d = Dut(raw=True)
d._trapq_last_pt = 10.0
d._emit_keepalive(10.0, 40.0)
segs = [seg_fields(a) for a in d.seg_cmd.sent]
xs = [s for s in segs if s['oid'] == 7]
ys = [s for s in segs if s['oid'] == 8]
check(len(xs) == len(ys) == 8, "both axes got %d/%d segments for 30 s at %.0f s chunks"
      % (len(xs), len(ys), KA))
check(all(s['ra'] == 0 for s in segs), "ra == 0 on every segment (never re-anchor)")
f2u = PhaseExec._f2u
check(all(s['vel'] == f2u(0.) and s['ha'] == f2u(0.) for s in segs),
      "every segment is zero-velocity, zero-accel")
check(all(s['pos'] == f2u(120.0 * 200.0) for s in xs),
      "x holds the last commanded axis position (120mm * 200 steps/mm)")
check(all(s['pos'] == f2u(45.0 * 200.0) for s in ys), "y holds 45mm * 200 steps/mm")
M32 = 0xffffffff
contig = all(((xs[i]['sc'] + xs[i]['dur']) & M32) == xs[i + 1]['sc']
             for i in range(len(xs) - 1))
check(contig, "chain is time-CONTIGUOUS mod 2^32 (no ring hole, wrap is normal)")
check(max(s['dur'] for s in xs) <= int(KA * CLOCK) + 1,
      "no chunk exceeds _KEEPALIVE_S (so no duration collides with the reanchor MSB)")
check(abs(d._trapq_last_pt - 40.0) < 1e-6,
      "_trapq_last_pt advanced to the present (%.3f)" % d._trapq_last_pt)
check(d._ka_segs == 8, "_ka_segs counted %d" % d._ka_segs)

# ---------------------------------------------------------------- legacy / host-shape path
print("\nlegacy (SHAPE=host) path, 30 s of silence:")
d = Dut(raw=False)
d._trapq_last_pt = 10.0
d._emit_keepalive(10.0, 40.0)
segs = [seg_fields(a) for a in d.seg_cmd.sent]
xs = [s for s in segs if s['oid'] == 7]
ys = [s for s in segs if s['oid'] == 8]
check(len(xs) == len(ys) == 8, "both motors got %d/%d segments" % (len(xs), len(ys)))
check(all(s['pos'] == f2u(1234.0) for s in xs) and all(s['pos'] == f2u(-567.0) for s in ys),
      "holds _last_end_posA/_last_end_posB (MOTOR space, not axis space)")
check(all(s['ra'] == 0 for s in segs), "ra == 0 on every segment")
check(all(((xs[i]['sc'] + xs[i]['dur']) & 0xffffffff) == xs[i + 1]['sc']
          for i in range(len(xs) - 1)), "chain is time-CONTIGUOUS mod 2^32")

print("\nlegacy path before anything has streamed:")
d = Dut(raw=False)
d._last_end_posA = d._last_end_posB = None
d._emit_keepalive(10.0, 40.0)
check(not d.seg_cmd.sent, "emits nothing (MCU holds its own value; no position to assert)")

# ---------------------------------------------------------------- dormancy
print("\ndormancy -- the timer must not emit while flushes are feeding the ring:")
d = Dut(raw=True)
d._trapq_last_pt = 100.5                 # flush horizon AHEAD of the present (printing)
nxt = d._keepalive_event(100.0)
check(not d.seg_cmd.sent, "silent while printing (_trapq_last_pt ahead of est)")
check(abs(nxt - (100.0 + KA)) < 1e-9, "reschedules itself at +%.1f s" % KA)

d = Dut(raw=True)
d._trapq_last_pt = 100.0 - (KA * 0.5)    # idle, but only half a period behind
d._keepalive_event(100.0)
check(not d.seg_cmd.sent, "silent when the ring head is less than _KEEPALIVE_S behind")

d = Dut(raw=True)
d._trapq_last_pt = 100.0 - (KA * 3)      # genuinely stale
d._keepalive_event(100.0)
check(bool(d.seg_cmd.sent), "FIRES once the head is %.0f s behind" % (KA * 3))
check(all(seg_fields(a)['sc'] <= int(100.0 * CLOCK) for a in d.seg_cmd.sent),
      "every hold is stamped at or before the present (never future-dated, so a resume"
      " move at est+250ms cannot be overlapped or skipped)")

# ------------------------------------------------------------- the KA=0 witness counters
# WHY. A clean print reports KA=0, and that is the right answer -- while the toolhead is fed the
# dormancy is negative and there is nothing to keep alive. But a keep-alive that was never
# armed, whose timer died, or whose threshold is unreachable reports the SAME zero. These
# counters are what separates "watched and not needed" from "silently dead", so they have to be
# right, and -- more importantly -- they must never be able to BREAK the thing they watch.
print("\nthe KA=0 witness (fires / maxlag):")
d = Dut(raw=True)
d._trapq_last_pt = 100.5                 # printing: flush horizon ahead of the present
d._keepalive_event(100.0)
check(d._ka_fires == 1, "counts a fire even when it emits nothing (_ka_fires=%d)" % d._ka_fires)
check(d._ka_segs == 0, "and emits nothing")
check(d._ka_maxlag is not None and d._ka_maxlag < 0,
      "maxlag is NEGATIVE while printing (%.2f s) -- the signature of 'fed, not dormant'"
      % d._ka_maxlag)

d = Dut(raw=True)
for i in range(3):
    d._trapq_last_pt = 100.0 - (KA * 0.25 * (i + 1))
    d._keepalive_event(100.0)
check(d._ka_fires == 3, "fires accumulate across firings (%d)" % d._ka_fires)
check(d._ka_segs == 0, "still under the threshold, so still no hold emitted")
check(abs(d._ka_maxlag - (KA * 0.75)) < 1e-9,
      "maxlag keeps the WORST approach to the period (%.2f s of %.1f s) -- this is what says"
      " how much margin a KA=0 print actually had" % (d._ka_maxlag, KA))

d = Dut(raw=True)
d._trapq_last_pt = 100.0 - (KA * 3)
d._keepalive_event(100.0)
check(d._ka_fires == 1 and d._ka_segs > 0, "a real dormancy both fires and emits")
check(d._ka_maxlag >= KA, "and maxlag reaches the period (%.2f s)" % d._ka_maxlag)

# THE ORDERING GUARD. The witness updates AFTER the emission, inside the same try/except that
# keeps a transient failure from killing the timer. If it updated first, a broken counter would
# be swallowed by that except and the keep-alive would silently stop emitting -- reintroducing
# the 2026-10-11 rail strike from inside its own fix. Simulate exactly that: make the counter
# update explode and assert the hold still went out.
print("\na BROKEN witness must not suppress the hold (the ordering guard):")


class Exploder(object):
    """Raises on += but is otherwise an int-ish."""
    def __iadd__(self, other):
        raise RuntimeError("instrumentation bug")


d = Dut(raw=True)
d._trapq_last_pt = 100.0 - (KA * 3)
d._ka_fires = Exploder()
d._keepalive_event(100.0)
check(bool(d.seg_cmd.sent),
      "the keep-alive STILL emitted its hold with the counter raising")
check(d._ka_segs > 0, "and _ka_segs recorded it (%d)" % d._ka_segs)

print("\ndisengaged / shutdown:")
d = Dut(raw=True)
d._trapq_engaged = False
d._trapq_last_pt = 0.0
check(d._keepalive_event(100.0) == d.reactor.NEVER and not d.seg_cmd.sent,
      "returns NEVER and emits nothing when not engaged")

# ---------------------------------------------------------------- backpressure
print("\nbackpressure:")
d = Dut(raw=True)
d._trapq_last_pt = 10.0
d._max_inflight = 3
d._emit_keepalive(10.0, 100.0)
check(len([s for s in d.seg_cmd.sent if seg_fields(s)['oid'] == 7]) == 3,
      "stops at _max_inflight (3) instead of flooding the ring")
check(d._trapq_last_pt < 100.0,
      "_trapq_last_pt left mid-chain (%.1f) so the next fire resumes there"
      % d._trapq_last_pt)

# ---------------------------------------------------------------- chunk cap vs period
# REGRESSION GUARD for the hazard that making the period CONFIGURABLE introduces. The chunk cap
# used to BE the period, which was safe only because 4.0 < _BRIDGE_MAX (8.0). A configurable
# period breaks that coincidence: phase_shaper.c derives positions from a SIGNED 32-bit tick
# delta, so a segment must stay well inside 2^31 ticks = 12.783 s. A 30 s chunk is 5.04e9 ticks
# -- past 2^32 entirely -- so it would corrupt `trel` outright rather than merely ageing the ring
# out. That would recreate the exact fault the keep-alive exists to prevent, from INSIDE the fix.
print()
print("chunk cap is independent of the period (the 2^31 hazard):")
d = Dut(raw=True)
d._ka_period = 30.0
d._trapq_last_pt = 0.0
d._emit_keepalive(0.0, 90.0)
durs = [seg_fields(a)['dur'] for a in d.seg_cmd.sent]
secs = [x / CLOCK for x in durs]
check(bool(secs), "emitted something at a 30s period")
check(max(secs) <= PhaseExec._BRIDGE_MAX + 1e-9,
      "longest chunk %.2fs is capped at _BRIDGE_MAX (%.1fs), NOT the 30s period"
      % (max(secs), PhaseExec._BRIDGE_MAX))
check(max(durs) < 2**31,
      "longest chunk %d ticks stays inside the signed-32 window" % max(durs))
check(abs(d._ka_chunk() - PhaseExec._BRIDGE_MAX) < 1e-9,
      "_ka_chunk() clamps a 30s period to %.1fs" % PhaseExec._BRIDGE_MAX)
d2 = Dut(raw=True)
d2._ka_period = 2.0
check(abs(d2._ka_chunk() - 2.0) < 1e-9,
      "_ka_chunk() follows the period when it is BELOW the cap (2.0s)")

print("\n%s -- %d check(s) failed" % ("FAIL" if fails else "PASS", len(fails)))
sys.exit(1 if fails else 0)
