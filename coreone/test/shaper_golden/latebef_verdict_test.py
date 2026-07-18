"""Unit checks for the lateBEF verdict in extras/phase_exec.py (_latebef_verdict).

    python3 latebef_verdict_test.py

WHY THIS GATE EXISTS. The teardown banner used to assert "XY IS SILENTLY WRONG" on any nonzero
lateBEF. The real fault needs TWO things: the shaper holding, AND the host advancing commanded
position while it holds. On 2026-10-12 the A1 print finished clean (net=0, JUMP=0, snapMAX=0,
segERR=0, PHASE_XY_CHECK=0 mid-print) and the teardown still printed lateBEF=11631, because
every tick of it accrued in the ~14 s parked tail after the last move. The banner scared the
operator about a part that was dimensionally fine.

The verdict now downgrades ONLY when the measured dormancy accounts for the whole count, and it
must FAIL CLOSED in both directions that matter:
  - a saturated/wrapped counter is a bound, not a count, so it can never buy a downgrade
  - the dormancy is discounted by _LATEBEF_IDLE_GUARD_S, because the flush stamps the clock
    ahead of execution

Fakes only the reactor/printer seams; the method under test is the real one, loaded from the
real file -- same approach as keepalive_unit_test.py, and the golden.PHASE_EXEC_PY path is
borrowed for the same fold-safety reason documented there.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

spec = importlib.util.spec_from_file_location('pe_lb', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec
PE_CNT_MAX = mod.PE_CNT_MAX

CLOCK = 168000000.0
INTERVAL = 4200          # ticks per ISR tick -> 168 MHz / 4200 = 40 kHz
TICK_HZ = CLOCK / INTERVAL
GUARD = PhaseExec._LATEBEF_IDLE_GUARD_S
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class FakeReactor(object):
    def __init__(self):
        self.t = 1000.0
    def monotonic(self):
        return self.t


class FakePrinter(object):
    def __init__(self, r):
        self._r = r
    def get_reactor(self):
        return self._r


class Dut(PhaseExec):
    """Skip the config-driven __init__; wire only what the verdict touches."""
    def __init__(self):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(self.reactor)
        self.stepper_name = 'stepper_x'
        self._interval = INTERVAL
        self._inv_clock_s = 1.0 / CLOCK
        self._last_motion_wall = 0.0


def verdict(lb, idle_s):
    """Run the real method with the gantry parked for idle_s seconds."""
    d = Dut()
    now = d.reactor.t
    d._last_motion_wall = now - idle_s
    return d._latebef_verdict(lb, now=now)


print("tick rate %.0f Hz   idle guard %.1f s   ceiling %d" % (TICK_HZ, GUARD, PE_CNT_MAX))

# ------------------------------------------------------------------ lb == 0: nothing to say
print("\nlateBEF = 0:")
benign, text = verdict(0, 0.0)
check(benign and text == "", "no banner at all, and reported benign")

# ------------------------------------------------------- the A1 case: parked tail explains it
print("\nthe real 2026-10-12 A1 case (lateBEF=11631 after a ~14 s parked tail):")
benign, text = verdict(11631, 14.0)
check(benign, "downgraded: %.1f s of dormancy covers 11631 ticks"
      % (14.0 - GUARD,))
check("PARKED" in text and "NOT" in text, "banner says parked / not implicated")
check("SILENTLY WRONG" not in text, "banner does NOT claim XY is silently wrong")

# ------------------------------------------------- a real mid-print storm must still scream
print("\na genuine storm with no idle tail to hide behind:")
benign, text = verdict(11631, 0.0)
check(not benign, "kept the hard warning when nothing was parked")
check("SILENTLY WRONG" in text, "banner still says XY IS SILENTLY WRONG")
check("Re-home" in text, "banner still tells the operator to re-home")

# the boundary: dormancy worth exactly N ticks must cover N but not N+1
print("\nboundary at the exact tick budget of the dormancy:")
idle = 1.0 + GUARD                      # 1.0 s of CREDITED dormancy
exact = int(1.0 * TICK_HZ)              # = 40000 ticks
check(verdict(exact, idle)[0], "lb == credited ticks (%d) downgrades" % exact)
check(not verdict(exact + 1, idle)[0], "lb == credited ticks + 1 does NOT downgrade")

# ---------------------------------------------------------- the guard must actually discount
print("\nthe idle guard must discount the dormancy, not be ignored:")
# A dormancy of exactly GUARD seconds credits ZERO ticks, so even lb=1 must warn.
check(not verdict(1, GUARD)[0],
      "dormancy == guard (%.1f s) credits nothing, so lb=1 still warns" % GUARD)
check(verdict(1, GUARD + 0.001)[0], "a hair more than the guard credits enough for lb=1")

# ------------------------------------------------------- saturated counter can never downgrade
print("\nfail-closed on an unreadable counter (saturated or pre-patch wrapped):")
benign, text = verdict(PE_CNT_MAX, 3600.0)
check(not benign, "a ceiling value warns even after an hour parked")
check("unbounded" in text, "banner says the true value is unbounded")
check(verdict(PE_CNT_MAX - 1, 3600.0)[0], "one below the ceiling is readable and downgrades")

# ------------------------------------------------ a DUT that never engaged must not downgrade
print("\nfail-closed when the tick rate is unknown (never engaged):")
d = Dut()
d._interval = 0
d._last_motion_wall = d.reactor.t - 3600.0
benign, text = verdict(50, 0.0)       # control
check(not d._latebef_verdict(100, now=d.reactor.t)[0],
      "no interval -> no tick rate -> cannot credit dormancy -> warns")

# ----------------------------------------------- an unmirrored slave must not silently downgrade
print("\nthe slave-mirror hazard (_last_motion_wall left at 0.0 reads as idle-since-epoch):")
d = Dut()                             # _last_motion_wall = 0.0, monotonic = 1000.0
sl_benign, _ = d._latebef_verdict(11631, now=d.reactor.t)
check(sl_benign, "an UNMIRRORED object does downgrade -- which is exactly why"
                 " _mirror_trust_caches must copy _last_motion_wall (see phase_exec.py)")

print()
if fails:
    print("LATEBEF-VERDICT GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("LATEBEF-VERDICT GATE: PASS -- the parked tail is excused, a real storm and an"
      " unreadable counter are not")
