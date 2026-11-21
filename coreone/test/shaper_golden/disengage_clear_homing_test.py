"""Gate: the disengage restore path CLEARS XY homing, independently, and says so when it cannot.

    python3 disengage_clear_homing_test.py

WHY THIS EXISTS. `_suppress_step_gen`'s restore branch does two things that matter after a
phase-stepping run:
  1. re-base the toolhead (`set_position(get_position())`) so the first post-disengage move
     cannot try to bridge engage->now in one impossible step interval ("Internal error in
     stepcompress");
  2. clear the XY homing state, because the re-base neither homes nor clears the flag, so
     `homed_axes` kept reading 'xy' while the open-loop XY reference was invalid -- a macro
     gating on it got a stale-true.

Both used to sit under ONE bare `except Exception: pass`. That cost three things, and this gate
pins the fix for each:
  * you could not tell which had failed, or that anything had;
  * a re-base failure SKIPPED the clear entirely, silently -- the worst case, because a failed
    re-base makes the XY reference LESS trustworthy, so that is precisely when you least want to
    leave the axis marked homed;
  * it made the clear UNFALSIFIABLE from outside: "it worked" and "it was never reached" both
    produce an empty log, so a hardware check for homed_axes dropping could only ever confirm,
    never refute.
Still swallowing on purpose -- a disengage must complete even if these fail, since raising here
would strand direct_mode -- but now it logs.

The cleared axes string is asserted to be exactly "xy": corexy.py does
`for axis, axis_name in enumerate("xyz"): if axis_name in clear_axes`, so it takes a string, and
Z is genuinely untouched by a disengage (clearing it would force a needless Z re-home).

Real method under test: PhaseExec._suppress_step_gen, loaded from the real file.
"""
import sys, os, importlib.util, logging
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

spec = importlib.util.spec_from_file_location('pe_clr', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class Boom(Exception):
    pass


class LogCatcher(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.records = []

    def emit(self, record):
        self.records.append(record.getMessage())

    def text(self):
        return " | ".join(self.records)


class FakeKin(object):
    def __init__(self, fail=False):
        self.cleared = []
        self.fail = fail

    def clear_homing_state(self, clear_axes):
        if self.fail:
            raise Boom("kinematics refused the clear")
        self.cleared.append(clear_axes)


class FakeToolhead(object):
    def __init__(self, fail_rebase=False, fail_clear=False):
        self.kin = FakeKin(fail_clear)
        self.fail_rebase = fail_rebase
        self.flushed = 0
        self.rebased = 0

    def flush_step_generation(self):
        self.flushed += 1

    def get_position(self):
        return [1., 2., 3., 0.]

    def set_position(self, pos):
        if self.fail_rebase:
            raise Boom("set_position refused the re-base")
        self.rebased += 1

    def get_kinematics(self):
        return self.kin


class FakePrinter(object):
    def __init__(self, th):
        self.th = th

    def lookup_object(self, name, default=None):
        if name == 'toolhead':
            return self.th
        return default


class FakeStepper(object):
    def __init__(self):
        self.trapq = object()
        self.restored = None
        self.noted = 0

    def get_trapq(self):
        return self.trapq

    def set_trapq(self, t):
        self.restored = t

    def note_homing_end(self):
        self.noted += 1


class Dut(PhaseExec):
    def __init__(self, fail_rebase=False, fail_clear=False):
        self.th = FakeToolhead(fail_rebase, fail_clear)
        self.printer = FakePrinter(self.th)
        self.stepper_name = 'stepper_x'
        self._suppressed_trapq = []


def run(fail_rebase=False, fail_clear=False, suppress_first=True):
    d = Dut(fail_rebase, fail_clear)
    cap = LogCatcher()
    root = logging.getLogger()
    root.addHandler(cap)
    prev = root.level
    root.setLevel(logging.DEBUG)
    try:
        ms = FakeStepper()
        if suppress_first:
            d._suppress_step_gen([ms], True)      # engage: detach
        d._suppress_step_gen(None, False)         # disengage: restore
    finally:
        root.removeHandler(cap)
        root.setLevel(prev)
    return d, cap


print("ARM A -- the normal path: re-base AND clear, nothing logged:")
d, cap = run()
check(d.th.rebased == 1, "toolhead was re-based")
check(d.th.kin.cleared == ["xy"],
      'clear_homing_state called exactly once with "xy" (got %r)' % (d.th.kin.cleared,))
check(cap.records == [], "no log noise on the happy path (got %r)" % (cap.records,))

print("\nARM B -- THE KEY ONE: a failed re-base must LOG and must NOT skip the clear:")
d, cap = run(fail_rebase=True)
check(d.th.rebased == 0, "the re-base did fail")
check(d.th.kin.cleared == ["xy"],
      "the clear STILL RAN -- a failed re-base is when you least want XY left marked homed")
check(any("RE-BASE failed" in r for r in cap.records),
      "the re-base failure was logged (%s)" % (cap.text()[:90],))
check(any("stepcompress" in r for r in cap.records),
      "and the log names the consequence it leads to")
check(not any("STALE-TRUE" in r for r in cap.records),
      "and it did NOT also claim the homing clear failed")

print("\nARM C -- a failed clear must LOG, and say what it means:")
d, cap = run(fail_clear=True)
check(d.th.rebased == 1, "the re-base still happened")
check(d.th.kin.cleared == [], "the clear did fail")
check(any("STALE-TRUE" in r for r in cap.records),
      "the failure was logged as a stale-true hazard (%s)" % (cap.text()[:90],))
check(any("RE-HOME" in r for r in cap.records), "and it tells the operator to re-home")
check(not any("RE-BASE failed" in r for r in cap.records),
      "and it did NOT also claim the re-base failed -- the two are distinguishable now")

print("\nARM D -- both failing must produce BOTH messages, not one swallowing the other:")
d, cap = run(fail_rebase=True, fail_clear=True)
check(any("RE-BASE failed" in r for r in cap.records)
      and any("STALE-TRUE" in r for r in cap.records),
      "both failures reported (%d records)" % len(cap.records))

print("\nARM E -- nothing was ever suppressed: no re-base, no clear, no log:")
d, cap = run(suppress_first=False)
check(d.th.rebased == 0 and d.th.kin.cleared == [],
      "restore with an empty ledger touches nothing")
check(cap.records == [], "and says nothing")

print("\nARM F -- the ENGAGE direction must never clear homing:")
d = Dut()
d._suppress_step_gen([FakeStepper()], True)
check(d.th.kin.cleared == [], "suppressing step-gen does not clear homing")
check(d.th.rebased == 0, "and does not re-base")

print()
if fails:
    print("DISENGAGE-CLEAR-HOMING GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("DISENGAGE-CLEAR-HOMING GATE: PASS -- XY homing is cleared on disengage, the clear is"
      " independent of the re-base, and each failure is logged distinguishably instead of"
      " swallowed")
