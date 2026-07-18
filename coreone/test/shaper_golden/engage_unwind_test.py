"""Gate: a FAILED engage unwinds its own damage -- and only its own.

    python3 engage_unwind_test.py

TWO FAULTS THIS PINS, both in cmd_TRAPQ_ENGAGE's failure handling.

(1) direct_mode left set. `_engage_motor` flips direct_mode per motor as its last act. If the
    second motor of a CoreXY pair fails, or the flip's own SPI write exhausts its five retries,
    the earlier motor stays in direct_mode -- an axis that accepts step/dir and does not move.
    Silent until a FIRMWARE_RESTART. The engage now keeps an unwind ledger, enrolled BEFORE the
    write (a raise there may still have landed it, so trusting a successful return would leave
    exactly the motor that failed stuck).

(2) 🚨 THE UNWIND MUST NOT RUN ON THE RE-ENTRY GUARD. A second PHASE_STEP_ON raises while the
    FIRST engage is still live and streaming. The initial version of this wrapper restored
    unconditionally, which would have restored the TMC bus and re-armed Klipper's error checks
    underneath a running phase-stepped print -- exactly the bus collision the suppression
    exists to prevent, corrupting XDIRECT mid-print. So `_engage_undo` doubles as an arming
    flag: None means "this call broke nothing", and the unwind is a no-op.
    ARM A below is that regression test, and it is the reason this file exists.

Unwind order is load bearing and asserted: restore the bus FIRST (a shadowed set_register
swallows the direct_mode clear, so the wrong order LOOKS like it worked), then re-align each
rotor to its frozen MSCNT before releasing direct_mode (releasing blind lets the sequencer snap
the rotor by up to +-1/2 electrical period -- the MSCNT_HOME "phase off 512" failure), and
release in reverse order of engagement.

Real methods under test: cmd_TRAPQ_ENGAGE, _engage_unwind, _engage_motor, _set_tmc_checks.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

spec = importlib.util.spec_from_file_location('pe_unwind', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec
PHASE_UNITS = mod.PHASE_UNITS

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class Boom(Exception):
    pass


class FakeFields(object):
    REGS = {'intpol': 'CHOPCONF', 'direct_mode': 'GCONF'}

    def __init__(self):
        self.vals = {'intpol': 1, 'direct_mode': 0}

    def lookup_register(self, f, default=None):
        return self.REGS.get(f, default)

    def get_field(self, f):
        return self.vals.get(f, 0)

    def set_field(self, f, v):
        self.vals[f] = v
        return v


class FakeMcuTmc(object):
    def __init__(self, name, fail_direct_mode=False):
        self.name = name
        self.fields = FakeFields()
        self.writes = []
        self.fail_direct_mode = fail_direct_mode
        self.direct_mode = 0

    def get_fields(self):
        return self.fields

    def get_register(self, reg_name):
        return {'MSCNT': 520}.get(reg_name, 0)

    def set_register(self, reg_name, val, print_time=None):
        self.writes.append(reg_name)
        if reg_name == 'GCONF':
            # mirror the hardware: the flip may LAND and still report failure
            self.direct_mode = self.fields.get_field('direct_mode')
            if self.fail_direct_mode:
                raise Boom("spi write to GCONF exhausted its retries")


class FakeErrorCheck(object):
    def __init__(self):
        self.check_timer = None

    def start_checks(self):
        self.check_timer = object()
        return False

    def stop_checks(self):
        self.check_timer = None


class FakeCmdHelper(object):
    def __init__(self):
        self.echeck_helper = FakeErrorCheck()

    def get_status(self, eventtime=None):
        return {}


class FakeTmc(object):
    def __init__(self, name, fail_direct_mode=False):
        self.mcu_tmc = FakeMcuTmc(name, fail_direct_mode)
        self._cmdhelper = FakeCmdHelper()
        self.get_status = self._cmdhelper.get_status


class FakeGcode(object):
    def __init__(self):
        self.scripts = []

    def run_script_from_command(self, s):
        self.scripts.append(s)


class FakeToolhead(object):
    def dwell(self, t):
        pass


class FakePrinter(object):
    def __init__(self, tmcs, gcode):
        self._tmcs = tmcs
        self._gcode = gcode
        self._th = FakeToolhead()

    def lookup_objects(self, module=None):
        if module == 'tmc2130':
            return [('tmc2130 %s' % t.mcu_tmc.name, t) for t in self._tmcs]
        return []

    def lookup_object(self, name, default=None):
        if name == 'gcode':
            return self._gcode
        if name == 'toolhead':
            return self._th
        return default


class Dut(PhaseExec):
    def __init__(self, fail_on=None):
        self.tmc = FakeTmc('stepper_x', fail_direct_mode=(fail_on == 'x'))
        self.slave_tmc = FakeTmc('stepper_y', fail_direct_mode=(fail_on == 'y'))
        self.gcode = FakeGcode()
        self.printer = FakePrinter([self.tmc, self.slave_tmc], self.gcode)
        self.stepper_name = 'stepper_x'
        self._n_tmc = 0

    def tmcs(self):
        return [self.tmc, self.slave_tmc]


def regs_shadowed(t):
    return getattr(t.mcu_tmc.get_register, '__self__', None) is not t.mcu_tmc


def checks_shadowed(t):
    ec = t._cmdhelper.echeck_helper
    return getattr(ec.start_checks, '__self__', None) is not ec


# ============================================================ ARM A -- the regression that matters
print("ARM A -- the RE-ENTRY guard must NOT unwind (a live engage is streaming):")
d = Dut()
# Simulate an engage already in progress: fully suppressed, as during a real run.
d._engage_undo = []
d._set_tmc_checks(False, regs=False)
d._set_tmc_checks(False)
check(all(regs_shadowed(t) and checks_shadowed(t) for t in d.tmcs()),
      "precondition: the live run has the bus suppressed on both motors")

# Now a SECOND PHASE_STEP_ON arrives. The guard raises before the body arms anything.
def guard_raises(gcmd):
    raise Boom("phase-stepping already ENGAGED -- PHASE_STEP_OFF before re-engaging")


d._trapq_engage_body = guard_raises
try:
    d.cmd_TRAPQ_ENGAGE(None)
    check(False, "the wrapper swallowed the guard error")
except Boom:
    check(True, "the guard error propagated to the operator")

check(all(regs_shadowed(t) for t in d.tmcs()),
      "THE BUS IS STILL SUPPRESSED -- the unwind did not reach into the running print")
check(all(checks_shadowed(t) for t in d.tmcs()),
      "and the TMC error checks are still shadowed, so nothing will poll the bus")
check(all(t._cmdhelper.echeck_helper.check_timer is None for t in d.tmcs()),
      "no periodic check was armed behind the running engage")
check(d.gcode.scripts == [],
      "and no direct_mode clear was issued mid-run (scripts: %r)" % (d.gcode.scripts,))

# ================================================= ARM B -- an armed failure DOES unwind cleanly
print("\nARM B -- a failure after both motors engaged must clear direct_mode on both:")
d2 = Dut()


def body_engages_then_fails(gcmd):
    d2._engage_undo = []
    d2._set_tmc_checks(False, regs=False)
    d2._engage_motor(d2.tmc, 255, 1, 0)
    d2._engage_motor(d2.slave_tmc, 255, 1, 0)
    d2._set_tmc_checks(False)
    raise Boom("engage blew up after both motors were in direct_mode")


d2._trapq_engage_body = body_engages_then_fails
try:
    d2.cmd_TRAPQ_ENGAGE(None)
    check(False, "the wrapper swallowed the failure")
except Boom:
    check(True, "the failure propagated")

check(not any(regs_shadowed(t) for t in d2.tmcs()), "bus restored on both motors")
check(not any(checks_shadowed(t) for t in d2.tmcs()), "TMC checks restored on both motors")
check(all(t._cmdhelper.echeck_helper.check_timer is not None for t in d2.tmcs()),
      "and the periodic check is armed again")
cleared = [s for s in d2.gcode.scripts if 'direct_mode VALUE=0' in s]
check(len(cleared) == 2, "direct_mode cleared on BOTH motors (%d script(s))" % len(cleared))
check('stepper_y' in cleared[0] and 'stepper_x' in cleared[1],
      "released in REVERSE order of engagement: %r" % (cleared,))
# the align-before-release ordering: XDIRECT must be written before the clear
for t in d2.tmcs():
    w = t.mcu_tmc.writes
    check('XDIRECT' in w, "%s: XDIRECT re-aligned during the unwind" % t.mcu_tmc.name)

# ========================== ARM C -- the ledger is enrolled BEFORE the write, so a failed flip unwinds
print("\nARM C -- a direct_mode write that RAISES must still be unwound (it may have landed):")
d3 = Dut(fail_on='y')


def body_slave_flip_raises(gcmd):
    d3._engage_undo = []
    d3._set_tmc_checks(False, regs=False)
    d3._engage_motor(d3.tmc, 255, 1, 0)          # x engages fine
    d3._engage_motor(d3.slave_tmc, 255, 1, 0)    # y's GCONF write raises


d3._trapq_engage_body = body_slave_flip_raises
try:
    d3.cmd_TRAPQ_ENGAGE(None)
    check(False, "the wrapper swallowed the SPI failure")
except Boom:
    check(True, "the SPI failure propagated")

cleared3 = [s for s in d3.gcode.scripts if 'direct_mode VALUE=0' in s]
check(len(cleared3) == 2,
      "BOTH motors cleared -- including the one whose write raised (%d)" % len(cleared3))
check(any('stepper_y' in s for s in cleared3),
      "the FAILED motor is in the unwind, because it is enrolled before the write")
check(not any(regs_shadowed(t) for t in d3.tmcs()), "bus restored")

# ===================================================== ARM D -- a clean engage disarms the ledger
print("\nARM D -- a SUCCESSFUL engage leaves nothing armed (PHASE_STEP_OFF owns teardown):")
d4 = Dut()


def body_succeeds(gcmd):
    d4._engage_undo = []
    d4._set_tmc_checks(False, regs=False)
    d4._engage_motor(d4.tmc, 255, 1, 0)
    d4._set_tmc_checks(False)
    return 'ok'


d4._trapq_engage_body = body_succeeds
check(d4.cmd_TRAPQ_ENGAGE(None) == 'ok', "the return value is passed through")
check(d4._engage_undo is None, "ledger disarmed after success")
check(regs_shadowed(d4.tmc), "and the suppression is left in place for the run")
check(d4.gcode.scripts == [], "no direct_mode clear on the success path")

# a later unwind call (e.g. a stray) must now be a no-op
d4._engage_unwind()
check(regs_shadowed(d4.tmc), "a stray _engage_unwind() after success does nothing")

print()
if fails:
    print("ENGAGE-UNWIND GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("ENGAGE-UNWIND GATE: PASS -- a failed engage clears direct_mode and restores the checks,"
      " a write that raised is still unwound, and the re-entry guard cannot reach into a"
      " running phase-stepped print")
