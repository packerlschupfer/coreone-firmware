"""Gate: a QUEUED Klipper enable callback can never see the TMC register shadow without the
check shadow (the 2026-10-15 GSTAT shutdown).

    python3 tmc_enable_race_test.py

THE BUG THIS PINS. phase_exec's engage issues `SET_STEPPER_ENABLE ... ENABLE=1`. tmc.py answers
the enable-line state change with

    self.printer.get_reactor().register_callback(enable_disable_cb)

so `_do_enable` runs LATER, at some reactor pass. Its second act is
`echeck_helper.start_checks()`, and start_checks READS GSTAT SYNCHRONOUSLY before arming its
timer (tmc.py:198-201) -- it is not only the periodic check that touches the bus:

    self._query_register(self.drv_status_reg_info)
    if self.gstat_reg_info is not None:
        cleared_flags = self._query_register(self.gstat_reg_info, try_clear=self.clear_gstat)

The register shadow used to go up ~100 lines after the enable, with reactor yield points in
between (SET_TMC_CURRENT, _engage_motor's dwell). So the callback could land in a window where
`get_register` was shadowed; if `start_checks` was live at that instant it read the shadow's
GSTAT, which TMCErrorCheck masks with err_mask=0xFFFFFFFF and (on a TMC2130, clear_gstat=False)
cannot clear, so _query_register raises after three tries -> invoke_shutdown. That is the
2026-10-15 shutdown. The two engages in that log differ precisely in whether a reactor pass
landed mid-engage.

THE INVARIANT, which is what this file actually asserts: `start_checks` must be shadowed no
later than `get_register`, at every observable point. Then the fatal combination is unreachable
by construction rather than merely unlikely, and no reasoning about reactor timing is required.

Fakes mirror Klipper's real object graph, because phase_exec resolves the error-check object by
walking it: TMC2130 does `self.get_status = cmdhelper.get_status` (tmc2130.py:318), so
`obj.get_status.__self__` is the TMCCommandHelper, which owns `.echeck_helper`. Get that
resolution wrong and the suppression silently lands on the wrong object -- so the fake keeps the
same shape rather than exposing a convenient attribute.

The method under test is the REAL `_set_tmc_checks`, loaded from the real file.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

spec = importlib.util.spec_from_file_location('pe_race', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
PhaseExec = mod.PhaseExec

GSTAT_ERR_MASK = 0xFFFFFFFF      # TMCErrorCheck.__init__: [0, reg, 0xffffffff, 0xffffffff, 0]
CS_ACTUAL_MASK = 0x001f0000      # DRV_STATUS cs_actual; 0 reads as "driver reset" and raises
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class Raised(Exception):
    pass


class FakeMcuTmc(object):
    """Stand-in for MCU_TMC_SPI. Real bus answers: GSTAT clean, DRV_STATUS cs_actual=0x1F."""
    REAL = {'GSTAT': 0x00000000, 'DRV_STATUS': 0x8112003d, 'MSCNT': 520}

    def __init__(self):
        self.reads = []

    def get_register(self, reg_name):
        self.reads.append(reg_name)
        return self.REAL.get(reg_name, 0)

    def set_register(self, reg_name, val, print_time=None):
        pass


class FakeErrorCheck(object):
    """Replays tmc.py's start_checks closely enough to raise where the real one raises."""
    def __init__(self, mcu_tmc):
        self.mcu_tmc = mcu_tmc
        self.clear_gstat = False          # TMC2130 quirk: cannot clear, so cannot recover
        self.check_timer = None
        self.started = 0

    def _query(self, reg_name, err_mask, cs_actual_mask):
        count = 0
        while 1:
            val = self.mcu_tmc.get_register(reg_name)
            if not val & err_mask:
                if not cs_actual_mask or val & cs_actual_mask:
                    return
                # cs_actual == 0 -> Klipper reads a driver reset
            count += 1
            if count >= 3:
                raise Raised("TMC reports error: %s: %08x" % (reg_name, val))

    def start_checks(self):
        self.started += 1
        self._query('DRV_STATUS', 0, CS_ACTUAL_MASK)
        self._query('GSTAT', GSTAT_ERR_MASK, 0)
        self.check_timer = object()
        return False

    def stop_checks(self):
        self.check_timer = None


class FakeCmdHelper(object):
    def __init__(self, mcu_tmc):
        self.echeck_helper = FakeErrorCheck(mcu_tmc)

    def get_status(self, eventtime=None):
        return {}

    # tmc.py: _do_enable writes registers, THEN reads GSTAT via start_checks
    def do_enable(self):
        self.echeck_helper.start_checks()


class FakeTmc2130(object):
    def __init__(self):
        self.mcu_tmc = FakeMcuTmc()
        self._cmdhelper = FakeCmdHelper(self.mcu_tmc)
        # tmc2130.py:318 -- the bound method IS the handle phase_exec resolves through
        self.get_status = self._cmdhelper.get_status


class FakePrinter(object):
    def __init__(self, objs):
        self._objs = objs

    def lookup_objects(self, module=None):
        if module == 'tmc2130':
            return [('tmc2130 stepper_%s' % n, o) for n, o in self._objs]
        return []


class Dut(PhaseExec):
    """Skip the config-driven __init__; wire only what _set_tmc_checks touches."""
    def __init__(self):
        self.objs = [('x', FakeTmc2130()), ('y', FakeTmc2130())]
        self.printer = FakePrinter(self.objs)
        self._n_tmc = 0

    def tmcs(self):
        return [o for _, o in self.objs]


def _is_shadowed(fn, owner):
    """True if `fn` is NOT owner's own bound method, i.e. phase_exec replaced it.

    Compare __self__, never the function object. `obj.m is obj.m` is False for a bound method
    every time -- a fresh wrapper is built on each attribute access -- so an identity test
    against the method would report "shadowed" always and this gate would pass on anything.
    A module-level function or a lambda has no __self__, which is exactly the distinction.
    """
    return getattr(fn, '__self__', None) is not owner


def state(obj):
    """(registers shadowed?, checks shadowed?) as phase_exec leaves them."""
    mt = obj.mcu_tmc
    ec = obj._cmdhelper.echeck_helper
    return (_is_shadowed(mt.get_register, mt),
            _is_shadowed(ec.start_checks, ec))


def fire_queued_enable(obj):
    """Simulate the deferred enable_disable_cb landing NOW. Returns the exception, or None."""
    try:
        obj._cmdhelper.do_enable()
        return None
    except Raised as e:
        return e


def probe(label, d, expect_regs, expect_checks):
    """Assert the invariant AND that a queued enable landing here is harmless."""
    ok = True
    for i, obj in enumerate(d.tmcs()):
        regs, checks = state(obj)
        if (regs, checks) != (expect_regs, expect_checks):
            check(False, "%s: axis %d state regs=%s checks=%s, expected regs=%s checks=%s"
                  % (label, i, regs, checks, expect_regs, expect_checks))
            ok = False
        # THE INVARIANT
        if regs and not checks:
            check(False, "%s: axis %d has REGISTERS shadowed with CHECKS LIVE"
                         " -- the fatal combination" % (label, i))
            ok = False
        exc = fire_queued_enable(obj)
        if exc is not None:
            check(False, "%s: axis %d -- a queued enable RAISED here: %s" % (label, i, exc))
            ok = False
    if ok:
        check(True, "%s: regs=%s checks=%s, and a queued enable landing here is harmless"
              % (label, expect_regs, expect_checks))


print("=== the engage sequence, probed at every observable point ===")
d = Dut()
probe("before anything", d, False, False)

d._set_tmc_checks(False, regs=False)      # <-- the fix: checks off BEFORE the enable
probe("after pre-suppress (regs=False)", d, False, True)

d._set_tmc_checks(False)                  # the register shadow, ~100 lines later in the engage
probe("after full suppress", d, True, True)

d._set_tmc_checks(True)                   # disengage
probe("after restore", d, False, False)

print("\n=== THE CALL SITES: the pre-suppress must come BEFORE the enable in the source ===")
# Everything above tests the PRIMITIVE. This arm tests that the engage paths actually use it in
# the right order -- without it, deleting the pre-suppress or moving it below the enable would
# reintroduce the exact 2026-10-15 window with every check above still green.
import inspect


def code_only(src):
    """Drop whole-line comments. The comment ABOVE the pre-suppress names
    SET_STEPPER_ENABLE in prose, and searching raw source finds that mention first -- which
    made an earlier version of this arm fail on correct code, i.e. have no discriminating
    power at all. Match statements, not documentation.
    """
    return '\n'.join(ln for ln in src.splitlines()
                     if not ln.lstrip().startswith('#'))


for fname, fn in (('_trapq_engage_body', PhaseExec._trapq_engage_body),
                  ('cmd_SEG_TEST', PhaseExec.cmd_SEG_TEST)):
    src = code_only(inspect.getsource(fn))
    i_pre = src.find('_set_tmc_checks(False, regs=False)')
    # the quoted command string appears only in the real gcode call
    i_en = src.find('"SET_STEPPER_ENABLE STEPPER=')
    i_full = src.find('_set_tmc_checks(False)')
    check(i_pre >= 0, "%s calls _set_tmc_checks(False, regs=False)" % fname)
    check(i_en >= 0, "%s issues SET_STEPPER_ENABLE" % fname)
    check(i_pre >= 0 and i_en >= 0 and i_pre < i_en,
          "%s: pre-suppress precedes the enable (offsets %d < %d)" % (fname, i_pre, i_en))
    check(i_full >= 0 and i_pre < i_full,
          "%s: and the register shadow comes after it (%d < %d)" % (fname, i_pre, i_full))

print("\n=== the registers really are shadowed in the suppressed state ===")
d2 = Dut()
d2._set_tmc_checks(False, regs=False)
d2._set_tmc_checks(False)
vals = [o.mcu_tmc.get_register('GSTAT') for o in d2.tmcs()]
check(all(v == 0 for v in vals), "GSTAT reads 0 through the shadow (%r)" % (vals,))
drv = [o.mcu_tmc.get_register('DRV_STATUS') for o in d2.tmcs()]
check(all(v & CS_ACTUAL_MASK for v in drv),
      "DRV_STATUS keeps cs_actual nonzero through the shadow (%r)"
      % (["%08x" % v for v in drv],))

print("\n=== TEETH: the OLD ordering (registers shadowed, checks live) must be caught ===")
d3 = Dut()
# Reproduce the pre-fix window by hand: shadow ONLY the registers, exactly as the old code did
# when the queued callback beat _set_tmc_checks' check half to the punch.
for obj in d3.tmcs():
    obj.mcu_tmc.get_register = mod.pe_tmc_shadow_get
caught = [fire_queued_enable(o) for o in d3.tmcs()]
check(all(c is None for c in caught),
      "with the FIXED per-register shadow even that window does not raise"
      " (GSTAT->0) -- the shadow is still the backstop")
for obj in d3.tmcs():
    obj.mcu_tmc.get_register = lambda *a, **k: 0x001f0000      # the shadow that shut it down
caught = [fire_queued_enable(o) for o in d3.tmcs()]
check(all(c is not None for c in caught),
      "and with the ONE-CONSTANT shadow it DOES raise on every axis -- so this gate can"
      " actually detect the 2026-10-15 fault (%s)" % (caught[0],))

print("\n=== the failure net: a raise mid-engage must not leave checks suppressed ===")
d4 = Dut()


def boom(gcmd):
    # Mirror the real body: ARM the unwind ledger, then pre-suppress. The arming is what tells
    # the wrapper this call owns the damage -- an unarmed failure deliberately restores NOTHING,
    # because the re-entry guard raises while a PREVIOUS engage is still streaming and
    # restoring there would un-suppress the bus underneath it. That path is asserted in
    # engage_unwind_test.py ARM A; here we are testing the opposite case.
    d4._engage_undo = []
    d4._set_tmc_checks(False, regs=False)      # what the body does next
    raise Raised("engage blew up after the pre-suppress")


d4._trapq_engage_body = boom
try:
    d4.cmd_TRAPQ_ENGAGE(None)
    check(False, "the wrapper swallowed the exception (it must re-raise)")
except Raised:
    check(True, "the wrapper re-raised, so the operator still sees the failure")
for i, obj in enumerate(d4.tmcs()):
    regs, checks = state(obj)
    check(not checks, "axis %d: checks RESTORED after the failed engage (not left blind)" % i)
    check(not regs, "axis %d: registers restored too" % i)
check(all(o._cmdhelper.echeck_helper.check_timer is not None for o in d4.tmcs()),
      "and start_checks() was re-run, so the periodic check is armed again")

print()
if fails:
    print("TMC-ENABLE-RACE GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("TMC-ENABLE-RACE GATE: PASS -- checks are shadowed no later than the registers at every"
      " point, a queued _do_enable is harmless throughout, and a failed engage restores the"
      " checks instead of leaving the driver unmonitored")
