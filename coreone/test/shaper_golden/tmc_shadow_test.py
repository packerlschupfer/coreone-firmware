"""Regression gate for phase_exec's TMC bus shadow (pe_tmc_shadow_get).

    python3 tmc_shadow_test.py

WHY THIS GATE EXISTS -- IT SHUT THE PRINTER DOWN, 2026-10-15.

During a phase-stepping run nothing on the host may touch the TMC SPI bus, so
_set_tmc_checks() stops Klipper's error checks AND shadows get_register/set_register on every
tmc2130 so a stray read cannot collide with the executor's XDIRECT stream. The shadow has to
return something, and it used to return one constant for every register:

    mt.get_register = lambda *a, **k: 0x001f0000

with a comment reasoning that GSTAT's named bits (reset / drv_err / uv_cp) are all zero in
that value. True, and irrelevant: tmc.py's TMCErrorCheck.__init__ builds

    self.gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]

so GSTAT's err_mask is 0xFFFFFFFF and ANY nonzero value is an unconditional error. The named
bits are never consulted. On TMC2130 clear_gstat is False, so _query_register cannot even try
to clear it -- it loops three times and raises. A re-engage on 2026-10-15 did exactly that:

    TMC 'stepper_x' reports error: GSTAT:      001f0000
    Transition to shutdown state: TMC 'stepper_x' reports error

and the 001f0000 in that message was never read off the bus. It was the sentinel.

The test replays Klipper's REAL _query_register control flow against the shadow rather than
just asserting the return values, because the thing that matters is "does the error check
raise", and that depends on masks living in tmc.py -- not on anything this file can see.
A value-only assertion would have passed on the broken code too, since 0x001f0000 was
"correct" by the old reasoning.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# Borrow the path from golden.py rather than deriving one here -- the same reason
# keepalive_unit_test.py and latebef_verdict_test.py do. The overlay keeps extras at
# klipper-port/extras, the release fork at <root>/klippy/extras, and '../../extras' would
# resolve to <root>/coreone/extras in the fork layout: a FileNotFoundError on fold, which is
# the loud failure, but only because this file happens to import at module scope.
import golden
spec = importlib.util.spec_from_file_location('pe_tmc', golden.PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
shadow = mod.pe_tmc_shadow_get

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


# --------------------------------------------------------------- tmc.py's real reg_info
# GSTAT, verbatim from TMCErrorCheck.__init__:
#     self.gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]
GSTAT_INFO = [0, 'GSTAT', 0xffffffff, 0xffffffff, 0]

# DRV_STATUS for a tmc2130, from the same constructor: mask/err_mask are the OR of the
# ot/s2ga/s2gb/s2vsa/s2vsb error fields plus the otpw/t120/... warn fields, and
# cs_actual_mask is the cs_actual field. TMC2130 register layout:
DRV_OT     = 0x00000002        # bit 1
DRV_S2GA   = 0x08000000        # bit 27
DRV_S2GB   = 0x10000000        # bit 28
DRV_OTPW   = 0x00000001        # bit 0
DRV_CS_ACT = 0x001f0000        # bits 20:16
DRV_ERR_MASK = DRV_OT | DRV_S2GA | DRV_S2GB
DRV_INFO = [0, 'DRV_STATUS', DRV_ERR_MASK | DRV_OTPW, DRV_ERR_MASK, DRV_CS_ACT]


class RaisedError(Exception):
    pass


def query_register(reg_info, get_register, irun=18, ihold=18, check_timer=True,
                   clear_gstat=False):
    """Faithful replay of tmc.py TMCErrorCheck._query_register.

    Returns normally if Klipper would accept the value; raises RaisedError if Klipper would
    raise command_error and shut the printer down.
    """
    reg_info = list(reg_info)
    last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
    try_clear = clear_gstat
    count = 0
    while 1:
        val = get_register(reg_name)
        reg_info[0] = last_value = val
        if not val & err_mask:
            if not cs_actual_mask or val & cs_actual_mask:
                return
            if check_timer is None or irun < 4:
                return
            if not ihold:
                return
            # cs_actual == 0 -> Klipper reads this as a driver reset
        count += 1
        if count >= 3:
            raise RaisedError("TMC reports error: %s: %08x" % (reg_name, val))
        if try_clear and val & err_mask:
            try_clear = False


# ------------------------------------------------------------------- the shutdown, replayed
print("the 2026-10-15 shutdown must no longer reproduce:")
try:
    query_register(GSTAT_INFO, shadow)
    gstat_ok = True
    err = None
except RaisedError as e:
    gstat_ok = False
    err = str(e)
check(gstat_ok, "a shadowed GSTAT read does NOT raise" + ("" if gstat_ok else " (got %r)" % err))
check(shadow('GSTAT') == 0,
      "because GSTAT returns exactly 0, not the DRV_STATUS sentinel (got %#x)"
      % shadow('GSTAT'))

print("\nand the old one-constant shadow must STILL fail -- proving the gate has teeth:")
old_shadow = lambda *a, **k: 0x001f0000
try:
    query_register(GSTAT_INFO, old_shadow)
    old_raised = False
except RaisedError:
    old_raised = True
check(old_raised,
      "the pre-fix shadow raises on GSTAT (this is the bug, reproduced from tmc.py's masks)")

# ----------------------------------------------------------------- DRV_STATUS still benign
print("\nDRV_STATUS must stay benign -- the reason the sentinel existed at all:")
try:
    query_register(DRV_INFO, shadow)
    drv_ok, drv_err = True, None
except RaisedError as e:
    drv_ok, drv_err = False, str(e)
check(drv_ok, "a shadowed DRV_STATUS read does NOT raise" + ("" if drv_ok else " (%s)" % drv_err))
v = shadow('DRV_STATUS')
check(v & DRV_CS_ACT == DRV_CS_ACT,
      "cs_actual reads 0x1F, so Klipper does not see a driver reset")
check(not (v & DRV_ERR_MASK), "and no error field (ot/s2ga/s2gb) is set")

print("\nthe cs_actual=0 trap the sentinel was invented to dodge:")
try:
    query_register(DRV_INFO, lambda *a, **k: 0)
    zero_raised = False
except RaisedError:
    zero_raised = True
check(zero_raised,
      "a plain 0 for DRV_STATUS WOULD raise (cs_actual==0 reads as a driver reset) --"
      " which is why one register cannot simply return 0 for everything")

# ---------------------------------------------------------------------- shape of the shadow
print("\nshadow plumbing:")
check(shadow('MSCNT') == mod.PE_TMC_SHADOW_DEFAULT,
      "an unlisted register falls back to the DRV_STATUS-safe default")
check(shadow() == mod.PE_TMC_SHADOW_DEFAULT, "a no-arg call does not explode")
check(shadow('GSTAT', 1, 2, foo=3) == 0,
      "extra positional/keyword args are tolerated (Klipper calls vary)")
# It is installed as an attribute on the mcu_tmc OBJECT, so it must not want an implicit self.


class FakeMcuTmc(object):
    pass


mt = FakeMcuTmc()
mt.get_register = shadow
check(mt.get_register('GSTAT') == 0,
      "installed as an instance attribute it still sees reg_name as the first arg"
      " (a bound method would have eaten it as self)")

print()
if fails:
    print("TMC-SHADOW GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("TMC-SHADOW GATE: PASS -- GSTAT reads 0 so the error check cannot trip, DRV_STATUS keeps"
      " cs_actual=0x1F, and the one-constant shadow that caused the shutdown still fails here")
