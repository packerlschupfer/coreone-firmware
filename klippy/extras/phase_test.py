# Phase 0 — host-only TMC2130 XDIRECT bring-up / spike
#
# Proves the electrical foundation of phase stepping WITHOUT firmware changes:
# puts one TMC2130 into direct_mode (GCONF bit16) and writes coil currents
# straight to XDIRECT (0x2d) from the host. Verified on hardware 2026-06-13:
# direct_mode+XDIRECT drives the coils (zero=free, set=hard hold) and the motor
# moves bidirectionally. NOTE the host's ~80 Hz update rate is the ceiling for
# smooth motion (stick-slip ~1 mm then stall) -> sustained motion needs the
# MCU+DMA executor (Phase 1/2). This tool is for register/electrical spot-checks.
#
# Hard-won lessons baked in here:
#  - The driver MUST be ENABLED or the coils freewheel (direct_mode only sets the
#    current target). ENABLE now asserts SET_STEPPER_ENABLE.
#  - IHOLD (not IRUN) scales the XDIRECT current (Prusa: "IHOLD is always used in
#    XDIRECT"). ENABLE sets BOTH; use SET_TMC_CURRENT ... HOLDCURRENT= for more.
#  - MSCURACT is NOT a valid readback in direct mode (it mirrors the internal sine
#    table). We read XDIRECT back instead.
#  - TMC swaps coils in XDIRECT, so SWAP defaults ON (Prusa writes coil_A=b,coil_B=a).
#
# SAFETY: IRUN capped low while enabled, AMP capped, refuses during a print, zeros
# current before/after switching. In direct mode the motor physically moves but
# Klipper loses sync -> RE-HOME the axis after (extruder is relative, no re-home).
#
# Deploy: scp to the Pi's klippy/extras/, clear __pycache__, restart klipper.
# Load with an empty [phase_test] section in printer.cfg.

import math

PHASE_UNITS = 1024          # phase index units per electrical period
AMP_HARD_CAP = 160          # absolute ceiling on digital amplitude (of 255)
IRUN_HARD_CAP = 16          # absolute ceiling on the IRUN/IHOLD we'll program (of 31)


def pack_xdirect(coil_a, coil_b):
    # XDIRECT: bits 8:0 = coil A (signed 9-bit), bits 24:16 = coil B (signed)
    return ((coil_b & 0x1ff) << 16) | (coil_a & 0x1ff)


def unpack_signed9(v):
    v &= 0x1ff
    return v - 0x200 if v >= 0x100 else v


class PhaseTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.saved = {}   # stepper_name -> (gconf, irun, ihold)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PHASE_TEST_ENABLE', self.cmd_ENABLE,
                                    desc="Enable driver + direct_mode for XDIRECT testing")
        self.gcode.register_command('PHASE_TEST_VECTOR', self.cmd_VECTOR,
                                    desc="Write one XDIRECT current vector; report XDIRECT readback")
        self.gcode.register_command('PHASE_TEST_ROTATE', self.cmd_ROTATE,
                                    desc="Sweep the electrical angle to rotate the motor")
        self.gcode.register_command('PHASE_TEST_DISABLE', self.cmd_DISABLE,
                                    desc="Restore normal step/dir operation")

    # --- helpers -----------------------------------------------------------
    def _get_tmc(self, gcmd):
        name = gcmd.get('STEPPER')
        tmc = self.printer.lookup_object('tmc2130 %s' % (name,), None)
        if tmc is None:
            raise gcmd.error("No [tmc2130 %s] found" % (name,))
        return name, tmc

    def _check_idle(self, gcmd):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is not None and ps.get_status(self.reactor.monotonic()).get(
                'state') == 'printing':
            raise gcmd.error("Refusing: a print is active")

    def _write_vector(self, tmc, angle, amp, swap, inva, invb):
        theta = 2.0 * math.pi * (angle % PHASE_UNITS) / PHASE_UNITS
        ca = int(round(amp * math.cos(theta)))
        cb = int(round(amp * math.sin(theta)))
        if swap:
            ca, cb = cb, ca
        if inva:
            ca = -ca
        if invb:
            cb = -cb
        tmc.mcu_tmc.set_register('XDIRECT', pack_xdirect(ca, cb))
        return ca, cb

    def _read_xdirect(self, tmc):
        raw = tmc.mcu_tmc.get_register('XDIRECT')
        return unpack_signed9(raw), unpack_signed9(raw >> 16)

    # --- commands ----------------------------------------------------------
    def cmd_ENABLE(self, gcmd):
        self._check_idle(gcmd)
        name, tmc = self._get_tmc(gcmd)
        irun = min(gcmd.get_int('IRUN', 12, minval=1, maxval=IRUN_HARD_CAP),
                   IRUN_HARD_CAP)
        # save GCONF + irun + ihold so DISABLE can restore them exactly
        gconf = tmc.mcu_tmc.get_register('GCONF')
        self.saved[name] = (gconf, tmc.fields.get_field('irun'),
                            tmc.fields.get_field('ihold'))
        # energize the driver power stage FIRST — direct_mode only sets the
        # current target; a disabled driver freewheels (no torque) no matter what
        self.gcode.run_script_from_command(
            'SET_STEPPER_ENABLE STEPPER=%s ENABLE=1' % (name,))
        # IHOLD (not IRUN) scales the XDIRECT current — set BOTH to irun
        tmc.fields.set_field('irun', irun)
        ihr = tmc.fields.set_field('ihold', irun)
        tmc.mcu_tmc.set_register('IHOLD_IRUN', ihr)
        self._write_vector(tmc, 0, 0, 0, 0, 0)   # zero current before switching
        gconf_new = tmc.fields.set_field('direct_mode', 1)
        tmc.mcu_tmc.set_register('GCONF', gconf_new)
        gcmd.respond_info(
            "[%s] driver ENABLED, direct_mode ON, IRUN=IHOLD=%d/31. For more "
            "torque: SET_TMC_CURRENT STEPPER=%s HOLDCURRENT=... . "
            "PHASE_TEST_DISABLE to restore (then RE-HOME)." % (name, irun, name))

    def cmd_VECTOR(self, gcmd):
        name, tmc = self._get_tmc(gcmd)
        if name not in self.saved:
            raise gcmd.error("Run PHASE_TEST_ENABLE %s first" % (name,))
        angle = gcmd.get_int('ANGLE', 0) % PHASE_UNITS
        amp = min(gcmd.get_int('AMP', 40, minval=0), AMP_HARD_CAP)
        swap = gcmd.get_int('SWAP', 1)
        inva = gcmd.get_int('INVA', 0)
        invb = gcmd.get_int('INVB', 0)
        ca, cb = self._write_vector(tmc, angle, amp, swap, inva, invb)
        # read XDIRECT back to confirm the write landed (MSCURACT is invalid here)
        xa, xb = self._read_xdirect(tmc)
        gcmd.respond_info(
            "[%s] angle=%d amp=%d swap=%d -> wrote(a=%d,b=%d)  XDIRECT(a=%d,b=%d)"
            % (name, angle, amp, swap, ca, cb, xa, xb))

    def cmd_ROTATE(self, gcmd):
        name, tmc = self._get_tmc(gcmd)
        if name not in self.saved:
            raise gcmd.error("Run PHASE_TEST_ENABLE %s first" % (name,))
        count = gcmd.get_int('COUNT', 256, minval=1, maxval=200000)
        step = gcmd.get_int('STEP', 8)            # phase units per update
        amp = min(gcmd.get_int('AMP', 40, minval=1), AMP_HARD_CAP)
        delay = gcmd.get_float('DELAY', 0.02, minval=0.001, maxval=1.0)
        swap = gcmd.get_int('SWAP', 1)
        inva = gcmd.get_int('INVA', 0)
        invb = gcmd.get_int('INVB', 0)
        angle = gcmd.get_int('START', 0)
        for _ in range(count):
            angle += step
            self._write_vector(tmc, angle, amp, swap, inva, invb)
            self.reactor.pause(self.reactor.monotonic() + delay)
        total = count * step
        periods = total / float(PHASE_UNITS)
        # 1 electrical period = 4 full steps; assume a 1.8deg (200 step) motor
        mech_deg = periods * 4.0 * (360.0 / 200.0)
        gcmd.respond_info(
            "[%s] swept %d phase units = %.2f electrical periods "
            "(~%.1f deg mech on a 1.8deg motor); end angle=%d"
            % (name, total, periods, mech_deg, angle % PHASE_UNITS))

    def cmd_DISABLE(self, gcmd):
        name, tmc = self._get_tmc(gcmd)
        if name not in self.saved:
            raise gcmd.error("%s was not enabled" % (name,))
        gconf, saved_irun, saved_ihold = self.saved.pop(name)
        self._write_vector(tmc, 0, 0, 0, 0, 0)    # zero current
        # guarantee the driver LEAVES direct_mode even if the IRUN/IHOLD restore
        # raises. A driver stranded in direct_mode ignores STEP/DIR -> the next G28
        # silently fails (the "interrupted-enable-leaves-axis-stuck" gotcha). The
        # current restore is cosmetic by comparison, so exiting direct_mode + restoring
        # GCONF is the finally.
        try:
            tmc.fields.set_field('irun', saved_irun)
            ihr = tmc.fields.set_field('ihold', saved_ihold)
            tmc.mcu_tmc.set_register('IHOLD_IRUN', ihr)
        finally:
            tmc.fields.set_field('direct_mode', 0)
            tmc.mcu_tmc.set_register('GCONF', gconf)   # restore exact saved GCONF
        gcmd.respond_info(
            "[%s] direct_mode OFF, IRUN/IHOLD restored. RE-HOME this axis "
            "(motor moved out of sync); extruder needs no re-home." % (name,))


def load_config(config):
    return PhaseTest(config)
