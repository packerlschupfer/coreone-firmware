# Precise sensorless homing for a marginal-StallGuard axis (Core One Y).
#
# Klipper's native sensorless homing triggers on the TMC diag1 pin, which
# fires on a SINGLE StallGuard dip -- so a noisy axis false-trips partway
# through a long sweep and "homes" short (and then a move-to-centre can ram
# the far frame). This wraps native per-axis homing in a multi-pass loop:
#
#   The real frame stall always stops at the SAME physical step count; a
#   noisy false-trip stops SHORT (further from the frame). So: home, retract
#   a fixed distance R, re-home, and measure the travel via the stepper's
#   absolute MCU step position. If travel ~= R, the head repeatably hit the
#   real frame. If travel > R, the previous home was premature (the re-home
#   had to cover the extra gap to the frame) -- and that re-home, being a
#   SHORT sweep, is reliable (short sweeps work where long ones false-trip).
#   N consecutive R-travel homes => converged, head at the true frame.
#
# Config:
#   [precise_home stepper_y]
#     #axis: y                  # defaults to the suffix of the section name
#     retract_dist: 15          # mm to back off + re-home each pass (SHORT,
#                               #   reliable sweep). Also the travel yardstick.
#     tolerance: 0.8            # mm; |travel-retract| under this == "at frame"
#     converge_passes: 2        # consecutive good passes required
#     max_passes: 10
#     retract_speed: 40         # mm/s for the back-off move
#     home_current:             # optional A: TMC run_current during homing
#     home_accel:               # optional: cap toolhead accel during homing
#
# Use:  PRECISE_HOME AXIS=Y     (wire into your G28 / homing flow)

class PreciseHome:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]          # e.g. "stepper_y"
        self.axis = config.get('axis', self.name.split('_')[-1]).upper()  # "Y"
        if self.axis not in ('X', 'Y', 'Z'):
            raise config.error("precise_home: axis must be X, Y or Z")
        self.retract = config.getfloat('retract_dist', 15., above=1.)
        self.tol = config.getfloat('tolerance', 0.8, above=0.)
        self.converge = config.getint('converge_passes', 2, minval=2)
        self.max_passes = config.getint('max_passes', 10,
                                        minval=self.converge + 1)
        self.retract_speed = config.getfloat('retract_speed', 40., above=0.)
        self.home_current = config.getfloat('home_current', 0., minval=0.)
        self.home_accel = config.getfloat('home_accel', 0., minval=0.)
        # which end does this axis home to? (endstop at min => retract +)
        sc = config.getsection('stepper_' + self.axis.lower())
        endstop = sc.getfloat('position_endstop')
        pmin = sc.getfloat('position_min')
        pmax = sc.getfloat('position_max')
        self.homes_to_min = abs(endstop - pmin) <= abs(endstop - pmax)
        self.run_current = None
        if self.home_current:
            self.run_current = config.getsection(
                'tmc2130 ' + self.name).getfloat('run_current')
        self.stepper = self.toolhead = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            'PRECISE_HOME', 'AXIS', self.axis, self.cmd_PRECISE_HOME,
            desc="Robust multi-pass sensorless home of one axis")

    def _connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.stepper = self.printer.lookup_object(
            'force_move').lookup_stepper(self.name)
        self.step_dist = self.stepper.get_step_dist()

    def _home_once(self):
        # native (diag1) sensorless home of this axis; return trigger step pos.
        # G28.1 (native homing), NOT G28 -- the unified [gcode_macro G28] routes
        # X/Y through MSCNT_HOME, whose own validate/retry loop homes up to ~50x. Calling
        # it here (up to max_passes times) nests two multi-pass homers = a homing storm.
        # PRECISE_HOME IS the multi-pass layer; it wants ONE raw native home per call.
        self.gcode.run_script_from_command("G28.1 %s" % (self.axis,))
        self.toolhead.wait_moves()
        return self.stepper.get_mcu_position()

    def _retract(self):
        # back off the frame by `retract` mm (axis is "homed" to endstop now)
        sign = 1.0 if self.homes_to_min else -1.0
        self.gcode.run_script_from_command(
            "G91\nG1 %s%.3f F%d\nG90"
            % (self.axis, sign * self.retract, int(self.retract_speed * 60)))
        self.toolhead.wait_moves()
        return self.stepper.get_mcu_position()

    def _set_homing_limits(self, on):
        if self.home_accel:
            if on:
                self._saved_accel = self.toolhead.max_accel
                self.gcode.run_script_from_command(
                    "SET_VELOCITY_LIMIT ACCEL=%.0f" % (self.home_accel,))
            else:
                self.gcode.run_script_from_command(
                    "SET_VELOCITY_LIMIT ACCEL=%.0f" % (self._saved_accel,))
        if self.home_current:
            cur = self.home_current if on else self.run_current
            self.gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f" % (self.name, cur))

    def cmd_PRECISE_HOME(self, gcmd):
        # refuse while phase-stepping is engaged -- StallGuard can't sense the frame in
        # direct_mode and the axis can drive the wrong way -> frame ram (same hazard the
        # G28/MSCNT guards prevent; this entry point bypasses the G28 macro via G28.1).
        if any(getattr(o, '_trapq_engaged', False)
               for _, o in self.printer.lookup_objects('phase_exec')):
            raise gcmd.error("PRECISE_HOME refused: phase-stepping is ENGAGED "
                             "-- run PHASE_STEP_OFF first")
        retract = gcmd.get_float('RETRACT', self.retract, above=1.)
        tol = gcmd.get_float('TOL', self.tol, above=0.)
        self._set_homing_limits(True)
        try:
            self._home_once()              # pass 0 (long sweep; may be short)
            good = 0
            for i in range(self.max_passes):
                before = self._retract()
                after = self._home_once()
                travel = abs(after - before) * self.step_dist
                err = abs(travel - retract)
                hit = err <= tol
                gcmd.respond_info(
                    "PRECISE_HOME %s pass %d: travel=%.2fmm (R=%.2f, err=%.2f) %s"
                    % (self.axis, i + 1, travel, retract, err,
                       "OK" if hit else "short->retry"))
                if hit:
                    good += 1
                    if good >= self.converge:
                        gcmd.respond_info(
                            "PRECISE_HOME %s: converged at frame (%d good passes)"
                            % (self.axis, good))
                        return
                else:
                    good = 0               # was premature; head now deeper
            raise gcmd.error(
                "PRECISE_HOME %s FAILED to converge in %d passes "
                "(StallGuard too noisy even for short sweeps -- check belt)"
                % (self.axis, self.max_passes))
        finally:
            self._set_homing_limits(False)


def load_config_prefix(config):
    return PreciseHome(config)
