# Temperature-robust, sub-step-precise sensorless homing (Prusa-style), ported
# from Prusa Buddy firmware: lib/Marlin/.../prusa/homing_cart.cpp + homing_modus.cpp.
#
# WHY: a fixed-threshold StallGuard endstop (our sg_endstop) false-triggers when
# the motor warms (hot chamber) -- sg_result drifts below the threshold and the
# guard trips mid-travel. Prusa does NOT temperature-compensate sg_result;
# instead it makes homing self-correcting and repeatable:
#
#   1. Coarse StallGuard home gets the axis NEAR the frame.
#   2. Validate + retry-slower: the homing travel must be plausible. A hot/noisy
#      premature trip travels short -> reject + re-home slower (short/slow sweeps
#      are reliable where a long/fast one false-trips). Catches the hot trip.
#   3. Phase-pin (the precision): the rotor's electrical angle (TMC MSCNT) at the
#      frame is repeatable modulo one electrical period (4 full steps). Snap the
#      motor onto the CALIBRATED phase so the home lands at the same rotor angle
#      every time -- sub-step repeatable, hot or cold. (Prusa physically moves
#      the motor to a full phase via plan_corexy_raw_move; we do the same with a
#      tiny FORCE_MOVE, sidestepping the CoreXY per-motor->axis mapping problem.)
#
# This module currently provides the verified Prusa math + the MSCNT read + the
# calibration data path. The motion integration (steps 2/3 wired into homing) is
# layered on top once this foundation is proven offline.

MSCNT_CIRCLE = 1024          # TMC2130 MSCNT spans one electrical period
MSCNT_HALF = 512
MSCNT_PER_FULLSTEP = 256     # MSCNT normalized: 256 microsteps per full step

# ---- Prusa homing_modus.cpp, ported verbatim --------------------------------

def to_calibrated(calibrated, value):
    # Shortest signed distance value->calibrated on the 1024 circle, range -512..+512.
    diff = calibrated - value
    if abs(diff) <= MSCNT_HALF:
        return diff
    return MSCNT_CIRCLE + diff if diff < 0 else diff - MSCNT_CIRCLE

def home_modus(positions, rng=96):
    # Circular weighted mode of MSCNT samples (robust to outliers); returns 0..1023.
    best_sum = best_n = best_pos = 0
    same = 1
    for i in range(MSCNT_CIRCLE):
        s = n = 0
        for p in positions:
            w = max(0, 1 + rng - abs(to_calibrated(i, p)))
            s += w
            n += 1 if w > 0 else 0
        if s > best_sum or (s == best_sum and n > best_n):
            best_sum, best_n, best_pos, same = s, n, i, 1
        elif s == best_sum and n == best_n:
            same += 1
    return (best_pos + (same - 1) // 2) % MSCNT_CIRCLE


# ---- Klipper integration -----------------------------------------------------

class MscntHome:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]            # e.g. "stepper_y"
        self.tmc_name = config.get('tmc', 'tmc2130 ' + self.name)
        # one electrical period in mm: rotation_distance / (256 * full_steps)
        sc = config.getsection(self.name)
        rot = sc.getfloat('rotation_distance')
        full_steps = sc.getint('full_steps_per_rotation', 200)
        self.mm_per_mscnt = rot / (MSCNT_PER_FULLSTEP * full_steps)
        self.period_mm = self.mm_per_mscnt * MSCNT_CIRCLE
        # calibrated target phase (MSCNT mode), persisted via save_variables
        self.cal_phase = config.getint('calibrated_phase', -1)
        # calibration / homing-experiment params
        self.axis = config.get('axis', self.name.split('_')[-1]).upper()
        self.cal_samples = config.getint('calibrate_samples', 8, minval=3)
        self.retract = config.getfloat('retract_dist', 15., above=1.)
        self.retract_speed = config.getfloat('retract_speed', 40., above=0.)
        es = sc.getfloat('position_endstop')
        self.pos_min = sc.getfloat('position_min')
        self.pos_max = sc.getfloat('position_max')
        self.homes_to_min = (abs(es - self.pos_min) <= abs(es - self.pos_max))
        # Prusa MOVE_BACK_BEFORE_HOMING: nudge off the frame before the bump so the
        # home has run-in and native diag1 isn't already triggered (Prusa uses 0.5mm
        # first / 10mm on retries; we use one value). Only applied when already homed.
        self.move_back = config.getfloat('move_back_dist', 10., minval=0.)
        # Bounded phase-snap ([endstop_phase]-style sub-step correction). After a
        # travel-validated home, re-label the axis position so the rotor is treated as
        # sitting at the calibrated phase -> removes the StallGuard full-step homing
        # ambiguity (the ~0.08mm CoreXY Y offset). BOUNDED + default-OFF + TEMP CAVEAT:
        # a FIXED cal phase can mis-snap if the rotor phase drifts > half a period with
        # temperature (Prusa re-calibrates per print to avoid exactly this). We only snap
        # when |moff| <= phase_snap_limit, staying clear of the +/-512 period boundary
        # where the snap direction is ambiguous. SIGN is empirical (set per machine).
        self.phase_snap = config.getboolean('phase_snap', False)
        self.phase_snap_sign = config.getint('phase_snap_sign', 1)
        self.phase_snap_limit = config.getint('phase_snap_limit', 384,
                                              minval=1, maxval=512)
        self.mcu_tmc = self.toolhead = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            'MSCNT_READ', 'STEPPER', self.name, self.cmd_MSCNT_READ,
            desc="Read this motor's TMC MSCNT (rotor electrical angle 0..1023)")
        self.gcode.register_mux_command(
            'MSCNT_HOME_CALIBRATE', 'AXIS', self.axis, self.cmd_CALIBRATE,
            desc="Home this axis N times; report the MSCNT cluster (phase-pin premise)")
        self.gcode.register_mux_command(
            'MSCNT_HOME', 'AXIS', self.axis, self.cmd_MSCNT_HOME,
            desc="Validated home: re-home until the rotor lands at the calibrated phase + travel checks out")
        self.gcode.register_mux_command(
            'CALIBRATE_HOMING_SGT', 'AXIS', self.axis, self.cmd_CALIBRATE_SGT,
            desc="Sweep driver_SGT (Prusa HomingSensitivityCalibration); pick the most repeatable value")
        self.gcode.register_mux_command(
            'MSCNT_SNAP_CALIBRATE', 'AXIS', self.axis, self.cmd_SNAP_CALIBRATE,
            desc="Auto-derive cal_phase + phase_snap_sign for the bounded phase-snap; predicts the residual")

    def _connect(self):
        tmc = self.printer.lookup_object(self.tmc_name)
        self.mcu_tmc = tmc.mcu_tmc
        self.toolhead = self.printer.lookup_object('toolhead')
        self.stepper = self.printer.lookup_object(
            'force_move').lookup_stepper(self.name)
        self.step_dist = self.stepper.get_step_dist()

    def read_mscnt(self):
        # MSCNT register (0x6a), low 10 bits = rotor electrical angle 0..1023.
        # Valid only at standstill (read after the move settles).
        return self.mcu_tmc.get_register("MSCNT") & 0x3ff

    def cmd_MSCNT_READ(self, gcmd):
        m = self.read_mscnt()
        gcmd.respond_info(
            "%s MSCNT=%d  (period=%.3fmm, %.5fmm/unit)%s"
            % (self.name, m, self.period_mm, self.mm_per_mscnt,
               ("  cal_phase=%d off=%d" % (
                   self.cal_phase, to_calibrated(self.cal_phase, m))
                if self.cal_phase >= 0 else "  (uncalibrated)")))


    def _prehome_backoff(self):
        # Prusa MOVE_BACK_BEFORE_HOMING: move away from the home frame before the bump
        # so there's run-in (and native diag1 isn't already triggered when we start).
        # Needs the axis homed for a coordinated CoreXY move; a cold start skips it and
        # relies on the post-home retract instead. Clamp to range -- never ram the far frame.
        if self.move_back <= 0.:
            return
        eventtime = self.printer.get_reactor().monotonic()
        if self.axis.lower() not in self.toolhead.get_status(eventtime)['homed_axes']:
            return
        ai = 'XYZ'.index(self.axis)
        pos = self.toolhead.get_position()
        sign = 1.0 if self.homes_to_min else -1.0
        target = pos[ai] + sign * self.move_back
        target = max(self.pos_min + 1.0, min(self.pos_max - 1.0, target))
        if abs(target - pos[ai]) < 0.5:
            return                       # already far from the home frame -> run-in exists
        self.gcode.run_script_from_command(
            "G90\nG1 %s%.3f F%d" % (self.axis, target, int(self.retract_speed * 60)))
        self.toolhead.wait_moves()

    def _home(self):
        self.gcode.run_script_from_command("G28.1 %s" % (self.axis,))  # native home, NOT the G28 macro (recursion)
        self.toolhead.wait_moves()

    def _retract(self):
        # back off the frame (axis is at its endstop after a home)
        sign = 1.0 if self.homes_to_min else -1.0
        self.gcode.run_script_from_command(
            "G91\nG1 %s%.3f F%d\nG90"
            % (self.axis, sign * self.retract, int(self.retract_speed * 60)))
        self.toolhead.wait_moves()

    def _refuse_if_phasestep_engaged(self, gcmd):
        # sensorless homing/calibration MUST NOT run in phase-stepping direct_mode --
        # StallGuard can't sense the frame stall AND the axis drives the wrong way -> frame ram
        # (the exact crash the [gcode_macro G28] disengage guard prevents). These commands call
        # G28.1 directly (recursion-safe), which BYPASSES that macro guard; and read_mscnt returns
        # a canned value under the phase run's TMC monkeypatch -> silently poisoned calibration.
        # So re-check engagement here, at every entry point.
        if any(getattr(o, '_trapq_engaged', False)
               for _, o in self.printer.lookup_objects('phase_exec')):
            raise gcmd.error("MSCNT homing/cal refused: phase-stepping is ENGAGED "
                             "-- run PHASE_STEP_OFF first")

    def cmd_CALIBRATE(self, gcmd):
        self._refuse_if_phasestep_engaged(gcmd)
        n = gcmd.get_int('SAMPLES', self.cal_samples, minval=3)
        gcmd.respond_info("MSCNT_HOME_CALIBRATE %s: homing %d times (cold) — "
                          "watching the rotor angle at the frame..." % (self.axis, n))
        # suppress safe_z_home's z_hop for the whole calibration -- its siblings
        # (MSCNT_HOME / SNAP_CALIBRATE) already do this, but this loop homes N times
        # and each unhomed G28.1 would otherwise lift/drop the bed by z_hop, drifting
        # the bed toward the chamber floor over many passes.
        sz = self.printer.lookup_object('safe_z_home', None)
        saved_zhop = sz.z_hop if sz is not None else None
        if sz is not None:
            sz.z_hop = 0.
        try:
            self._home()                   # establish a home to retract from
            samples = []
            for i in range(n):
                self._retract()
                self._home()
                self.toolhead.dwell(0.05)  # let the rotor settle before the SPI read
                m = self.read_mscnt()
                samples.append(m)
                gcmd.respond_info("  pass %d: MSCNT=%d" % (i + 1, m))
        finally:
            if sz is not None:
                sz.z_hop = saved_zhop
        mode = home_modus(samples)
        spread = max(abs(to_calibrated(mode, s)) for s in samples)
        gcmd.respond_info(
            "MSCNT_HOME_CALIBRATE %s: mode=%d, spread=+/-%d units (%.3fmm) over %d passes"
            % (self.axis, mode, spread, spread * self.mm_per_mscnt, n))
        if spread <= 128:
            gcmd.respond_info("  TIGHT (<half full-step): phase-pin viable. "
                              "Set 'calibrated_phase: %d' in [mscnt_home %s]."
                              % (mode, self.name))
        elif spread <= 256:
            gcmd.respond_info("  ~1 full-step spread: phase-pin usable; "
                              "validate/retry carries the rest.")
        else:
            gcmd.respond_info("  WIDE (>1 full-step): StallGuard home too noisy to "
                              "phase-pin alone — validate/retry is the primary fix.")


    def _move_axis_to(self, coord):
        coord = max(self.pos_min + 1.0, min(self.pos_max - 1.0, coord))
        self.gcode.run_script_from_command(
            "G90\nG1 %s%.4f F%d" % (self.axis, coord, int(self.retract_speed * 60)))
        self.toolhead.wait_moves()

    # ---- REFUTED: in-place MSCNT reset (MSCNT_RESYNC) -- do NOT re-attempt -------
    # HYPOTHESIS (2026-07-19): the +/-512 desync could be cleared without a FIRMWARE_RESTART by
    # resetting the TMC's microstep counter in place. MSCNT is READ-ONLY (tmc2130.py register
    # list), so the only candidate was an indirect reset via a CHOPCONF/mres rewrite (mres lives
    # in CHOPCONF; Klipper rewrites CHOPCONF at init, which was assumed to be what a restart did).
    #
    # MEASURED ON HARDWARE -- BOTH HALVES REFUTED:
    #   1. mres rewrite does NOT reset MSCNT.  X: 392 -> 393, Y: 280 -> 279 across a
    #      mres 4->5->4 cycle. That is +/-1 microstep of jitter/settle, not a reset to 0.
    #   2. FIRMWARE_RESTART does NOT reset MSCNT either.  The TMC2130 is never power-cycled by an
    #      MCU reset: MSCNT read 392/280 BOTH before (logged at the phase-step disengage) and
    #      after a full `systemctl restart klipper`. A reset would have zeroed it.
    #
    # CONSEQUENCE: there is no known in-place MSCNT reset on the 2130, and "restart it" was never
    # the real fix -- past "a restart cleared it" reports were the self-heal (a post-restart
    # re-home landing in the cal well). The deterministic recovery is MSCNT_SNAP_CALIBRATE, which
    # re-derives cal_phase against the CURRENT relationship and applies it live.
    # The MSCNT_RESYNC command was removed rather than shipped non-functional.

    def cmd_SNAP_CALIBRATE(self, gcmd):
        self._refuse_if_phasestep_engaged(gcmd)
        # Auto-derive the bounded phase-snap parameters, ALL measured in-Python
        # (read_mscnt is reliable; console scraping is not):
        #   1. dir: move the axis a known amount, watch which way MSCNT travels ->
        #      snap sign = -dir (from the position-correction math).
        #   2. cal_phase: the circular mode of N raw homes.
        #   3. residual: PREDICT the post-snap spread analytically from those same
        #      samples (snapped to 0 when |moff|<=limit, else left) -- no extra homing.
        n = gcmd.get_int('SAMPLES', 8, minval=4)
        limit = gcmd.get_int('SNAPLIMIT', self.phase_snap_limit, minval=1, maxval=512)
        apply_live = bool(gcmd.get_int('APPLY', 1))
        ai = 'XYZ'.index(self.axis)
        sz = self.printer.lookup_object('safe_z_home', None)
        saved_zhop = sz.z_hop if sz is not None else None
        if sz is not None:
            sz.z_hop = 0.
        try:
            eventtime = self.printer.get_reactor().monotonic()
            if self.axis.lower() not in self.toolhead.get_status(eventtime)['homed_axes']:
                self._home()
            # --- 1. direction of MSCNT vs axis position ---
            mid = 0.5 * (self.pos_min + self.pos_max)
            self._move_axis_to(mid)
            self.toolhead.dwell(0.1)
            m0 = self.read_mscnt()
            test_units = 300                       # < half period -> unambiguous sign
            dmm = test_units * self.mm_per_mscnt
            self._move_axis_to(mid + dmm)
            self.toolhead.dwell(0.1)
            m1 = self.read_mscnt()
            delta_m = to_calibrated(m1, m0)        # shortest signed (m1 - m0)
            self._move_axis_to(mid)
            direction = 1 if delta_m > 0 else -1
            sign = -direction
            gcmd.respond_info(
                "MSCNT_SNAP_CALIBRATE %s: moved +%.4fmm -> MSCNT %d->%d (delta %+d) "
                "=> dir %+d, snap sign %+d" % (self.axis, dmm, m0, m1, delta_m,
                                               direction, sign))
            # --- 2. cal_phase = mode of N raw homes ---
            self._home()
            samples = []
            for i in range(n):
                self._retract()
                self._home()
                self.toolhead.dwell(0.05)
                samples.append(self.read_mscnt())
            mode = home_modus(samples)
            raw_spread = max(abs(to_calibrated(mode, s)) for s in samples)
            # --- 3. predicted post-snap residual ---
            snapped = 0
            residuals = []
            for s in samples:
                moff = to_calibrated(mode, s)
                if abs(moff) <= limit:
                    residuals.append(0.0)
                    snapped += 1
                else:
                    residuals.append(moff)        # outside the limit -> left uncorrected
            res_spread = max(abs(r) for r in residuals)
            gcmd.respond_info(
                "  samples=%s" % (samples,))
            gcmd.respond_info(
                "  cal_phase(mode)=%d | raw spread +/-%d (%.3fmm) -> post-snap +/-%d "
                "(%.3fmm), %d/%d within +/-%d limit"
                % (mode, raw_spread, raw_spread * self.mm_per_mscnt, res_spread,
                   res_spread * self.mm_per_mscnt, snapped, n, limit))
            if apply_live:
                self.cal_phase = mode
                self.phase_snap_sign = sign
                self.phase_snap = True
                gcmd.respond_info(
                    "  APPLIED LIVE (this session): cal_phase=%d, sign=%+d, snap ON. "
                    "PERSIST in [mscnt_home %s]: calibrated_phase: %d / phase_snap_sign: "
                    "%d / phase_snap: True  (re-run hot to confirm temp stability)."
                    % (mode, sign, self.name, mode, sign))
            else:
                gcmd.respond_info(
                    "  (APPLY=0: not applied) Set [mscnt_home %s] calibrated_phase: %d / "
                    "phase_snap_sign: %d / phase_snap: True" % (self.name, mode, sign))
        finally:
            if sz is not None:
                sz.z_hop = saved_zhop

    def cmd_MSCNT_HOME(self, gcmd):
        self._refuse_if_phasestep_engaged(gcmd)
        # Re-home until the home both lands at the calibrated rotor phase (sub-step
        # repeatable) AND covers the expected travel (rejects a hot/noisy premature
        # trip). A clean home passes on the first try; a bad one is rejected + retried.
        if self.cal_phase < 0:
            raise gcmd.error("MSCNT_HOME %s: not calibrated — run "
                             "MSCNT_HOME_CALIBRATE AXIS=%s first" % (self.axis, self.axis))
        mtol = gcmd.get_int('TOL', 128, minval=1)            # MSCNT units; 128 = half full-step
        rtol = gcmd.get_float('TRAVELTOL', 0.6, above=0.)    # mm
        # Default 8 (was 5): the ~+/-512 half-period slave-settling case (see DESYNC RECOVERY
        # below) sometimes sits in the "other" electrical well for several homes before a re-home
        # flips it to the cal well; more tries catch that good-well landing (and its snap) before
        # falling back to travel-only. A clean home still passes on try 1, so no cost in the normal
        # case; the creep cap (30mm) still bounds a genuinely-noisy StallGuard regardless of count.
        retries = gcmd.get_int('RETRIES', 8, minval=1)
        # Bounded phase-snap (default per [mscnt_home] phase_snap; SNAP=0/1 to override)
        do_snap = bool(gcmd.get_int('SNAP', 1 if self.phase_snap else 0))
        snap_sign = gcmd.get_int('SIGN', self.phase_snap_sign)
        snap_limit = gcmd.get_int('SNAPLIMIT', self.phase_snap_limit,
                                  minval=1, maxval=512)
        # safe_z_home does a z_hop on EVERY G28; while Z is unhomed the hop calls
        # toolhead.set_position(..., homing_axes="z") (safe_z_home.py). On the SECOND axis's home
        # (Z still unhomed) that reset DROPS the already-homed FIRST axis -> the next move aborts
        # with "Must home axis first" (mid-PRINT_START, recurring). So suppress the hop for the
        # WHOLE home, not just after the first -- the old "let one hop for clearance" design
        # un-homed X on the Y home. Also kills the per-retry bed creep. No clearance hop is needed:
        # the nozzle is parked far from the bed at PRINT_START (PRINT_END drops the bed ~100mm).
        sz = self.printer.lookup_object('safe_z_home', None)
        saved_zhop = sz.z_hop if sz is not None else None
        if sz is not None:
            sz.z_hop = 0.                                    # suppress ALL z_hops for this home
        self._prehome_backoff()                              # Prusa MOVE_BACK: run-in
        self._home()                                         # initial home (no z_hop -> no un-home)
        creep = 0.0   # cumulative drift toward the FAR frame from premature re-homes (safety)
        frame_hits = 0    # tries that REACHED the frame (StallGuard reliable, not noisy)
        desync_hits = 0   # tries that reached the frame BUT were unsnappable (fixed phase off)
        try:
            for attempt in range(1, retries + 1):
                self._retract()
                before = self.stepper.get_mcu_position()
                self._home()
                self.toolhead.dwell(0.05)
                after = self.stepper.get_mcu_position()
                travel = abs(after - before) * self.step_dist
                m = self.read_mscnt()
                moff = to_calibrated(self.cal_phase, m)
                phase_ok = abs(moff) <= mtol
                # CURRENT home must reach the frame (travel >= retract); travel > retract
                # only means the PREVIOUS home was short; only travel << retract is premature.
                travel_ok = travel >= self.retract - rtol
                # SAFETY: a premature re-home drifts (retract-travel) toward the far frame;
                # cap the cumulative drift so a noisy StallGuard can never walk into a rail.
                creep += max(0.0, self.retract - travel)
                if creep > 30.0:
                    raise gcmd.error(
                        "MSCNT_HOME %s ABORTED: drifted %.0fmm toward the far frame in %d tries — "
                        "StallGuard too noisy. Home COLD (before the heat soak), not hot."
                        % (self.axis, creep, attempt))
                # Lock on TRAVEL (reached the frame). MSCNT is diagnostic only: the rotor
                # angle at the trip DRIFTS with temperature, so a fixed-cal phase-lock can't
                # hold across temps -- the travel-validated home is the robust signal.
                # Accept only a home that (a) reached the frame (travel_ok) AND, when
                # snapping, (b) landed within +/-1 full-step of cal so the snap can null
                # it (|moff| <= snap_limit). A home that scattered >1 full-step (noisy
                # StallGuard) is REJECTED + re-homed -- Prusa's instability rejection
                # (ORIGIN_BUMP_MAX_ERR / point_is_unstable) -- so the accepted home always
                # lands on a consistent, snappable phase instead of a 0.16mm full-step jump.
                snappable = (not do_snap) or abs(moff) <= snap_limit
                if travel_ok:
                    frame_hits += 1                      # reliable trip (reached the frame)
                    if abs(moff) > snap_limit:
                        desync_hits += 1                 # ...but a fixed >1-full-step phase off
                if travel_ok and snappable:
                    snapped = ""
                    if do_snap:
                        ai = 'XYZ'.index(self.axis)
                        corr = moff * self.mm_per_mscnt * snap_sign
                        npos = self.toolhead.get_position()
                        npos[ai] += corr
                        self.toolhead.set_position(npos)
                        snapped = ", SNAP %+.4fmm (moff %d)" % (corr, moff)
                    ph = ("phase-locked" if phase_ok
                          else "phase off %d%s" % (moff, "" if snapped else " (temp drift)"))
                    gcmd.respond_info(
                        "MSCNT_HOME %s: HOMED (travel=%.2fmm, MSCNT=%d, %s%s -- "
                        "validated, %d %s)"
                        % (self.axis, travel, m, ph, snapped, attempt,
                           "try" if attempt == 1 else "tries"))
                    return
                if not travel_ok:
                    gcmd.respond_info(
                        "MSCNT_HOME %s try %d: travel=%.2f << retract %.0f (premature trip) "
                        "-> re-home" % (self.axis, attempt, travel, self.retract))
                else:
                    gcmd.respond_info(
                        "MSCNT_HOME %s try %d: phase off %d > snap limit %d (~%.2fmm full-step "
                        "scatter) -> re-home for a snappable landing"
                        % (self.axis, attempt, moff, snap_limit,
                           abs(moff) * self.mm_per_mscnt))
            # DESYNC RECOVERY: EVERY try REACHED the frame (StallGuard reliable, NOT noisy) but
            # NONE was snappable, all clustered at ~+/-512 (2 full steps = 1/2 electrical period).
            # (i.e. diametrically opposite cal_phase on the 1024 circle = maximally ambiguous).
            # ROOT MECHANISM (ONE cause, SEVERAL triggers): MSCNT is the TMC's microstep-table
            # counter and it advances ONLY on STEP PULSES. Anything that moves the rotor WITHOUT
            # step pulses silently breaks the MSCNT <-> physical-position relationship:
            #   (a) PHASE-STEPPING -- XDIRECT drives the coils directly while step-gen is
            #       suppressed, so MSCNT sits frozen while the gantry moves.
            #   (b) HAND-MOVING the gantry while the steppers are DE-ENERGIZED (e.g. pushing the
            #       head aside to clean the nozzle). MSCNT cannot see your hand. CONFIRMED LIVE
            #       2026-07-19: a post-print hand-move produced exactly this on BOTH X and Y.
            #   (c) an ABNORMAL phase-step exit (cancel / MCU shutdown) where _disengage_motor's
            #       rotor<->MSCNT re-align could not run.
            # NB (a)+(c) alone were the old explanation here and were INCOMPLETE -- klippy.log shows
            # this also happens when the disengage align DID run (it logs "rotor re-synced to
            # MSCNT ..."), and (b) reproduces it with no phase-stepping involved at all.
            # Snapping at +/-512 would ADD that 0.16mm error (the MSCNT reference is 2 full steps
            # out) rather than remove it, and the correction DIRECTION is a coin-flip there -- so we
            # correctly SKIP the snap. The home is still physically valid (frame-locked,
            # StallGuard-native precision, exactly like stock Prusa homing); only the OPTIONAL
            # sub-step refinement is forgone. First-layer Z is loadcell-probed (NOT MSCNT), so a
            # skipped XY snap does NOT affect squish -- impact is <=0.08mm XY registration.
            # RECOVERY, cheapest first: (1) just RE-HOME -- it self-heals when a try lands in the
            # cal well (observed live: Y succeeded on try 6; the RETRIES default of 8 exists for
            # exactly this); (2) MSCNT_SNAP_CALIBRATE re-derives cal_phase against the CURRENT
            # relationship and applies it live -- the ONLY deterministic no-restart recovery (it
            # adopts a reference up to 0.16mm shifted, so do NOT persist it unless you want that).
            # /!\ FIRMWARE_RESTART does NOT reset MSCNT and is NOT the fix -- MEASURED 2026-07-19:
            # the TMC2130 is never power-cycled by an MCU reset, and MSCNT read 392(X)/280(Y) BOTH
            # before (logged at the phase-step disengage) and after a full `systemctl restart
            # klipper`; a reset would have zeroed it. A CHOPCONF/mres rewrite doesn't reset it
            # either (see the REFUTED note above). Historic "a restart cleared it" reports were
            # the self-heal in (1) after the post-restart re-home.
            # Real noise (premature trips or scattered moff) still falls through to the error below.
            if frame_hits == retries and desync_hits == retries:
                gcmd.respond_info(
                    "MSCNT_HOME %s: HOMED travel-only -- reliable trip, but phase off %d = the "
                    "1/2-period MSCNT desync (~%.2fmm full-step ambiguity, physically at the "
                    "frame) left by rotor motion WITHOUT step pulses (phase-step exit, or the "
                    "gantry pushed by hand while de-energized). Snap skipped -- the home IS valid "
                    "(StallGuard-native precision; first-layer Z is loadcell-probed, unaffected). "
                    "Recovery: re-home (often self-heals), or MSCNT_SNAP_CALIBRATE AXIS=%s to "
                    "re-derive cal_phase live. NOTE: FIRMWARE_RESTART does NOT reset MSCNT."
                    % (self.axis, moff, abs(moff) * self.mm_per_mscnt, self.axis))
                return
            raise gcmd.error(
                "MSCNT_HOME %s: no acceptable home in %d tries (premature trip or scattered "
                "phase -- StallGuard genuinely noisy; check belt tension)" % (self.axis, retries))
        finally:
            if sz is not None:
                sz.z_hop = saved_zhop


    def _set_sgt(self, sgt):
        self.gcode.run_script_from_command(
            "SET_TMC_FIELD STEPPER=%s FIELD=sgt VALUE=%d" % (self.name, sgt))

    def cmd_CALIBRATE_SGT(self, gcmd):
        self._refuse_if_phasestep_engaged(gcmd)
        # Prusa HomingSensitivityCalibration analogue. Sweep driver_SGT low->high; at each
        # value home N times and score on two axes:
        #   - reliability: travel must reach the frame (travel ~ retract). A SHORT travel is
        #     a premature trip => SGT too sensitive. A no-trip (homing error) => too insensitive.
        #   - repeatability: the rotor MSCNT must cluster tight across the N homes.
        # Sweep low->high because low SGT = MORE sensitive = trips EARLY (safe); high SGT =
        # insensitive = may NOT trip = ram, so we STOP at the first miss (all higher miss too).
        # Pick the middle of the contiguous fully-reliable range (Prusa's "consistent middle"),
        # and also report the tightest-cluster value. driver_SGT is restored at the end; the
        # winner is reported for the user to persist in [tmc2130 <stepper>] (like calibrated_phase).
        cmd_error = self.printer.command_error
        lo = gcmd.get_int('MIN', -4, minval=-64, maxval=63)
        # Default MAX stays LOW on purpose: the useful, *detectable* edge is the premature
        # one (short travel). The high-SGT end RUMBLES (insensitive -> the head rams the rail
        # and only trips on the dead stall) but still reaches the frame, so travel can't see it
        # -- only an ear can. We recommend just above the premature edge, so there's no need to
        # sweep up into the rumble zone. Raise MAX explicitly (and LISTEN) if you want to probe it.
        hi = gcmd.get_int('MAX', 1, minval=lo, maxval=63)
        n = gcmd.get_int('SAMPLES', 5, minval=3)
        rtol = gcmd.get_float('TRAVELTOL', 1.0, above=0.)
        orig_sgt = self.mcu_tmc.get_fields().get_field("sgt")
        sz = self.printer.lookup_object('safe_z_home', None)
        saved_zhop = sz.z_hop if sz is not None else None
        gcmd.respond_info(
            "CALIBRATE_HOMING_SGT %s: sweeping SGT %d..%d, %d homes each "
            "(low=sensitive/safe, high=may ram -> stops at first miss). orig SGT=%d"
            % (self.axis, lo, hi, n, orig_sgt))
        results = {}   # sgt -> (reliable_count, spread or None)
        try:
            # kill z_hop BEFORE the first _home() -- the establishing home is an
            # unhomed G28.1 too, so leaving z_hop live for it drops the bed once per
            # CALIBRATE_SGT run (the sweep loop below was already guarded).
            if sz is not None:
                sz.z_hop = 0.
            self._prehome_backoff()
            self._home()                       # establish a frame to retract from
            for sgt in range(lo, hi + 1):
                self._set_sgt(sgt)
                travels = []; mscnts = []; missed = rammed = False; done = 0
                for i in range(n):
                    self._retract()
                    before = self.stepper.get_mcu_position()
                    try:
                        self._home()
                    except cmd_error:
                        missed = True
                        break
                    self.toolhead.dwell(0.05)
                    after = self.stepper.get_mcu_position()
                    travel = abs(after - before) * self.step_dist
                    if travel > self.retract + 3.0:   # gross overshoot past first contact = ram
                        rammed = True
                        break
                    travels.append(travel)
                    mscnts.append(self.read_mscnt())
                    done += 1
                if missed or rammed:
                    gcmd.respond_info(
                        "  SGT %+d: %s after %d/%d -- too insensitive; stopping sweep"
                        % (sgt, "MISSED (no trip -> ram)" if missed else "RAMMED (overshoot)",
                           done, n))
                    results[sgt] = (0, None)
                    break
                premature = sum(1 for t in travels if t < self.retract - rtol)
                reliable = n - premature
                if reliable == n:
                    mode = home_modus(mscnts)
                    spread = max(abs(to_calibrated(mode, m)) for m in mscnts)
                    results[sgt] = (n, spread)
                    gcmd.respond_info(
                        "  SGT %+d: reliable %d/%d, MSCNT spread +/-%d (%.3fmm)"
                        % (sgt, reliable, n, spread, spread * self.mm_per_mscnt))
                else:
                    results[sgt] = (reliable, None)
                    gcmd.respond_info("  SGT %+d: premature %d/%d (too sensitive)"
                                      % (sgt, premature, n))
        finally:
            self._set_sgt(orig_sgt)            # leave the driver as we found it
            if sz is not None:
                sz.z_hop = saved_zhop
        good = sorted(s for s, (rel, sp) in results.items() if sp is not None)
        if not good:
            raise gcmd.error(
                "CALIBRATE_HOMING_SGT %s: no SGT homed reliably in %d..%d -- widen the "
                "range or check belt tension/current" % (self.axis, lo, hi))
        # Recommend ONE notch above the premature edge (= lowest reliable SGT + 1): far enough
        # off the premature edge for temperature margin (hot -> more sensitive -> premature),
        # but as LOW as possible so we stay clear of the high-SGT rumble zone that travel-based
        # reliability CANNOT detect. Bias low, not middle -- the rumble end only looks reliable.
        lo_rel = good[0]
        pick = lo_rel + 1 if (lo_rel + 1) in good else lo_rel
        gcmd.respond_info(
            "CALIBRATE_HOMING_SGT %s: premature for SGT<%d, reliable from %d. Recommend %d "
            "(one notch off the premature edge = hot margin). Set 'driver_SGT: %d' in "
            "[tmc2130 %s] (currently %d)."
            % (self.axis, lo_rel, lo_rel, pick, pick, self.name, orig_sgt))
        gcmd.respond_info(
            "  WARNING: travel-reliability CANNOT detect a rail RUMBLE -- a too-insensitive "
            "(high) SGT rams the rail and trips on the dead stall, which still reaches the "
            "frame. LISTEN during homing and ear-confirm; if it grinds, drop SGT by 1.")


def load_config_prefix(config):
    return MscntHome(config)


# ---- offline self-test: prove the ported math matches Prusa's ----------------
if __name__ == "__main__":
    # to_calibrated: wrap-around correctness on the 1024 circle
    assert to_calibrated(10, 5) == 5
    assert to_calibrated(5, 10) == -5
    assert to_calibrated(0, 1020) == 4        # 0 is just past 1020 going forward
    assert to_calibrated(1020, 0) == -4
    assert to_calibrated(0, 512) == -512      # boundary: |diff|<=512 returns diff
    assert to_calibrated(0, 513) == 511       # past the half -> wrap forward
    # home_modus: tight cluster -> its center; ignores a far outlier
    assert home_modus([100, 101, 99, 100, 102]) == 100
    assert home_modus([300, 301, 299, 300, 700]) == 300   # 700 outlier dropped
    # cluster straddling the 0/1024 seam
    assert home_modus([1022, 1023, 0, 1, 2]) == 0
    print("mscnt_home math self-test: PASS")
