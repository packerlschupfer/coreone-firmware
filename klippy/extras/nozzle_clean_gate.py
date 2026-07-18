# Nozzle tip-clean gate (Prusa Core One cleanup_probe / G29 P9 analogue)
#
# Prusa's cleanup_probe() taps the bed with the loadcell and requires N
# CONSECUTIVE "clean" contacts before it will purge -- a dirty/oozy tip gives a
# soft/early/erratic touch that fails the clean test and RESETS the consecutive
# counter. The probed Z is DISCARDED; this is purely a pass/fail "is the tip
# clean enough to purge" gate. See /tmp/coreone_g29_purge_probe.md.
#
# We don't (yet) have Prusa's force-curve "isGood" classifier -- our
# load_cell_probe TapAnalysis.is_valid is a stub. So this Tier-1 gate uses
# tap-Z REPEATABILITY as the cleanliness proxy: a still-oozing tip keeps
# shifting its trigger height, so it can never produce clean_count taps in a row
# that agree within z_tolerance. Requiring a tight consecutive run = "the
# reading has STABILISED" = "the tip is clean". (Tier-2 would implement the real
# waveform isGood on the samples we already capture; deferred.)
#
# On failure we RAISE (abort PRINT_START with an actionable message) -- chosen
# for bring-up so a dirty tip is loud and manual, not silently worked around.
# Z is discarded (Prusa-faithful); the front-band purge mesh is a separate pass.

class NozzleCleanGate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        # Number of consecutive in-tolerance taps that means "clean".
        self.clean_count = config.getint('clean_count', 3, minval=1)
        # Two taps are "consecutive-clean" if their bed_z agree within this (mm).
        self.z_tolerance = config.getfloat('z_tolerance', 0.03, above=0.)
        # Absolute-height guard: a tap only counts if its bed_z lands within this
        # window (mm) of the EXPECTED bed height. Closes the Tier-1 hole where a
        # STABLE blob gives consistent (repeatable) but WRONG-height taps -- those
        # agree with each other and would pass repeatability alone. Anchored at
        # EXPECTED=0 when tapping at the G28-Z reference point (bed centre), where
        # the bed is ~0; a blob then reads high and is rejected. 0 = disabled.
        self.height_window = config.getfloat('height_window', 0.4, minval=0.)
        # Hard cap on taps before we give up and declare the tip dirty.
        self.max_taps = config.getint('max_taps', 9, minval=1)
        # Serpentine spot geometry (Prusa walks -X in ~2mm steps over a small
        # window so a blob fixed on one side of the tip gets re-checked
        # off-centre). step = spacing between spots; cols = spots per row before
        # stepping +Y to the next row (boustrophedon).
        self.step = config.getfloat('step', 2.0, above=0.)
        self.cols = config.getint('cols', 3, minval=1)
        # Lift above the trigger between taps / when repositioning (mm).
        self.clearance = config.getfloat('clearance', 2.0, above=0.)
        self.gcode.register_command(
            'NOZZLE_CLEAN_GATE', self.cmd_NOZZLE_CLEAN_GATE,
            desc=self.cmd_NOZZLE_CLEAN_GATE_help)

    # Boustrophedon serpentine spot around the start XY: walk -X for `cols`
    # spots, step +Y, walk +X back, etc. -- mirrors Prusa's small-window scan.
    def _spot(self, bx, by, i):
        row = i // self.cols
        col = i % self.cols
        if row % 2 == 0:
            x = bx - col * self.step
        else:
            x = bx - (self.cols - 1) * self.step + col * self.step
        y = by + row * self.step
        return x, y

    cmd_NOZZLE_CLEAN_GATE_help = (
        "Tap the bed until N consecutive in-tolerance loadcell touches confirm "
        "the nozzle tip is clean (Prusa cleanup_probe analogue). Raises if the "
        "tip stays dirty. Probed Z is discarded.")
    def cmd_NOZZLE_CLEAN_GATE(self, gcmd):
        clean_count = gcmd.get_int('CLEAN_COUNT', self.clean_count, minval=1)
        tol = gcmd.get_float('TOLERANCE', self.z_tolerance, above=0.)
        max_taps = gcmd.get_int('MAX_TAPS', self.max_taps, minval=1)
        # Absolute-height guard (0 window = disabled). EXPECTED bed height at the
        # tap spot (0 at the G28-Z reference / bed centre).
        expected = gcmd.get_float('EXPECTED', 0.)
        height_window = gcmd.get_float('HEIGHT_WINDOW', self.height_window,
                                       minval=0.)
        toolhead = self.printer.lookup_object('toolhead')
        probe = self.printer.lookup_object('probe', None)
        if probe is None:
            raise gcmd.error("NOZZLE_CLEAN_GATE: no [probe] (load_cell_probe)")
        # Require a homed Z -- the taps descend from the current height.
        if 'z' not in toolhead.get_status(
                self.printer.get_reactor().monotonic())['homed_axes']:
            raise gcmd.error("NOZZLE_CLEAN_GATE: must home Z first")
        params = probe.get_probe_params(gcmd)
        lift_speed = params['lift_speed']
        start_pos = toolhead.get_position()
        base_x, base_y = start_pos[0], start_pos[1]
        safe_z = start_pos[2]
        # Force a single tap per run_probe (like PROBE_ACCURACY's SAMPLES=1).
        fo_params = dict(gcmd.get_command_parameters())
        fo_params['SAMPLES'] = '1'
        fo_gcmd = self.gcode.create_gcode_command("", "", fo_params)

        streak = []
        zs = []
        clean = False
        for i in range(max_taps):
            x, y = self._spot(base_x, base_y, i)
            # Reposition above the next spot at the safe (lifted) height.
            toolhead.manual_move([x, y, safe_z], lift_speed)
            # One loadcell tap. Use a fresh session per tap so the loadcell
            # re-tares each time (Prusa re-tares before every contact) and the
            # session suspend/resume hooks (TMC poll etc.) wrap cleanly.
            session = probe.start_probe_session(fo_gcmd)
            try:
                session.run_probe(fo_gcmd)
                positions = session.pull_probed_results()
            finally:
                session.end_probe_session()
            z = positions[-1].bed_z
            zs.append(z)
            # Lift clear before the next reposition/tap.
            trig_z = toolhead.get_position()[2]
            safe_z = max(start_pos[2], trig_z + self.clearance)
            toolhead.manual_move([x, y, safe_z], lift_speed)
            # Absolute-height guard: a tap landing too far from the expected bed
            # height is over a blob/contamination -> reject it (break the streak,
            # never anchor a run on an off-height tap). Catches a STABLE blob that
            # repeatability alone would pass.
            off = abs(z - expected)
            if height_window > 0. and off > height_window:
                streak = []
                gcmd.respond_info(
                    "NOZZLE_CLEAN_GATE tap %d/%d: bed_z=%.4f  OFF-HEIGHT "
                    "(%.3f > %.3f from expected %.3f) -- streak=0/%d"
                    % (i + 1, max_taps, z, off, height_window, expected,
                       clean_count))
                continue
            # Consecutive-clean streak (repeatability): a tap that disagrees with
            # the previous by more than tol breaks the run (still oozing/unstable).
            if streak and abs(z - streak[-1]) > tol:
                streak = []
            streak.append(z)
            gcmd.respond_info(
                "NOZZLE_CLEAN_GATE tap %d/%d: bed_z=%.4f  streak=%d/%d"
                % (i + 1, max_taps, z, len(streak), clean_count))
            if len(streak) >= clean_count:
                clean = True
                break
        if not clean:
            rng = (max(zs) - min(zs)) if zs else 0.
            far = max((abs(z - expected) for z in zs), default=0.)
            raise gcmd.error(
                "NOZZLE_CLEAN_GATE: tip NOT clean -- %d taps, never %d "
                "consecutive within %.3fmm AND within %.3fmm of expected %.3f "
                "(z range %.4fmm, worst off-height %.3fmm). Purge/wipe the "
                "nozzle and restart the print."
                % (len(zs), clean_count, tol, height_window, expected, rng, far))
        gcmd.respond_info(
            "NOZZLE_CLEAN_GATE: tip clean (%d consecutive taps within %.3fmm, "
            "within %.3fmm of expected %.3f)"
            % (clean_count, tol, height_window, expected))
        # Z discarded (Prusa-faithful pass/fail gate). Leave the tip lifted.


def load_config(config):
    return NozzleCleanGate(config)
