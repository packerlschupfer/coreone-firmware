# BELT_TUNE -- measure XY belt resonant frequency (a belt-tension proxy), the
# Klipper analogue of Prusa's M960: drive a motor-excited frequency sweep and
# find the resonance peak. Reports X and Y so they can be tensioned to match.
#
# The Core One has NO permanent accelerometer (it only clips to the nozzle for
# input-shaper calibration), so there are TWO sensor modes:
#
#   SENSOR=sg    (default, NO accelerometer): read the TMC StallGuard load
#                (sg_result) during the sweep. The belt resonance shows up as an
#                extremum in the load response (max std-dev). Always available.
#                EXPERIMENTAL -- validate on hardware; if the SG signal is too
#                weak, use SENSOR=accel.
#   SENSOR=accel (accelerometer clipped to the nozzle): the accurate M960-style
#                method -- accelerometer RMS response peak.
#
# Reuses [resonance_tester]'s vibration excitation (same accel-pulse drive) and
# [lis3dh]/[adxl345] for the accel mode. Config:
#   [belt_tune]
#     accel_per_hz: 75           # excitation strength (mm/s^2 per Hz)
#     cycles: 40                 # excitation half-periods per frequency dwell
#     sensor: sg                 # default sensor mode
#     #measure_position: 125,105 # where to vibrate (default: current pos)
#
# Usage:  BELT_TUNE [AXIS=X|Y|BOTH] [SENSOR=sg|accel]
#                   [FREQ_START=50] [FREQ_END=150] [STEP=2]
import math
from . import resonance_tester


class BeltTune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.accel_per_hz = config.getfloat('accel_per_hz', 75., above=0.)
        self.cycles = config.getint('cycles', 40, minval=8)
        self.default_sensor = config.get('sensor', 'sg').lower()
        self.freq_start = config.getfloat('freq_start', 50., minval=10.)
        self.freq_end = config.getfloat('freq_end', 150.,
                                        minval=self.freq_start, maxval=300.)
        self.step = config.getfloat('step', 2., above=0.)
        mp = config.get('measure_position', None)
        self.measure_pos = [float(v) for v in mp.split(',')] if mp else None
        # Optional tension readout: T = 4*mu*L^2*f^2 (Prusa's formula). Set
        # belt_length_m to enable N output; mu defaults to Prusa's GT2 belt,
        # target to Prusa's 18 N. (Core One belt length isn't published, so
        # frequency + X/Y matching stays the primary output.)
        self.belt_length_m = config.getfloat('belt_length_m', 0., minval=0.)
        self.belt_mass_kg_m = config.getfloat('belt_mass_kg_m', 0.007569,
                                              above=0.)
        self.target_tension_n = config.getfloat('target_tension_n', 18.,
                                                above=0.)
        # SG sampling state
        self._sg_tmc = self._sg_fields = self._sg_timer = None
        self._sg_samples = []
        self.gcode.register_command('BELT_TUNE', self.cmd_BELT_TUNE,
                                    desc="Measure XY belt resonant frequency "
                                         "(SG or accelerometer)")

    # ---- shared vibration driver (replicates resonance_tester's move loop) ----
    def _gen_burst(self, freq):
        # constant-frequency accel pulses (no sweep): self.cycles half-periods
        t_seg = 0.25 / freq
        accel = self.accel_per_hz * freq
        res = []
        t = 0.
        sign = 1.
        for _ in range(self.cycles):
            t += t_seg
            res.append((t, sign * accel, freq))
            t += t_seg
            res.append((t, -sign * accel, freq))
            sign = -sign
        return res

    def _vibrate(self, toolhead, taxis, burst, tpos):
        X, Y, Z = tpos[:3]
        last_v = last_t = 0.
        for next_t, accel, freq in burst:
            t_seg = next_t - last_t
            abs_last_v = abs(last_v)
            last_v2 = last_v * last_v
            if abs(accel) < 1e-6:
                v, abs_v = last_v, abs_last_v
                if abs_v < 1e-6:
                    toolhead.dwell(t_seg)
                    last_t = next_t
                    continue
                half_inv_accel = 0.
                d = v * t_seg
            else:
                toolhead.set_max_velocities(None, abs(accel), None, None)
                v = last_v + accel * t_seg
                abs_v = abs(v)
                if abs_v < 1e-6:
                    v = abs_v = 0.
                half_inv_accel = .5 / accel
                d = (v * v - last_v2) * half_inv_accel
            dX, dY, dZ = taxis.get_point(d)
            nX, nY, nZ = X + dX, Y + dY, Z + dZ
            toolhead.limit_next_junction_speed(abs_last_v)
            if v * last_v < 0:
                d_decel = -last_v2 * half_inv_accel
                ddX, ddY, ddZ = taxis.get_point(d_decel)
                toolhead.move([X + ddX, Y + ddY, Z + ddZ] + tpos[3:],
                              abs_last_v)
                toolhead.move([nX, nY, nZ] + tpos[3:], abs_v)
            else:
                toolhead.move([nX, nY, nZ] + tpos[3:], max(abs_v, abs_last_v))
            X, Y, Z = nX, nY, nZ
            last_t, last_v = next_t, v
        # bursts are symmetric (+a,-a pairs) so last_v ~= 0; return to start to
        # clear any residual drift, then settle.
        toolhead.move(list(tpos), 50.)
        toolhead.wait_moves()

    # ---- StallGuard sensor (no accelerometer) --------------------------------
    def _sg_setup(self, axis):
        name = 'stepper_' + axis.lower()
        tmc = self.printer.lookup_object('tmc2130 ' + name)
        self._sg_tmc = tmc
        self._sg_fields = tmc.mcu_tmc.get_fields()

    def _sg_sample(self, eventtime):
        try:
            st = self._sg_tmc.mcu_tmc.get_register_raw("DRV_STATUS")
            self._sg_samples.append(
                self._sg_fields.get_field("sg_result", st["data"]))
        except Exception:
            pass
        return eventtime + 0.002

    def _sg_response(self, toolhead, taxis, burst, tpos):
        reactor = self.printer.get_reactor()
        self._sg_samples = []
        self._sg_timer = reactor.register_timer(self._sg_sample, reactor.NOW)
        try:
            self._vibrate(toolhead, taxis, burst, tpos)
        finally:
            reactor.unregister_timer(self._sg_timer)
            self._sg_timer = None
        s = self._sg_samples
        if len(s) < 4:
            return 0.
        m = sum(s) / len(s)
        return math.sqrt(sum((x - m) ** 2 for x in s) / len(s))   # std-dev

    # ---- accelerometer sensor (clipped to nozzle) ----------------------------
    def _accel_chip(self, rt):
        for chip_axis, chip in rt.accel_chips:
            if 'x' in chip_axis or 'y' in chip_axis:
                return chip
        raise self.gcode.error("no XY accelerometer found in [resonance_tester]")

    def _accel_response(self, toolhead, taxis, burst, tpos, chip, ai):
        aclient = chip.start_internal_client()
        try:
            self._vibrate(toolhead, taxis, burst, tpos)
        finally:
            aclient.finish_measurements()
        comp = [s[1 + ai] for s in aclient.get_samples()]   # ax/ay column
        if len(comp) < 4:
            return 0.
        m = sum(comp) / len(comp)
        return math.sqrt(sum((c - m) ** 2 for c in comp) / len(comp))  # RMS

    # ---- sweep one axis -------------------------------------------------------
    def _sweep_axis(self, gcmd, axis, sensor, fstart, fend, step):
        toolhead = self.printer.lookup_object('toolhead')
        rt = self.printer.lookup_object('resonance_tester', None)
        if rt is None:
            raise gcmd.error("[resonance_tester] is required")
        taxis = resonance_tester.TestAxis(
            vib_dir=(1., 0., 0.) if axis == 'X' else (0., 1., 0.))
        ai = 0 if axis == 'X' else 1
        if self.measure_pos is not None:
            toolhead.manual_move(
                [self.measure_pos[0], self.measure_pos[1], None], 80.)
        toolhead.wait_moves()
        tpos = toolhead.get_position()
        chip = self._accel_chip(rt) if sensor == 'accel' else None
        if sensor == 'sg':
            self._sg_setup(axis)
        # raise accel ceiling for the high-frequency bursts
        old = toolhead.get_status(self.printer.get_reactor().monotonic())
        self.gcode.run_script_from_command(
            "SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=%.0f MINIMUM_CRUISE_RATIO=0"
            % (self.accel_per_hz * fend + 500,))
        ishaper = self.printer.lookup_object('input_shaper', None)
        if ishaper is not None:
            ishaper.disable_shaping()
        results = []
        try:
            freq = fstart
            while freq <= fend + 1e-6:
                burst = self._gen_burst(freq)
                if sensor == 'accel':
                    r = self._accel_response(toolhead, taxis, burst, tpos,
                                             chip, ai)
                else:
                    r = self._sg_response(toolhead, taxis, burst, tpos)
                results.append((freq, r))
                freq += step
        finally:
            if ishaper is not None:
                ishaper.enable_shaping()
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT VELOCITY=%.0f ACCEL=%.0f"
                " MINIMUM_CRUISE_RATIO=%.2f"
                % (old['max_velocity'], old['max_accel'],
                   old['minimum_cruise_ratio']))
        peak = max(results, key=lambda fr: fr[1])
        spectrum = "  ".join("%.0f:%.2f" % (f, r) for f, r in results)
        tstr = ""
        if self.belt_length_m:
            t = (4. * self.belt_mass_kg_m * self.belt_length_m ** 2
                 * peak[0] ** 2)
            tstr = " (~%.1f N, target %.0f)" % (t, self.target_tension_n)
        gcmd.respond_info("BELT_TUNE %s (%s): PEAK = %.1f Hz%s\n  %s"
                          % (axis, sensor, peak[0], tstr, spectrum))
        return peak[0]

    def cmd_BELT_TUNE(self, gcmd):
        axis = gcmd.get('AXIS', 'BOTH').upper()
        sensor = gcmd.get('SENSOR', self.default_sensor).lower()
        if sensor not in ('sg', 'accel'):
            raise gcmd.error("SENSOR must be sg or accel")
        fstart = gcmd.get_float('FREQ_START', self.freq_start, minval=10.)
        fend = gcmd.get_float('FREQ_END', self.freq_end, minval=fstart)
        step = gcmd.get_float('STEP', self.step, above=0.)
        th = self.printer.lookup_object('toolhead')
        homed = th.get_status(self.printer.get_reactor().monotonic())[
            'homed_axes']
        if 'x' not in homed or 'y' not in homed:
            raise gcmd.error("home X and Y first")
        axes = ['X', 'Y'] if axis == 'BOTH' else [axis]
        peaks = {}
        for a in axes:
            peaks[a] = self._sweep_axis(gcmd, a, sensor, fstart, fend, step)
        if len(peaks) == 2:
            msg = ("X=%.0f Y=%.0f Hz (d%.0f)"
                   % (peaks['X'], peaks['Y'], abs(peaks['X'] - peaks['Y'])))
            gcmd.respond_info("BELT_TUNE result: %s -- tension X and Y to match."
                              % (msg,))
        else:
            a = axes[0]
            msg = "%s=%.0f Hz" % (a, peaks[a])
        # show the peak on the LCD status line
        self.gcode.run_script_from_command("M117 Belt %s" % (msg,))


def load_config(config):
    return BeltTune(config)
