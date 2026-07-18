# Cogging-correction calibration for phase_exec (Stage B) — Prusa-style sweep.
#
# Instead of the old discrete 2-probe solve (too noisy for the small cogging), this
# mirrors Prusa's calibration: the MCU SWEEPS one harmonic's correction continuously
# as the head makes ONE slow constant-velocity move, while the lis3dh records. We
# then lock-in (demodulate at the cogging frequency f_h + sliding-Hann window) to get
# the f_h magnitude vs the swept parameter, and take the ARGMIN as the optimum. One
# move covers the whole parameter range -> no executor stall, and the windowed lock-in
# pulls the small signal out of the noise.
#
# Per harmonic: sweep PHA (0..1024 at a fixed mag) -> best phase; then sweep MAG
# (0..max at that phase) -> best magnitude + the baseline/residual for the report.
#
# WORKFLOW (accel clips ABOVE the nozzle tip; loadcell shielded by its sleeve so we
# home X/Y only and pause the loadcell during the run, FIRMWARE_RESTART to restore):
#   clip lis3dh -> G28 X + G28 Y -> CALIBRATE_PHASE_COGGING [STEPPER=..] ->
#   review -> SAVE_CONFIG -> FIRMWARE_RESTART -> remove sensor -> full G28.
# CoreXY isolation: stepper_x=X+Y motor -> drive X=Y; stepper_y=X-Y -> drive X=-Y.

import math
import numpy as np


class PhaseCogging:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.accel_chip = config.get('accel_chip', 'lis3dh')
        self.harmonics = [int(x) for x in
                          config.get('harmonics', '2, 4').replace(' ', '').split(',')]
        self.velocity = config.getfloat('velocity', 4., above=0.)     # slow = clean read
        self.distance = config.getfloat('distance', 60., above=1.)    # long = mostly cruise
        self.travel_v = config.getfloat('travel_velocity', 50., above=0.)
        self.fixed_mag = config.getint('probe_mag', 60, minval=10, maxval=400)
        # per-harmonic sweep cap = mag_limit/h. Above ~163/h the modulated angle goes
        # non-monotonic -> the motor skips -> position desync -> crash. Stay well under.
        self.mag_limit = config.getint('mag_limit', 120, minval=10)
        self.rate = config.getint('rate', 1500, minval=500)
        # keep the calibration in the FRONT half: phase-stepping drifts position a
        # little during sweeps, so leave a big margin to the rear frame (Y~220).
        self.center = [config.getfloat('center_x', 125.),
                       config.getfloat('center_y', 65.)]
        # belt mm per electrical period (4 full steps * rotation_distance/full_steps)
        self.eperiod = config.getfloat('electrical_period', 0.32, above=0.)
        self.window_s = config.getfloat('window_s', 0.12, above=0.01)  # lock-in window
        # stage 1 DEFAULT (Prusa-faithful): MOVING speed-sweep. Drive the motor diagonal
        # at rising constant speeds; the cogging excitation freq f_h = h*v*sqrt2/eperiod
        # scales WITH speed, so the sweep walks the cogging bin up through the structural
        # resonance. A parked hum can only reach its top listed freq -> clips high modes
        # (our X clipped at 130Hz); Prusa calibration.cpp:996 sweeps SPEED. Bounded passes
        # (vs Prusa's single long ramp) keep us inside the front-half frame margin.
        self.speed_list = [float(x) for x in
                           config.get('speed_list', '8,14,20,26,32,40,48,56').replace(' ', '').split(',')]
        self.scan_half = config.getfloat('scan_half', 35., above=5.)   # bounded pass half-len
        # per-(speed,harmonic) NYQUIST gate vs the MEASURED sample rate (lis3dh ~1344Hz)
        self.nyquist_frac = config.getfloat('nyquist_frac', 0.42, above=0.1, below=0.5)
        self.peak_speed_shift = config.getfloat('peak_speed_shift', 0.9, above=0.1)  # Prusa 0.9
        self.max_sweep_v = config.getfloat('max_sweep_v', 60., above=1.)  # stage-2 v cap
        self.use_moving = config.getboolean('moving_resonance', True)  # False = parked-hum
        # MEASUREMENT METHOD. 'coherent' (DEFAULT, the rebuild): direct complex synchronous
        # detection at the rotor's h-th electrical harmonic + a 2-probe complex solve,
        # measured OFF the structural resonance. Rejects the on-resonance structural-ringing
        # contamination that made the magnitude-sweep ('sweep') regress (it argmin'd
        # |cogging + uncontrollable ringing| -> wrong correction). 'sweep' = the old
        # ramp+argmin path (kept for comparison).
        self.method = config.get('method', 'moving')
        # PROJECTION (Prusa Core One reads ONE accel axis per motor, NOT the CoreXY sqrt2
        # diagonal which mixes both motors -> cross-contamination = spurious fwd/bwd
        # asymmetry). 'auto' = pick the axis with the strongest cogging response per motor;
        # or force 'x'/'y'/'z'/'diag'.
        self.proj_axis = config.get('proj_axis', 'auto').lower()
        # marker/settle window (Prusa VIBRATION_SETTLE_TIME 0.2): drop accel/decel ends of a
        # capture using the move's precise print-time clock (we have the time-sync Prusa
        # fakes with marker pulses), so the lock-in sees only the constant-velocity cruise.
        self.settle_s = config.getfloat('settle_s', 0.15, above=0.01)
        self._cur_proj = ('diag', 1.0)   # active projection (set per motor in cmd_CAL)
        # target cogging bin (Hz) for the coherent measurement -- place in a GAP between the
        # structural modes (we've measured ~70 / ~130 / ~283 Hz), NOT on a resonance.
        self.meas_freq = config.getfloat('meas_freq', 100., above=10.)
        self.meas_distance = config.getfloat('meas_distance', 40., above=5.)  # long = SNR
        self.probe_mag2 = config.getint('probe_mag2', 24, minval=4)   # complex-solve probe
        self.iterations = config.getint('iterations', 1, minval=0)    # Newton refine steps
        self.edge_drop = config.getfloat('edge_drop', 0.12, above=0., below=0.4)
        # stage 1 FALLBACK: STATIONARY hum (gantry parked) at rising freqs
        self.freq_list = [float(x) for x in
                          config.get('freq_list', '40, 55, 70, 85, 100, 115, 130').replace(' ', '').split(',')]
        self.osc_amp = config.getint('osc_amp', 200, minval=10, maxval=500)
        self.hum_dwell = config.getfloat('hum_dwell', 0.3, above=0.05)
        self.motors = {'stepper_x': +1.0, 'stepper_y': -1.0}
        self.gcode.register_command('CALIBRATE_PHASE_COGGING', self.cmd_CAL,
                                    desc="Accelerometer cogging calibration (Prusa-style sweep)")

    def _pause_loadcell(self, accel_obj):
        # Stop every F427 bulk reader EXCEPT the accel (the HX717 stream starved the
        # MCU). Not restored (raw _start corrupts the LoadCell) -> FIRMWARE_RESTART after.
        accel_mcu = accel_obj.get_mcu() if hasattr(accel_obj, 'get_mcu') else None
        paused = []
        seen = set()
        for name, obj in self.printer.lookup_objects():
            if obj is accel_obj:
                continue
            for o in (obj, getattr(obj, 'sensor', None)):
                if o is None or o is accel_obj:
                    continue
                bb = getattr(o, 'batch_bulk', None)
                if bb is None or id(bb) in seen:
                    continue
                if not (hasattr(bb, '_stop') and hasattr(bb, '_start')):
                    continue
                omcu = o.get_mcu() if hasattr(o, 'get_mcu') else None
                if accel_mcu is not None and omcu is not None and omcu is not accel_mcu:
                    continue
                seen.add(id(bb))
                try:
                    bb._stop()
                    paused.append(name)
                except Exception:
                    pass
        return paused

    def _get_accel(self):
        obj = self.printer.lookup_object(self.accel_chip, None)
        if obj is None:
            raise self.gcode.error("accel chip '%s' not found" % (self.accel_chip,))
        if not hasattr(obj, 'start_internal_client'):
            raise self.gcode.error("'%s' has no start_internal_client" % (self.accel_chip,))
        return obj

    def cmd_CAL(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self.printer.get_reactor().monotonic())['homed_axes']
        if 'x' not in homed or 'y' not in homed:
            raise gcmd.error("Home X/Y first (G28 X + G28 Y), clip the accelerometer, "
                             "THEN run CALIBRATE_PHASE_COGGING.")
        accel = self._get_accel()
        self.rate = gcmd.get_int('RATE', self.rate, minval=500)
        self.velocity = gcmd.get_float('VELOCITY', self.velocity, above=0.)
        self.distance = gcmd.get_float('DISTANCE', self.distance, above=1.)
        self.fixed_mag = gcmd.get_int('PROBE_MAG', self.fixed_mag, minval=10, maxval=400)
        self.window_s = gcmd.get_float('WINDOW_S', self.window_s, above=0.01)
        # coherent-method overrides (live, no restart): MEAS_FREQ Hz of the cogging bin
        # (higher = nearer resonance = more signal/SNR; coherence rejects the structural
        # ringing), MEAS_DIST mm cruise (longer = more averaging), ITERS Newton refines,
        # PROBE2 the complex-solve probe magnitude (larger = cleaner gradient).
        self.meas_freq = gcmd.get_float('MEAS_FREQ', self.meas_freq, above=10.)
        self.meas_distance = gcmd.get_float('MEAS_DIST', self.meas_distance, above=5.)
        self.iterations = gcmd.get_int('ITERS', self.iterations, minval=0, maxval=6)
        self.probe_mag2 = gcmd.get_int('PROBE2', self.probe_mag2, minval=4, maxval=200)
        self.method = gcmd.get('METHOD', self.method).lower()
        self.proj_axis = gcmd.get('PROJ', self.proj_axis).lower()   # auto|x|y|z|diag
        self.settle_s = gcmd.get_float('SETTLE', self.settle_s, above=0.01)
        which = gcmd.get('STEPPER', 'BOTH').lower()
        names = list(self.motors) if which in ('both', 'all') else [which]
        harms = gcmd.get('HARMONICS', None)
        harmonics = ([int(x) for x in harms.replace(' ', '').split(',')]
                     if harms else self.harmonics)
        pes = {}
        for n in names:
            pe = self.printer.lookup_object('phase_exec %s' % (n,), None)
            if pe is None:
                raise gcmd.error("no [phase_exec %s]" % (n,))
            pes[n] = pe
        gcmd.respond_info("Cogging cal: method=%s proj=%s settle=%.2fs motors=%s harmonics=%s. "
                          "Accel must be clipped."
                          % (self.method, self.proj_axis, self.settle_s, names, harmonics))
        paused = self._pause_loadcell(accel)
        gcmd.respond_info("Paused %d bulk sensor(s): %s. (FIRMWARE_RESTART after to "
                          "restore the loadcell.)" % (len(paused), ', '.join(paused) or 'none'))
        self.gcode.run_script_from_command("PHASE_STEP_ON RATE=%d" % (self.rate,))
        results = []
        # Solve + apply each harmonic INDEPENDENTLY per travel direction (Prusa
        # forward_current / backward_current). The +Y-per-pad real-print drift is a
        # direction-ANTISYMMETRIC cogging term a single symmetric LUT can't cancel.
        DIRS = [(0, 'fwd', False), (1, 'bwd', True)]
        try:
            for n in names:
                osign = self.motors[n]
                pe = pes[n]
                res_speeds = None
                # Prusa single-axis projection: pick (or honor) the accel axis that isolates
                # THIS motor before any measurement. This is the #1 fix vs the CoreXY sqrt2
                # diagonal, which mixed both motors -> spurious fwd/bwd asymmetry.
                self._cur_proj = ('diag', osign)
                if self.method != 'coherent':
                    proj, presp = self._detect_proj_axis(toolhead, accel, pe, osign, harmonics)
                    self._cur_proj = proj
                    pname = ('x', 'y', 'z')[proj[1]] if proj[0] == 'axis' else 'diag'
                    if presp:
                        gcmd.respond_info(
                            "  %s proj-detect: x=%.0f y=%.0f z=%.0f diag=%.0f -> using %s"
                            % (n, presp['x'], presp['y'], presp['z'], presp['diag'], pname))
                    else:
                        gcmd.respond_info("  %s proj: forced %s" % (n, pname))
                    res_speeds = self._find_resonance(toolhead, accel, pe, osign, harmonics)
                    gcmd.respond_info("  %s: peak speeds (x%.2f) %s mm/s"
                                      % (n, self.peak_speed_shift,
                                         {h: round(res_speeds[h], 1) for h in harmonics}))
                for h in harmonics:
                    if self.method == 'coherent':
                        # put the cogging bin at meas_freq (off-resonance) -> velocity/h
                        self.velocity = max(2., min(
                            self.meas_freq * self.eperiod / (h * math.sqrt(2.)),
                            self.max_sweep_v))
                    for d, dname, rev in DIRS:
                        if self.method == 'coherent':
                            # solves AND applies internally (probes + Newton)
                            res = self._calibrate_one_coherent(toolhead, accel, pe, n,
                                                               osign, h, rev)
                        else:
                            res = self._calibrate_one(toolhead, accel, pe, n, osign, h,
                                                      res_speeds[h], reverse=rev)
                            pe.set_correction(h, res['mag'], res['pha'], direction=d)
                        results.append((n, h, d, dname, res))
                        gcmd.respond_info(
                            "  %s H%d %s: mag=%d pha=%d  (cogging |a0|=%.3f -> |a|=%.3f, %s)"
                            % (n, h, dname, res['mag'], res['pha'], res['a0'],
                               res['av'],
                               ("-%.0f%%" % res['red'] if res['red'] >= 0 else "WORSE")))
        finally:
            self.gcode.run_script_from_command("PHASE_STEP_OFF REHOME=0")
        ok = [r for r in results if r[4]['red'] >= 20.]
        gcmd.respond_info(
            "Done: %d/%d (motor,harmonic,dir) improved >=20%%. Values live. Persist per "
            "direction as harmonic<H>_mag/_pha (fwd) + harmonic<H>_mag_bwd/_pha_bwd (bwd) "
            "in each [phase_exec <m>] + SAVE_CONFIG. LOADCELL PAUSED -> FIRMWARE_RESTART, "
            "remove the accel, full G28 before printing."
            % (len(ok), len(results)))

    def _find_resonance(self, toolhead, accel, pe, osign, harmonics):
        # Stage 1 (Prusa-faithful MOVING speed-sweep): drive the motor diagonal at each
        # speed_list speed (bounded, front-half); DFT each harmonic's cogging bin
        # f_h = h*v*sqrt2/eperiod; the speed whose bin peaks = that harmonic's resonance
        # speed. Back off x peak_speed_shift (Prusa 0.9 = just below resonance, stable).
        # Per-(speed,h) NYQUIST gate vs the measured fs so an aliased bin can't pose as
        # a peak. Returns {h: stage-2 sweep velocity mm/s}.
        if not self.use_moving:
            f = self._find_resonance_hum(toolhead, accel, pe, osign)   # parked fallback
            return {h: max(2., f * self.eperiod / (h * math.sqrt(2.))) for h in harmonics}
        # Build the full speed x harmonic response grid.
        grid = {}
        for v in self.speed_list:
            grid[v] = self._measure_moving(toolhead, accel, pe, osign, v, harmonics)
        # HARMONIC-COMB fit (Prusa find_harmonic_peaks): the cogging bin f_h = h*v*k hits the
        # SAME structural mode f* when h*v is constant, so harmonic h peaks at v_h = vb*hmax/h
        # (a comb in speed). Score each base speed vb (for the top harmonic) by the comb sum;
        # the speed whose comb is best-supported across BOTH harmonics is the real cogging
        # resonance -- a lone spurious structural peak in one harmonic can't win. This is the
        # fix for the old per-harmonic argmax locking onto random structure.
        hmax = max(harmonics)

        def nearest(vq):
            return min(self.speed_list, key=lambda s: abs(s - vq))
        best_vb, best_score = None, -1.
        for vb in self.speed_list:
            score = 0.
            for h in harmonics:
                score += grid[nearest(vb * hmax / float(h))].get(h, 0.)
            if score > best_score:
                best_score, best_vb = score, vb
        if best_vb is None:
            best_vb = self.speed_list[len(self.speed_list) // 2]
        out = {}
        for h in harmonics:
            vh = best_vb * hmax / float(h)
            out[h] = max(2., min(vh * self.peak_speed_shift, self.max_sweep_v))
        return out

    def _measure_moving(self, toolhead, accel, pe, osign, v, harmonics):
        # ONE bounded constant-velocity diagonal pass at speed v; per-harmonic single-bin
        # DFT, Nyquist-gated against the capture's measured sample rate.
        cx, cy = self.center
        hl = self.scan_half
        toolhead.manual_move([cx - hl, cy - osign * hl, None], self.travel_v)
        toolhead.wait_moves()
        lo_t = toolhead.get_last_move_time() + self.settle_s
        aclient = accel.start_internal_client()
        toolhead.manual_move([cx + hl, cy + osign * hl, None], v)
        hi_t = toolhead.get_last_move_time() - self.settle_s
        toolhead.wait_moves()
        aclient.finish_measurements()
        samples = aclient.get_samples()
        out = {}
        if len(samples) >= 64:
            span = samples[-1].time - samples[0].time
            fs = (len(samples) - 1) / span if span > 0 else 1000.
            for h in harmonics:
                f_h = h * v * math.sqrt(2.) / self.eperiod
                if f_h < self.nyquist_frac * fs:
                    out[h] = self._lockin_mag(samples, self._cur_proj, f_h, lo_t, hi_t)
        return out

    def _find_resonance_hum(self, toolhead, accel, pe, osign):
        # FALLBACK (moving_resonance: False): park the gantry, HUM the motor in place at
        # rising frequencies, find the accel-response peak. No translation -> no drift.
        cx, cy = self.center
        toolhead.manual_move([cx, cy, None], self.travel_v)
        toolhead.wait_moves()
        best_f, best_m = self.freq_list[0], -1.
        for f in self.freq_list:
            inc = max(1, int(round(f * 1024. / self.rate)))   # osc_phase units per tick
            m = self._measure_hum(toolhead, accel, pe, osign, f, inc)
            if m > best_m:
                best_m, best_f = m, f
        return best_f

    def _measure_hum(self, toolhead, accel, pe, osign, f, inc):
        # gantry parked; motor hums at f; return the accel response at f
        pe.osc(self.osc_amp, inc)
        aclient = accel.start_internal_client()
        toolhead.dwell(self.hum_dwell)
        toolhead.wait_moves()
        aclient.finish_measurements()
        pe.osc_stop()
        return self._dft_mag(aclient.get_samples(), osign, f)

    def _dft_mag(self, samples, osign, f_h):
        if not samples:
            return 0.
        t = np.array([s.time for s in samples])
        ap = (np.array([s.accel_x for s in samples])
              + osign * np.array([s.accel_y for s in samples])) / math.sqrt(2.)
        n = len(ap)
        lo = int(n * 0.15); hi = n - lo                  # drop accel/decel
        if hi - lo < 32:
            return 0.
        tr = t[lo:hi] - t[lo]
        demod = ap[lo:hi] * np.exp(-2j * np.pi * f_h * tr)
        return float(abs(demod.mean()) * 2.)

    # ---- Prusa-faithful projection + windowing ------------------------------ #
    def _project(self, samples, proj):
        # proj = ('diag', osign) -> CoreXY sqrt2 mix (legacy); ('axis', idx) -> a SINGLE
        # accelerometer axis (Prusa Core One), which isolates one motor.
        if proj[0] == 'diag':
            return (np.array([s.accel_x for s in samples])
                    + proj[1] * np.array([s.accel_y for s in samples])) / math.sqrt(2.)
        key = ('accel_x', 'accel_y', 'accel_z')[proj[1]]
        return np.array([getattr(s, key) for s in samples], dtype=float)

    def _lockin_mag(self, samples, proj, f_h, lo_t, hi_t):
        # complex synchronous detection of the cogging bin over the precise [lo_t, hi_t]
        # constant-velocity window (markers via clock, not pulses). |mean| -> bin amplitude.
        if not samples:
            return 0.
        t = np.array([s.time for s in samples])
        ap = self._project(samples, proj)
        mask = (t >= lo_t) & (t <= hi_t)
        if int(mask.sum()) < 32:
            return 0.
        trm = t[mask]; apm = ap[mask]
        demod = apm * np.exp(-2j * np.pi * f_h * (trm - trm[0]))
        return float(abs(demod.mean()) * 2.)

    def _detect_proj_axis(self, toolhead, accel, pe, osign, harmonics):
        # ONE mid-band cruise; pick the accel axis with the strongest cogging response at the
        # lowest harmonic. Replaces the CoreXY sqrt2 diagonal (which folds in BOTH motors).
        if self.proj_axis in ('x', 'y', 'z'):
            return ('axis', {'x': 0, 'y': 1, 'z': 2}[self.proj_axis]), {}
        if self.proj_axis == 'diag':
            return ('diag', osign), {}
        v = self.speed_list[len(self.speed_list) // 2]
        cx, cy = self.center; hl = self.scan_half
        toolhead.manual_move([cx - hl, cy - osign * hl, None], self.travel_v)
        toolhead.wait_moves()
        lo_t = toolhead.get_last_move_time() + self.settle_s
        aclient = accel.start_internal_client()
        toolhead.manual_move([cx + hl, cy + osign * hl, None], v)
        hi_t = toolhead.get_last_move_time() - self.settle_s
        toolhead.wait_moves()
        aclient.finish_measurements()
        samples = aclient.get_samples()
        h = min(harmonics)
        f_h = h * v * math.sqrt(2.) / self.eperiod
        resp = {nm: self._lockin_mag(samples, ('axis', i), f_h, lo_t, hi_t)
                for i, nm in enumerate(('x', 'y', 'z'))}
        resp['diag'] = self._lockin_mag(samples, ('diag', osign), f_h, lo_t, hi_t)
        best = max(range(3), key=lambda i: resp[('x', 'y', 'z')[i]])
        return ('axis', best), resp

    def _calibrate_one(self, toolhead, accel, pe, name, osign, h, res_speed,
                       reverse=False):
        # run THIS harmonic's diagonal sweep at its moving-sweep peak speed (already
        # x peak_speed_shift). reverse=False measures the FORWARD cogging LUT (motor
        # rotating +), reverse=True the BACKWARD LUT -- each direction solved + applied
        # independently (Prusa forward_current/backward_current). Returns WITHOUT applying;
        # cmd_CAL applies it to the matching direction's LUT.
        # Stage-2 velocity = the moving-sweep peak speed, frame/Nyquist-bounded -- NOT the
        # old 2-16 mm/s clamp (which starved SNR; Prusa runs 6-128 mm/s).
        self.velocity = max(2., min(res_speed, self.max_sweep_v))
        cap = max(8, int(self.mag_limit / h))    # monotonicity-safe cap (no motor skip)
        fixed = min(self.fixed_mag, cap)
        # 1) sweep PHASE (full circle) at a fixed magnitude -> best phase
        rp = self._sweep_measure(toolhead, accel, pe, osign, h, 0, 1024, fixed, 0,
                                 reverse=reverse)
        pha_opt = int(round(rp['prog'] * 1024.)) % 1024
        # 2) sweep MAGNITUDE (0..cap) at that phase -> best magnitude + baseline/residual
        rm = self._sweep_measure(toolhead, accel, pe, osign, h, pha_opt, 0, 0, cap,
                                 reverse=reverse)
        mag_opt = int(round(rm['prog'] * cap))
        a0, av = rm['baseline'], rm['minval']
        if mag_opt > 0 and a0 > 0 and av < a0:
            red = 100. * (1. - av / a0)
        else:
            mag_opt, pha_opt, red = 0, 0, 0.
        return {'mag': mag_opt, 'pha': pha_opt, 'a0': a0, 'av': av, 'red': red}

    def _sweep_measure(self, toolhead, accel, pe, osign, h, pha_s, pha_d, mag_s, mag_d,
                       reverse=False):
        cx, cy = self.center
        half = self.distance / 2.0
        # forward = move along +diagonal (motor rotates forward); reverse swaps the
        # endpoints so the motor rotates backward -> measures the BACKWARD cogging LUT.
        sgn = -1.0 if reverse else 1.0
        toolhead.manual_move([cx - sgn * half, cy - sgn * osign * half, None], self.travel_v)
        toolhead.wait_moves()
        start_pt = toolhead.get_last_move_time()
        aclient = accel.start_internal_client()
        # the MCU sweep MUST be stopped even if the move / arm / wait raises. If it is
        # left armed (sweep_active=1), pe_sweep_update force-overwrites this harmonic's live
        # cogging correction with the stale sweep-endpoint magnitude (up to the sweep cap ~120
        # vs the calibrated 2-4) on EVERY subsequent engage -> a severe, silent cogging
        # mis-correction on every print until a FIRMWARE_RESTART. Wrap in try/finally.
        try:
            toolhead.manual_move([cx + sgn * half, cy + sgn * osign * half, None], self.velocity)
            end_pt = toolhead.get_last_move_time()
            duration = end_pt - start_pt
            # arm the MCU sweep anchored to the move's print-time window (reaches the MCU
            # before the move runs via the lookahead buffer; ramp is keyed off start_clock)
            start_clock = pe.mcu.print_time_to_clock(start_pt)
            dur_per_1024 = max(1, int(pe.mcu.seconds_to_clock(duration) // 1024))
            pe.sweep(h, start_clock, dur_per_1024, pha_s, pha_d, mag_s, mag_d)
            toolhead.wait_moves()
        finally:
            aclient.finish_measurements()
            pe.sweep_stop(h)                # idempotent if the sweep was never armed
        return self._analyze_sweep(aclient.get_samples(), osign, h, start_pt, duration)

    def _analyze_sweep(self, samples, osign, h, start_pt, duration):
        if not samples or duration <= 0:
            return {'prog': 0., 'baseline': 0., 'minval': 0.}
        t = np.array([s.time for s in samples])
        ap = self._project(samples, self._cur_proj)       # Prusa single-axis projection
        tr = t - start_pt
        prog = tr / duration
        mask = (prog > 0.05) & (prog < 0.95)              # drop accel/decel + edges
        if int(mask.sum()) < 64:
            return {'prog': 0., 'baseline': 0., 'minval': 0.}
        trm = tr[mask]; apm = ap[mask]; progm = prog[mask]
        f_h = h * (self.velocity * math.sqrt(2.)) / self.eperiod
        # lock-in: demodulate at f_h then sliding-Hann low-pass -> |amplitude| vs time
        demod = apm * np.exp(-2j * np.pi * f_h * trm)
        span = trm[-1] - trm[0]
        fs = len(trm) / span if span > 0 else 1000.
        win_len = max(8, int(fs * self.window_s))
        win = np.hanning(win_len)
        win = win / win.sum()
        mag = np.abs(np.convolve(demod, win, mode='same'))
        imin = int(np.argmin(mag))
        prog_opt = float(progm[imin])
        minval = float(mag[imin])
        base_mask = progm < (float(progm.min()) + 0.12)   # near sweep start (~uncorrected)
        baseline = float(mag[base_mask].mean()) if bool(base_mask.any()) else float(mag.max())
        return {'prog': prog_opt, 'baseline': baseline, 'minval': minval}

    # ------------------------------------------------------------------ #
    # ANGLE-COHERENT REBUILD (method='coherent') -- direct complex solve. #
    # ------------------------------------------------------------------ #
    def _measure_cogging_complex(self, toolhead, accel, pe, osign, h, reverse):
        # one constant-velocity diagonal cruise; return the COMPLEX cogging amplitude at
        # the rotor's h-th electrical harmonic (phase preserved). self.velocity is set by
        # the caller to put f_h at meas_freq (off-resonance).
        cx, cy = self.center
        half = self.meas_distance / 2.0
        sgn = -1.0 if reverse else 1.0
        toolhead.manual_move([cx - sgn * half, cy - sgn * osign * half, None],
                             self.travel_v)
        toolhead.wait_moves()
        start_pt = toolhead.get_last_move_time()
        aclient = accel.start_internal_client()
        toolhead.manual_move([cx + sgn * half, cy + sgn * osign * half, None],
                             self.velocity)
        end_pt = toolhead.get_last_move_time()
        toolhead.wait_moves()
        aclient.finish_measurements()
        return self._sync_detect(aclient.get_samples(), osign, h, start_pt,
                                 end_pt - start_pt, sgn)

    def _sync_detect(self, samples, osign, h, start_pt, duration, sgn):
        # COMPLEX synchronous detection at the rotor's h-th electrical harmonic over the
        # constant-velocity cruise. theta_elec = 2*pi*f1*t (f1 = v*sqrt2/eperiod elec
        # periods/s); the cogging is phase-locked to theta -> coherent integration over
        # many periods builds it up while rotor-INCOHERENT structural ringing averages
        # toward zero. Keeping the complex value (not |.|) preserves the phase the solve
        # needs. Returns C_h as a python complex (0 if too few samples).
        if not samples or duration <= 0:
            return 0j
        t = np.array([s.time for s in samples])
        # (dead-path note): this hardcodes the (accel_x + osign*accel_y)/sqrt(2) diagonal
        # projection -- the exact cross-motor contamination `_detect_proj_axis`/`_cur_proj` was
        # built to eliminate (and which the default 'moving' method uses). Harmless ONLY because
        # the 'coherent' method that calls this is retired ('method: moving' is the default). If
        # 'coherent' is ever revived, switch this to `self._project(samples, self._cur_proj)` and
        # run proj-detect first, or it will measure contaminated cogging.
        ap = (np.array([s.accel_x for s in samples])
              + osign * np.array([s.accel_y for s in samples])) / math.sqrt(2.)
        tr = t - start_pt
        m = (tr > self.edge_drop * duration) & (tr < (1. - self.edge_drop) * duration)
        if int(m.sum()) < 64:
            return 0j
        trm = tr[m]
        apm = ap[m] - ap[m].mean()                      # drop DC / ramp bias
        f1 = self.velocity * math.sqrt(2.) / self.eperiod
        theta = 2. * math.pi * f1 * trm * sgn           # rotor electrical phase (signed)
        c = np.mean(apm * np.exp(-1j * h * theta)) * 2.
        return complex(c)

    def _complex_to_magpha(self, corr, cap):
        mag = int(round(min(abs(corr), float(cap))))
        pha = int(round((math.atan2(corr.imag, corr.real) % (2. * math.pi))
                        * 1024. / (2. * math.pi))) % 1024
        return mag, pha

    def _apply_complex(self, pe, h, d, corr, cap):
        mag, pha = self._complex_to_magpha(corr, cap)
        pe.set_correction(h, mag, pha, direction=d)

    def _calibrate_one_coherent(self, toolhead, accel, pe, name, osign, h, reverse):
        # DIRECT complex solve. Cogging = a complex vector C in the accel at rotor harmonic
        # h. Applying correction `corr` adds G*corr (G = local complex gain from
        # correction-space to accel-space). Measure C0 (corr=0), probe two ORTHOGONAL
        # corrections to estimate G, solve corr=-C0/G, apply + Newton-refine, re-measure.
        # Leaves the solved correction APPLIED on this direction's LUT.
        d = 1 if reverse else 0
        # snapshot this (h,d) correction before the solve. The solve applies probe
        # corrections (mag ~12-60, arbitrary phase) via set_correction, which mutates the
        # PERSISTENT pe.corrections/corr_bwd dict. If a measurement raises mid-solve, the last
        # probe value would be left in the dict and re-sent at every future engage. Restore the
        # pre-solve value on error so nothing stale persists, then re-raise.
        _store = pe.corrections if d == 0 else pe.corr_bwd
        _saved = _store.get(h)
        try:
            cap = max(8, int(self.mag_limit / h))
            pm = max(2, min(self.probe_mag2, cap // 2))
            pe.set_correction(h, 0, 0, direction=d)
            c0 = self._measure_cogging_complex(toolhead, accel, pe, osign, h, reverse)
            if abs(c0) < 1e-9:
                return {'mag': 0, 'pha': 0, 'a0': 0., 'av': 0., 'red': 0.}
            # two orthogonal probes: pha=0 (real) and pha=+90deg (imag)
            self._apply_complex(pe, h, d, complex(pm, 0.), cap)
            c1 = self._measure_cogging_complex(toolhead, accel, pe, osign, h, reverse)
            self._apply_complex(pe, h, d, complex(0., pm), cap)
            c2 = self._measure_cogging_complex(toolhead, accel, pe, osign, h, reverse)
            g = 0.5 * ((c1 - c0) / complex(pm, 0.) + (c2 - c0) / complex(0., pm))
            if abs(g) < 1e-12:
                pe.set_correction(h, 0, 0, direction=d)
                return {'mag': 0, 'pha': 0, 'a0': abs(c0), 'av': abs(c0), 'red': 0.}
            corr = -c0 / g
            for _ in range(self.iterations):           # Newton: drive residual to 0 via G
                self._apply_complex(pe, h, d, corr, cap)
                cr = self._measure_cogging_complex(toolhead, accel, pe, osign, h, reverse)
                corr = corr - cr / g
            mag, pha = self._complex_to_magpha(corr, cap)
            pe.set_correction(h, mag, pha, direction=d)
            cf = self._measure_cogging_complex(toolhead, accel, pe, osign, h, reverse)
            a0, av = abs(c0), abs(cf)
            if mag > 0 and av < a0:
                red = 100. * (1. - av / a0)
            else:                                       # no real improvement -> disarm
                pe.set_correction(h, 0, 0, direction=d)
                mag, pha, red = 0, 0, 0.
            return {'mag': mag, 'pha': pha, 'a0': a0, 'av': av, 'red': red}
        except Exception:
            if _saved is not None:
                pe.set_correction(h, _saved[0], _saved[1], direction=d)
            else:
                pe.set_correction(h, 0, 0, direction=d)
            raise


def load_config(config):
    return PhaseCogging(config)
