# loadcell_pa_probe.py -- pressure-advance bench probe (calibration tool)
#
# Measures whether the Core One's HX717 nozzle loadcell resolves a pressure-advance
# step-response in AIR well enough to pick an optimal K -- the approach CNC Kitchen's
# PrusaPATuner / markniu's bd_pressure / the Bambu A1 use. A bench/calibration tool,
# NOT a print-path module: enable it temporarily in printer.cfg while tuning PA, then
# remove it. (If the loadcell-PA approach is made permanent it graduates to a proper
# native Klipper module; this stays the hands-on measurement tool.)
#
# Method: for each PA value, set SET_PRESSURE_ADVANCE, run a slow->fast extrude
# velocity step in air (the accel event PA reacts to) then stop (the decel event),
# while capturing the raw channel-A loadcell force. Writes one CSV per PA + a
# one-line summary. Reuses the exact HX717 ch-A client estall_detect uses.
#
# USAGE (idle, hotend HOT, filament loaded, nozzle over a safe spot -- it extrudes
# ~2*ELEN mm of filament in air per PA value):
#   [loadcell_pa_probe]              # add temporarily to printer.cfg
#   #probe: probe                    # the [load_cell_probe] sharing the HX717
#   #out_dir: /tmp                   # where the CSVs go (on the Pi)
#
#   LOADCELL_PA_PROBE PA_LIST=0.0,0.03,0.06 TEMP=220
#   -> /tmp/loadcell_pa_probe_<i>_pa<k>.csv (t_s,raw) + a summary line per PA.
#
# It suppresses estall around the sweep and restores PA + interleave after.
# Copyright (C) 2026 -- GNU GPLv3.
import logging
import math
import os
import random

try:
    import numpy as np          # klippy already requires numpy for load_cell_probe
except ImportError:
    np = None


def _detrend(y):
    x = np.arange(len(y), dtype=float)
    a, b = np.polyfit(x, y, 1)
    return y - (a * x + b)


def phase_lag_ms(force, cmd, dt, max_lag_s=0.6):
    # Cross-correlate the measured force against the COMMANDED extrusion waveform
    # and return the peak lag in ms. THIS is the metric that makes calibration
    # possible: it is SIGNED -- force LAGS the command when PA is too low
    # (positive), LEADS when too high (negative), and passes through ZERO at the
    # optimum. So the optimum is a zero crossing, needing no weights or tuning.
    # (A peak/overshoot amplitude metric is monotonic in PA and therefore can
    # never have an optimum -- that dead end cost us three test rounds.)
    # Using the whole waveform also means we never have to resolve the ~34ms
    # transient's shape sample-by-sample.
    if np is None or len(force) < 16 or len(cmd) < 16:
        return float('nan')
    f = _detrend(np.asarray(force, dtype=float))
    c = _detrend(np.asarray(cmd, dtype=float))
    fs, cs = f.std(), c.std()
    if fs < 1e-9 or cs < 1e-9:
        return float('nan')
    f = f / fs
    c = c / cs
    corr = np.correlate(f, c, 'full')
    lags = np.arange(-len(c) + 1, len(f)) * dt
    m = np.abs(lags) <= max_lag_s
    if not m.any():
        return float('nan')
    cc, ll = corr[m], lags[m]
    i = int(np.argmax(cc))
    # Reject boundary-saturated peaks: they are aliased to the next cycle or are
    # noise, and letting them through is what produces non-monotonic lag-vs-PA
    # scatter (the reference implementation documents the same failure).
    if i == 0 or i == len(cc) - 1:
        return float('nan')
    den = cc[i - 1] - 2 * cc[i] + cc[i + 1]
    off = 0.5 * (cc[i - 1] - cc[i + 1]) / den if den != 0 else 0.
    return float((ll[i] + off * dt) * 1000.)


class LoadcellPAProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.probe_name = config.get('probe', 'probe')
        self.out_dir = config.get('out_dir', '/tmp')
        self.sensor = None
        self._cap = None            # list of (t, raw) while capturing, else None
        self.printer.register_event_handler('klippy:connect', self._connect)
        self.gcode.register_command('LOADCELL_PA_PROBE', self.cmd_PROBE,
                                    desc="SPIKE: capture loadcell force vs PA in"
                                         " air (signal-resolution check)")

    def _connect(self):
        probe = self.printer.lookup_object(self.probe_name, None)
        if probe is None or not hasattr(probe, 'get_sensor'):
            raise self.printer.config_error(
                "[loadcell_pa_probe] needs a [load_cell_probe] (probe: %s)"
                " sharing the HX717" % (self.probe_name,))
        self.sensor = probe.get_sensor()

    def _on_batch(self, msg):
        if self._cap is not None:
            for row in msg.get('data', []):
                self._cap.append((row[0], row[1]))
        return True

    def _check_abort(self, gcmd):
        """Bail out cleanly if soft_cancel's abort flag is set.

        WHY THIS EXISTS (2026-09-26). This sweep runs 20-30 minutes inside ONE gcode command,
        so it holds the gcode mutex for its whole duration. That makes it exactly the class
        [soft_cancel] was built for -- but soft_cancel only made the BLOCKING WAITS flag-aware
        (M190/M109, TEMPERATURE_WAIT, the probe loops). A long motion loop like this one was
        never covered, so the flag could be set and nothing would look at it.

        Found the hard way: this probe was started at the wrong Z, `SOFT_ABORT` was issued, and
        the sweep kept running until an emergency stop. Two separate reasons, both worth knowing:
          * the SOFT_ABORT *gcode command* is itself mutex-bound, so it queued behind this macro
            and never executed -- out-of-band callers must use the soft_cancel/abort WEBHOOK
            (see scripts/soft-abort.sh);
          * and even once the flag IS set, nothing here was checking it.
        This closes the second half. Checked between PA measurements rather than mid-capture, so
        an abort lands on a clean boundary and never truncates a trace into the CSV.
        """
        sc = self.printer.lookup_object('soft_cancel', None)
        if sc is not None and getattr(sc, 'aborting', False):
            raise gcmd.error("LOADCELL_PA_PROBE: soft-abort requested -- stopping between"
                             " measurements. PA is restored by the caller's cleanup;"
                             " SOFT_ABORT_RESET to clear the flag.")

    def cmd_PROBE(self, gcmd):
        extruder = self.printer.lookup_object('extruder', None)
        toolhead = self.printer.lookup_object('toolhead')
        if extruder is None:
            raise gcmd.error("no extruder")
        eventtime = self.reactor.monotonic()
        temp = gcmd.get_float('TEMP', 0., minval=0., maxval=300.)
        mintemp = gcmd.get_float('MINTEMP', 180., minval=0.)
        pa_list = []
        for tok in gcmd.get('PA_LIST', '0.0,0.03,0.06').split(','):
            tok = tok.strip()
            if tok:
                pa_list.append(float(tok))
        if not pa_list:
            raise gcmd.error("PA_LIST empty")
        elen = gcmd.get_float('ELEN', 4.0, above=0.)
        # FAST leg lengthened (was = ELEN): the PA signal lives in the velocity-STEP
        # transient, and 4mm @F900 is only ~0.27s = ~87 samples at 320Hz -- too few to
        # characterise. 8mm @F900 ~= 0.53s (~170 samples) resolves it properly.
        elen_fast = gcmd.get_float('ELEN_FAST', 8.0, above=0.)
        f_slow = gcmd.get_float('FSLOW', 120., above=0.)    # mm/min (~2 mm/s)
        f_fast = gcmd.get_float('FFAST', 900., above=0.)    # mm/min (~15 mm/s)
        settle = gcmd.get_float('SETTLE', 0.4, minval=0.05)
        # REPEATS>1 = the repeatability test: does the PA-driven difference exceed
        # run-to-run variance? Without it a single sweep cannot distinguish the two.
        repeats = gcmd.get_int('REPEATS', 3, minval=1)   # 3 = calibration default
        # RANDOMIZE: sweeping PA in a fixed ascending order makes PA perfectly
        # correlated with time-within-sweep, so ANY drift masquerades as a PA trend
        # (this is exactly what the 2026-07-19 runs measured -- the two positions
        # trended in OPPOSITE directions). Shuffling per repeat decouples the two:
        # a PA effect that survives randomisation is real.
        randomize = gcmd.get_int('RANDOMIZE', 1, minval=0, maxval=1)
        # PRIME: the first measurement of a sweep reads systematically LOW (cold /
        # unprimed melt). Extruding a slug before EACH measurement normalises the
        # melt state so every sample starts from the same condition.
        prime = gcmd.get_float('PRIME', 15.0, minval=0.)
        # REFINE: run an automatic narrow second stage around the bracket found by
        # the first sweep (see the two-stage note below). REFINE=0 = single sweep.
        refine = gcmd.get_int('REFINE', 1, minval=0, maxval=1)
        f_prime = gcmd.get_float('FPRIME', 300., above=0.)
        # CIRCLE: the extrudate is part of the measurement chain -- a strand hanging
        # from a STATIC nozzle piles up under itself and (2026-07-19) got manually
        # pulled mid-run, applying hand force straight to the loadcell and invalidating
        # two whole test rounds. Tracing a big circle while extruding LAYS THE STRAND
        # OUT along the arc: nothing accumulates under the nozzle, nothing ever needs
        # touching, and the run is fully hands-off.
        # Why a circle specifically: tangential speed is CONSTANT (no speed-change
        # inertia), and the only residual acceleration is centripetal (v^2/r) which
        # acts in the XY PLANE -- while the loadcell reads Z force, so it barely
        # couples in. A big radius shrinks v^2/r further. A straight line with a speed
        # step would inject an in-plane accel TRANSIENT at exactly the measurement
        # moment; the circle keeps it constant and steady instead.
        circle_r = gcmd.get_float('CIRCLE_R', 40., minval=0.)   # 0 disables
        circle_f = gcmd.get_float('CIRCLE_F', 2400., above=0.)  # mm/min XY, CONSTANT
        # SQUARE WAVE (CYCLES>0) -- the PrusaPATuner-style excitation. A single step
        # gives one transition and forces a peak/shape metric, which is monotonic in
        # PA and therefore has NO optimum. A repeating square wave gives many
        # transitions to CROSS-CORRELATE against the commanded waveform, yielding a
        # SIGNED phase lag: force LAGS the command when PA is too low, LEADS when too
        # high, and passes through ZERO at the optimum. The zero crossing is
        # parameter-free -- no weights to choose (which is what sank the composite).
        cycles = gcmd.get_int('CYCLES', 6, minval=0)     # 0 = legacy single step
        period = gcmd.get_float('PERIOD', 0.8, above=0.05)   # s per full cycle
        slow_dur = elen / (f_slow / 60.)        # s, the slow leg -> step boundary
        # Optional heat + wait.
        if temp > 0.:
            self.gcode.run_script_from_command("M104 S%.0f" % temp)
            self.gcode.run_script_from_command("M109 S%.0f" % temp)
        cur = extruder.get_status(eventtime).get('temperature', 0.)
        if cur < mintemp:
            raise gcmd.error("extruder %.0fC < MINTEMP %.0f -- heat first "
                             "(TEMP=<t> or preheat)" % (cur, mintemp))
        # Circle centre = wherever the caller parked the nozzle. Validate the whole
        # circle fits the axis limits BEFORE any motion -- an out-of-range G1 would
        # abort mid-sweep and leave the heater on and the melt in an unknown state.
        self._ang = 0.
        pos = toolhead.get_position()
        self._circle_c = (pos[0], pos[1])
        if circle_r > 0.:
            kin = toolhead.get_kinematics()
            lo = [r.get_range() for r in kin.get_rails()[:2]] \
                if hasattr(kin, 'get_rails') else None
            st = toolhead.get_status(eventtime)
            amin, amax = st.get('axis_minimum'), st.get('axis_maximum')
            if amin and amax:
                for c, lo_, hi_ in ((pos[0], amin[0], amax[0]),
                                    (pos[1], amin[1], amax[1])):
                    if c - circle_r < lo_ or c + circle_r > hi_:
                        raise gcmd.error(
                            "CIRCLE_R=%.0f around (%.1f,%.1f) leaves the bed "
                            "(limits X %.0f..%.0f Y %.0f..%.0f). Park the nozzle "
                            "nearer the centre or lower CIRCLE_R."
                            % (circle_r, pos[0], pos[1],
                               amin[0], amax[0], amin[1], amax[1]))
        gcmd.respond_info(
            "LOADCELL_PA_PROBE: %d PA values x %d repeat(s) in AIR%s. "
            "HANDS OFF the extrudate for the whole run."
            % (len(pa_list), repeats,
               (", tracing r=%.0fmm circles @%.0fmm/s so the strand lays out "
                "instead of piling" % (circle_r, circle_f / 60.))
               if circle_r > 0. else " (STATIC nozzle -- strand will pile!)"))
        # Save PA to restore; suppress estall; turn on interleave for ch-A stream.
        pa_cfg = None
        try:
            pa_cfg = self.printer.lookup_object('configfile').get_status(
                eventtime)['settings']['extruder'].get('pressure_advance')
        except Exception:
            pass
        has_estall = self.printer.lookup_object('estall_detect', None) is not None
        has_inter = hasattr(self.sensor, 'enable_interleave')
        if not self._cap_subscribed():
            self.sensor.add_client(self._on_batch, channel='A')
            self._subscribed = True
        results = []
        try:
            if has_estall:
                self.gcode.run_script_from_command("ESTALL_BLOCK BLOCK=1")
            if has_inter:
                self.sensor.enable_interleave()
            self.gcode.run_script_from_command("M83")
            # TWO-STAGE by default. Measured 2026-07-21 (PLA HS): a wide sweep
            # extrapolates the crossing from points far off-optimum and reads
            # systematically HIGH -- its own raw data crossed zero at ~0.020 while
            # the fits returned 0.0228-0.0259. A narrow second stage straddling the
            # bracket interpolates instead, collapsing the fit spread 0.005 -> 0.0001.
            # So: sweep coarse to BRACKET, then re-sweep narrow to MEASURE.
            stage_pas = list(pa_list)
            for _stage in (0, 1):
                results = []
                for rep in range(repeats):
                    order = list(enumerate(stage_pas))
                    if randomize:
                        random.shuffle(order)       # break the PA<->time correlation
                    gcmd.respond_info(
                        "--- repeat %d/%d  order=%s%s ---"
                        % (rep + 1, repeats,
                           " ".join("%.3f" % p for _, p in order),
                           "" if randomize else " (FIXED order -- drift confound!)"))
                    for i, pa in order:
                        self._check_abort(gcmd)
                        self.gcode.run_script_from_command(
                            "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % pa)
                        # PRIME the melt to a consistent state (NOT captured).
                        if prime > 0.:
                            # Prime on the arc too -- a static prime would dump its slug
                            # exactly where the measurement then starts.
                            if circle_r > 0.:
                                self._extrude_arc(prime, f_prime, circle_r, circle_f)
                            else:
                                self.gcode.run_script_from_command(
                                    "G1 E%.3f F%.0f" % (prime, f_prime))
                            toolhead.dwell(settle)
                            toolhead.wait_moves()
                        toolhead.dwell(settle)      # settle at the new PA
                        toolhead.wait_moves()
                        self._cap = []              # start capture
                        # velocity STEP: slow leg (steady state) -> fast leg (the accel
                        # event PA acts on) -> stop (decel).
                        segs = []
                        if cycles > 0:
                            # SQUARE WAVE: N cycles of slow/fast half-periods. Record each
                            # half's print_time span + commanded rate so the analysis can
                            # rebuild the command waveform to correlate against.
                            for _c in range(cycles):
                                for f_e in (f_slow, f_fast):
                                    t_a = toolhead.get_last_move_time()
                                    e_amt = (f_e / 60.) * (period / 2.)
                                    if circle_r > 0.:
                                        self._extrude_arc(e_amt, f_e, circle_r, circle_f)
                                    else:
                                        self.gcode.run_script_from_command(
                                            "G1 E%.4f F%.0f" % (e_amt, f_e))
                                    t_b = toolhead.get_last_move_time()
                                    segs.append((t_a, t_b, f_e / 60.))
                        elif circle_r > 0.:
                            self._extrude_arc(elen, f_slow, circle_r, circle_f)
                            self._extrude_arc(elen_fast, f_fast, circle_r, circle_f)
                        else:
                            self.gcode.run_script_from_command("G1 E%.3f F%.0f"
                                                               % (elen, f_slow))
                            self.gcode.run_script_from_command("G1 E%.3f F%.0f"
                                                               % (elen_fast, f_fast))
                        toolhead.dwell(settle)      # capture the post-stop decay
                        toolhead.wait_moves()
                        cap = self._cap
                        self._cap = None
                        r = self._summarise(gcmd, i, pa, rep, cap, slow_dur, segs)
                        if r:
                            results.append(r)
                self._report_repeatability(gcmd, results, repeats)
                info = self._report_phase_lag(gcmd, results)
                if _stage or not refine:
                    break
                nxt = self._refine_list(gcmd, info, stage_pas)
                if nxt is None:
                    break
                stage_pas = nxt
        finally:
            # Restore PA, interleave, estall -- always.
            if pa_cfg is not None:
                self.gcode.run_script_from_command(
                    "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % float(pa_cfg))
            if has_inter:
                try:
                    self.sensor.disable_interleave()
                except Exception:
                    logging.exception("loadcell_pa_probe: interleave restore")
            if has_estall:
                self.gcode.run_script_from_command("ESTALL_BLOCK BLOCK=0")
        gcmd.respond_info("LOADCELL_PA_PROBE done. Eyeball the CSVs in %s -- if the "
                          "force traces SEPARATE across PA, a native module is worth "
                          "building; if not, drop it." % (self.out_dir,))

    def _cap_subscribed(self):
        return getattr(self, '_subscribed', False)

    def _extrude_arc(self, e_total, f_e, radius, f_xy):
        # Extrude e_total at E-feedrate f_e while tracing a constant-speed arc of
        # radius `radius` around the circle centre captured at run start. The arc
        # LENGTH is chosen so the XY move lasts exactly as long as the E move would
        # have on its own -- i.e. the E rate (and therefore the velocity step the
        # measurement depends on) is UNCHANGED from the static version; the motion
        # only carries the extrudate away. The angle advances monotonically across
        # calls so successive legs/measurements never retrace the same ground.
        dur = e_total / (f_e / 60.)             # s, unchanged E timing
        arc_len = (f_xy / 60.) * dur
        nseg = max(2, int(arc_len / 2.))        # ~2mm segments
        cx, cy = self._circle_c
        for _ in range(nseg):
            self._ang += (arc_len / nseg) / radius      # radians
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f E%.5f F%.0f"
                % (cx + radius * math.cos(self._ang),
                   cy + radius * math.sin(self._ang),
                   e_total / nseg, f_xy))

    def _summarise(self, gcmd, i, pa, rep, cap, slow_dur, segs=None):
        if not cap:
            gcmd.respond_info("  PA=%.4f: NO samples captured (interleave off?)" % pa)
            return None
        # CLOCK FIX: rows carry MCU print_time; the old code subtracted a
        # reactor.monotonic() t0 -- a DIFFERENT clock -- so the CSV's t_s had a
        # meaningless absolute offset. Anchor on the first captured sample instead.
        t0 = cap[0][0]
        vals = [r[1] for r in cap]
        base = sum(vals[:min(10, len(vals))]) / min(10, len(vals))
        # WINDOW ON THE STEP: the PA-sensitive transient is the fast leg, not the
        # whole trace. Split at the slow-leg duration; report the step amplitude
        # (fast-leg peak above the slow-leg steady state), which is what PA changes.
        slow = [v for (t, v) in cap if (t - t0) < slow_dur]
        fast = [v for (t, v) in cap if (t - t0) >= slow_dur]
        if len(slow) >= 5 and fast:
            n_tail = max(1, len(slow) // 4)
            steady = sum(slow[-n_tail:]) / n_tail   # end-of-slow steady state
            pk_fast = max(fast)
            step = pk_fast - steady
        else:                                       # window fell over: whole-trace
            steady = base
            pk_fast = max(vals)
            step = pk_fast - steady
        path = os.path.join(self.out_dir, "loadcell_pa_probe_r%d_%d_pa%.4f.csv"
                            % (rep, i, pa))
        try:
            with open(path, 'w') as f:
                # `cmd` = commanded E rate (mm/s) at that sample, reconstructed from
                # the recorded print_time spans. This is the reference waveform the
                # phase-lag cross-correlation needs; without it there is nothing to
                # correlate the force against.
                f.write("t_s,raw,leg,cmd\n")
                for t, raw in cap:
                    rt = t - t0
                    cmd = ''
                    if segs:
                        for (ta, tb, rate) in segs:
                            if ta <= t < tb:
                                cmd = "%.4f" % rate
                                break
                    f.write("%.5f,%d,%s,%s\n"
                            % (rt, raw, 'slow' if rt < slow_dur else 'fast', cmd))
        except Exception:
            logging.exception("loadcell_pa_probe: CSV write failed")
            path = "(write failed)"
        # --- PHASE LAG (the calibration metric) -------------------------------
        lag = float('nan')
        if segs and np is not None:
            ts, fs_, cs_ = [], [], []
            for t, raw in cap:
                for (ta, tb, rate) in segs:
                    if ta <= t < tb:
                        ts.append(t); fs_.append(raw); cs_.append(rate)
                        break
            if len(ts) >= 32:
                ts = np.asarray(ts); dt = float(np.median(np.diff(ts)))
                if dt > 0:
                    grid = np.arange(ts[0], ts[-1], dt)     # uniform resample
                    lag = phase_lag_ms(np.interp(grid, ts, np.asarray(fs_)),
                                       np.interp(grid, ts, np.asarray(cs_)), dt)
        gcmd.respond_info(
            "  PA=%.4f rep%d: n=%d STEP=%.0f%s -> %s"
            % (pa, rep, len(cap), step,
               ("  LAG=%+.1fms" % lag) if lag == lag else "  LAG=n/a",
               os.path.basename(path)))
        return {'pa': pa, 'rep': rep, 'n': len(cap), 'steady': steady,
                'peak': pk_fast, 'step': step, 'lag': lag}

    def _refine_list(self, gcmd, info, prev):
        # Build the narrow stage-2 sweep from the stage-1 bracket. Straddling the
        # crossing with real points turns the estimate from an EXTRAPOLATION into an
        # INTERPOLATION, which is where the wide sweep's high bias came from.
        if not info:
            return None
        lo, hi = info.get('lo'), info.get('hi')
        if lo != lo or hi != hi or hi <= lo:
            return None
        w = hi - lo
        # Already fine-grained: a second stage would only re-measure the same points.
        if w <= 0.004:
            gcmd.respond_info(
                "  REFINE: bracket is already %.4f wide -- no second stage needed."
                % w)
            return None
        pad = 0.25 * w
        n = 6
        step = (w + 2. * pad) / (n - 1)
        out = []
        for i in range(n):
            v = round(max(0., lo - pad + i * step), 4)
            if v not in out:
                out.append(v)
        if len(out) < 3:
            return None
        gcmd.respond_info(
            "=== STAGE 2 (REFINE): narrowing to %s ===\n"
            "  Stage 1 bracketed the crossing between %.4f and %.4f; re-measuring "
            "INSIDE that bracket removes the extrapolation bias of the wide sweep."
            % (" ".join("%.4f" % v for v in out), lo, hi))
        return out

    def _report_phase_lag(self, gcmd, results):
        # Aggregate the signed phase lag per PA and solve lag(PA) = 0. The zero
        # crossing IS the optimal pressure advance -- parameter-free.
        if np is None:
            gcmd.respond_info("PA-LAG: numpy unavailable, skipping")
            return
        good = [r for r in results if r.get('lag') == r.get('lag')]
        if not good:
            gcmd.respond_info(
                "PA-LAG: no usable lags (run with CYCLES>0 -- a square wave is "
                "required; a single step gives nothing to correlate)")
            return
        pas = sorted(set(r['pa'] for r in good))
        gcmd.respond_info("=== PHASE LAG by PA (signed; zero = optimum) ===")
        ks, ys, spreads = [], [], []
        for pa in pas:
            xs = [r['lag'] for r in good if r['pa'] == pa]
            m = sum(xs) / len(xs)
            sd = (max(xs) - min(xs)) if len(xs) > 1 else 0.
            ks.append(pa); ys.append(m)
            if len(xs) > 1:
                spreads.append(sd)
            gcmd.respond_info("  PA=%.4f  n=%d  lag=%+7.1f ms  spread=%.1f  [%s]"
                              % (pa, len(xs), m, sd,
                                 " ".join("%+.0f" % x for x in xs)))
        if len(ks) < 2:
            gcmd.respond_info("  need >=2 PA values to solve for a crossing")
            return
        k = np.asarray(ks, dtype=float); y = np.asarray(ys, dtype=float)
        if y.min() > 0 or y.max() < 0:
            # Two very different faults both show up as "no sign change", and they
            # need OPPOSITE actions -- so tell them apart before advising:
            #  (a) FLAT lag across the whole sweep, with per-PA spread of the same
            #      order as the total span => the PA signal is GONE, not mis-centred.
            #      Widening PA_LIST would waste a run. The known cause is the
            #      extrudate coupling force back into the loadcell -- measured
            #      2026-07-19: identical PLA sweeps gave a 44.6ms span with
            #      CIRCLE_R=40 but only 3.1ms (no trend) with CIRCLE_R=0.
            #  (b) a genuine bracket miss: strong monotonic trend, just offset.
            span = float(y.max() - y.min())
            typ = (sum(spreads) / len(spreads)) if spreads else 0.
            if span < max(5.0, 2.0 * typ):
                gcmd.respond_info(
                    "  /!\\ lag is FLAT (%.1f..%.1f ms, span %.1f vs typical "
                    "per-PA spread %.1f) -- this is a DEAD SIGNAL, not a bad "
                    "bracket. Widening PA_LIST will NOT help."
                    % (y.min(), y.max(), span, typ))
                gcmd.respond_info(
                    "      Most likely the extrudate is loading the loadcell: use "
                    "CIRCLE_R>0 (a static nozzle piles the strand under itself), "
                    "keep HANDS OFF it, and make sure it can fall away freely.")
                return
            gcmd.respond_info(
                "  /!\\ lag never changes sign (%.1f..%.1f ms) but IS trending -- the "
                "sweep did not bracket the optimum. Widen PA_LIST %s and re-run."
                % (y.min(), y.max(), "UPWARD" if y.min() > 0 else "DOWNWARD"))
            return
        a1, b1 = np.polyfit(k, y, 1)
        lin = -b1 / a1 if a1 != 0 else float('nan')
        quad = float('nan')
        # Distinguish "not enough points for a quadratic" from "quadratic fitted
        # but found no usable root" -- they need different user action.
        too_few = len(k) < 4 or len(set(ks)) < 3
        if not too_few:
            a2, b2, c2 = np.polyfit(k, y, 2)
            lo, hi = k.min() - 0.2 * np.ptp(k), k.max() + 0.2 * np.ptp(k)
            for r in np.roots([a2, b2, c2]):
                if abs(r.imag) < 1e-9 and lo <= r.real <= hi:
                    # descending branch = the physical over-PA crossing
                    if 2 * a2 * r.real + b2 < 0:
                        quad = float(r.real)
        # WHICH fit to trust? Measured against two independently pa-tower-calibrated
        # materials, the two fits BRACKET the truth -- linear reads HIGH, quadratic
        # reads LOW -- so their MEAN cancels the bias and beats either alone:
        #            linear    quadratic   MEAN     reference
        #   PETG     +0.0037   -0.0028    +0.0004    0.034
        #   PLA HS   +0.0007   -0.0021    -0.0007    0.022
        # (The reference implementation's "prefer the quadratic root" did NOT
        # reproduce here -- linear won on PLA HS, quadratic on PETG. n=2 materials,
        # so treat the mean as empirical-best, not proven, and read the
        # linear-quadratic spread as the honest uncertainty band.)
        best = 0.5 * (lin + quad) if quad == quad else lin
        if quad == quad:
            why = "  quadratic=%.4f" % quad
        elif too_few:
            why = ("  (quadratic needs >=4 PA values -- using LINEAR, which reads "
                   "HIGH; add PA points to improve)")
        else:
            why = "  (quadratic fitted but no in-range descending root -- using linear)"
        # THE honest uncertainty = the two MEASURED points straddling the crossing.
        # It is bounded by data rather than by how well two fits happen to agree.
        lo_i = None
        for i in range(len(ks) - 1):
            if (y[i] > 0.) != (y[i + 1] > 0.):
                lo_i = i
                break
        gcmd.respond_info("  zero crossing: linear=%.4f%s" % (lin, why))
        if quad == quad:
            gcmd.respond_info(
                "*** OPTIMAL PRESSURE ADVANCE = %.4f ***  (mean of the two fits)"
                % best)
            # NOT an error bar. On a smooth lag curve both fits converge on the SAME
            # answer whether or not that answer is accurate. Measured 2026-07-21 on
            # PLA HS: a narrow sweep reported a linear-quadratic spread of 0.0001
            # while sitting ~0.003 from the pa-tower reference -- i.e. it understated
            # the true error by ~30x. It is a smoothness diagnostic, nothing more.
            gcmd.respond_info(
                "  fit-consistency (linear vs quadratic) = %.4f -- a DIAGNOSTIC, "
                "NOT an error bar. Small only means the lag curve is smooth; it "
                "says nothing about accuracy." % abs(lin - quad))
        else:
            gcmd.respond_info(
                "*** OPTIMAL PRESSURE ADVANCE = %.4f ***  (LINEAR fit only -- add PA "
                "values to get the quadratic and a tighter estimate)" % best)
        if lo_i is not None:
            gcmd.respond_info(
                "  UNCERTAINTY: crossing is bracketed by MEASURED points %.4f "
                "(lag %+.1f ms) and %.4f (lag %+.1f ms). Quote THAT interval -- "
                "the fit's extra decimals are not resolution."
                % (ks[lo_i], y[lo_i], ks[lo_i + 1], y[lo_i + 1]))
        gcmd.respond_info(
            "  Set it per-filament, e.g. M572 S%.3f in the filament start g-code."
            % best)
        gcmd.respond_info(
            "  NB per-FILAMENT: valid only for the material/temperature just tested."
            " Validated against pa-tower references on PETG (0.034) and PLA HS "
            "(0.022); the PLA HS agreement is to within the bracket width, not to "
            "the fit's last digit.")
        return {'best': best,
                'lo': ks[lo_i] if lo_i is not None else float('nan'),
                'hi': ks[lo_i + 1] if lo_i is not None else float('nan')}

    def _report_repeatability(self, gcmd, results, repeats):
        # THE decision metric: is the PA-driven change in STEP bigger than the
        # run-to-run spread at a fixed PA? If not, the signal cannot calibrate PA.
        if not results:
            return
        pas = sorted(set(r['pa'] for r in results))
        gcmd.respond_info("=== STEP amplitude by PA ===")
        means = {}
        worst_spread = 0.
        for pa in pas:
            xs = [r['step'] for r in results if r['pa'] == pa]
            m = sum(xs) / len(xs)
            means[pa] = m
            spread = (max(xs) - min(xs)) if len(xs) > 1 else float('nan')
            if len(xs) > 1:
                worst_spread = max(worst_spread, spread)
            gcmd.respond_info("  PA=%.4f  n=%d  mean STEP=%.0f  spread=%s  %s"
                              % (pa, len(xs), m,
                                 ("%.0f" % spread) if len(xs) > 1 else "n/a",
                                 " ".join("%.0f" % x for x in xs)))
        if len(pas) > 1:
            rng = max(means.values()) - min(means.values())
            gcmd.respond_info("  PA-driven range (max-min of means) = %.0f" % rng)
            if repeats > 1 and worst_spread > 0:
                ratio = rng / worst_spread
                gcmd.respond_info(
                    "  VERDICT: PA-range/worst-repeat-spread = %.2f -- %s"
                    % (ratio,
                       "USABLE: PA effect exceeds run-to-run noise" if ratio >= 3.
                       else ("MARGINAL: needs more repeats/averaging" if ratio >= 1.5
                             else "NOT USABLE: PA effect is buried in variance")))
            else:
                gcmd.respond_info("  (run with REPEATS=3 for the variance verdict)")


def load_config(config):
    return LoadcellPAProbe(config)
