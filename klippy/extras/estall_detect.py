# Loadcell-based extruder clog / stuck-filament detection (host side).
#
# A faithful Klipper port of Prusa's EMotorStallDetector (Core One). Prusa's
# clog detector is NOT extruder StallGuard and NOT a filament-motion encoder --
# it watches the toolhead LOADCELL force signal (the same HX717 used for the
# Z-probe). When the extruder keeps pushing but the filament can't advance,
# pressure builds and the loadcell sees a force EDGE; a 5-tap Gaussian x edge
# FIR over the raw force stream fires on that transient.
#
# Mechanism (mirrors lib/.../feature/prusa/e-stall_detector.cpp):
#  - FIR = {0.1939811, 0.592259, 0, -0.592259, -0.1939811} over the last 5 raw
#    loadcell counts. The coefficients SUM TO ZERO (no DC gain): it measures a
#    derivative/edge, not absolute load, so the tare offset is irrelevant.
#  - Trip when filtered > detection_threshold (default 700000, a RAW-count
#    threshold). Our counts_per_gram (52.083) and HX717 ODR (320 Hz) match
#    Prusa's, so the threshold AND the taps port as-is. The compare is SIGNED;
#    Prusa empirically inverted the edge sign, so `invert:` is provided and the
#    polarity MUST be confirmed on hardware (see notes below).
#  - GATE: only ever reports while the extruder is MOVING AND PUSHING FORWARD
#    (derived here from extruder.find_past_position deltas). Retraction, travel,
#    and idle continuously CLEAR the transient flag, so they can't false-fire.
#    Cold extrude is implicitly covered (E won't move cold -> gate stays shut).
#  - LATCH: one over-threshold sample while pushing latches `reported`; the
#    action fires once, then `reported` clears after report_interval (Core One
#    re-arms log-only every 1 s) so a persistent clog re-reports.
#
# This pairs with the firmware HX717 channel interleave (src/sensor_hx71x.c +
# hx71x.py enable_interleave): arming the detector turns interleave ON so the
# channel-A loadcell feed stays live DURING A PRINT (without it, the port parks
# the HX717 on channel B for the filament-presence sensor and there is no force
# stream mid-print). Disarming / probing restores the prior channel mode.
#
#   [estall_detect]
#     #probe: probe                 # the [load_cell_probe] sharing the HX717
#     detection_threshold: 700000   # raw-count FIR trip level (Prusa default)
#     invert: False                 # flip edge polarity (verify on hardware!)
#     on_stall: pause               # pause | log | m600
#     min_extrude_speed: 0.5        # mm/s of filament below which the gate is shut
#     report_interval: 1.0          # s before a still-stalled print re-reports
#     default_enabled: True         # Core One config_store default is ON
#     #stall_gcode:                  # extra g-code to run on a stall (template)
#
#   ESTALL_DETECT ENABLE=1   ; arm  (also enables interleave) -- PRINT_START
#   ESTALL_DETECT ENABLE=0   ; disarm (also disables interleave) -- PRINT_END
#   ESTALL_BLOCK   BLOCK=1   ; suppress (nestable) -- wrap M600/load/unload/ram
#   ESTALL_BLOCK   BLOCK=0   ; un-suppress
#   M591 S1|S0|P|R           ; Prusa-compatible arm/disarm/persist/restore
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging

# Prusa combinedFilter (e-stall_detector.cpp:12): a Gaussian smoother convolved
# with a [-1,0,1] edge detector, sign-inverted in Prusa's code. Newest sample is
# the LAST element of the ring (see _on_batch); `invert` flips the trip polarity.
FIR_TAPS = (0.1939811, 0.592259, 0.0, -0.592259, -0.1939811)

# I2 shadow-telemetry: histogram-bin count used to characterise the per-print
# distribution of the gated 'filtered' edge (for offline threshold tuning).
TELE_NBINS = 80


class EStallDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.config_name = config.get_name()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object('gcode')
        # Config
        self.probe_name = config.get('probe', 'probe')
        self.threshold = config.getfloat('detection_threshold', 700000.,
                                         above=0.)
        self.invert = config.getboolean('invert', False)
        self.on_stall = config.getchoice(
            'on_stall', {'pause': 'pause', 'log': 'log', 'm600': 'm600'},
            'pause')
        self.min_extrude_speed = config.getfloat('min_extrude_speed', 0.5,
                                                 above=0.)
        self.report_interval = config.getfloat('report_interval', 1.0,
                                               above=0.)
        # Core One's config_store default for stuck_filament_detection is ON.
        self.default_enabled = config.getboolean('default_enabled', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.stall_template = gcode_macro.load_template(config, 'stall_gcode',
                                                        '')
        # Bound at connect
        self.probe = self.sensor = self.extruder = None
        self._supports_interleave = False
        # FIR + gate + latch state
        self._buf = collections.deque([0.0] * len(FIR_TAPS),
                                      maxlen=len(FIR_TAPS))
        self._last_filtered = 0.
        self._trip_filtered = 0.      # the FIR value that last crossed threshold
        self._last_epos = None
        self._last_t = None
        self._detected = False        # transient flag (FIR over + pushing)
        self._reported = False        # latched: action already fired
        self._report_time = 0.
        self._enabled = False
        self._block_depth = 0         # >0 = suppressed (probe / M600 / load ...)
        self._subscribed = False
        self._tele_reset()            # I2 shadow-telemetry accumulator
        # Wire up
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("klippy:ready", self._ready)
        self.gcode.register_command('ESTALL_DETECT', self._cmd_ESTALL_DETECT,
                                    desc=self.cmd_ESTALL_DETECT_help)
        self.gcode.register_command('ESTALL_BLOCK', self._cmd_ESTALL_BLOCK,
                                    desc=self.cmd_ESTALL_BLOCK_help)
        self.gcode.register_command('ESTALL_STATS', self._cmd_ESTALL_STATS,
                                    desc=self.cmd_ESTALL_STATS_help)
        try:
            self.gcode.register_command('M591', self._cmd_M591,
                                        desc="Prusa stuck-filament detection")
        except self.printer.config_error:
            pass        # already provided elsewhere

    def _connect(self):
        self.extruder = self.printer.lookup_object('extruder', None)
        self.probe = self.printer.lookup_object(self.probe_name, None)
        if self.probe is None or not hasattr(self.probe, 'get_sensor'):
            raise self.printer.config_error(
                "[estall_detect] requires a [load_cell_probe] (probe: %s)"
                " sharing the HX717" % (self.probe_name,))
        self.sensor = self.probe.get_sensor()
        self._supports_interleave = hasattr(self.sensor, 'enable_interleave')
        # Suppress while a probe session pins channel A (no extrusion there
        # anyway, but the loadcell force is dominated by the probe touch).
        if hasattr(self.probe, 'register_session_hooks'):
            self.probe.register_session_hooks(self._probe_suspend,
                                              self._probe_resume)

    def _ready(self):
        # Defer subscription so the loadcell's stream is up first (matches the
        # filament-sensor module's 1 s activation delay).
        self.reactor.register_callback(self._activate,
                                       self.reactor.monotonic() + 1.0)

    def _activate(self, eventtime):
        if not self._subscribed:
            # Channel A = loadcell / e-stall feed (the demux routes chB elsewhere).
            self.sensor.add_client(self._on_batch, channel='A')
            self._subscribed = True
        if self.default_enabled:
            self._set_enabled(True)

    # ---- arming -------------------------------------------------------------
    def _set_enabled(self, enable):
        enable = bool(enable)
        if enable == self._enabled:
            return
        # I2 shadow-telemetry: dump the accumulated per-print distribution on the
        # disarm edge (PRINT_END's ESTALL_DETECT ENABLE=0), reset on the arm edge.
        if not enable and self._enabled:
            self._tele_report("print")
        self._enabled = enable
        # reset detection state on every arm/disarm edge
        self._detected = self._reported = False
        if enable:
            self._tele_reset()
            # a print start (ENABLE=1) is an unambiguous "no blocks outstanding" point. A leaked
            # BLOCK (a cancel mid-M600 left _block_depth >= 1) would otherwise silently disable
            # detection across ALL future prints. Clear it on the arm edge.
            if self._block_depth:
                logging.warning("estall_detect: cleared leaked _block_depth=%d on arm",
                                self._block_depth)
            self._block_depth = 0
        self._buf.clear()
        self._buf.extend([0.0] * len(FIR_TAPS))
        self._last_epos = self._last_t = None
        # Arming needs the channel-A loadcell feed live during the print -> turn
        # on the HX717 interleave; disarming hands the chip back to presence.
        if self._supports_interleave:
            try:
                if enable:
                    self.sensor.enable_interleave()
                else:
                    self.sensor.disable_interleave()
            except Exception:
                logging.exception("estall_detect: interleave toggle failed")

    # ---- suppression (BlockEStallDetection equivalent) ----------------------
    def _block(self, on):
        if on:
            self._block_depth += 1
        elif self._block_depth > 0:
            self._block_depth -= 1
        self._detected = False        # discard anything seen while blocked

    def _probe_suspend(self):
        # Probing pins channel A at full rate -> interleave must be off. Derive
        # the post-probe intent from _enabled (no saved-state needed), so this is
        # correct whether or not a filament sensor is also managing interleave.
        self._block(True)
        if self._supports_interleave:
            try:
                self.sensor.disable_interleave()
            except Exception:
                logging.exception("estall_detect: disable_interleave failed")

    def _probe_resume(self):
        self._block(False)
        if self._enabled and self._supports_interleave:
            try:
                self.sensor.enable_interleave()
            except Exception:
                logging.exception("estall_detect: enable_interleave failed")

    # ---- sample processing --------------------------------------------------
    def _gate_pushing(self, t):
        # True when the extruder is advancing filament forward at >= the min
        # speed at print_time t (derived from the commanded extruder position;
        # Klipper has no direct motor_direction(E)).
        if self.extruder is None:
            return False
        try:
            epos = self.extruder.find_past_position(t)
        except Exception:
            return False
        pushing = False
        if self._last_epos is not None and self._last_t is not None:
            dt = t - self._last_t
            if dt > 0.:
                pushing = ((epos - self._last_epos) / dt) >= self.min_extrude_speed
        self._last_epos = epos
        self._last_t = t
        return pushing

    def _on_batch(self, msg):
        data = msg.get('data')
        if not data:
            return True
        # Re-arm the latch once the cooldown elapses (Core One clears every 1 s).
        if self._reported:
            now = self.reactor.monotonic()
            if now - self._report_time >= self.report_interval:
                self._reported = False
        for row in data:
            t = row[0]
            raw = row[1]              # raw loadcell count (channel A)
            # FIR runs continuously so it has valid history when extrusion starts
            self._buf.append(float(raw))
            filtered = 0.
            for coeff, val in zip(FIR_TAPS, self._buf):
                filtered += coeff * val
            if self.invert:
                filtered = -filtered
            self._last_filtered = filtered
            over = filtered > self.threshold
            # GATE: only a forward push lets a detection stick; anything else
            # clears the transient (so retract/travel/idle can't false-fire).
            if not self._gate_pushing(t):
                self._detected = False
                continue
            # I2 shadow-telemetry: record the gated-pushing 'filtered' distribution
            # (skip suppressed/purge regions so stats reflect real print extrusion).
            if self._enabled and not self._block_depth:
                self._tele_add(filtered)
            if over:
                self._detected = True
                self._trip_filtered = filtered   # the value that crossed
            # Evaluate(): latch + fire once. Log the value that actually CROSSED
            # (not the current decayed sample), and clear _detected so a re-report
            # after the latch cool-down needs a genuinely NEW over-threshold edge
            # -- otherwise a multi-second push re-fires on the stale sticky flag
            # with a sub-threshold/negative filtered value.
            if (self._detected and self._enabled and not self._block_depth
                    and not self._reported):
                self._reported = True
                self._report_time = self.reactor.monotonic()
                self._fire(self._trip_filtered)
                self._detected = False
        return True

    def _fire(self, filtered):
        msg = ("extruder clog/stall detected (filtered=%.0f thr=%.0f)"
               % (filtered, self.threshold))
        logging.warning("estall_detect: %s", msg)
        # Defer all g-code off the sensor batch timer (re-entrancy-safe).
        self.reactor.register_callback(lambda e, m=msg: self._run_action(m))

    def _run_action(self, msg):
        try:
            self.gcode.respond_info("// estall: " + msg)
            self.gcode.run_script("M118 ESTALL: " + msg)
            extra = self.stall_template.render().strip()
            if extra:
                self.gcode.run_script(extra)
            if self.on_stall == 'pause':
                self.gcode.run_script("M117 Clog detected - PAUSED\nPAUSE")
            elif self.on_stall == 'm600':
                self.gcode.run_script("M600")
            # on_stall == 'log' -> alert only (Core One stock behaviour)
        except Exception:
            logging.exception("estall_detect: stall action failed")

    # ---- gcode --------------------------------------------------------------
    cmd_ESTALL_DETECT_help = ("Arm/disarm loadcell clog detection for a print"
                              " (ESTALL_DETECT ENABLE=1 | ENABLE=0)")

    def _cmd_ESTALL_DETECT(self, gcmd):
        enable = gcmd.get_int('ENABLE', 1, minval=0, maxval=1)
        self._set_enabled(enable)
        gcmd.respond_info("Clog detection %s" % ("ON" if enable else "OFF"))

    cmd_ESTALL_BLOCK_help = ("Suppress/resume clog detection (nestable) around"
                             " expected force spikes (ESTALL_BLOCK BLOCK=1|0)")

    def _cmd_ESTALL_BLOCK(self, gcmd):
        self._block(gcmd.get_int('BLOCK', 1, minval=0, maxval=1))

    def _cmd_M591(self, gcmd):
        # Prusa M591 [S|P|R]: S1/S0 enable/disable, P persist, R restore default.
        # P and R are bare flags (no value); S takes 1/0.
        params = gcmd.get_command_parameters()
        if 'R' in params:
            self._set_enabled(self.default_enabled)
        if 'S' in params:
            self._set_enabled(gcmd.get_int('S'))
        if 'P' in params:
            configfile = self.printer.lookup_object('configfile')
            configfile.set(self.config_name, 'default_enabled',
                           '%s' % (bool(self._enabled),))
            gcmd.respond_info("M591: run SAVE_CONFIG to persist (default_enabled"
                              "=%s)" % (bool(self._enabled),))
        gcmd.respond_info("Stuck-filament detection: %s"
                          % ("ON" if self._enabled else "OFF"))

    # ---- I2 shadow-telemetry: per-print 'filtered' distribution -------------
    # estall is already on_stall:log (alert-only) after the 2026-06-28 revert, so
    # the shadow mode is effectively live -- what was missing is the DATA. Over a
    # normal print, characterise how close the gated-pushing edge gets to the
    # threshold (max/thr headroom) and how far a real clog sits above it, so
    # detection_threshold can be set to separate them and on_stall re-promoted
    # log->pause. Streaming + memory-bounded (a fixed histogram); O(1) per sample;
    # NO effect on the trip path (accumulate-only).
    def _tele_reset(self):
        self._tele_n = 0
        self._tele_sum = 0.
        self._tele_min = self._tele_max = None
        self._tele_over = 0                     # samples > threshold while pushing
        self._tele_lo = -0.5 * self.threshold   # histogram spans normal..clog
        self._tele_hi = 2.5 * self.threshold
        self._tele_bw = (self._tele_hi - self._tele_lo) / TELE_NBINS
        self._tele_hist = [0] * TELE_NBINS

    def _tele_add(self, filtered):
        self._tele_n += 1
        self._tele_sum += filtered
        if self._tele_min is None or filtered < self._tele_min:
            self._tele_min = filtered
        if self._tele_max is None or filtered > self._tele_max:
            self._tele_max = filtered
        if filtered > self.threshold:
            self._tele_over += 1
        b = int((filtered - self._tele_lo) / self._tele_bw)
        if b < 0:
            b = 0
        elif b >= TELE_NBINS:
            b = TELE_NBINS - 1
        self._tele_hist[b] += 1

    def _tele_pct(self, p):
        # Approximate percentile from the streaming histogram (bin-center estimate).
        target = p * self._tele_n
        cum = 0
        for i, c in enumerate(self._tele_hist):
            cum += c
            if cum >= target:
                return self._tele_lo + (i + 0.5) * self._tele_bw
        return self._tele_hi

    def _tele_report(self, label):
        n = self._tele_n
        if not n:
            return
        mean = self._tele_sum / n
        headroom = (self._tele_max / self.threshold) if self.threshold else 0.
        msg = ("estall telemetry [%s]: n=%d gated-push samples | filtered "
               "min=%.0f mean=%.0f p50=%.0f p90=%.0f p99=%.0f p99.9=%.0f "
               "max=%.0f | thr=%.0f would-trip=%d (%.3f%%) | max/thr=%.2f"
               % (label, n, self._tele_min, mean, self._tele_pct(0.50),
                  self._tele_pct(0.90), self._tele_pct(0.99),
                  self._tele_pct(0.999), self._tele_max, self.threshold,
                  self._tele_over, 100. * self._tele_over / n, headroom))
        logging.warning("estall_detect: %s", msg)
        try:
            self.gcode.respond_info("// " + msg)
        except Exception:
            pass

    cmd_ESTALL_STATS_help = ("Report the per-print clog-telemetry distribution"
                             " (RESET=1 to also zero the accumulator)")

    def _cmd_ESTALL_STATS(self, gcmd):
        self._tele_report("on-demand")
        if gcmd.get_int('RESET', 0, minval=0, maxval=1):
            self._tele_reset()
            gcmd.respond_info("estall telemetry reset")

    def get_status(self, eventtime):
        return {
            'enabled': self._enabled,
            'blocked': self._block_depth > 0,
            'reported': self._reported,
            'filtered': self._last_filtered,
            'threshold': self.threshold,
            # I2 shadow-telemetry (per-print, since last arm)
            'tele_n': self._tele_n,
            'tele_max': self._tele_max if self._tele_max is not None else 0.,
            'tele_would_trip': self._tele_over,
        }


def load_config(config):
    return EStallDetect(config)
