# Filament sensor on the 2nd channel of the loadcell's HX717 (Prusa Core One).
#
# The extruder filament sensor is wired to HX717 channel B (gain 8), time-shared
# with the loadcell Z-probe on channel A (gain 128) on the same chip. This module
# OWNS channel B while idle/printing and YIELDS it to the probe during a probe
# session (mutually exclusive in time — interleaving would feed gain-8 values into
# the probe's per-sample MCU trigger and break it). The probe re-tares + rebuilds
# its filter at the start of every probing move, so any channel-B perturbation
# between sessions is erased. See the klipper-filament-sensor plan.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

GAIN_CH_LOADCELL = 1     # HX717 channel A, gain 128 (probe)
GAIN_CH_FILAMENT = 4     # HX717 channel B, gain 8 (filament sensor)
FS_RAW_MIN = 2000        # below this -> sensor not connected
FS_RAW_MAX = 2000000     # above this -> not connected / not calibrated
FS_MIN_SEPARATION = 100  # |ins-nins| must exceed this to be a usable signal
FS_CAL_DURATION = 1.0    # seconds of channel-B samples to average per capture


class HX71xFilamentSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config_name = config.get_name()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        # Calibration reference raw values (set by HX71X_FS_CALIBRATE / SAVE_CONFIG)
        self.fs_ref_ins = config.getint('fs_ref_ins', None)
        self.fs_ref_nins = config.getint('fs_ref_nins', None)
        # Shared HX717 sensor (resolved from the loadcell probe at connect)
        self.sensor = None
        self.probe = None
        # State
        self._active = False          # True when the stream is on our channel (B)
        self._discard = 0             # samples to drop after a channel switch
        self._median_buf = []
        self._last_median = None
        # Calibration capture state (FSENSOR_CALIBRATE)
        self._cal_buf = []
        self._cal_collecting = False
        self._cal_nins = None
        self._cal_ins = None
        # Reuse the standard runout/insert + QUERY/SET machinery
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.get_status = self.runout_helper.get_status
        # Wire up
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_cmd_error)
        gcode = self.printer.lookup_object('gcode')
        # NB: command names must NOT contain digits — Klipper's gcode parser would
        # split "HX71X_..." into the traditional command "HX71".
        gcode.register_mux_command("FSENSOR_READ", "SENSOR", self.name,
            self.cmd_FSENSOR_READ,
            desc="Report the HX717-channel-B filament sensor raw value + presence")
        gcode.register_mux_command("FSENSOR_CALIBRATE", "SENSOR", self.name,
            self.cmd_FSENSOR_CALIBRATE,
            desc="Calibrate the filament sensor: STATE=NINS|INS|SAVE")

    def _handle_connect(self):
        self.probe = self.printer.lookup_object('probe', None)
        if self.probe is None or not hasattr(self.probe, 'register_session_hooks'):
            raise self.printer.config_error(
                "[%s] requires a [load_cell_probe] sharing the same HX717"
                % (self.config_name,))
        self.sensor = self.probe.get_sensor()
        self.probe.register_session_hooks(self._suspend, self._resume)

    def _handle_ready(self):
        # Defer FS activation so the loadcell's continuous stream is up first.
        self.reactor.register_callback(self._activate,
                                       self.reactor.monotonic() + 1.0)

    def _activate(self, eventtime):
        # channel='B' is REQUIRED: add_client() defaults to channel='A' (the
        # LOADCELL), so omitting it subscribed this filament sensor to the probe's
        # channel and fed it loadcell counts. Symptom (diagnosed 2026-07-19): with
        # filament visibly loaded, FSENSOR_READ reported raw=-254179 present=False
        # -- a value that tracked probe.tare_counts (-254143) to within noise, sat
        # far below FS_RAW_MIN, and matched neither calibrated reference. While
        # interleaving (12:1 chA:chB) the ch-A rows also dominate the 3-sample
        # median, so the sensor read loadcell data essentially always.
        self.sensor.add_client(self._on_batch, channel='B')
        self._resume()

    # --- channel mode-switch (driven by the probe's session hooks) ---
    def _suspend(self):
        # Probe session starting: hand channel A to the loadcell, stop evaluating.
        # Block until the switch actually lands so homing never tares on ch B.
        self._active = False
        if self.sensor is not None:
            self.sensor.set_gain_channel(GAIN_CH_LOADCELL)
            self.sensor.wait_for_channel_switch()

    def _resume(self):
        # Idle/printing: take channel B back for the filament sensor.
        if self.sensor is None:
            return
        self.sensor.set_gain_channel(GAIN_CH_FILAMENT)
        self._discard = 2             # first post-switch sample(s) are stale
        self._median_buf = []
        self._active = True

    def _handle_cmd_error(self):
        # A probe that aborts mid-session still restores FS mode (idempotent).
        # (invariant): _resume() flips the HX717 back to channel B. This is only
        # reached when NOT self._active (i.e. the probe path had yielded ch A and did
        # not restore it), and the gcode mutex serializes probe vs. this error handler,
        # so there is no window where a probe read and this resume touch the channel
        # concurrently. If ch A is ever driven outside the gcode mutex, revisit.
        if self.sensor is not None and not self._active:
            self._resume()

    # --- sample processing ---
    def _on_batch(self, msg):
        if not self._active:
            return True
        data = msg.get('data')
        if not data:
            return True
        median = None
        for row in data:
            raw = row[1]              # HX71xBase batch row = (time, raw_counts, adc)
            if self._discard > 0:
                self._discard -= 1
                continue
            if self._cal_collecting:
                self._cal_buf.append(raw)
            self._median_buf.append(raw)
            if len(self._median_buf) > 3:
                self._median_buf.pop(0)
            median = sorted(self._median_buf)[len(self._median_buf) // 2]
        if median is not None:
            self._last_median = median
            # Never let a presence-eval bug shut down the printer -- this runs on
            # the probe's shared batch stream.
            try:
                self._eval_presence(median)
            except Exception:
                logging.exception("hx71x_filament_sensor: presence eval failed")
        return True

    def _in_range(self, v):
        return v is not None and FS_RAW_MIN <= v <= FS_RAW_MAX

    def is_calibrated(self):
        return (self._in_range(self.fs_ref_ins)
                and self._in_range(self.fs_ref_nins)
                and abs(self.fs_ref_ins - self.fs_ref_nins) > 6)

    def _eval_presence(self, median):
        if not self._in_range(median) or not self.is_calibrated():
            return
        mid = (self.fs_ref_ins + self.fs_ref_nins) / 2.
        hyst = abs(self.fs_ref_ins - self.fs_ref_nins) / 6.
        prev = self.runout_helper.filament_present
        if abs(median - mid) < hyst:
            present = prev            # within the hysteresis band: hold state
        else:
            present = (median > mid) == (self.fs_ref_ins > mid)
        self.runout_helper.note_filament_present(self.reactor.monotonic(), present)

    # --- calibration ---
    def _collect_avg(self, gcmd, duration=FS_CAL_DURATION):
        # Average channel-B raw counts over `duration` seconds. The FS reader
        # keeps running on the reactor while we pause, filling self._cal_buf.
        if not self._active:
            raise gcmd.error("filament sensor not active (probe session in "
                             "progress?) -- retry when idle")
        self._cal_buf = []
        self._cal_collecting = True
        try:
            eventtime = self.reactor.monotonic()
            end = eventtime + duration
            while eventtime < end:
                eventtime = self.reactor.pause(eventtime + 0.1)
        finally:
            self._cal_collecting = False
        buf = sorted(self._cal_buf)
        self._cal_buf = []
        if len(buf) < 3:
            raise gcmd.error("filament sensor: only %d samples collected -- is "
                             "the stream running?" % (len(buf),))
        # trimmed mean: drop the single min+max to reject outliers
        trimmed = buf[1:-1] if len(buf) > 4 else buf
        return int(round(float(sum(trimmed)) / len(trimmed)))

    def cmd_FSENSOR_CALIBRATE(self, gcmd):
        state = gcmd.get('STATE').upper()
        if state in ('NINS', 'INS'):
            avg = self._collect_avg(gcmd)
            if not self._in_range(avg):
                raise gcmd.error(
                    "filament sensor: raw=%d outside valid range %d..%d -- "
                    "check wiring/channel" % (avg, FS_RAW_MIN, FS_RAW_MAX))
            if state == 'NINS':
                self._cal_nins = avg
                gcmd.respond_info(
                    "filament sensor: captured NO-filament ref = %d.\n"
                    "Now LOAD filament and run: "
                    "FSENSOR_CALIBRATE SENSOR=%s STATE=INS" % (avg, self.name))
            else:
                self._cal_ins = avg
                gcmd.respond_info(
                    "filament sensor: captured WITH-filament ref = %d.\n"
                    "Run: FSENSOR_CALIBRATE SENSOR=%s STATE=SAVE" % (avg, self.name))
        elif state == 'SAVE':
            nins = self._cal_nins if self._cal_nins is not None else self.fs_ref_nins
            ins = self._cal_ins if self._cal_ins is not None else self.fs_ref_ins
            if nins is None or ins is None:
                raise gcmd.error("filament sensor: capture both STATE=NINS and "
                                 "STATE=INS before STATE=SAVE")
            sep = abs(ins - nins)
            if sep < FS_MIN_SEPARATION:
                raise gcmd.error(
                    "filament sensor: ins=%d nins=%d differ by only %d counts "
                    "(< %d) -- sensor not responding to filament; recapture"
                    % (ins, nins, sep, FS_MIN_SEPARATION))
            configfile = self.printer.lookup_object('configfile')
            configfile.set(self.config_name, 'fs_ref_nins', '%d' % (nins,))
            configfile.set(self.config_name, 'fs_ref_ins', '%d' % (ins,))
            self.fs_ref_nins = nins
            self.fs_ref_ins = ins
            mid = (ins + nins) / 2.
            hyst = sep / 6.
            gcmd.respond_info(
                "filament sensor calibrated: nins=%d ins=%d "
                "(mid=%.0f hyst=%.0f, separation=%d).\n"
                "Run SAVE_CONFIG to persist (klippy restarts)."
                % (nins, ins, mid, hyst, sep))
        else:
            raise gcmd.error("STATE must be NINS, INS, or SAVE")

    def cmd_FSENSOR_READ(self, gcmd):
        m = self._last_median
        if m is None:
            gcmd.respond_info("filament sensor (HX717 ch B): no sample yet "
                              "(active=%s)" % (self._active,))
            return
        if self.is_calibrated():
            mid = (self.fs_ref_ins + self.fs_ref_nins) / 2.
            hyst = abs(self.fs_ref_ins - self.fs_ref_nins) / 6.
            gcmd.respond_info(
                "filament sensor (HX717 ch B): raw=%d present=%s "
                "(ins=%d nins=%d mid=%.0f hyst=%.0f)"
                % (m, self.runout_helper.filament_present, self.fs_ref_ins,
                   self.fs_ref_nins, mid, hyst))
        else:
            gcmd.respond_info(
                "filament sensor (HX717 ch B): raw=%d -- NOT calibrated "
                "(run FSENSOR_CALIBRATE with/without filament)" % (m,))


def load_config_prefix(config):
    return HX71xFilamentSensor(config)
