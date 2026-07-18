# Soft-abort: a graceful, non-destructive cancel that interrupts in-flight
# BLOCKING waits without an emergency stop -- so the printer parks to idle and
# stays connected (no FIRMWARE_RESTART needed).
#
# The problem it solves: M190/M109 (heater waits), TEMPERATURE_WAIT (chamber
# soak) and the probe loops (BED_MESH_CALIBRATE, G28 Z) all run inside a single
# gcode command that holds the gcode mutex for its whole duration. A normal
# CANCEL_PRINT is itself queued gcode, so it can't run until the blocking wait
# finishes -- which is why mid-heat / mid-soak cancel "hangs" today.
#
# The fix has two halves:
#   1) An OUT-OF-BAND trigger. A klippy webhook endpoint ("soft_cancel/abort")
#      is dispatched from the reactor and NOT under the gcode mutex (same path
#      as emergency_stop, see webhooks.py). So it can flip a flag + drop heater
#      targets while a blocking wait still holds the mutex.
#   2) The blocking waits are made flag-aware. We monkey-patch the heater wait
#      (_wait_for_temperature -- used by M190 AND M109) and cmd_TEMPERATURE_WAIT
#      (chamber soak), and we listen on probe:update_results (mesh / G28 Z), so
#      each raises a command_error the instant the flag is seen. That unwinds the
#      running macro; we then queue CANCEL_PRINT for the normal cleanup.
#
# Config (empty section enables it):
#   [soft_cancel]
#   #poll: 0.25     # seconds between flag checks during a wait

import logging


class SoftCancel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.aborting = False
        self.poll = config.getfloat('poll', 0.25, above=0.)
        self.pheaters = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("probe:update_results",
                                            self._on_probe)
        # Out-of-band trigger: runs in the reactor, NOT under the gcode mutex.
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint("soft_cancel/abort", self._web_abort)
        # Mutex-bound helpers (only useful when idle / for clearing the flag).
        self.gcode.register_command(
            "SOFT_ABORT", self.cmd_SOFT_ABORT,
            desc="Soft-cancel in-flight waits, then CANCEL_PRINT (out-of-band "
                 "callers should use the soft_cancel/abort webhook)")
        self.gcode.register_command(
            "SOFT_ABORT_RESET", self.cmd_SOFT_ABORT_RESET,
            desc="Clear the soft-abort flag")
        self.gcode.register_command(
            "SOAK_WAIT", self._cmd_SOAK_WAIT,
            desc="Timed heat-soak dwell, interruptible by SOFT_ABORT (unlike a bare G4)")

    def _connect(self):
        self.pheaters = self.printer.lookup_object('heaters')
        # Make the heater wait (covers M190 + M109 + SET_HEATER_TEMPERATURE wait)
        # honour the flag. Instance-attribute shadowing: set_temperature() calls
        # self._wait_for_temperature(heater), so our version takes over.
        self.pheaters._wait_for_temperature = self._wait_for_temperature
        # Re-point the registered TEMPERATURE_WAIT command at our flag-aware copy.
        self.gcode.register_command("TEMPERATURE_WAIT", None)
        self.gcode.register_command(
            "TEMPERATURE_WAIT", self._cmd_TEMPERATURE_WAIT,
            desc="Wait for temperature on a sensor (soft-abortable)")

    # --- flag-aware reimplementations of the blocking waits -----------------

    def _wait_for_temperature(self, heater):
        # Mirror of heaters.PrinterHeaters._wait_for_temperature with a flag check.
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        eventtime = self.reactor.monotonic()
        i = 0
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            if self.aborting:
                raise self.printer.command_error("Soft abort during heat wait")
            toolhead.get_last_move_time()
            if i % 4 == 0:
                self.gcode.respond_raw(self.pheaters._get_temp(eventtime))
            i += 1
            eventtime = self.reactor.pause(eventtime + self.poll)

    def _cmd_TEMPERATURE_WAIT(self, gcmd):
        # Mirror of heaters.PrinterHeaters.cmd_TEMPERATURE_WAIT with a flag check.
        sensor_name = gcmd.get('SENSOR')
        min_temp = gcmd.get_float('MINIMUM', float('-inf'))
        max_temp = gcmd.get_float('MAXIMUM', float('inf'), above=min_temp)
        # TIMEOUT (s): 0 = wait forever (native/default, backward compatible). >0 = warn + CONTINUE
        # if the sensor doesn't reach the band in time, so a passive chamber warm-up can't hang the
        # print indefinitely (cellar-cold day, or CHAMBER_MIN set above the passive ceiling). The
        # print proceeds -- better a possibly-cool chamber than a silent 15+ min idle needing a
        # manual SOFT_ABORT (slicer chat hit exactly this twice calibrating ASA, 2026-07-07). Still
        # soft-abortable (the flag check below fires every poll regardless of the deadline).
        timeout = gcmd.get_float('TIMEOUT', 0., minval=0.)
        if min_temp == float('-inf') and max_temp == float('inf'):
            raise gcmd.error(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.")
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        if sensor_name in self.pheaters.heaters:
            sensor = self.pheaters.heaters[sensor_name]
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        eventtime = self.reactor.monotonic()
        deadline = (eventtime + timeout) if timeout > 0. else None
        i = 0
        while not self.printer.is_shutdown():
            if self.aborting:
                raise gcmd.error("Soft abort during temperature wait")
            temp, target = sensor.get_temp(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            if deadline is not None and eventtime >= deadline:
                goal = min_temp if min_temp != float('-inf') else max_temp
                gcmd.respond_info(
                    "TEMPERATURE_WAIT: %s only reached %.1f (target %.0f) in %.0fs -- "
                    "continuing anyway" % (sensor_name, temp, goal, timeout))
                return
            toolhead.get_last_move_time()
            if i % 4 == 0:
                gcmd.respond_raw(self.pheaters._get_temp(eventtime))
            i += 1
            eventtime = self.reactor.pause(eventtime + self.poll)

    def _cmd_SOAK_WAIT(self, gcmd):
        # Abortable timed dwell (heat-soak). A bare `G4` dwell holds the gcode mutex
        # for its whole duration and is BLIND to the soft-abort flag, so SOFT_ABORT
        # can't break a soak -- it only bailed at the NEXT flag-aware wait after the
        # soak finished. This mirrors the temp-wait poll loop so a soak aborts instantly.
        minutes = gcmd.get_float('MINUTES', 0., minval=0.)
        seconds = gcmd.get_float('SECONDS', minutes * 60., minval=0.)
        if seconds <= 0.:
            return
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        eventtime = self.reactor.monotonic()
        deadline = eventtime + seconds
        i = 0
        while not self.printer.is_shutdown():
            if self.aborting:
                raise gcmd.error("Soft abort during heat soak")
            if eventtime >= deadline:
                return
            toolhead.get_last_move_time()
            if i % 40 == 0:                 # periodic temp report (~10s at poll 0.25)
                gcmd.respond_raw(self.pheaters._get_temp(eventtime))
            i += 1
            eventtime = self.reactor.pause(eventtime + self.poll)

    def _on_probe(self, results):
        # Fired once per probed point (BED_MESH_CALIBRATE, G28 Z, PROBE_ACCURACY).
        # Raising here unwinds a running mesh / probe sequence on abort.
        # (invariant): this raises a command_error from the probe:update_results
        # callback. That is SAFE because probe:update_results only fires synchronously
        # inside a probe command (which holds the gcode mutex), so the exception unwinds
        # that command's own call stack -- it never fires from the reactor's idle path.
        # If probing is ever driven from a non-gcode context, this must become a flag.
        if self.aborting:
            raise self.printer.command_error("Soft abort during probe")

    # --- trigger + cleanup ---------------------------------------------------

    def _web_abort(self, web_request):
        self._do_abort("webhook")
        web_request.send({"status": "aborting"})

    def cmd_SOFT_ABORT(self, gcmd):
        self._do_abort("gcode")
        gcmd.respond_info("Soft abort requested")

    def cmd_SOFT_ABORT_RESET(self, gcmd):
        self.aborting = False
        gcmd.respond_info("Soft-abort flag cleared")

    def _do_abort(self, source):
        if self.aborting:
            return
        self.aborting = True
        logging.info("soft_cancel: abort requested via %s", source)
        # Immediately stop heating. This also lets a *stock* heater wait exit via
        # check_busy(), as defence in depth if the patch ever isn't in place.
        try:
            for heater in self.pheaters.heaters.values():
                heater.set_temp(0.)
        except Exception:
            logging.exception("soft_cancel: error dropping heater targets")
        # Queue the normal cleanup. This reactor callback grabs the gcode mutex
        # cooperatively -- it waits for the now-raising blocking wait to release
        # it, then runs CANCEL_PRINT just like a normal cancel would.
        self.reactor.register_callback(self._finish_abort)

    def _finish_abort(self, eventtime):
        # If the aborted print's sdcard work-handler hasn't finished unwinding
        # yet, its cmd_from_sd flag is still set and CANCEL_PRINT's
        # SDCARD_RESET_FILE trips "cannot be run from the sdcard" -- the rest of
        # the cleanup (CANCEL_PRINT_BASE) is then skipped, leaving print_stats
        # stuck at 'error'. Do NOT force cmd_from_sd=False to dodge this: that
        # deadlocks do_pause()'s wait loop (it spins while work_timer is set and
        # cmd_from_sd is False, holding the gcode mutex -> full hang). Instead
        # wait for the work-handler to stop on its own (it clears work_timer and
        # cmd_from_sd when the aborted command unwinds), then run the cleanup.
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        retries = getattr(self, '_abort_retries', 0)
        if vsd is not None and vsd.is_active() and retries < 100:
            self._abort_retries = retries + 1
            self.reactor.register_callback(
                self._finish_abort, self.reactor.monotonic() + 0.05)
            return
        self._abort_retries = 0
        try:
            ps = self.printer.lookup_object('print_stats', None)
            state = None
            if ps is not None:
                state = ps.get_status(eventtime).get('state')
            # 'error' is included on purpose: when the abort interrupts a PRINT,
            # the sdcard work-handler catches our raised command_error and sets
            # print_stats to 'error' (note_error) as it unwinds -- so by the time
            # we get here (after the defer above), the state is 'error', not
            # 'printing'. Without it we'd skip CANCEL_PRINT and leave the print
            # half-cancelled (error state + file still loaded). cmd_from_sd is
            # clear by now, so CANCEL_PRINT's SDCARD_RESET_FILE runs cleanly.
            if state in ('printing', 'paused', 'error'):
                self.gcode.run_script("CANCEL_PRINT")
            else:
                self.gcode.run_script("TURN_OFF_HEATERS")
        except Exception:
            logging.exception("soft_cancel: cleanup error")
        finally:
            self.aborting = False


def load_config(config):
    return SoftCancel(config)
