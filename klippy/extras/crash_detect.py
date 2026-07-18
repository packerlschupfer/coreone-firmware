# StallGuard crash detection (host side).  Pairs with src/crash_detect.c.
#
# Prusa Core One firmware ships "Crash Detection" (default-on): the TMC2130 X/Y
# drivers flag a motor stall via StallGuard during moves and the printer halts so
# the nozzle stops grinding into the bed/part.  Our Klipper port had NO such guard
# on a plain G1 move (the loadcell only protects homing/probing).  This module is
# the equivalent: while ARMED (for the duration of a print) the MCU reads sg_result
# over SPI every sample_period and, when it stays at/near 0 (Prusa's DIAG1 comparator
# fires at SG_RESULT==0), calls shutdown() -- an immediate halt of all steppers + heaters.
#
# DETECTION + STOP only: Prusa's Crash_s also does re-home + g-code replay recovery,
# which Klipper has no infrastructure for.  After a crash: FIRMWARE_RESTART + re-print.
#
# Trigger = an ABSOLUTE sg_result <= sg_floor (==0 with SGT biasing where 0 lands),
# matching Prusa's CRASH_STALL_GUARD (SGT +2) + CRASH_FILTER (sfilt off), NOT the
# earlier % -drop-from-baseline (which false-tripped: sg_result is low just above the
# gate and climbs with speed, so a % drop reads fast moves as stalls).  A VELOCITY GATE
# (TSTEP) still skips samples below the coolStep velocity, where sg_result is invalid-
# low, so accel/decel/slow moves can't false-trip.  The gate value is taken from the
# driver's own configured TCOOLTHRS (coolstep_threshold) -- no clock math.
#
#   [crash_detect stepper_x]
#     #tmc: tmc2130 stepper_x   # default; the driver to read sg_result from
#     sg_floor: 2               # stall when sg_result stays <= this (Prusa SG==0) ...
#     sample_count: 4           # ... for this many consecutive samples ...
#     sample_period: 0.0003     # ... spaced this far apart (s)
#     crash_sgt: 2              # TMC SGT applied on arm (Prusa +2; higher = less sensitive)
#
#   CRASH_DETECT ENABLE=1   ; arm  (PRINT_START, after homing+mesh+prime)
#   CRASH_DETECT ENABLE=0   ; disarm (PRINT_END / CANCEL / abort paths)
#
# NOTE: place [crash_detect stepper_x] AFTER [tmc2130 stepper_x] in the config so the
# driver object exists when this binds to it.
import collections


class CrashDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]          # e.g. "stepper_x"
        self.tmc_name = config.get('tmc', 'tmc2130 ' + self.name)
        # Prusa SG==0 trigger: stall when sg_result stays AT/below this absolute floor.
        # With SGT biasing where 0 lands, near-0 = a real stall at any speed (velocity-
        # robust), unlike a % drop from a baseline. 0 = exact Prusa DIAG1 (SG_RESULT==0);
        # a small floor adds margin for the SPI-polled read.
        self.sg_floor = config.getint('sg_floor', 2, minval=0, maxval=1023)
        # consecutive at-floor samples required -- debounces a stray single-sample 0
        # (DIAG/line noise). Prusa uses a single hardware DIAG1 edge; a few SPI samples
        # is the polled equivalent. sample_count*sample_period = the sustained window.
        self.sample_count = config.getint('sample_count', 4,
                                          minval=1, maxval=255)
        self.sample_period = config.getfloat('sample_period', 0.0003, above=0.)
        # CRASH_STALL_GUARD: the TMC2130 SGT applied on arm (separate from homing SGT).
        # Higher = LESS sensitive. Prusa = +2 on X/Y. Restored on disarm.
        self.crash_sgt = config.getint('crash_sgt', 2, minval=-64, maxval=63)
        self.mcu = self.spi = self.mcu_tmc = self.fields = self.tmc = None
        self._cmd_queue = self._arm_cmd = self._query_cmd = None
        self._armed = False
        self._prev = collections.OrderedDict()
        self._dirty = collections.OrderedDict()
        self._bind_tmc()
        self._oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self._build_config)
        # One shared CRASH_DETECT command drives every [crash_detect ...] instance;
        # the first-created instance registers it, the rest piggy-back.
        gcode = self.printer.lookup_object('gcode')
        try:
            gcode.register_command('CRASH_DETECT', self._cmd_CRASH_DETECT,
                                   desc=self.cmd_CRASH_DETECT_help)
        except self.printer.config_error:
            pass        # already registered by a sibling instance

    def _bind_tmc(self):
        if self.mcu is not None:
            return
        tmc = self.printer.lookup_object(self.tmc_name)
        self.tmc = tmc
        self.mcu_tmc = tmc.mcu_tmc
        self.fields = self.mcu_tmc.get_fields()
        self.spi = self.mcu_tmc.tmc_spi.spi
        self.mcu = self.spi.get_mcu()

    def _build_config(self):
        spi_oid = self.spi.get_oid()
        self.mcu.add_config_cmd("config_crash_detect oid=%d spi_oid=%d"
                                % (self._oid, spi_oid))
        # disarm on (re)start so a FIRMWARE_RESTART after a crash comes up unarmed
        self.mcu.add_config_cmd(
            "crash_detect_arm oid=%d clock=0 rest_ticks=0 sample_count=0"
            " sg_floor=0 gate_tstep=0" % (self._oid,),
            on_restart=True)
        self._cmd_queue = self.mcu.alloc_command_queue()
        self._arm_cmd = self.mcu.lookup_command(
            "crash_detect_arm oid=%c clock=%u rest_ticks=%u sample_count=%c"
            " sg_floor=%hu gate_tstep=%u",
            cq=self._cmd_queue)
        self._query_cmd = self.mcu.lookup_query_command(
            "crash_detect_query_state oid=%c",
            "crash_detect_state oid=%c armed=%c",
            oid=self._oid, cq=self._cmd_queue)

    # --- TMC register save/restore (mirrors sg_endstop / TMCVirtualPinHelper) ---
    def _set_field(self, name, value):
        if name not in self._prev:
            self._prev[name] = self.fields.get_field(name)
        reg = self.fields.lookup_register(name)
        self._dirty[reg] = self.fields.set_field(name, value)

    def _send(self):
        for reg, val in self._dirty.items():
            self.mcu_tmc.set_register(reg, val)
        self._dirty.clear()

    def _set_tmc_check(self, enable):
        # Suspend Klipper's periodic TMC error-check (the ~1 Hz GSTAT poll) on THIS
        # driver while crash detection is armed. The MCU's own sg_result reads use the
        # TMC2130's PIPELINED SPI read latch (a read returns the register addressed in
        # the PREVIOUS transfer); a concurrent host GSTAT poll then reads back the wrong
        # register and Klipper sees a bogus "reset=1" -> driver-error shutdown. (Same
        # shared-spi3a collision the phase-stepping executor hit.) Also no-op
        # get/set_register so a stray host access (e.g. idle-timeout disable) can't
        # collide. All restored on disarm. enable=True restores, False suspends.
        try:
            cmdhelper = self.tmc.get_status.__self__
            ec = getattr(cmdhelper, 'echeck_helper', cmdhelper)
            mt = self.mcu_tmc
            if enable:
                if hasattr(ec, '_cd_saved_start'):
                    ec.start_checks = ec._cd_saved_start
                    del ec._cd_saved_start
                if hasattr(mt, '_cd_saved_get'):
                    mt.get_register = mt._cd_saved_get
                    mt.set_register = mt._cd_saved_set
                    del mt._cd_saved_get, mt._cd_saved_set
                ec.start_checks()
            else:
                ec.stop_checks()
                # idempotence: if suspend ever runs twice, don't re-save the no-op shadow
                # (that would lose the REAL get/set_register until klippy restart). self._armed
                # guards double-arm today; this is defense in depth.
                if not hasattr(ec, '_cd_saved_start'):
                    ec._cd_saved_start = ec.start_checks
                    ec.start_checks = lambda *a, **k: False
                if not hasattr(mt, '_cd_saved_get'):
                    mt._cd_saved_get = mt.get_register
                    mt._cd_saved_set = mt.set_register
                    # benign read value: DRV_STATUS cs_actual=0x1F, GSTAT reset/drv_err/uv=0
                    mt.get_register = lambda *a, **k: 0x001f0000
                    mt.set_register = lambda *a, **k: None
        except Exception:
            pass

    def arm(self):
        if self._armed:
            return
        # Match Prusa's crash StallGuard config with REAL TMC writes BEFORE suspending the
        # checks (which no-op set_register); all restored from self._prev on disarm:
        #  - en_pwm_mode=0  : spreadCycle (stealthChop has no StallGuard)
        #  - sgt=crash_sgt  : CRASH_STALL_GUARD (+2), less sensitive than homing
        #  - sfilt=0        : CRASH_FILTER off (per-fullstep, no 4-sample averaging)
        self._set_field("en_pwm_mode", 0)
        self._set_field("sgt", self.crash_sgt)
        self._set_field("sfilt", 0)
        self._send()
        # gate = the driver's own coolStep/StallGuard TSTEP window (coolstep_threshold),
        # read from the cached field (no SPI): the MCU skips a sample when TSTEP > gate.
        gate = self.fields.get_field("tcoolthrs")
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        clock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.seconds_to_clock(self.sample_period)
        # suspend host TMC polling on this driver BEFORE the MCU starts reading sg_result
        self._set_tmc_check(False)
        self._arm_cmd.send(
            [self._oid, clock, rest_ticks, self.sample_count, self.sg_floor, gate],
            reqclock=clock)
        self._armed = True

    def disarm(self):
        if not self._armed:
            return
        # stop the MCU's SPI reads first, then confirm (a query round-trip) the disarm
        # was processed before resuming host polling -- so the two never overlap on spi3a.
        self._arm_cmd.send([self._oid, 0, 0, 0, 0, 0])
        if not self.mcu.is_fileoutput():
            self._query_cmd.send([self._oid])
        self._set_tmc_check(True)           # restore host TMC polling (get/set real again)
        for field, val in list(self._prev.items()):   # restore en_pwm_mode (real write now)
            self._set_field(field, val)
        self._send()
        self._prev.clear()
        self._armed = False

    cmd_CRASH_DETECT_help = ("Arm/disarm StallGuard crash detection on X/Y for the"
                             " duration of a print (CRASH_DETECT ENABLE=1 | ENABLE=0)")

    def _cmd_CRASH_DETECT(self, gcmd):
        # Klipper extended gcode needs KEY=VALUE -- a bare "ON"/"OFF" token is a
        # malformed command, so arm/disarm via ENABLE=1/0 (default ENABLE=1).
        enable = gcmd.get_int('ENABLE', 1, minval=0, maxval=1)
        # (shared-SPI3 hazard): crash_detect's task-level TMC sg_result reads run on the SAME
        # SPI3 the phase-stepping executor owns. While phase-stepping is engaged (direct_mode) a
        # blocking read can collide mid-frame with the prio-0 TIM8/DMA XDIRECT write -> a corrupted
        # coil vector; and it is INERT anyway (no step pulses -> TSTEP pegs -> the velocity gate
        # never passes). So refuse to ARM while any phase_exec is engaged: lose nothing, remove the
        # hazard. (Disarm is always allowed; disarm() is a no-op if not armed.) PRINT_START engages
        # phase-stepping BEFORE arming crash detect, so this guard sees it.
        if enable and any(getattr(o, '_trapq_engaged', False)
                          for _n, o in self.printer.lookup_objects('phase_exec')):
            gcmd.respond_info("Crash detection NOT armed: phase-stepping engaged "
                              "(shares SPI3; StallGuard gate inert in direct_mode)")
            return
        for _name, obj in self.printer.lookup_objects('crash_detect'):
            if enable:
                obj.arm()
            else:
                obj.disarm()
        gcmd.respond_info("Crash detection %s" % ("ON" if enable else "OFF"))


def load_config_prefix(config):
    return CrashDetect(config)
