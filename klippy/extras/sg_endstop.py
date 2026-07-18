# StallGuard-filtered sensorless endstop (host side).  Pairs with
# src/sg_endstop.c.
#
# Klipper's native sensorless homing uses the TMC diag1 pin, which trips on a
# single StallGuard dip -- so a noisy axis (e.g. a per-belt-tooth load ripple
# that punches sg_result to 0) false-trips on a long homing sweep. This driver
# reads sg_result over SPI during the move and triggers only after N consecutive
# sub-threshold samples (a SUSTAINED stall), filtering the brief dips.
#
# It registers a `virtual_endstop` pin so it drops straight into the rail:
#   [sg_endstop stepper_y]
#     sg_threshold: 100      # trigger when sg_result stays < this ...
#     sample_count: 16       # ... for this many consecutive samples ...
#     sample_period: 0.0003  # ... spaced this far apart (s) => ~5ms sustained
#     homing_velocity: 120   # only used to set TCOOLTHRS (StallGuard window)
#   [stepper_y]
#     endstop_pin: sg_endstop_stepper_y:virtual_endstop   # was tmc..:virtual_endstop
#
# It also sets TCOOLTHRS + spreadCycle during the homing move (so sg_result is
# valid), mirroring tmc.TMCVirtualPinHelper, and restores them after.
import collections
import mcu


class MCU_sg_endstop:
    def __init__(self, helper):
        self._helper = helper
        self._mcu = helper.mcu
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = None
        self._rest_ticks = 0
        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._mcu.register_config_callback(self._build_config)

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def _build_config(self):
        spi_oid = self._helper.spi.get_oid()
        self._mcu.add_config_cmd("config_sg_endstop oid=%d spi_oid=%d"
                                 % (self._oid, spi_oid))
        self._mcu.add_config_cmd(
            "sg_endstop_home oid=%d clock=0 rest_ticks=0 sample_count=0"
            " threshold=0 trsync_oid=0 trigger_reason=0 drop_permille=0 warmup=0"
            % (self._oid,), on_restart=True)
        cmd_queue = self._dispatch.get_command_queue()
        self._home_cmd = self._mcu.lookup_command(
            "sg_endstop_home oid=%c clock=%u rest_ticks=%u sample_count=%c"
            " threshold=%hu trsync_oid=%c trigger_reason=%c drop_permille=%hu"
            " warmup=%hu", cq=cmd_queue)
        self._query_cmd = self._mcu.lookup_query_command(
            "sg_endstop_query_state oid=%c",
            "sg_endstop_state oid=%c homing=%c next_clock=%u",
            oid=self._oid, cq=cmd_queue)

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # We ignore Klipper's sample_time/sample_count/rest_time and use our own
        # SG sample rate + sustained-count + threshold from config.
        # Accel-skip: don't start sampling sg_result until the axis is up to speed.
        # Below ~homing_speed the TMC sg_result is invalid-low and would false-trigger
        # ("micro-movement" trip). Offset only the SAMPLING start-clock; the trsync is
        # still armed from print_time (below), so the move is fully guarded — we just
        # ignore the ramp where SG is meaningless. A real frame-stall is held forever
        # and triggers fine once sampling begins.
        clock = self._mcu.print_time_to_clock(print_time + self._helper.home_accel_skip)
        rest_ticks = self._mcu.seconds_to_clock(self._helper.sample_period)
        self._rest_ticks = rest_ticks
        trigger_completion = self._dispatch.start(print_time)
        self._home_cmd.send(
            [self._oid, clock, rest_ticks, self._helper.sample_count,
             self._helper.sg_threshold, self._dispatch.get_oid(),
             mcu.MCU_trsync.REASON_ENDSTOP_HIT,
             self._helper.drop_permille, self._helper.warmup], reqclock=clock)
        return trigger_completion

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0, 0])
        res = self._dispatch.stop()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self._mcu.get_printer().command_error(
                "Communication timeout during homing")
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.
        if self._mcu.is_fileoutput():
            return home_end_time
        params = self._query_cmd.send([self._oid])
        next_clock = self._mcu.clock32_to_clock64(params['next_clock'])
        return self._mcu.clock_to_print_time(next_clock - self._rest_ticks)

    def query_endstop(self, print_time):
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid])
        return params['homing']


class SGEndstopHelper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]          # e.g. "stepper_y"
        self.tmc_name = config.get('tmc', 'tmc2130 ' + self.name)
        # trigger when sg_result stays BELOW this. Must sit between the real
        # stall floor (~0) and normal-motion sg_result (X~52, Y~40+). ~20 is a
        # safe start; tune from an SG_PROFILE after the belt is set.
        self.sg_threshold = config.getint('sg_threshold', 20,
                                          minval=0, maxval=1023)
        # consecutive sub-threshold samples required (sustained-stall filter);
        # sample_count*sample_period must exceed the longest transient SG dip.
        self.sample_count = config.getint('sample_count', 16,
                                          minval=1, maxval=255)
        self.sample_period = config.getfloat('sample_period', 0.0003, above=0.)
        self.homing_velocity = config.getfloat('homing_velocity', 120.,
                                               above=0.)
        # Skip sampling for this long after the homing move starts (clears the
        # low-velocity accel ramp where sg_result is invalid-low). ~2mm @ homing_speed.
        self.home_accel_skip = config.getfloat('home_accel_skip', 0.05,
                                               minval=0., maxval=0.5)
        # ADAPTIVE trigger: stall = sg_result falling below this fraction (per-mille)
        # of the live decaying-max BASELINE, which rides the temperature drift. This is
        # what makes hot homing work; sg_threshold above is now just a low absolute
        # floor. warmup = samples to learn the baseline before arming the trigger.
        self.drop_permille = config.getint('drop_permille', 400, minval=50, maxval=950)
        self.warmup = config.getint('warmup', 256, minval=0, maxval=4000)
        self.mcu = self.spi = self.mcu_tmc = self.fields = None
        self.mcu_endstop = None
        self._prev = collections.OrderedDict()
        self._dirty = collections.OrderedDict()
        ppins = self.printer.lookup_object('pins')
        ppins.register_chip('sg_endstop_' + self.name, self)

    def _bind_tmc(self):
        if self.mcu is not None:
            return
        tmc = self.printer.lookup_object(self.tmc_name)
        self.mcu_tmc = tmc.mcu_tmc
        self.fields = self.mcu_tmc.get_fields()
        self.spi = self.mcu_tmc.tmc_spi.spi
        self.mcu = self.spi.get_mcu()

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise ppins.error("sg_endstop only useful as a virtual endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise ppins.error("Can not pullup/invert sg_endstop virtual pin")
        self._bind_tmc()
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._end)
        self.mcu_endstop = MCU_sg_endstop(self)
        return self.mcu_endstop

    # --- TCOOLTHRS / spreadCycle management (mirrors TMCVirtualPinHelper) ----
    def _set_field(self, name, value):
        self._prev[name] = self.fields.get_field(name)
        reg = self.fields.lookup_register(name)
        self._dirty[reg] = self.fields.set_field(name, value)

    def _send(self):
        for reg, val in self._dirty.items():
            self.mcu_tmc.set_register(reg, val)
        self._dirty.clear()

    def _begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # sg_result requires spreadCycle + the StallGuard velocity window open
        self._set_field("en_pwm_mode", 0)        # disable stealthChop
        if self.fields.get_field("tcoolthrs") == 0:
            self._set_field("tcoolthrs", 0xfffff)
        if self.fields.lookup_register("thigh", None) is not None:
            self._set_field("thigh", 0)
        self._send()

    def _end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        for field, val in list(self._prev.items()):
            self._set_field(field, val)
        self._send()
        self._prev.clear()


def load_config_prefix(config):
    return SGEndstopHelper(config)
