# GT911 touch panel on the Prusa xLCD, driving Klipper's menu
#
# Copyright (C) 2026  coreone-firmware contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The controller sits on the I2C3 pins (PA8 SCL, PC9 SDA) of the xBuddy and
# shares its reset line with the ILI9488 panel, so it is reset whenever the
# display is initialised. Its I2C address (0x5D or 0x14) depends on the INT
# line level at that moment, so both are probed. The bus is bit-banged by
# default: the stm32f4 hardware I2C code leaves the NACK flag set after the
# probe of the absent address, which then fails later transfers. The touch
# status is polled; INT (PC8) is not used. Register layout, configuration
# blob and gesture rules follow Prusa-Firmware-Buddy
# src/hw/touchscreen/touchscreen_gt911*.
#
# Touches drive the [display] menu:
#   tap            status screen: open the menu; menu: run the tapped row
#   long press     same, with a long click
#   swipe up/down  scroll the list (or change the value being edited)
#   swipe right    back
#
# [gt911_touch]
# #i2c_software_scl_pin: PA8
# #i2c_software_sda_pin: PC9
# #i2c_bus:              use a hardware bus instead (see above)
# #i2c_speed: 100000
# #swap_xy: False
# #invert_x: False
# #invert_y: False
# #poll_interval: 0.03
# #swipe_distance: 40     (pixels; anything shorter is a tap)
# #long_press_time: 0.8
# #min_touch_polls: 2     (shorter touches are dropped as noise)
# #update_config: True    (write Prusa's controller config if it differs)
# #enable: True
import logging
import mcu
from . import bus

REG_ESD_CHECK = 0x8041
REG_CONFIG = 0x8047
REG_PRODUCT_ID = 0x8140
REG_STATUS = 0x814E
CONFIG_LEN = 0x8100 - REG_CONFIG + 1
ADDRESSES = (0x5D, 0x14)
I2C_CHUNK = 32
STATUS_OK = 'SUCCESS'

# Buddy touchscreen_gt911_config.hpp (0x8047..0x8100). Its stored checksum
# (0xaf) is stale; like Buddy, the checksum is recomputed before writing.
GT911_CONFIG = bytes([
    0x63, 0x40, 0x01, 0xe0, 0x01, 0x02, 0x87, 0x00, 0x01, 0x08, 0x28, 0x05,
    0x5a, 0x30, 0x03, 0x00, 0x02, 0x02, 0x00, 0x00, 0x55, 0x00, 0x00, 0x18,
    0x1a, 0x1e, 0x14, 0x87, 0x27, 0x09, 0x2b, 0x29, 0x31, 0x0d, 0x00, 0x00,
    0x00, 0x02, 0x03, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64, 0x32,
    0x00, 0x00, 0x00, 0x14, 0x58, 0x94, 0xc5, 0x02, 0x07, 0x00, 0x00, 0x04,
    0x95, 0x17, 0x00, 0x7f, 0x1f, 0x00, 0x71, 0x2a, 0x00, 0x62, 0x39, 0x00,
    0x58, 0x4c, 0x00, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x12, 0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x04,
    0x02, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x04, 0x06, 0x08, 0x0a, 0x0c, 0x24, 0x22, 0x21, 0x20, 0x1f, 0x1e, 0x1d,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xaf, 0x01])

def config_with_checksum(cfg):
    body = bytes(cfg[:CONFIG_LEN - 2])
    return body + bytes([(-sum(body)) & 0xff, 0x01])

# Gesture recognition (result of one touch, decided on release)
def classify(start, end, duration, swipe_min, long_press_time):
    # Fingers drift 10-20 px during a tap, so anything shorter than a
    # swipe counts as a tap
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    adx, ady = abs(dx), abs(dy)
    if adx < swipe_min and ady < swipe_min:
        if duration >= long_press_time:
            return 'long_press', 0
        return 'tap', 0
    if ady >= swipe_min and adx <= .5 * ady:
        return ('swipe_up' if dy < 0 else 'swipe_down'), ady
    if adx >= swipe_min and ady <= .5 * adx:
        return ('swipe_left' if dx < 0 else 'swipe_right'), adx
    return None, 0

class GT911Touch:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.name = config.get_name()
        i2c_mcu = mcu.get_printer_mcu(printer, config.get('i2c_mcu', 'mcu'))
        i2c_bus = config.get('i2c_bus', None)
        sw_pins = None
        if i2c_bus is None:
            sw_pins = (config.get('i2c_software_scl_pin', 'PA8'),
                       config.get('i2c_software_sda_pin', 'PC9'))
        speed = config.getint('i2c_speed', 100000, minval=10000)
        self.devs = [bus.MCU_I2C(i2c_mcu, i2c_bus, addr, speed, sw_pins)
                     for addr in ADDRESSES]
        self.transfer_cmds = None
        i2c_mcu.register_config_callback(self._build_config)
        self.mcu = i2c_mcu
        self.width = config.getint('width', 480, minval=1)
        self.height = config.getint('height', 320, minval=1)
        self.swap_xy = config.getboolean('swap_xy', False)
        self.invert_x = config.getboolean('invert_x', False)
        self.invert_y = config.getboolean('invert_y', False)
        self.poll_time = config.getfloat('poll_interval', .03,
                                         minval=.02, maxval=.5)
        self.start_delay = config.getfloat('start_delay', 2., minval=0.)
        self.swipe_min = config.getint('swipe_distance', 40, minval=1)
        self.long_press_time = config.getfloat('long_press_time', .8,
                                               above=0.)
        self.min_polls = config.getint('min_touch_polls', 2, minval=1)
        self.update_config = config.getboolean('update_config', True)
        self.enabled = config.getboolean('enable', True)
        # State
        self.dev = None
        self.state = 'init'
        self.errors = 0
        self.total_errors = 0
        self.config_state = 'unknown'
        self.probing = False
        self.touch = None
        self.last_event = None
        self.test_until = 0.
        self.display = None
        self.timer = None
        printer.register_event_handler('klippy:ready', self._handle_ready)
        printer.register_event_handler('homing:homing_move_begin',
                                       self._homing_begin)
        printer.register_event_handler('homing:homing_move_end',
                                       self._homing_end)
        gcode = printer.lookup_object('gcode')
        gcode.register_command('TOUCH_STATUS', self.cmd_TOUCH_STATUS,
                               desc=self.cmd_TOUCH_STATUS_help)
        gcode.register_command('TOUCH_TEST', self.cmd_TOUCH_TEST,
                               desc=self.cmd_TOUCH_TEST_help)
        gcode.register_command('TOUCH_ENABLE', self.cmd_TOUCH_ENABLE,
                               desc=self.cmd_TOUCH_ENABLE_help)
    def _build_config(self):
        # Query the bus directly: a NACK at the address that is not in use
        # must not shut the printer down (MCU_I2C.i2c_transfer would)
        self.transfer_cmds = [self.mcu.lookup_query_command(
            "i2c_transfer oid=%c write=%*s read_len=%u",
            "i2c_response oid=%c i2c_bus_status=%c response=%*s",
            oid=d.get_oid(), cq=d.get_command_queue()) for d in self.devs]
    # I2C helpers
    def _xfer(self, idx, write, read_len=0):
        if self.mcu.is_fileoutput():
            return None
        params = self.transfer_cmds[idx].send(
            [self.devs[idx].get_oid(), write, read_len])
        if params['i2c_bus_status'] != STATUS_OK:
            return None
        return bytes(bytearray(params['response']))
    def _read(self, reg, length, idx=None):
        if idx is None:
            idx = self.dev
        out = b''
        while len(out) < length:
            n = min(I2C_CHUNK, length - len(out))
            r = self._xfer(idx, [(reg >> 8) & 0xff, reg & 0xff], n)
            if r is None or len(r) != n:
                return None
            out += r
            reg += n
        return out
    def _write(self, reg, data):
        pos = 0
        while pos < len(data):
            chunk = list(data[pos:pos + I2C_CHUNK])
            r = self._xfer(self.dev, [(reg >> 8) & 0xff, reg & 0xff] + chunk)
            if r is None:
                return False
            pos += len(chunk)
            reg += len(chunk)
        return True
    # Startup
    def _handle_ready(self):
        self.display = self.printer.lookup_object('display', None)
        self.timer = self.reactor.register_timer(
            self._poll_event, self.reactor.monotonic() + self.start_delay)
    def _detect(self):
        for idx, addr in enumerate(ADDRESSES):
            pid = self._read(REG_PRODUCT_ID, 4, idx)
            if pid is not None:
                self.dev = idx
                logging.info("gt911_touch: controller at 0x%02x, product"
                             " id %r", addr, pid)
                return True
        return False
    def _check_config(self):
        cur = self._read(REG_CONFIG, CONFIG_LEN)
        if cur is None:
            return False
        if cur[:CONFIG_LEN - 2] == GT911_CONFIG[:CONFIG_LEN - 2]:
            self.config_state = 'matches stock'
        elif not self.update_config:
            self.config_state = 'differs (version 0x%02x), left as is' % (
                cur[0],)
        else:
            logging.info("gt911_touch: config differs (version 0x%02x),"
                         " writing the stock one", cur[0])
            if not self._write(REG_CONFIG, config_with_checksum(GT911_CONFIG)):
                return False
            self.config_state = 'written (was version 0x%02x)' % (cur[0],)
        # ESD protection, as Buddy does after configuring
        self._write(REG_ESD_CHECK, [0])
        return True
    def _start(self):
        if not self._detect():
            return False
        if not self._check_config():
            return False
        self._write(REG_STATUS, [0])
        logging.info("gt911_touch: ready, config %s", self.config_state)
        return True
    # Polling
    def _homing_begin(self, hmove):
        # Keep the MCU free while the load cell probes
        self.probing = True
    def _homing_end(self, hmove):
        self.probing = False
    def _poll_event(self, eventtime):
        if self.printer.is_shutdown():
            return self.reactor.NEVER
        if not self.enabled or self.probing:
            self.touch = None
            return eventtime + self.poll_time
        if self.state != 'running':
            if self._start():
                self.state = 'running'
                self.errors = 0
            else:
                self.state = 'not found'
                return eventtime + 5.
            return eventtime + self.poll_time
        data = self._read(REG_STATUS, 1)
        if data is None:
            return self._poll_error(eventtime)
        self.errors = 0
        status = data[0]
        if not status & 0x80:
            # No new report. The controller only reports when the point
            # moves, so a still finger is still down.
            if self.touch is not None:
                self.touch['polls'] += 1
            return eventtime + self.poll_time
        count = status & 0x0f
        point = b''
        if 0 < count <= 5:
            point = self._read(REG_STATUS + 1, 8)
            if point is None:
                return self._poll_error(eventtime)
        if not self._write(REG_STATUS, [0]):
            return self._poll_error(eventtime)
        if count > 5:
            self.touch = None
            return eventtime + self.poll_time
        self._update_touch(eventtime, count, point)
        return eventtime + self.poll_time
    def _poll_error(self, eventtime):
        self.errors += 1
        self.total_errors += 1
        self.touch = None
        if self.errors >= 8:
            logging.warning("gt911_touch: %d read errors, re-detecting",
                            self.errors)
            self.state = 'lost'
            self.errors = 0
            return eventtime + 5.
        return eventtime + self.poll_time
    def _point(self, pt):
        # Buddy reads the first coordinate as screen Y and the second as X
        y = pt[1] | (pt[2] << 8)
        x = pt[3] | (pt[4] << 8)
        if self.swap_xy:
            x, y = y, x
        if self.invert_x:
            x = self.width - 1 - x
        if self.invert_y:
            y = self.height - 1 - y
        return (x, y)
    def _update_touch(self, eventtime, count, pt):
        t = self.touch
        if count == 0:
            self.touch = None
            if t is None:
                return
            if t['valid'] and t['polls'] >= self.min_polls:
                self._gesture(eventtime, t)
            elif eventtime < self.test_until:
                self._respond("touch: ignored (%s)" % (
                    "several fingers" if not t['valid'] else "too short",))
            return
        if t is None:
            pos = self._point(pt)
            self.touch = {'start': pos, 'last': pos, 'time': eventtime,
                          'polls': 1, 'valid': count == 1}
            return
        t['polls'] += 1
        if count > 1:
            t['valid'] = False
        else:
            t['last'] = self._point(pt)
    # Gestures -> menu
    def _gesture(self, eventtime, t):
        kind, dist = classify(t['start'], t['last'], eventtime - t['time'],
                              self.swipe_min, self.long_press_time)
        self.last_event = (kind, t['start'], t['last'])
        if eventtime < self.test_until:
            self._respond("touch: %s from %d,%d to %d,%d (%.2fs)" % (
                kind or "unclear (diagonal)", t['start'][0], t['start'][1],
                t['last'][0], t['last'][1], eventtime - t['time']))
            return
        if kind is None:
            return
        try:
            self._menu_action(eventtime, kind, dist, t['start'])
        except Exception:
            logging.exception("gt911_touch: menu action failed")
    def _respond(self, msg):
        self.printer.lookup_object('gcode').respond_info(msg)
    def _menu_action(self, eventtime, kind, dist, pos):
        display = self.display
        menu = getattr(display, 'menu', None)
        if menu is None:
            return
        lcd = getattr(display, 'lcd_chip', None)
        if getattr(lcd, '_bl_dimmed', False):
            # The first touch on a dimmed screen only wakes it
            lcd._note_activity(eventtime)
            return
        rows = max(1, menu.rows)
        if hasattr(lcd, 'menu_row_height'):
            row_px = lcd.menu_row_height()
        else:
            row_px = 16 * getattr(lcd, 'scale', 0) or self.height // rows
        target = None
        if hasattr(lcd, 'touch_target'):
            target = lcd.touch_target(pos[0], pos[1])
        if kind in ('tap', 'long_press') and target is not None:
            self._target_action(menu, target, kind, eventtime)
        elif kind in ('tap', 'long_press'):
            key = 'click' if kind == 'tap' else 'long_click'
            if menu.is_running():
                self._select_row(menu, pos[1] // row_px)
            menu.key_event(key, eventtime)
        elif kind in ('swipe_up', 'swipe_down') and menu.is_running():
            # The list follows the finger: swiping up shows later items
            key = 'down' if kind == 'swipe_up' else 'up'
            for i in range(max(1, min(10, int(round(dist / row_px))))):
                menu.key_event(key, eventtime)
        elif kind == 'swipe_right' and menu.is_running():
            menu.key_event('back', eventtime)
    def _target_action(self, menu, target, kind, eventtime):
        # A touch on the graphical UI: a status screen button, the menu title
        # bar (back) or a menu row
        what, arg = target
        key = 'click' if kind == 'tap' else 'long_click'
        if what == 'back':
            if menu.is_running():
                menu.key_event('back', eventtime)
        elif what == 'row':
            if menu.is_running():
                self._select_row(menu, arg)
                menu.key_event(key, eventtime)
        elif what == 'button':
            if arg == 'menu':
                if not menu.is_running():
                    menu.key_event('click', eventtime)
            elif arg in ('pause', 'resume'):
                self._run_gcode(arg.upper())
    def _run_gcode(self, script):
        gcode = self.printer.lookup_object('gcode')
        def run(eventtime):
            try:
                gcode.run_script(script)
            except Exception:
                logging.exception("gt911_touch: %s failed", script)
        self.reactor.register_callback(run)
    def _select_row(self, menu, row):
        from .display import menu as menu_mod
        container = menu.stack_peek()
        if not isinstance(container, menu_mod.MenuList):
            return
        current = container.selected_item()
        if isinstance(current, menu_mod.MenuInput) and current.is_editing():
            return
        if row >= menu.rows:
            return
        index = getattr(container, '_viewport_top', 0) + row
        if index < len(container):
            container.select_at(index)
    # G-code commands
    cmd_TOUCH_STATUS_help = "Report the GT911 touch controller state"
    def cmd_TOUCH_STATUS(self, gcmd):
        addr = "none" if self.dev is None else "0x%02x" % (ADDRESSES[self.dev],)
        msg = ["GT911: %s, address %s, enabled %d" % (
                   self.state, addr, self.enabled),
               "config: %s" % (self.config_state,),
               "read errors: %d" % (self.total_errors,)]
        if self.last_event is not None:
            kind, s, e = self.last_event
            msg.append("last touch: %s from %d,%d to %d,%d" % (
                kind, s[0], s[1], e[0], e[1]))
        gcmd.respond_info("\n".join(msg))
    cmd_TOUCH_TEST_help = ("Report touches in the console instead of acting"
                           " on them: SECONDS=<n>")
    def cmd_TOUCH_TEST(self, gcmd):
        secs = gcmd.get_float('SECONDS', 30., minval=1., maxval=600.)
        self.test_until = self.reactor.monotonic() + secs
        gcmd.respond_info("Touch the screen; touches are reported for %.0fs"
                          " and do not operate the menu" % (secs,))
    cmd_TOUCH_ENABLE_help = "Turn touch input on or off: ENABLE=0|1"
    def cmd_TOUCH_ENABLE(self, gcmd):
        self.enabled = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        self.touch = None

def load_config(config):
    return GT911Touch(config)
