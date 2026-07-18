# ILI9488 SPI colour TFT as a Klipper display "lcd_chip" (Prusa Core One / STM32F427).
#
# The 320x480 ILI9488 is on the F427's dedicated SPI6 (PG13 SCK / PG14 MOSI /
# PG12 MISO), CS=PD11, DC/RS=PD15, RST=PG4. This implements the Klipper display
# lcd_chip interface (init/get_dimensions/clear/write_text/write_glyph/
# write_graphics/set_glyphs/flush) so the stock [display] + [menu] framework drives
# it (status screen + navigable menus + rotary-encoder/click) with no MCU change.
# Register it via display.py LCD_chips: 'ili9488': ili9488_display.ILI9488.
#
# Model: a character grid of cols x rows, each cell a 16-byte row-major MSB mono
# bitmap (an 8x16 font glyph, an icon half, or graphics). flush() diffs the grid
# and re-renders only changed cells (each a scaled RGB666 blit), so updates are
# small. Over 4-wire SPI the ILI9488 is RGB666 (3 B/px); the panel is BGR-ordered.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

NSEG_HINT = 512   # src/display_dma.c NSEG -- for reporting ring depth
from . import bus
from .display.font8x14 import VGA_FONT

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000
TextGlyphs = {'right_arrow': b'\x1a', 'degrees': b'\xf8'}

# ILI9488 opcodes
CMD_SLPOUT = 0x11; CMD_INVON = 0x21; CMD_DISPOFF = 0x28; CMD_DISPON = 0x29
CMD_CASET = 0x2A; CMD_RASET = 0x2B; CMD_RAMWR = 0x2C; CMD_MADCTL = 0x36
CMD_COLMOD = 0x3A; CMD_WRDISBV = 0x51; CMD_WRCTRLD = 0x53; CMD_CABCCTRL2 = 0xC8
CTRLD_BCTRL = 0x20; CTRLD_BL = 0x04

GAMMA  = [0x00,0x08,0x0c,0x02,0x0e,0x04,0x30,0x45,0x47,0x04,0x0c,0x0a,0x2e,0x34,0x0F]
NGAMMA = [0x00,0x11,0x0d,0x01,0x0f,0x05,0x39,0x36,0x51,0x06,0x0f,0x0d,0x33,0x37,0x0F]
INIT_SEQ = [
    (0xF7, [0xA9, 0x51, 0x2C, 0x82], 0.), (CMD_MADCTL, [0xE0], 0.),
    (CMD_COLMOD, [0x66], 0.), (0xB1, [0xA0, 0x11], 0.), (0xB4, [0x02], 0.),
    (0xC0, [0x0F, 0x0F], 0.), (0xC1, [0x41], 0.), (0xC2, [0x22], 0.),
    (0xC5, [0x00, 0x53, 0x80], 0.), (0xB7, [0xC6], 0.), (0xE0, GAMMA, 0.),
    (0xE1, NGAMMA, 0.), (CMD_INVON, [], 0.), (CMD_SLPOUT, [], 0.120),
    (CMD_DISPON, [], 0.020),
]
SPI_CHUNK = 48   # bytes per spi_send (pixel-aligned: 48 % 3 == 0)
EMPTY = bytes(16)
# Cap cells/flush while busy so a whole flush still fits the MCU DMA ring with no
# back-pressure (back-pressure spins in command context = the loadcell starvation
# this engine exists to prevent). A glyph costs 78 ring segments (5 framing + 72
# data + CS-up) against NSEG 512, so 6 fits and 7 does not. The cap is now purely
# RING-bound: with the MCU glyph expander a cell is 1 command-link message instead
# of 72, so the link is no longer what limits it. Back off to 4 (the long-validated
# value) if the display ever misbehaves during a probe.
DMA_CELLS_PER_FLUSH = 6

# Colour palette, BGR666 (panel is BGR-ordered; low 2 bits ignored).
C_WHITE = bytes([0xFC, 0xFC, 0xFC])
C_HOT   = bytes([0x00, 0x00, 0xFC])  # red   -> heater >= 50C
C_WARM  = bytes([0x00, 0xA8, 0xFC])  # amber -> heater >= 35C
C_COOL  = bytes([0xFC, 0x90, 0x00])  # light blue -> cold heater
C_CYAN  = bytes([0xFC, 0xFC, 0x00])  # feedrate

# Largest font_scale the MCU glyph expander supports (must match GLYPH_SCALE_MAX
# in src/display_dma.c -- above it the host falls back to expanded pixels).
GLYPH_SCALE_MAX = 3

def _rgb(c):
    # 3-byte panel colour -> the u32 the glyph command carries.
    return (c[0] << 16) | (c[1] << 8) | c[2]
C_GREEN = bytes([0x00, 0xFC, 0x00])  # fan
C_ACC   = bytes([0xFC, 0xC0, 0x40])  # usb / sd accent
HOT_C, WARM_C = 50., 35.             # heater colour thresholds


class ILI9488:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mcu = self.printer.lookup_object('mcu')
        self.width = config.getint('width', 480, minval=1)
        self.height = config.getint('height', 320, minval=1)
        self.scale = config.getint('font_scale', 3, minval=1, maxval=6)
        speed = config.getint('spi_speed', 20000000, minval=100000)
        spi_bus = config.get('spi_bus', 'spi6_PG12_PG14_PG13')
        self.spi = bus.MCU_SPI(self.mcu, spi_bus, None, 0, speed)   # no HW CS
        cq = self.spi.get_command_queue()
        self.cs = bus.MCU_bus_digital_out(self.mcu, config.get('cs_pin', 'PD11'),
                                          cq, value=1)
        self.dc = bus.MCU_bus_digital_out(self.mcu, config.get('dc_pin', 'PD15'),
                                          cq, value=1)
        self.rst = bus.MCU_bus_digital_out(self.mcu, config.get('rst_pin', 'PG4'),
                                           cq, value=0)
        # Async SPI6 TX-DMA play-out engine (src/stm32/display_dma.c): runtime
        # cell blits go through it so flushing never blocks command_task long
        # enough to starve the loadcell. The one-time init() below keeps the
        # generic (blocking) SPI path for its real-time power-up delays.
        # Pass the pin NAMES straight through: msgproto maps any *_pin arg via the
        # "pin" enumeration, so we avoid a second pin reservation (the init path's
        # MCU_bus_digital_out above already owns CS/DC).
        cs_pin = config.get('cs_pin', 'PD11')
        dc_pin = config.get('dc_pin', 'PD15')
        self._dma_oid = self.mcu.create_oid()
        self.mcu.add_config_cmd(
            "config_display_dma oid=%d spi_bus=%s mode=%d rate=%d"
            " cs_pin=%s dc_pin=%s"
            % (self._dma_oid, spi_bus, 0, speed, cs_pin, dc_pin))
        self.dma_cells = config.getint('dma_cells_per_flush',
                                       DMA_CELLS_PER_FLUSH, minval=1)
        # When a menu is open during a print, the status-row churn (live temps,
        # XYZ) eats the small dma_cells budget row-major before the menu rows
        # repaint -> stale status bleeds under the menu. Raise the cap while a
        # menu is showing so it repaints in a few flushes. NOT applied during a
        # loadcell probe tap (that stays at dma_cells -- the critical window).
        # ⚠ UNEXERCISED REGIME after the MCU glyph expander (2026-09-28): a cell is
        # 78 ring segments, so any value above ~6 cannot fit NSEG 512 and the MCU
        # back-pressures. That was equally true before (77 segs/cell), but the spin
        # now happens INSIDE one display_dma_glyph command instead of spread across
        # 72 separate seg messages, so command_task is held for a longer unbroken
        # stretch (~0.7ms/cell once saturated). Total occupancy should be LOWER
        # than before -- far fewer messages to decode -- but the shape is new and
        # menu-open-during-a-probe has NOT been re-tested. Lower this to 6 if the
        # display ever misbehaves during a probe.
        self.menu_cells = config.getint('menu_dma_cells', 16, minval=1)
        self._seg_cmd = self._dma_wait_cmd = self._glyph_cmd = None
        self.mcu.register_config_callback(self._build_dma_cmds)
        self.backlight = config.getboolean('backlight', True)
        self.brightness = config.getint('brightness', 255, minval=0, maxval=255)
        self._fg = bytes([0xFC, 0xFC, 0xFC])    # white, BGR666
        self._bg = bytes([0x00, 0x00, 0x00])    # black
        # character grid of 16-byte mono cells
        self.cols = self.width // (8 * self.scale)
        self.rows = self.height // (16 * self.scale)
        self.font = VGA_FONT
        self.icons = {}
        self.fb = [[bytearray(EMPTY) for _ in range(self.cols)]
                   for _ in range(self.rows)]
        self.shown = [[bytearray(EMPTY) for _ in range(self.cols)]
                      for _ in range(self.rows)]
        # Per-cell foreground colour (icons get tinted; text stays white).
        self.fgmap = [[C_WHITE for _ in range(self.cols)]
                      for _ in range(self.rows)]
        self.shown_col = [[C_WHITE for _ in range(self.cols)]
                          for _ in range(self.rows)]
        self._extruder = self._heater_bed = None
        self._idle_timeout = None
        self._display = None
        self._init_pt = 0.
        self._probing = False
        self.printer.register_event_handler('homing:homing_move_begin',
                                            self._homing_begin)
        self.printer.register_event_handler('homing:homing_move_end',
                                            self._homing_end)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_DISPLAY_BACKLIGHT',
                               self.cmd_SET_DISPLAY_BACKLIGHT,
                               desc="ILI9488 backlight: ON=0/1 [BRIGHTNESS=0-255]")
        gcode.register_command('DUMP_DISPLAY_FB', self.cmd_DUMP_DISPLAY_FB,
                               desc="debug: dump the display cell-fill grid")
        gcode.register_command('SET_MENU_CELLS', self.cmd_SET_MENU_CELLS,
                               desc="live-tune menu repaint cells/flush while "
                                    "busy: CELLS=<n>")
        # Backlight idle-dim / wake-on-click. backlight_timeout=0 disables it.
        self.bl_timeout = config.getint('backlight_timeout', 0, minval=0)
        self.idle_brightness = config.getint('idle_brightness', 0,
                                             minval=0, maxval=255)
        self._bl_dimmed = False
        self._last_activity = 0.
        self._bl_timer = None
        # --- PANEL HEALTH (2026-11-10, redesigned after review) --------------
        # 🚨 THE FIRST DESIGN WAS WRONG AND IS WORTH RECORDING. It watched the MCU
        # DMA engine for a lost transfer-complete (count>0, running, tc frozen).
        # That state cannot persist: a changed cell costs ~78 ring segments against
        # NSEG 512, so the 7th changed cell fills the ring, dd_push spins 500 ms and
        # calls shutdown("display_dma ring stall") -- within SECONDS on a live print
        # screen whose temps, progress and Z change constantly. A two-poll detector
        # at 30 s loses that race every time, and the thing it would have reported
        # aborts the print anyway.
        #
        # And the 2026-10-25 freeze had NO shutdown and NOTHING logged, so by that
        # same arithmetic it cannot have been a dead engine at all: it was a healthy
        # engine streaming pixels at a panel whose CONTROLLER had stopped. Engine
        # counters read perfectly normal in that state -- which is why they are kept
        # here as diagnostic colour only, and why the detector is a read-back from
        # the panel itself.
        self.health_period = config.getfloat('health_period', 30., minval=0.)
        # RDDPM (0x0A) = power mode: bit6 sleep-OUT, bit2 display-ON when alive.
        self.health_reg = config.getint('health_reg', 0x0a, minval=0, maxval=0xff)
        self._rd_cmd = None
        self._rd_unavailable = False   # board does not drive SDO -> detector OFF
        self._rd_base = None           # baseline bytes from the at-ready read
        self._hq_cmd = None
        self._hq_last = {}             # last engine-counter reply (diagnostics)
        self._panel_bad = 0
        self._panel_warned = False
        self._hq_timer = None
        gcode.register_command('DISPLAY_HEALTH', self.cmd_DISPLAY_HEALTH,
                               desc="Read the panel's own status register back over "
                                    "MISO plus the DMA engine counters. The only "
                                    "check that can see a wedged PANEL; M117 and "
                                    "display_status cannot.")
        if self.health_period > 0.:
            self.printer.register_event_handler('klippy:ready', self._hq_ready)
        if self.bl_timeout > 0:
            self._hook_menu_keys()
            self.printer.register_event_handler('klippy:ready', self._bl_ready)

    # --- panel health -------------------------------------------------------
    def _hq_ready(self):
        # Establish the read-back BASELINE first. ⚠ THE DISCRIMINATOR FOR THIS
        # DESIGN: a board that does not route SDO reads all 0x00 or all 0xFF, which
        # is INDISTINGUISHABLE from a dead panel. So the detector only arms if the
        # first read returns something plausible; otherwise it says read-back is
        # unavailable and stays off. Failing safe beats crying wolf -- an alarm that
        # fires on healthy hardware is how a real one gets ignored.
        st = self._panel_read(self.health_reg, 2)
        if st is None or self._implausible(st):
            self._rd_unavailable = True
            logging.info("ili9488: panel read-back unavailable (reg 0x%02x -> %s)."
                         " Leaving the panel-liveness check OFF: on a board that does"
                         " not drive SDO this reads identically to a dead panel, so"
                         " arming it would alarm on healthy hardware. DISPLAY_HEALTH"
                         " still reports the engine counters.",
                         self.health_reg,
                         "no reply" if st is None else " ".join("%02x" % b for b in st))
        else:
            self._rd_base = bytes(st)
            logging.info("ili9488: panel read-back baseline reg 0x%02x = %s;"
                         " liveness check armed at %.0f s",
                         self.health_reg, " ".join("%02x" % b for b in st),
                         self.health_period)
        self._hq_timer = self.reactor.register_timer(
            self._hq_event, self.reactor.monotonic() + self.health_period)

    @staticmethod
    def _implausible(data):
        """All-zero or all-ones means nobody drove the bus, not that the panel died."""
        if not data:
            return True
        return all(b == 0x00 for b in data) or all(b == 0xff for b in data)

    def _panel_read(self, reg, nbytes):
        """Raw read-back, or None. One extra byte is requested because most ILI9488
        reads clock out a dummy first -- the shift is panel-revision dependent and
        unmeasured on this hardware, so the bytes come back raw and uninterpreted."""
        if self._rd_cmd is None:
            try:
                self._rd_cmd = self.mcu.lookup_query_command(
                    "display_dma_read oid=%c cmd=%c nbytes=%c",
                    "display_dma_rdata oid=%c busy=%c n=%c data=%*s",
                    oid=self._dma_oid)
            except Exception as e:
                # ⚠ Distinguish the cases. A FORMAT MISMATCH is drift between this
                # file and display_dma.c and must not be filed as "old firmware":
                # reporting a mismatch as a missing command sent an earlier version
                # of this module looking in the wrong place entirely.
                if 'format mismatch' in str(e).lower():
                    logging.warning("ili9488: display_dma_read FORMAT MISMATCH (%s) --"
                                    " the host module and display_dma.c have drifted;"
                                    " reflash or re-deploy, do not read this as old"
                                    " firmware", e)
                else:
                    logging.info("ili9488: MCU has no display_dma_read (%s);"
                                 " panel liveness is not observable on this build", e)
                self._rd_cmd = False
                return None
        if self._rd_cmd is False:
            return None
        try:
            r = dict(self._rd_cmd.send([self._dma_oid, reg, nbytes + 1]))
        except Exception as e:
            # Transient. Do NOT disable anything -- the poll re-arms and tries again.
            logging.info("ili9488: panel read 0x%02x failed (%s); will retry", reg, e)
            return None
        if r.get('busy'):
            return None            # engine mid-flush; ask again next poll
        return bytes(bytearray(r.get('data', b'')))

    def _engine_counters(self):
        """Engine ring/completion counters. DIAGNOSTIC COLOUR ONLY -- see the note in
        __init__ for why these cannot detect the failure that prompted all this."""
        if self._hq_cmd is None:
            try:
                self._hq_cmd = self.mcu.lookup_query_command(
                    "display_dma_query oid=%c",
                    "display_dma_state oid=%c count=%hu head=%hu tail=%hu"
                    " running=%c tc=%u dmaerr=%u pushes=%u maxcount=%hu"
                    " last_tc=%u now=%u", oid=self._dma_oid)
            except Exception as e:
                logging.info("ili9488: MCU has no display_dma_query (%s)", e)
                self._hq_cmd = False
                return None
        if self._hq_cmd is False:
            return None
        try:
            st = dict(self._hq_cmd.send([self._dma_oid]))
        except Exception as e:
            logging.info("ili9488: display_dma_query failed (%s); will retry", e)
            return None
        self._hq_last = st
        return st

    def _hq_event(self, eventtime):
        if self.health_period <= 0.:
            return self.reactor.NEVER
        # ⚠ ALWAYS re-arm. An earlier version returned NEVER on any failed query,
        # so one transient error killed the poll for the rest of the session while
        # health_period still read 30 -- a dead detector that looked armed.
        nxt = eventtime + self.health_period
        if self._rd_unavailable or self._rd_base is None:
            return nxt
        st = self._panel_read(self.health_reg, 2)
        if st is None:
            return nxt             # busy, transient, or unsupported: no verdict
        if self._implausible(st) or bytes(st) != self._rd_base:
            self._panel_bad = 1
            if not self._panel_warned:
                self._panel_warned = True
                eng = self._engine_counters() or {}
                logging.warning(
                    "ili9488: PANEL LIVENESS CHANGED -- register 0x%02x now reads %s,"
                    " baseline was %s. The panel controller has reset, slept, or"
                    " stopped responding; the screen is probably frozen even though"
                    " the DMA engine looks fine (tc=%s count=%s dmaerr=%s). This is"
                    " the 2026-10-25 signature, which logged NOTHING at the time."
                    " The print is unaffected and must stay that way -- a cosmetic"
                    " panel never costs a job. M117 and display_status cannot see"
                    " this: they only set host-side text.",
                    self.health_reg, " ".join("%02x" % b for b in st),
                    " ".join("%02x" % b for b in self._rd_base),
                    eng.get('tc'), eng.get('count'), eng.get('dmaerr'))
        else:
            if self._panel_bad:
                logging.info("ili9488: panel register 0x%02x back to baseline %s",
                             self.health_reg,
                             " ".join("%02x" % b for b in self._rd_base))
            self._panel_bad = 0
            self._panel_warned = False
        return nxt

    def cmd_DISPLAY_HEALTH(self, gcmd):
        # ⚠ The verdict comes from the sample taken HERE, not from the poll's latch.
        # The first version printed self._panel_bad, so a wedge inside the first poll
        # interval -- or with health_period 0 -- reported "healthy" off a stale flag.
        rd = self._panel_read(self.health_reg, 2)
        eng = self._engine_counters()
        lines = []
        if rd is None:
            lines.append("  panel reg 0x%02x: NO READING (engine busy, unsupported,"
                         " or transient)" % self.health_reg)
        else:
            lines.append("  panel reg 0x%02x = %s   (baseline %s)"
                         % (self.health_reg, " ".join("%02x" % b for b in rd),
                            " ".join("%02x" % b for b in self._rd_base)
                            if self._rd_base else "not established"))
        if self._rd_unavailable:
            lines.append("  liveness check: OFF -- this board does not appear to drive"
                         " SDO, which reads the same as a dead panel")
        elif rd is None or self._rd_base is None:
            lines.append("  liveness verdict: UNKNOWN (no usable reading)")
        elif self._implausible(rd):
            lines.append("  liveness verdict: PANEL NOT RESPONDING")
        elif bytes(rd) != self._rd_base:
            lines.append("  liveness verdict: PANEL CHANGED STATE (reset/slept?)")
        else:
            lines.append("  liveness verdict: panel alive (matches baseline)")
        if eng is None:
            lines.append("  engine counters: unavailable on this firmware")
        else:
            # ⚠ THE AGE IS AMBIGUOUS MODULO THE WRAP, AND CANNOT BE BOUNDED.
            # last_tc/now are raw 32-bit tick counts; at 168 MHz they wrap every
            # 25.57 s. My first attempt compared the wrapped delta against the wrap
            # period and printed a precise figure when it looked small -- but a
            # WRAPPED DELTA *IS* A SMALL NUMBER: a one-hour gap came out as a
            # confident "20.86 s". There is no arithmetic that recovers the truth
            # from one sample, so the only honest output states the ambiguity.
            # The real long-gap signal is tc not advancing between polls, which the
            # host tracks itself; this field is for sub-wrap timing only.
            age = "n/a"
            try:
                freq = float(self.mcu.get_constant_float('CLOCK_FREQ'))
                wrap = (1 << 32) / freq
                if not eng['last_tc']:
                    age = "never completed"
                else:
                    d = ((eng['now'] - eng['last_tc']) & 0xffffffff) / freq
                    age = ("%.2f s AMBIGUOUS modulo %.2f s -- the tick counter wraps,"
                           " so a longer gap is indistinguishable from this one;"
                           " do not read it as a stall duration" % (d, wrap))
            except Exception:
                pass
            lines.append("  engine (DIAGNOSTIC ONLY, cannot see a dead panel):"
                         " count=%d/%d running=%d tc=%d dmaerr=%d pushes=%d"
                         % (eng['count'], NSEG_HINT, eng['running'], eng['tc'],
                            eng['dmaerr'], eng['pushes']))
            lines.append("  last engine completion: %s" % age)
            if eng['count'] and not eng['tc']:
                lines.append("  ⚠ segments queued with no completion ever: if this"
                             " persists the ring fills in ~7 changed cells and the"
                             " MCU will shutdown with \"display_dma ring stall\"")
        gcmd.respond_info("display health:\n" + "\n".join(lines))

    # ⚠ NO get_status HERE, DELIBERATELY. An lcd_chip is NOT a registered printer
    # object -- display.py does `self.lcd_chip = config.getchoice(...)(config)`, so
    # the instance is a plain attribute and display.py has no get_status either. A
    # get_status() here would never be reachable from Moonraker or a macro's
    # `printer[...]`: telemetry nothing can read, which is the same can't-fail shape
    # this whole patch exists to remove. I wrote one, found it unreachable, removed
    # it. The working channels are the logged warning above and DISPLAY_HEALTH.

    # --- low-level SPI (one CS-framed command + optional data) --------------
    def _w(self, cmd, data, minclock=0, reqclock=0):
        kc = {'minclock': minclock, 'reqclock': reqclock}
        self.cs.update_digital_out(0, **kc)
        self.dc.update_digital_out(0, **kc)
        self.spi.spi_send([cmd], **kc)
        if data:
            self.dc.update_digital_out(1, **kc)
            for i in range(0, len(data), SPI_CHUNK):
                self.spi.spi_send(list(data[i:i + SPI_CHUNK]), **kc)
        self.cs.update_digital_out(1, **kc)

    def _blit(self, x, y, w, h, data):
        kc = {'reqclock': BACKGROUND_PRIORITY_CLOCK}
        x1, y1 = x + w - 1, y + h - 1
        self._w(CMD_CASET, [x >> 8, x & 0xff, x1 >> 8, x1 & 0xff], **kc)
        self._w(CMD_RASET, [y >> 8, y & 0xff, y1 >> 8, y1 & 0xff], **kc)
        self.cs.update_digital_out(0, **kc)
        self.dc.update_digital_out(0, **kc)
        self.spi.spi_send([CMD_RAMWR], **kc)
        self.dc.update_digital_out(1, **kc)
        for i in range(0, len(data), SPI_CHUNK):
            self.spi.spi_send(list(data[i:i + SPI_CHUNK]), **kc)
        self.cs.update_digital_out(1, **kc)

    def _fill_phys(self, x, y, w, h, px, minclock=0):
        kc = {'minclock': minclock, 'reqclock': BACKGROUND_PRIORITY_CLOCK}
        x1, y1 = x + w - 1, y + h - 1
        self._w(CMD_CASET, [x >> 8, x & 0xff, x1 >> 8, x1 & 0xff], **kc)
        self._w(CMD_RASET, [y >> 8, y & 0xff, y1 >> 8, y1 & 0xff], **kc)
        self.cs.update_digital_out(0, **kc)
        self.dc.update_digital_out(0, **kc)
        self.spi.spi_send([CMD_RAMWR], **kc)
        self.dc.update_digital_out(1, **kc)
        ppc = SPI_CHUNK // 3
        chunk = list(px * ppc)
        total = w * h
        while total > 0:
            n = min(ppc, total)
            self.spi.spi_send(chunk if n == ppc else list(px * n), **kc)
            total -= n
        self.cs.update_digital_out(1, **kc)

    def _set_backlight(self, on, brightness, minclock=0, reqclock=0):
        kc = {'minclock': minclock, 'reqclock': reqclock}
        bv = (brightness & 0xff) if on else 0x00
        self._w(CMD_WRCTRLD, [CTRLD_BCTRL | CTRLD_BL], **kc)
        self._w(CMD_CABCCTRL2, [0xB1], **kc)
        self._w(CMD_WRDISBV, [bv], **kc)
        self._w(CMD_DISPON if on else CMD_DISPOFF, [], **kc)

    # --- async DMA play-out engine (runtime blits) --------------------------
    def _build_dma_cmds(self):
        self._seg_cmd = self.mcu.lookup_command(
            "display_dma_seg oid=%c flags=%u data=%*s")
        self._dma_wait_cmd = self.mcu.lookup_command("display_dma_wait oid=%c")
        # Glyph fast path. Optional: an MCU built before this command exists
        # still works, it just falls back to shipping expanded pixels.
        try:
            self._glyph_cmd = self.mcu.lookup_command(
                "display_dma_glyph oid=%c x=%u y=%u scale=%u fg=%u bg=%u"
                " bitmap=%*s")
        except Exception:
            self._glyph_cmd = None
            logging.info("ili9488: MCU has no display_dma_glyph;"
                         " using the expanded-pixel path")

    def _seg(self, cs, dc, data):
        # Enqueue one segment: CS/DC levels (applied first) + optional bytes.
        flags = (1 if cs else 0) | (2 if dc else 0)
        self._seg_cmd.send([self._dma_oid, flags, list(data)],
                           reqclock=BACKGROUND_PRIORITY_CLOCK)

    def _blit_dma(self, x, y, w, h, data):
        # CS held low across CASET/RASET/RAMWR+pixels, raised by the final seg.
        x1, y1 = x + w - 1, y + h - 1
        self._seg(0, 0, [CMD_CASET])
        self._seg(0, 1, [x >> 8, x & 0xff, x1 >> 8, x1 & 0xff])
        self._seg(0, 0, [CMD_RASET])
        self._seg(0, 1, [y >> 8, y & 0xff, y1 >> 8, y1 & 0xff])
        self._seg(0, 0, [CMD_RAMWR])
        for i in range(0, len(data), SPI_CHUNK):
            self._seg(0, 1, data[i:i + SPI_CHUNK])
        self._seg(1, 1, [])

    def _render_cell(self, cx, cy, bitmap, fg):
        sc = self.scale
        x, y = cx * 8 * sc, cy * 16 * sc
        if self._glyph_cmd is not None and sc <= GLYPH_SCALE_MAX:
            # Send the MONO bitmap (16 bytes) and let the MCU expand it. The
            # expanded cell is 8*sc*16*sc*3 bytes -- 3456 at sc=3 -- which as
            # display_dma_seg messages was 72 command-link messages AND 72 ring
            # segments per character. That flood is what the dma_cells_per_flush
            # cap exists to contain; this makes a cell 1 message / ~7 segments.
            self._glyph_cmd.send(
                [self._dma_oid, x, y, sc, _rgb(fg), _rgb(self._bg),
                 list(bitmap[:16])],
                reqclock=BACKGROUND_PRIORITY_CLOCK)
            return
        buf = bytearray()
        for row in range(16):
            rb = bitmap[row]
            srow = bytearray()
            for col in range(8):
                srow += (fg if (rb >> (7 - col)) & 1 else self._bg) * sc
            buf += srow * sc
        self._blit_dma(x, y, 8 * sc, 16 * sc, buf)

    # --- Klipper lcd_chip interface -----------------------------------------
    def init(self):
        pt = self.mcu.estimated_print_time(self.reactor.monotonic()) + 0.5
        clk = self.mcu.print_time_to_clock
        self.rst.update_digital_out(0, minclock=clk(pt)); pt += 0.020
        self.rst.update_digital_out(1, minclock=clk(pt)); pt += 0.150
        for cmd, data, delay in INIT_SEQ:
            self._w(cmd, data, minclock=clk(pt)); pt += max(delay, 0.002)
        # physical black clear so empty cells match the screen (backlight is set
        # last, so this happens dark); shown[] already all-empty -> flush() will
        # then paint only non-empty cells.
        self._fill_phys(0, 0, self.width, self.height, self._bg, minclock=clk(pt))
        pt += 0.5
        self._set_backlight(self.backlight, self.brightness, minclock=clk(pt))
        # Gate the DMA engine until this generic (blocking) init has run on the
        # MCU: otherwise the first flush's DMA blits race the init sequence on
        # SPI6 and corrupt the bus (intermittent garbage that the diff locks in).
        self._init_pt = pt + 0.3

    def get_dimensions(self):
        return (self.cols, self.rows)

    def clear(self):
        for row in self.fb:
            for cell in row:
                cell[:] = EMPTY
        for y in range(self.rows):
            row = self.fgmap[y]
            for x in range(self.cols):
                row[x] = C_WHITE

    def set_glyphs(self, glyphs):
        for name, gd in glyphs.items():
            icon = gd.get('icon16x16')
            if icon is not None:                  # [left8x16, right8x16], row-major
                self.icons[name] = (bytearray(icon[0]), bytearray(icon[1]))

    def write_text(self, x, y, data):
        if y >= self.rows:
            return
        for i, c in enumerate(bytearray(data)):
            if x + i >= self.cols:
                break
            self.fb[y][x + i][:] = self.font[c]

    def _heater_color(self, hname):
        obj = self._extruder if hname == 'extruder' else self._heater_bed
        if obj is None:
            obj = self.printer.lookup_object(hname, None)
            if hname == 'extruder':
                self._extruder = obj
            else:
                self._heater_bed = obj
        if obj is None:
            return C_WHITE
        t = obj.get_status(self.reactor.monotonic()).get('temperature', 0.)
        return C_HOT if t >= HOT_C else C_WARM if t >= WARM_C else C_COOL

    def _glyph_color(self, name):
        if name.startswith('extruder'):
            return self._heater_color('extruder')
        if name.startswith('bed'):
            return self._heater_color('heater_bed')
        if name.startswith('fan'):
            return C_GREEN
        if name == 'feedrate':
            return C_CYAN
        if name in ('usb', 'sd'):
            return C_ACC
        return C_WHITE

    def write_glyph(self, x, y, glyph_name):
        icon = self.icons.get(glyph_name)
        if icon is not None and y < self.rows and x + 1 < self.cols:
            self.fb[y][x][:] = icon[0]
            self.fb[y][x + 1][:] = icon[1]
            col = self._glyph_color(glyph_name)
            self.fgmap[y][x] = col
            self.fgmap[y][x + 1] = col
            return 2
        ch = TextGlyphs.get(glyph_name)
        if ch is not None:
            self.write_text(x, y, ch)
            return 1
        return 0

    def write_graphics(self, x, y, data):
        if x < self.cols and y < self.rows and len(data) == 16:
            cell = self.fb[y][x]
            for i in range(16):
                cell[i] ^= data[i]

    # Loadcell Z-probe + hotend filament sensor bit-bang the HX717 CONTINUOUSLY on
    # this same F427 (both register as bulk-sensor clients at ready and never
    # detach). Stock Prusa fw avoids contention by pushing the LCD over SPI6 DMA
    # (CPU-free) while sampling the loadcell in a high-priority ISR. Klipper's
    # spi_send is programmed-I/O, so a big burst occupies the MCU dispatcher and
    # can starve the HX717's timing-critical probe read (sensor_bulk_status
    # timeout -> shutdown). Lacking MCU DMA we approximate it two ways below.
    def _homing_begin(self, hmove):
        self._probing = True       # full pause across each loadcell probe tap

    def _homing_end(self, hmove):
        self._probing = False

    def _printing(self):
        if self._idle_timeout is None:
            self._idle_timeout = self.printer.lookup_object('idle_timeout', None)
        if self._idle_timeout is None:
            return False
        return self._idle_timeout.get_status(
            self.reactor.monotonic()).get('state') == 'Printing'

    def _menu_running(self):
        # True while a [menu] is showing, so flush() can repaint it without the
        # busy cap starving it. Cached display lookup; guarded so a fault never
        # breaks rendering.
        if self._display is None:
            self._display = self.printer.lookup_object('display', None)
        d = self._display
        if d is None or getattr(d, 'menu', None) is None:
            return False
        try:
            return d.menu.is_running()
        except Exception:
            return False

    def flush(self):
        # Blits go through the DMA engine, so a flush only memcpys into the ring
        # and returns -- it no longer blocks command_task or starves the
        # loadcell. We still cap cells/flush while busy (printing or probing) so
        # one flush fits the DMA ring (no producer back-pressure -> no stall);
        # the rest drain on later flushes (flush() runs every REDRAW_TIME). Idle
        # is uncapped so menu navigation repaints in one pass.
        # Hold off until the generic init sequence has run on the MCU, else the
        # first DMA blit races it on SPI6 -> locked-in garbage.
        if self._init_pt and self.mcu.estimated_print_time(
                self.reactor.monotonic()) < self._init_pt:
            return
        # Idle -> uncapped (snappy menu nav). Loadcell probe tap -> always the
        # protective dma_cells cap (the critical window). Printing with a menu
        # open -> raised menu_cells cap so the menu repaints instead of being
        # starved by the status-row churn. Printing, no menu -> dma_cells.
        if not (self._probing or self._printing()):
            cap = None
        elif self._probing:
            cap = self.dma_cells
        elif self._menu_running():
            cap = self.menu_cells
        else:
            cap = self.dma_cells
        rendered = 0
        for y in range(self.rows):
            for x in range(self.cols):
                cell = self.fb[y][x]
                col = self.fgmap[y][x]
                if cell != self.shown[y][x] or col != self.shown_col[y][x]:
                    self._render_cell(x, y, cell, col)
                    self.shown[y][x][:] = cell
                    self.shown_col[y][x] = col
                    rendered += 1
                    if cap is not None and rendered >= cap:
                        return

    # --- backlight idle-dim / wake-on-click ---------------------------------
    def _apply_backlight(self, on, br):
        # Backlight uses the generic SPI path; drain the DMA engine first so it
        # never collides with an in-flight flush on SPI6.
        if self._dma_wait_cmd is not None:
            self._dma_wait_cmd.send([self._dma_oid])
        self._set_backlight(on, br, reqclock=BACKGROUND_PRIORITY_CLOCK)

    def _hook_menu_keys(self):
        # Wrap MenuManager.key_event at the class level (the lcd_chip is built
        # BEFORE the menu, so this lands before MenuKeys captures the bound
        # method) -> every encoder/click wakes the backlight. Reaches us via
        # menu.display.lcd_chip; guarded so a fault never breaks navigation.
        from .display import menu
        mm = menu.MenuManager
        if getattr(mm, '_ili9488_wake_hooked', False):
            return
        orig = mm.key_event

        def key_event(mself, key, eventtime, _orig=orig):
            try:
                mself.display.lcd_chip._note_activity(eventtime)
            except Exception:
                pass
            return _orig(mself, key, eventtime)
        mm.key_event = key_event
        mm._ili9488_wake_hooked = True

    def _note_activity(self, eventtime):
        self._last_activity = eventtime
        if self._bl_dimmed:                      # wake immediately on a click/turn
            self._bl_dimmed = False
            self._force_repaint()                # self-heal any stale cells on wake
            self._apply_backlight(self.backlight, self.brightness)

    def _force_repaint(self):
        # Mark every cell dirty so the next flush re-renders the whole screen
        # (recovers from any one-off render glitch the cell-diff would lock in).
        for row in self.shown:
            for cell in row:
                cell[:] = b'\xff' * 16

    def _bl_ready(self):
        now = self.reactor.monotonic()
        self._last_activity = now
        self._bl_timer = self.reactor.register_timer(
            self._bl_check, now + self.bl_timeout)

    def _bl_check(self, eventtime):
        if self._bl_dimmed:
            if self._printing():                 # a print started -> wake to show it
                self._bl_dimmed = False
                self._force_repaint()
                self._apply_backlight(self.backlight, self.brightness)
        elif not self._printing() \
                and eventtime - self._last_activity >= self.bl_timeout:
            self._bl_dimmed = True
            self._apply_backlight(self.idle_brightness > 0, self.idle_brightness)
        return eventtime + 2.

    def cmd_DUMP_DISPLAY_FB(self, gcmd):
        # '.' empty, 'o' partial (text/icon), '#' mostly-filled cell.
        for y in range(self.rows):
            line = ''
            for x in range(self.cols):
                s = sum(bin(b).count('1') for b in self.fb[y][x])
                line += '.' if s == 0 else ('#' if s > 96 else 'o')
            gcmd.respond_info("r%d |%s|" % (y, line))

    def cmd_SET_MENU_CELLS(self, gcmd):
        self.menu_cells = gcmd.get_int('CELLS', self.menu_cells, minval=1)
        gcmd.respond_info("menu_dma_cells=%d" % (self.menu_cells,))

    # --- gcode --------------------------------------------------------------
    def cmd_SET_DISPLAY_BACKLIGHT(self, gcmd):
        on = gcmd.get_int('ON', 1, minval=0, maxval=1)
        br = gcmd.get_int('BRIGHTNESS', self.brightness, minval=0, maxval=255)
        self.brightness = br
        self._bl_dimmed = False                  # manual control wins
        self._last_activity = self.reactor.monotonic()
        self._apply_backlight(on, br)
        gcmd.respond_info("display backlight %s (brightness %d)"
                          % ("ON" if on else "OFF", br))


def load_config(config):
    return ILI9488(config)
