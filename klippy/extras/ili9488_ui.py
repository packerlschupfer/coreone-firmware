# Graphical status screen and menu for the ILI9488 xLCD
#
# Draws with two MCU primitives of src/stm32/display_dma.c: solid rectangle
# fills (display_dma_fill) and anti-aliased text (display_dma_aa plus run-length
# coded coverage in display_dma_aa_data). The screen is a set of widgets; each
# flush builds the current widgets and queues draws for the ones that changed.
# The queue is drained under a byte budget while printing or probing, so a full
# repaint is spread over several flushes and never floods the DMA ring.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import base64, collections, logging
from .display import font_aa

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000
AA_DATA_MAX = 48     # bytes of run-length data per display_dma_aa_data
FILL_BAND = 8000     # pixels per fill command while the budget applies

def color(rgb):
    # 0xRRGGBB -> the u32 the MCU commands carry; the panel is BGR ordered
    r, g, b = (rgb >> 16) & 0xfc, (rgb >> 8) & 0xfc, rgb & 0xfc
    return (b << 16) | (g << 8) | r

BG = color(0x101418)
PANEL = color(0x232a31)
HEADER = color(0x1a2026)
TRACK = color(0x3a434c)
TEXT = color(0xeef1f4)
DIM = color(0x8d99a6)
ORANGE = color(0xfa6831)
GREEN = color(0x3fbf60)
BLUE = color(0x4a9ff5)
RED = color(0xe5484d)
AMBER = color(0xf5a524)
GREY = color(0x5c6773)

HEADER_H = 44
MENU_ROW_H = 46
# Background between the status screen widgets: x, y, w, h
STATUS_GAPS = [(0, 44, 480, 8), (0, 52, 10, 120), (158, 52, 8, 120),
               (314, 52, 8, 120), (470, 52, 10, 120), (0, 172, 480, 8),
               (0, 180, 10, 80), (470, 180, 10, 80), (0, 260, 480, 8),
               (0, 268, 10, 44), (470, 268, 10, 44), (0, 312, 480, 8)]
DRAIN_TIME = 0.050   # seconds between queue drains while a budget applies

######################################################################
# Fonts
######################################################################

class Font:
    def __init__(self, height, ascent, chars, widths, blob):
        self.height = height
        self.ascent = ascent
        data = base64.b64decode(blob)
        self.glyphs = {}
        pos = 0
        for ch, w in zip(chars, bytearray(widths)):
            size = ((w + 1) // 2) * height
            raw = data[pos:pos + size]
            pos += size
            rows = []
            stride = (w + 1) // 2
            for y in range(height):
                row = bytearray()
                for b in raw[y * stride:(y + 1) * stride]:
                    row.append(b >> 4)
                    row.append(b & 0x0f)
                rows.append(bytes(row[:w]))
            self.glyphs[ch] = (w, rows)
        self.fallback = self.glyphs.get('?', self.glyphs[' '])
    def glyph(self, ch):
        return self.glyphs.get(ch, self.fallback)
    def width(self, text):
        return sum(self.glyph(ch)[0] for ch in text)
    def fit(self, text, width):
        # Shorten text to fit width, marking the cut with ".."
        if self.width(text) <= width:
            return text
        while text and self.width(text + "..") > width:
            text = text[:-1]
        return text.rstrip() + ".."

_fonts = {}
def get_font(name):
    if name not in _fonts:
        _fonts[name] = Font(*font_aa.FONTS[name])
    return _fonts[name]

def render_coverage(font, text, width, height, align='left', pad=0):
    # Coverage levels (0-15) of a width x height band holding the text
    rows = [bytearray(width) for _ in range(height)]
    text = font.fit(text, width - 2 * pad)
    tw = font.width(text)
    if align == 'center':
        x = (width - tw) // 2
    elif align == 'right':
        x = width - pad - tw
    else:
        x = pad
    top = (height - font.height) // 2
    for ch in text:
        w, grows = font.glyph(ch)
        x0, x1 = max(x, 0), min(x + w, width)
        if x1 > x0:
            for gy, grow in enumerate(grows):
                y = top + gy
                if 0 <= y < height:
                    rows[y][x0:x1] = grow[x0 - x:x1 - x]
        x += w
    return rows

def rle_encode(rows):
    # Run-length code coverage: (level << 4) | (count - 1), up to 16 a run
    out = bytearray()
    level, count = -1, 0
    for row in rows:
        for v in row:
            if v == level and count < 16:
                count += 1
                continue
            if count:
                out.append((level << 4) | (count - 1))
            level, count = v, 1
    if count:
        out.append((level << 4) | (count - 1))
    return bytes(out)

######################################################################
# Drawing
######################################################################

class Painter:
    def __init__(self, lcd):
        self.lcd = lcd
        self.fill_cmd = self.aa_cmd = self.aa_data_cmd = None
    def build(self, mcu):
        # Returns False on an MCU without the graphical primitives
        try:
            self.fill_cmd = mcu.lookup_command(
                "display_dma_fill oid=%c x=%u y=%u w=%u h=%u color=%u")
            self.aa_cmd = mcu.lookup_command(
                "display_dma_aa oid=%c x=%u y=%u w=%u h=%u fg=%u bg=%u")
            self.aa_data_cmd = mcu.lookup_command(
                "display_dma_aa_data oid=%c data=%*s")
        except Exception as e:
            logging.info("ili9488: MCU has no graphical primitives (%s);"
                         " using the text UI", e)
            return False
        return True
    # Each op is (cost in panel bytes, callable)
    def fill_ops(self, x, y, w, h, col):
        ops = []
        band = max(1, FILL_BAND // max(w, 1))
        for by in range(y, y + h, band):
            bh = min(band, y + h - by)
            ops.append((w * bh * 3, (self._fill, (x, by, w, bh, col))))
        return ops
    def text_ops(self, x, y, w, h, text, font, fg, bg, align='left', pad=0):
        rows = render_coverage(get_font(font), text, w, h, align, pad)
        data = rle_encode(rows)
        return [(w * h * 3, (self._aa, (x, y, w, h, fg, bg, data)))]
    def _fill(self, x, y, w, h, col):
        self.fill_cmd.send([self.lcd._dma_oid, x, y, w, h, col],
                           reqclock=BACKGROUND_PRIORITY_CLOCK)
    def _aa(self, x, y, w, h, fg, bg, data):
        oid = self.lcd._dma_oid
        self.aa_cmd.send([oid, x, y, w, h, fg, bg],
                         reqclock=BACKGROUND_PRIORITY_CLOCK)
        for i in range(0, len(data), AA_DATA_MAX):
            self.aa_data_cmd.send([oid, list(data[i:i + AA_DATA_MAX])],
                                  reqclock=BACKGROUND_PRIORITY_CLOCK)

class Widgets:
    """Collects the widgets of one frame: key -> (signature, ops builder)"""
    def __init__(self, painter):
        self.p = painter
        self.items = collections.OrderedDict()
    def fill(self, key, x, y, w, h, col):
        self.items[key] = ((x, y, w, h, col),
                           lambda: self.p.fill_ops(x, y, w, h, col))
    def text(self, key, x, y, w, h, text, font='normal', fg=TEXT, bg=BG,
             align='left', pad=0):
        self.items[key] = ((x, y, w, h, text, font, fg, bg, align, pad),
                           lambda: self.p.text_ops(x, y, w, h, text, font,
                                                   fg, bg, align, pad))

######################################################################
# Screens
######################################################################

def fmt_temp(t):
    return "%d\xb0" % (int(round(t)),)

def fmt_time(secs):
    secs = int(max(secs, 0.))
    h, m = secs // 3600, (secs // 60) % 60
    if h:
        return "%dh %02dm" % (h, m)
    return "%dm %02ds" % (m, secs % 60)

class GraphicUI:
    def __init__(self, lcd, config):
        self.lcd = lcd
        self.printer = lcd.printer
        self.reactor = self.printer.get_reactor()
        self.painter = Painter(lcd)
        self.enabled = False
        self.chamber_name = config.get('chamber_sensor', None)
        self.title = config.get('status_title', 'CORE One L')
        self.width, self.height = lcd.width, lcd.height
        self.shown = {}                            # key -> drawn signature
        self.pending = collections.OrderedDict()   # key -> [ops]
        self.layout = None
        self.buttons = []                          # (x, y, w, h, action)
        self.menu_rows = 0
        self.budget = None
        self.drain_timer = None
    def build(self, mcu):
        self.enabled = self.painter.build(mcu)
        return self.enabled
    def invalidate(self):
        # Repaint everything on the next flush
        self.shown = {}
    # Printer state
    def _lookup(self, name):
        return self.printer.lookup_object(name, None)
    def _heater(self, name, eventtime):
        pheaters = self._lookup('heaters')
        obj = None
        if pheaters is not None:
            try:
                obj = pheaters.lookup_heater(name)
            except self.printer.config_error:
                obj = None
            except Exception:
                obj = None
        if obj is None:
            obj = self._lookup(name)
        if obj is None:
            return None
        st = obj.get_status(eventtime)
        return st.get('temperature', 0.), st.get('target', None)
    def _chamber(self, eventtime):
        names = [self.chamber_name] if self.chamber_name else [
            'heater_generic chamber', 'temperature_sensor chamber',
            'temperature_fan chamber']
        for name in names:
            obj = self._lookup(name)
            if obj is not None:
                st = obj.get_status(eventtime)
                return st.get('temperature', 0.), st.get('target', None)
        return None
    def _job(self, eventtime):
        ps = self._lookup('print_stats')
        st = ps.get_status(eventtime) if ps is not None else {}
        ds = self._lookup('display_status')
        dst = ds.get_status(eventtime) if ds is not None else {}
        state = st.get('state', 'standby')
        if self.printer.is_shutdown():
            state = 'shutdown'
        return state, st, dst
    # Frame building
    def _status_frame(self, w, eventtime):
        state, st, dst = self._job(eventtime)
        active = state in ('printing', 'paused')
        names = {'printing': ("Printing", ORANGE), 'paused': ("Paused", AMBER),
                 'complete': ("Finished", GREEN),
                 'cancelled': ("Cancelled", GREY), 'error': ("Error", RED),
                 'shutdown': ("Shutdown", RED)}
        label, dot = names.get(state, ("Ready", GREEN))
        # The widgets tile the whole screen, so nothing needs a background
        # fill underneath; these are the gaps between them
        for i, (x, y, gw, gh) in enumerate(STATUS_GAPS):
            w.fill('gap%d' % (i,), x, y, gw, gh, BG)
        # Header
        w.fill('hdr.lead', 0, 0, 32, HEADER_H, HEADER)
        w.fill('hdr.dot', 14, 17, 10, 10, dot)
        w.text('hdr.state', 32, 0, 200, HEADER_H, label, 'bold', TEXT, HEADER)
        fan = self._lookup('fan')
        right = self.title
        if fan is not None:
            speed = fan.get_status(eventtime).get('speed', 0.)
            right = "Fan %d%%" % (int(round(speed * 100.)),)
        w.text('hdr.right', 232, 0, 248, HEADER_H, right, 'small', DIM,
               HEADER, 'right', 14)
        # Temperature tiles
        tiles = [("NOZZLE", self._heater('extruder', eventtime)),
                 ("BED", self._heater('heater_bed', eventtime)),
                 ("CHAMBER", self._chamber(eventtime))]
        for i, (name, val) in enumerate(tiles):
            self._tile(w, i, 10 + i * 156, 52, name, val)
        # Job / position panel
        px, py, pw = 10, 180, 460
        if active:
            fname = st.get('filename', '') or "Printing"
            fname = fname.split('/')[-1]
            if fname.lower().endswith('.gcode'):
                fname = fname[:-6]
            w.text('job.name', px, py, pw, 38, fname, 'normal', TEXT, PANEL,
                   'left', 12)
            prog = min(max(dst.get('progress', 0.) or 0., 0.), 1.)
            bx, by, bar_w = px + 12, py + 41, pw - 24
            done = int(bar_w * prog)
            barcol = AMBER if state == 'paused' else ORANGE
            w.fill('job.l', px, py + 38, 12, 16, PANEL)
            w.fill('job.r', bx + bar_w, py + 38, 12, 16, PANEL)
            w.fill('job.t', bx, py + 38, bar_w, 3, PANEL)
            w.fill('job.b', bx, py + 51, bar_w, 3, PANEL)
            w.fill('job.bar', bx, by, done, 10, barcol)
            w.fill('job.track', bx + done, by, bar_w - done, 10, TRACK)
            elapsed = st.get('print_duration', 0.)
            left = ""
            if prog > 0.01 and elapsed > 0.:
                left = "  \xb7  %s left" % (
                    fmt_time(elapsed / prog - elapsed),)
            info = "%d%%  \xb7  %s%s" % (int(prog * 100.), fmt_time(elapsed),
                                         left)
            w.text('job.info', px, py + 54, pw, 26, info, 'small', DIM, PANEL,
                   'left', 12)
        else:
            th = self._lookup('toolhead')
            pos = homed = None
            if th is not None:
                tst = th.get_status(eventtime)
                pos, homed = tst.get('position'), tst.get('homed_axes', '')
            if pos is not None:
                txt = "X %.1f   Y %.1f   Z %.2f" % (pos[0], pos[1], pos[2])
            else:
                txt = ""
            w.text('pos.xyz', px, py, pw, 40, txt, 'normal', TEXT, PANEL,
                   'left', 12)
            msg = dst.get('message') or (
                "Homed" if homed == 'xyz' else "Not homed")
            w.text('pos.msg', px, py + 40, pw, 40, msg, 'small', DIM, PANEL,
                   'left', 12)
        # Buttons
        self.buttons = []
        if active:
            act = 'resume' if state == 'paused' else 'pause'
            self._button(w, 'btn.menu', 10, 268, 226, 44, "Menu", 'menu',
                         PANEL)
            w.fill('btn.gap', 236, 268, 8, 44, BG)
            self._button(w, 'btn.job', 244, 268, 226, 44,
                         act.capitalize(), act,
                         GREEN if act == 'resume' else AMBER)
        else:
            self._button(w, 'btn.menu', 10, 268, 460, 44, "Menu", 'menu',
                         PANEL)
    def _tile(self, w, i, x, y, name, val):
        tw, th = 148, 120
        if val is None:
            cur, target, col = "--", "No sensor", GREY
        else:
            t, tgt = val
            cur = fmt_temp(t)
            if tgt is None:
                target, col = "", BLUE if t < 50. else ORANGE
            elif tgt > 0.:
                target = "Target %s" % (fmt_temp(tgt),)
                col = GREEN if abs(t - tgt) < 3. else ORANGE
            else:
                target = "Off"
                col = RED if t >= 50. else BLUE
        k = 'tile%d.' % (i,)
        w.fill(k + 'accent', x, y, tw, 4, col)
        w.text(k + 'name', x, y + 4, tw, 26, name, 'small', DIM, PANEL,
               'center')
        w.text(k + 'temp', x, y + 30, tw, 50, cur, 'large', TEXT, PANEL,
               'center')
        w.text(k + 'target', x, y + 80, tw, th - 80, target, 'small', col,
               PANEL, 'center')
    def _button(self, w, key, x, y, bw, bh, label, action, col):
        w.text(key, x, y, bw, bh, label, 'bold', TEXT, col, 'center')
        self.buttons.append((x, y, bw, bh, action))
    def _menu_frame(self, w, eventtime):
        lcd = self.lcd
        menu = lcd._display.menu
        title = "Menu"
        try:
            container = menu.stack_peek()
            title = str(container.render_name()) or title
            depth = menu.stack_size()
        except Exception:
            depth = 1
        back = "<  " if depth > 1 else ""
        w.text('menu.title', 0, 0, self.width, HEADER_H, back + title, 'bold',
               TEXT, HEADER, 'left', 14)
        self.menu_rows = lcd.rows
        for row in range(lcd.rows):
            chars = lcd.chars[row]
            prefix = chr(chars[0]) if chars[0] else ' '
            sub = chars[-1] == ord('>')
            body = bytes(c or 0x20 for c in chars[1:-1 if sub else None])
            text = body.decode('utf-8', 'replace').replace('\xf8', '\xb0')
            text = text.replace('\x1a', '>').strip()
            y = HEADER_H + row * MENU_ROW_H
            if prefix == '*':
                bg, fg = BLUE, TEXT
            elif prefix != ' ' and text:
                bg, fg = ORANGE, TEXT
            else:
                bg, fg = BG, TEXT
            k = 'menu.row%d' % (row,)
            w.text(k, 0, y, self.width - 40, MENU_ROW_H - 2, text, 'normal',
                   fg, bg, 'left', 16)
            w.text(k + '.sub', self.width - 40, y, 40, MENU_ROW_H - 2,
                   ">" if sub else "", 'bold', fg if bg != BG else DIM, bg,
                   'center')
            w.fill(k + '.sep', 0, y + MENU_ROW_H - 2, self.width, 2, PANEL)
    # Touch
    def touch_target(self, x, y):
        if self.layout == 'menu':
            if y < HEADER_H:
                return ('back', None)
            return ('row', (y - HEADER_H) // MENU_ROW_H)
        if self.layout is None:
            return None
        for bx, by, bw, bh, action in self.buttons:
            if bx <= x < bx + bw and by <= y < by + bh:
                return ('button', action)
        return ('button', 'menu')
    # Flushing
    def flush(self, budget, menu_running, eventtime):
        layout = 'menu' if menu_running else 'status'
        w = Widgets(self.painter)
        try:
            if layout == 'menu':
                self._menu_frame(w, eventtime)
            else:
                self._status_frame(w, eventtime)
        except Exception:
            logging.exception("ili9488: building the screen failed")
            return
        self.layout = layout
        # Widgets no longer on screen are forgotten, so they are drawn again
        # when they come back
        for key in list(self.shown):
            if key not in w.items:
                del self.shown[key]
                self.pending.pop(key, None)
        for key, (sig, build) in w.items.items():
            if self.shown.get(key) != sig:
                self.shown[key] = sig
                self.pending[key] = build()
        self.budget = budget
        self._drain(budget)
        if self.pending and budget is not None:
            # Keep drawing between display updates, a budget at a time
            if self.drain_timer is None:
                self.drain_timer = self.reactor.register_timer(
                    self._drain_event)
            self.reactor.update_timer(self.drain_timer,
                                      eventtime + DRAIN_TIME)
    def _drain_event(self, eventtime):
        if self.lcd._probing or not self.pending:
            return self.reactor.NEVER
        self._drain(self.budget)
        if not self.pending:
            return self.reactor.NEVER
        return eventtime + DRAIN_TIME
    def _drain(self, budget):
        spent = 0
        while self.pending:
            key, ops = next(iter(self.pending.items()))
            while ops:
                cost, (fn, args) = ops[0]
                if budget is not None and spent and spent + cost > budget:
                    return
                fn(*args)
                spent += cost
                ops.pop(0)
            del self.pending[key]
