#!/usr/bin/env python3
"""Offline reproduction: ili9488_display panel-health vs. the REAL klippy reactor.

    python3 ready_pause_repro.py <path/to/ili9488_display.py> [klipper_dir]

REAL (imported unmodified from the Klipper tree):
    klippy/reactor.py            reactor, greenlets, ReactorCompletion
    klippy/klippy.py             Printer._connect (the klippy:ready loop) + Printer.run
    klippy/serialhdl.py          SerialReader._bg_thread / raw_send_wait_ack / SerialRetryCommand
    klippy/mcu.py                MCU.lookup_query_command -> CommandQueryWrapper
    klippy/msgproto.py           real encode/parse of the display_dma_* messages
    klippy/gcode.py              DISPLAY_HEALTH is run through the real dispatcher
    the module under test        its real _hq_ready/_panel_read/_engine_counters/_hq_event/
                                 cmd_DISPLAY_HEALTH (only __init__ is bypassed: it needs SPI config)
FAKE:
    the C serialqueue only. serialqueue_send() hands the command to a simulated MCU,
    which answers on the reader thread exactly like the wire does: first the response
    message, then the ack notification for the command.
MODELLED (affects only the three "panel model" checks, never the crash checks):
    the panel answers 08 00 00 until init() has finished and 9c 00 00 afterwards.
    08 00 00 is the real at-ready reply; 9c is the datasheet expectation, UNMEASURED.
    init() is scaled from the real ~0.5 .. 1.64 s after ready down to 0.10 .. 0.45 s,
    and health_period from 30 s down to 0.3 s -- so "a poll landed inside init()" is
    reachable here but, on hardware, only with health_period < ~2 s.

Exit status 0 = every arm passed, 1 = at least one failed.
"""
import sys, os, types, json, queue, threading, logging, importlib.util, time, termios
sys.dont_write_bytecode = True          # leave no __pycache__ in the trees under test
logging.getLogger().addHandler(logging.NullHandler())

ILI_PY = os.path.abspath(sys.argv[1])
KLIPPER = os.path.abspath(sys.argv[2] if len(sys.argv) > 2
                          else os.path.expanduser('~/git/klipper'))
sys.path.insert(0, os.path.join(KLIPPER, 'klippy'))
import reactor as kreactor, klippy as kklippy, serialhdl, mcu as kmcu, msgproto

# ---- load the module under test (same stub-package trick as display_health_test.py)
EXTRAS = os.path.dirname(ILI_PY)
pkg = types.ModuleType('ilipkg'); pkg.__path__ = [EXTRAS]; sys.modules['ilipkg'] = pkg
m = types.ModuleType('ilipkg.bus'); m.MCU_SPI = lambda *a, **k: None
sys.modules['ilipkg.bus'] = m
d = types.ModuleType('ilipkg.display'); d.__path__ = []; sys.modules['ilipkg.display'] = d
f = types.ModuleType('ilipkg.display.font8x14'); f.VGA_FONT = [b'\x00' * 16] * 256
sys.modules['ilipkg.display.font8x14'] = f
spec = importlib.util.spec_from_file_location('ilipkg.ili9488_display', ILI_PY)
mod = importlib.util.module_from_spec(spec)
sys.modules['ilipkg.ili9488_display'] = mod
spec.loader.exec_module(mod)
CLS = mod.ILI9488

RD_CMD = "display_dma_read oid=%c cmd=%c nbytes=%c"
RD_RSP = "display_dma_rdata oid=%c busy=%c n=%c data=%*s"
HQ_CMD = "display_dma_query oid=%c"
HQ_RSP = ("display_dma_state oid=%c count=%hu head=%hu tail=%hu running=%c tc=%u"
          " dmaerr=%u pushes=%u maxcount=%hu last_tc=%u now=%u")
OID = 17


class FakeSerialqueue(object):
    """Stands in for the C serialqueue + the MCU behind it."""
    def __init__(self, real_lib, ser, new_fw=True):
        self._real, self._ser = real_lib, ser
        self._q = queue.Queue()
        # PANEL MODEL. 08 00 00 is what the real panel returned to the at-ready read.
        # 9c 00 00 after init() is the DATASHEET expectation for RDDPM once SLPOUT +
        # DISPON have run (0x08 is its reset default) -- NOT measured on this hardware.
        self.pre_init = [0x08, 0x00, 0x00]
        self.post_init = [0x9c, 0x00, 0x00]
        self.wedged = False
        self.init_window = None             # (start, end) monotonic: init() playing on SPI6
        self.reads = 0                      # display_dma_read commands that reached the MCU
        self.reads_in_init = 0              # ...of which landed in the middle of init()
        self.queries = 0
    def _panel(self, now):
        if self.wedged:
            return [0x00, 0x00, 0x00]
        if self.init_window is None or now < self.init_window[1]:
            return self.pre_init
        return self.post_init
    def __getattr__(self, name):            # commandqueue alloc/free etc: the real thing
        return getattr(self._real, name)
    def set_thread_name(self, name):
        pass
    def serialqueue_send(self, sq, cq, cmd, clen, minclock, reqclock, nid):
        mp = self._ser.msgparser
        cmd = list(cmd)
        msgid, _ = mp.msgid_parser.parse(cmd, 0)
        mid = mp.messages_by_id[msgid]
        params, _ = mid.parse(cmd, 0)
        now = self._ser.reactor.monotonic()
        rsp = None
        if mid.name == 'display_dma_read':
            self.reads += 1
            if self.init_window and self.init_window[0] <= now < self.init_window[1]:
                self.reads_in_init += 1
            data = self._panel(now)[:params['nbytes']]
            rsp = mp.lookup_command(RD_RSP).encode([params['oid'], 0, len(data), data])
        elif mid.name == 'display_dma_query':
            self.queries += 1
            rsp = mp.lookup_command(HQ_RSP).encode(
                [params['oid'], 0, 1, 1, 0, 10, 0, 5, 3, 500, 1000])
        def deliver():                      # ~2 ms of "wire": response, then the ack
            if rsp is not None:
                self._q.put(([0, 0] + list(rsp) + [0, 0, msgproto.MESSAGE_SYNC], 0, now))
            if nid:
                self._q.put(([], nid, now))
        threading.Timer(0.002, deliver).start()
    def serialqueue_pull(self, sq, response):
        item = self._q.get()
        if item is None:
            response.len = -1
            return
        msg, nid, t = item
        for i, b in enumerate(msg):
            response.msg[i] = b
        response.len = len(msg)
        response.notify_id = nid
        response.sent_time = response.receive_time = t
    def stop(self):
        self._q.put(None)


class Cap(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.recs = []
        self.times = []
    def emit(self, r):
        msg = r.getMessage()
        if r.exc_info:
            import traceback
            msg += "\n" + "".join(traceback.format_exception(*r.exc_info))
        self.recs.append((r.levelno, msg))
        self.times.append((r.created, msg))
    def has(self, s):
        return [mm for _, mm in self.recs if s in mm]


def build(new_fw=True):
    """A real reactor + real Printer + real SerialReader, wired to the fake MCU."""
    r = kreactor.Reactor()
    created = []                                     # (completion, who created it)
    _orig_completion = r.completion
    def completion():
        c = _orig_completion()
        created.append((c, sys._getframe(1).f_code.co_name))
        return c
    r.completion = completion                        # observation only
    r._created = created
    pty_m, pty_s = os.openpty()                      # GCodeIO wants a gcode fd, as klippy.py gives it
    a = termios.tcgetattr(pty_s); a[3] &= ~termios.ECHO
    termios.tcsetattr(pty_s, termios.TCSANOW, a)
    printer = kklippy.Printer(r, None, {'gcode_fd': pty_m})
    printer._read_config = lambda: None              # no printer.cfg in this harness
    ser = serialhdl.SerialReader(r, "")
    ident = {"commands": {}, "responses": {}, "config": {"CLOCK_FREQ": 168000000},
             "version": "repro", "build_versions": ""}
    if new_fw:
        ident["commands"] = {RD_CMD: 70, HQ_CMD: 71}
        ident["responses"] = {RD_RSP: 72, HQ_RSP: 73}
    ser.msgparser.process_identify(json.dumps(ident).encode(), decompress=False)
    fake = FakeSerialqueue(ser.ffi_lib, ser)
    ser.ffi_lib = fake
    ser.background_thread = threading.Thread(target=ser._bg_thread, daemon=True)
    ser.background_thread.start()
    fmcu = types.SimpleNamespace(get_printer=lambda: printer, is_fileoutput=lambda: False)
    conn = types.SimpleNamespace(get_serial=lambda: ser, get_mcu=lambda: fmcu)
    # the module calls self.mcu.lookup_query_command(...): route it through the REAL
    # MCU.lookup_query_command so the xmit helper is whatever Klipper would really pick
    mstub = types.SimpleNamespace(_conn_helper=conn)
    def lqc(msgformat, respformat, oid=None, cq=None, is_async=False):
        return kmcu.MCU.lookup_query_command(mstub, msgformat, respformat, oid, cq, is_async)
    mstub.lookup_query_command = lqc
    mstub.get_constant_float = ser.msgparser.get_constant_float
    mstub.estimated_print_time = lambda eventtime: eventtime   # print_time == monotonic here
    return r, printer, ser, fake, mstub


class Dut(CLS):
    def __init__(self, printer, mstub, period):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.mcu = mstub
        self._dma_oid = OID
        self.health_period = period
        self.health_reg = 0x0a
        self._rd_cmd = None
        self._rd_unavailable = False
        self._rd_base = None
        self._hq_cmd = None
        self._hq_last = {}
        self._panel_bad = 0
        self._panel_warned = False
        self._hq_timer = None
        self._init_pt = 0.               # as the real __init__; init() sets it at ready
        # exactly what the real __init__ registers
        printer.lookup_object('gcode').register_command(
            'DISPLAY_HEALTH', self.cmd_DISPLAY_HEALTH)
        if self.health_period > 0.:
            printer.register_event_handler('klippy:ready', self._hq_ready)


fails = []
def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm_control():
    """Is a blocking MCU query from a reactor.register_timer callback supported at all?
    (the tmc.py _do_periodic_check pattern) -- no module code involved."""
    print("\nARM 0  control: blocking query straight from a register_timer callback")
    r, printer, ser, fake, mstub = build()
    out = {}
    def tick(eventtime):
        q = mstub.lookup_query_command(RD_CMD, RD_RSP, oid=OID)
        out['helper'] = q._xmit_helper.__name__
        out['a'] = dict(q.send([OID, 0x0a, 3]))
        out['b'] = dict(q.send([OID, 0x0a, 3]))      # second query: greenlet is now non-dispatch
        r.end()
        return r.NEVER
    r.register_timer(tick, r.NOW)
    exc = None
    try:
        r.run()
    except BaseException as e:
        exc = e
    fake.stop()
    check(exc is None, "reactor survived (exception: %r)" % (exc,))
    check(out.get('helper') == 'SerialRetryCommand',
          "lookup_query_command() default helper is SerialRetryCommand, not"
          " RetryAsyncCommand (got %s)" % out.get('helper'))
    check(out.get('a', {}).get('data') == b'\x08\x00\x00'
          and out.get('b', {}).get('data') == b'\x08\x00\x00',
          "both replies decoded: %r" % (out.get('a', {}).get('data'),))


def boot(title, new_fw=True, connect_pauses=True, period=0.3, runtime=2.4):
    """Boot a real Printer through klippy:connect -> klippy:ready with the module
    registered the way its __init__ registers it, then run for `runtime` seconds."""
    print("\n" + title)
    r, printer, ser, fake, mstub = build(new_fw)
    cap = Cap(); root = logging.getLogger(); root.addHandler(cap)
    root.setLevel(logging.DEBUG)
    ev = {'shutdown': None, 'ready_at': None, 'health': None, 'wedge_at': None}
    if connect_pauses:
        # the real klippy:connect (mcu identify/config) pauses many times, so the
        # greenlet that later runs klippy:ready is a NON-dispatch greenlet with a timer
        printer.register_event_handler(
            'klippy:connect', lambda: r.pause(r.monotonic() + 0.01))
    dut = Dut(printer, mstub, period)
    def display_ready():
        # display.py's handle_ready -> lcd_chip.init(). Registered AFTER the chip's own
        # handler, exactly as in display.py (chip constructed :183, handler added :204).
        # Real init(): reset pulse at +0.5 s, done at ~+1.9 s. Scaled here to 0.1 .. 0.45 s.
        now = r.monotonic()
        ev['ready_at'] = now
        fake.init_window = (now + 0.1, now + 0.45)
        dut._init_pt = mstub.estimated_print_time(now) + 0.45
    printer.register_event_handler('klippy:ready', display_ready)
    printer.register_event_handler(
        'klippy:shutdown', lambda: (ev.__setitem__('shutdown', r.monotonic()),
                                    printer.request_exit('error_exit')))
    gcode = printer.lookup_object('gcode')
    def run_health(eventtime):
        try:
            gcode.run_script("DISPLAY_HEALTH")
            ev['health'] = 'ran'
        except Exception as e:
            ev['health'] = 'raised %r' % (e,)
    def wedge(eventtime):
        ev['wedge_at'] = time.time()
        fake.wedged = True                          # panel controller stops answering
    r.register_callback(run_health, r.monotonic() + runtime * 0.45)
    r.register_callback(wedge, r.monotonic() + runtime * 0.55)
    r.register_callback(lambda e: printer.request_exit('exit'), r.monotonic() + runtime)
    result = printer.run()                          # REAL klippy main loop
    fake.stop()
    root.removeHandler(cap)
    return dict(dut=dut, cap=cap, ev=ev, fake=fake, result=result, printer=printer,
                reactor=r)


def report_crash(res):
    cap, ev = res['cap'], res['ev']
    crash = cap.has("Unhandled exception during run")
    if crash:
        tb = crash[0].strip().splitlines()
        print("       klippy logged: " + tb[0])
        for ln in tb[1:]:
            if '~~' in ln or '^^' in ln:
                continue
            print("       | " + ln)
        for c, who in res['reactor']._created:
            if c.waiting:
                print("       stale waiter left in a completion created by %s():"
                      " %d greenlet(s), .timer = %r"
                      % (who, len(c.waiting), [w.timer for w in c.waiting]))
    if ev['shutdown'] is not None and ev['ready_at'] is not None:
        print("       shutdown %.1f ms after klippy:ready; display_dma_read commands that"
              " had reached the MCU: %d" % ((ev['shutdown'] - ev['ready_at']) * 1e3,
                                            res['fake'].reads))
    for s in ("panel read 0x0a failed", "read-back unavailable", "read-back baseline"):
        for mm in cap.has(s):
            print("       module logged: " + mm[:150])
    return crash


def arm_boot(title, **kw):
    res = boot(title, **kw)
    crash = report_crash(res)
    dut, cap, ev, fake = res['dut'], res['cap'], res['ev'], res['fake']
    check(not crash, "no unhandled exception escaped into the reactor")
    check(ev['shutdown'] is None and res['result'] == 'exit',
          "klippy did NOT shut down (run result %r)" % (res['result'],))
    check(not cap.has("reactor pause disabled"),
          "nothing tried to pause inside the no-pause klippy:ready section")
    return res, crash


def arm_new_fw(title, **kw):
    res, crash = arm_boot(title, **kw)
    dut, cap, ev, fake = res['dut'], res['cap'], res['ev'], res['fake']
    check(dut._rd_base is not None and not dut._rd_unavailable,
          "a baseline was established and the check is ARMED"
          " (base=%r unavailable=%r)" % (dut._rd_base, dut._rd_unavailable))
    check(dut._rd_base == bytes(bytearray(fake.post_init)),
          "the baseline is the INITIALISED panel's value, not the pre-init one"
          " (base=%r)   [panel model: see FakeSerialqueue]" % (dut._rd_base,))
    check(fake.reads_in_init == 0,
          "no read was clocked onto SPI6 while init() was playing out (%d)"
          % fake.reads_in_init)
    early = [mm for t, mm in cap.times
             if "PANEL LIVENESS CHANGED" in mm and (ev['wedge_at'] is None or t < ev['wedge_at'])]
    check(not early, "no liveness warning BEFORE the simulated wedge (%d false alarm)"
          % len(early))
    check(fake.reads >= 4, "the periodic poll really queried the MCU repeatedly"
          " (%d reads)" % fake.reads)
    check(ev['health'] == 'ran', "DISPLAY_HEALTH ran through the real gcode dispatcher"
          " (%s)" % ev['health'])
    check(len(cap.has("PANEL LIVENESS CHANGED")) == 1,
          "the simulated wedge produced exactly one warning (%d)"
          % len(cap.has("PANEL LIVENESS CHANGED")))


def arm_old_fw(title):
    res, crash = arm_boot(title, new_fw=False)
    dut, cap, ev, fake = res['dut'], res['cap'], res['ev'], res['fake']
    check(dut._rd_cmd is False and dut._rd_unavailable and fake.reads == 0,
          "firmware without display_dma_read: check disabled, nothing sent")
    check(bool(cap.has("MCU has no display_dma_read")), "and it said so in the log")
    check(ev['health'] == 'ran', "DISPLAY_HEALTH still ran (%s)" % ev['health'])


print("module under test: %s" % ILI_PY)
print("klipper tree:      %s" % KLIPPER)
arm_control()
arm_new_fw("ARM 1  boot, connect phase paused (as the real MCU connect does)")
arm_new_fw("ARM 2  boot, connect phase never paused (ready runs on the dispatch greenlet)",
           connect_pauses=False)
arm_old_fw("ARM 3  boot against firmware WITHOUT the new commands")
print("\n%s  (%d failed checks)" % ("FAIL" if fails else "PASS", len(fails)))
sys.exit(1 if fails else 0)
