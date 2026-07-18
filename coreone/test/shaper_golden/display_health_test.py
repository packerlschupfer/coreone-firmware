"""Gate: panel liveness must be detectable -- and an unwired SDO must NOT look like a dead panel.

    python3 display_health_test.py

🚨 WHY, INCLUDING A DESIGN I GOT WRONG. The panel froze mid-print on 2026-10-25: six grey rows,
no glyphs, and NOTHING logged. My first attempt watched the MCU DMA engine for a lost
transfer-complete (count>0, running, tc frozen). An independent review killed it with arithmetic:
a changed cell costs ~78 ring segments against NSEG 512, so the 7th changed cell fills the ring,
dd_push spins 500 ms and calls shutdown("display_dma ring stall") -- within SECONDS on a live print
screen. A two-poll 30 s detector loses that race every time, AND the state it would report aborts
the print anyway. Worse: the 10-25 event logged nothing and did not shut down, so it CANNOT have
been a dead engine. It was a healthy engine streaming pixels at a panel whose controller had
stopped -- a state in which every engine counter reads perfectly normal.

So the detector is a READ-BACK from the panel's own status register, and the engine counters are
kept as diagnostic colour only.

🔑 ARM B IS THE ONE THAT MAKES THIS SAFE TO ARM. A board that does not route SDO reads all 0x00 or
all 0xFF -- INDISTINGUISHABLE from a dead panel. If the detector armed on that, it would alarm on
healthy hardware, and an alarm that fires on healthy hardware is how a real one gets ignored. The
baseline read must therefore DISARM the check rather than report a fault.

⚠ The previous version of this gate passed against eight separate mutations (reviewer-verified),
because its arms set two variables at once, never executed DISPLAY_HEALTH, and checked the MCU side
with greps that were disjunctions always true. Arms below pin one thing each, run the real command,
and compare the host/C format strings literally rather than grepping for substrings.
"""
import sys, os, types, importlib.util, logging, re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

EXTRAS = os.path.dirname(golden.PHASE_EXEC_PY)
ILI_PY = os.path.join(EXTRAS, 'ili9488_display.py')


def _find_src(name):
    """Locate src/**/<name> by SEARCHING, not by assuming a layout.

    🚨 WHY, AND THIS IS THE SECOND TIME. The first version computed
    `os.path.join(os.path.dirname(EXTRAS), 'src', name)`. In the overlay EXTRAS is
    `klipper-port/extras`, so that resolves to `klipper-port/src/display_dma.c` and
    works. In the FORK, EXTRAS is `klippy/extras`, so it computes
    `klippy/src/display_dma.c`, which does not exist -- the fork keeps MCU source at
    `src/stm32/display_dma.c`, at the repo root and nested by MCU family. The gate
    then died with FileNotFoundError at import, BEFORE A SINGLE CHECK RAN: zero
    coverage in the fork, while reporting 9 arms and 12/12 mutations killed in the
    overlay. A fork-tree run caught it; I did not.

    ⚠ This is EXACTLY the class `keepalive_margin_test._find_shaper_h` exists to
    solve, and I had already written and documented that fix. Flat-src-at-a-fixed-
    depth is the mistake; walk up and search instead. Any new gate that reaches for
    MCU source must come through here.
    """
    searched = []
    d = os.path.dirname(os.path.abspath(golden.PHASE_EXEC_PY))
    for _ in range(6):                  # extras/ -> klipper-port/ or klippy/ -> repo root
        root = os.path.join(d, 'src')
        searched.append(root)
        if os.path.isdir(root):
            for dirpath, _dirs, files in os.walk(root):
                if name in files:
                    return os.path.join(dirpath, name), searched
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return None, searched


DMA_C, DMA_C_SEARCHED = _find_src('display_dma.c')
HOST_SRC = open(ILI_PY).read()
# ⚠ FAIL, do not crash. A bare FileNotFoundError at import truncates the whole gate
# and reads as a broken harness rather than a missing file -- and run_all.sh would
# report it identically to a real assertion failure with no clue where to look.
C_SRC = open(DMA_C).read() if DMA_C else None
fails = []


def joined_literal(src, prefix):
    """The full message format, with ADJACENT STRING LITERALS JOINED.

    ⚠ Both C (`sendf(...)`) and Python write these across several lines and rely on
    adjacent-literal concatenation. A single `"([^"]*)"` capture therefore takes only
    the FIRST fragment -- which made an earlier version of this gate compare a
    truncated string against a full one and report a format mismatch that did not
    exist. It did it on the host side first, and then, after I "fixed" that, on the C
    side, because I had only generalised one of the two. Hence one helper for both.
    """
    i = src.find('"' + prefix)
    if i < 0:
        return None
    out, j = [], i
    while True:
        a = src.find('"', j)
        if a < 0:
            break
        b = src.find('"', a + 1)
        if b < 0:
            break
        out.append(src[a + 1:b])
        k = b + 1
        while k < len(src) and src[k] in ' \t\r\n':
            k += 1
        if k >= len(src) or src[k] != '"':
            break
        j = k
    return "".join(out)


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        import traceback
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))
        traceback.print_exc()


pkg = types.ModuleType('ilipkg'); pkg.__path__ = [EXTRAS]
sys.modules['ilipkg'] = pkg
m = types.ModuleType('ilipkg.bus'); m.MCU_SPI = lambda *a, **k: None
sys.modules['ilipkg.bus'] = m
d = types.ModuleType('ilipkg.display'); d.__path__ = [os.path.join(EXTRAS, 'display')]
sys.modules['ilipkg.display'] = d
f = types.ModuleType('ilipkg.display.font8x14'); f.VGA_FONT = [b'\x00' * 16] * 256
sys.modules['ilipkg.display.font8x14'] = f
spec = importlib.util.spec_from_file_location('ilipkg.ili9488_display', ILI_PY)
mod = importlib.util.module_from_spec(spec)
sys.modules['ilipkg.ili9488_display'] = mod
spec.loader.exec_module(mod)
CLS = [getattr(mod, n) for n in dir(mod)
       if isinstance(getattr(mod, n), type) and hasattr(getattr(mod, n), '_hq_event')][0]


class Reactor(object):
    NEVER = 9e99

    def __init__(self, no_pause=False):
        self.t = 1000.0
        # ⚠ THE FAKE MUST MODEL no-pause SECTIONS. The previous fake had no such
        # concept, so this gate passed 66/66 against the module that CRASHED KLIPPY
        # on the first deploy: klippy:ready handlers run inside
        # reactor.assert_no_pause(), a query command blocks, and the orphaned
        # completion waiter detonated when the ack landed. A fake that cannot
        # express the constraint cannot test it.
        self.no_pause = no_pause

    def verify_can_pause(self):
        if self.no_pause:
            raise Exception("Internal error - reactor pause disabled")

    def monotonic(self):
        return self.t

    def register_timer(self, cb, waketime=NEVER):
        return cb

    def update_timer(self, h, w):
        pass


class Q(object):
    """A bound query command. Separate object from the mcu -- an earlier harness made
    lookup_query_command return the mcu itself, which has no .send(), so every arm
    silently tested the exception path instead of the behaviour."""

    def __init__(self, replies, raise_on_lookup=None):
        self.replies = list(replies)
        self.n = 0

    def send(self, params):
        self.n += 1
        if not self.replies:
            raise AssertionError("harness: more sends than scripted replies")
        r = self.replies.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


class Mcu(object):
    def __init__(self, reads=(), engine=(), lookup_err=None):
        self.rq = Q(list(reads))
        self.eq = Q(list(engine))
        self.lookup_err = lookup_err

    def lookup_query_command(self, cmd, resp, oid=None):
        if self.lookup_err is not None:
            raise Exception(self.lookup_err)
        return self.rq if cmd.startswith('display_dma_read') else self.eq

    def get_constant_float(self, name):
        return 168e6


class Dut(CLS):
    def __init__(self, reads=(), engine=(), lookup_err=None, period=30., reg=0x0a,
                 no_pause=False):
        self.reactor = Reactor(no_pause)
        self._init_pt = 0.          # real __init__ sets this; init() advances it
        self.mcu = Mcu(reads, engine, lookup_err)
        self._dma_oid = 7
        self.health_period = period
        self.health_reg = reg
        self._rd_cmd = None
        self._rd_unavailable = False
        self._rd_base = None
        self._hq_cmd = None
        self._hq_last = {}
        self._panel_bad = 0
        self._panel_warned = False
        self._hq_last_bytes = None
        self._hq_timer = None
        self.segs = []              # DISPLAY_WEDGE_TEST's panel commands land here
        self.print_state = 'standby'
        self.printer = _FakePrinter(self)


class _FakePrinter(object):
    """Just enough for the wedge test's print-state guard."""

    def __init__(self, dut):
        self.dut = dut

    def lookup_object(self, name, default='__raise__'):
        if name == 'print_stats':
            d = self.dut
            return type('PS', (), {'get_status': staticmethod(
                lambda e: {'state': d.print_state})})()
        if default == '__raise__':
            raise Exception("no %s" % name)
        return default


class _Gcmd(object):
    class error(Exception):
        pass

    def __init__(self, **params):
        self.params = params
        self.msgs = []

    def get_int(self, name, default, minval=None, maxval=None):
        return self.params.get(name, default)

    def respond_info(self, m):
        self.msgs.append(m)


def rd(data, busy=0):
    return {'oid': 7, 'busy': busy, 'n': len(data), 'data': bytes(bytearray(data))}


def _wire_segs(d):
    d._seg = lambda cs, dc, data: d.segs.append((cs, dc, list(data)))
    d.reactor.pause = lambda t: None
    return d


def eng(tc=10, count=0, running=0, dmaerr=0, pushes=5, head=1, tail=1,
        maxcount=3, last_tc=500, now=1000):
    return {'oid': 7, 'count': count, 'head': head, 'tail': tail, 'running': running,
            'tc': tc, 'dmaerr': dmaerr, 'pushes': pushes, 'maxcount': maxcount,
            'last_tc': last_tc, 'now': now}


class Cap(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.recs = []

    def emit(self, r):
        self.recs.append((r.levelno, r.getMessage()))


def cap_run(fn):
    c = Cap(); root = logging.getLogger(); root.addHandler(c); lv = root.level
    root.setLevel(logging.DEBUG)
    try:
        out = fn()
    finally:
        root.removeHandler(c); root.setLevel(lv)
    return out, c.recs


def warns(recs):
    return [m for l, m in recs if l >= logging.WARNING]


BASE = [0x00, 0x9c, 0x00]        # a plausible RDDPM-ish reply (dummy + value)


def arm_a():
    # DETECTION: the register leaves its baseline -> exactly one warning.
    dut = Dut(reads=[rd(BASE), rd([0x00, 0x08, 0x00]), rd([0x00, 0x08, 0x00])],
              engine=[eng()])
    _, recs = cap_run(lambda: dut._hq_baseline())
    check(dut._rd_base == bytes(bytearray(BASE)), "the at-ready read sets a baseline")
    _, recs = cap_run(lambda: dut._hq_event(dut.reactor.t))
    check(dut._panel_bad == 1, "a register that no longer matches the baseline is flagged")
    w = warns(recs)
    check(len(w) == 1, "exactly one warning (got %d)" % len(w))
    if w:
        check('PANEL LIVENESS' in w[0], "the warning names panel liveness")
        check('2026-10-25' in w[0], "and ties it to the event that logged nothing")
        check('never costs a job' in w[0] or 'print is unaffected' in w[0].lower(),
              "and states the print is unaffected -- true for this design, unlike the"
              " engine-wedge version whose own warning claimed it falsely")
    # latched: a second poll must not re-warn
    _, recs2 = cap_run(lambda: dut._hq_event(dut.reactor.t))
    check(not warns(recs2), "a second poll does not repeat the warning")


def arm_b():
    # COUNTERWEIGHT: an unwired SDO must DISARM, never warn. Both all-00 and all-FF.
    for name, data in (("all 0x00", [0, 0, 0]), ("all 0xFF", [0xff, 0xff, 0xff])):
        dut = Dut(reads=[rd(data)] + [rd(data)] * 3)
        _, recs = cap_run(lambda: dut._hq_baseline())
        check(dut._rd_unavailable, "%s at baseline -> read-back marked UNAVAILABLE" % name)
        check(dut._rd_base is None, "   and no baseline is adopted")
        check(not warns(recs), "   and nothing is warned at startup")
        _, recs = cap_run(lambda: dut._hq_event(dut.reactor.t))
        check(not warns(recs), "   and polling stays silent forever (%s)" % name)
        check(dut._panel_bad == 0, "   and never flags a fault")
    # and the predicate itself, pinned independently of the poll
    check(CLS._implausible([]) is True, "_implausible([]) is True")
    check(CLS._implausible([0, 0]) is True, "_implausible(all zero) is True")
    check(CLS._implausible([0xff, 0xff]) is True, "_implausible(all ones) is True")
    check(CLS._implausible([0x00, 0x9c]) is False,
          "_implausible(mixed) is False -- a real reading is not rejected")


def arm_c():
    # A matching baseline is silent, repeatedly.
    dut = Dut(reads=[rd(BASE)] + [rd(BASE)] * 10)
    cap_run(lambda: dut._hq_baseline())
    _, recs = cap_run(lambda: [dut._hq_event(dut.reactor.t) for _ in range(10)])
    check(not warns(recs), "10 polls matching the baseline stay silent")
    check(dut._panel_bad == 0, "and nothing is flagged")


def arm_d():
    # Engine busy -> decline, no verdict, no warning, and the poll still re-arms.
    dut = Dut(reads=[rd(BASE), rd([], busy=1)])
    cap_run(lambda: dut._hq_baseline())
    (nxt, recs) = cap_run(lambda: dut._hq_event(dut.reactor.t))
    check(not warns(recs), "a busy reply produces no warning")
    check(dut._panel_bad == 0, "and no fault flag")
    check(nxt == dut.reactor.t + dut.health_period,
          "and the poll re-arms normally (got %r)" % (nxt,))


def arm_e():
    # THE F5 BUG: one transient failure must not kill the poll for the session.
    dut = Dut(reads=[rd(BASE), Exception("mcu timeout"), rd(BASE)])
    cap_run(lambda: dut._hq_baseline())
    (nxt, _) = cap_run(lambda: dut._hq_event(dut.reactor.t))
    check(nxt != dut.reactor.NEVER,
          "a failed read RE-ARMS the poll rather than returning NEVER (got %r)" % (nxt,))
    check(dut.health_period == 30.,
          "and does not zero health_period, which would leave a dead poll looking armed")
    (nxt2, recs) = cap_run(lambda: dut._hq_event(dut.reactor.t))
    check(nxt2 != dut.reactor.NEVER and not warns(recs),
          "and the NEXT poll works again")


def arm_f():
    # THE F5b BUG: a format mismatch is drift, NOT old firmware. Filing it as the
    # latter sent an earlier version of this module looking in the wrong place.
    dut = Dut(lookup_err="Command format mismatch: display_dma_read")
    _, recs = cap_run(lambda: dut._panel_read(0x0a, 2))
    w = warns(recs)
    check(len(w) == 1, "a format mismatch is logged at WARNING, not INFO (got %d)" % len(w))
    if w:
        check('MISMATCH' in w[0].upper(), "and says MISMATCH")
        check('old firmware' in w[0].lower() or 'do not read this as old' in w[0].lower(),
              "and explicitly tells the reader it is NOT old firmware")
    dut2 = Dut(lookup_err="Unknown command 'display_dma_read'")
    _, recs2 = cap_run(lambda: dut2._panel_read(0x0a, 2))
    check(not warns(recs2), "whereas a genuinely missing command is only INFO")


def arm_g():
    # THE F4 BUG + a command the old gate never executed at all.
    class Gcmd(object):
        def __init__(self):
            self.msgs = []

        def respond_info(self, m):
            self.msgs.append(m)

    # stale latch says bad, fresh read says good -> must report GOOD
    dut = Dut(reads=[rd(BASE), rd(BASE)], engine=[eng(), eng()])
    cap_run(lambda: dut._hq_baseline())
    dut._panel_bad = 1                       # stale latch from a previous poll
    g = Gcmd()
    cap_run(lambda: dut.cmd_DISPLAY_HEALTH(g))
    check(g.msgs, "DISPLAY_HEALTH actually produced output (the old gate never ran it)")
    out = "\n".join(g.msgs)
    check('alive' in out,
          "the verdict comes from the FRESH read, not the stale _panel_bad latch")
    # fresh read says changed -> must report changed even with the latch clear
    dut2 = Dut(reads=[rd(BASE), rd([0x00, 0x08, 0x00])], engine=[eng(), eng()])
    cap_run(lambda: dut2._hq_baseline())
    g2 = Gcmd()
    cap_run(lambda: dut2.cmd_DISPLAY_HEALTH(g2))
    check('CHANGED' in "\n".join(g2.msgs),
          "and a changed register is reported even with the latch clear")
    # engine section must label itself as unable to see a dead panel
    check('DIAGNOSTIC ONLY' in out or 'cannot see a dead panel' in out,
          "the engine counters are labelled as unable to detect a dead panel")
    check('ring depth' not in out,
          "and the ring depth prints a NUMBER, not the literal string 'ring depth'")


def arm_h():
    # THE F3 BUG: last_tc/now wrap at 2^32/168e6 = 25.57 s, SHORTER than the poll.
    class Gcmd(object):
        def __init__(self):
            self.msgs = []

        def respond_info(self, m):
            self.msgs.append(m)

    wrap = (1 << 32) / 168e6
    # an hour-long gap, which the 32-bit counter cannot express
    hour = int(3600 * 168e6) & 0xffffffff
    dut = Dut(reads=[rd(BASE), rd(BASE)],
              engine=[eng(last_tc=0x1000, now=(0x1000 + hour) & 0xffffffff)] * 2)
    cap_run(lambda: dut._hq_baseline())
    g = Gcmd()
    cap_run(lambda: dut.cmd_DISPLAY_HEALTH(g))
    out = "\n".join(g.msgs)
    # 🔑 A WRAPPED DELTA *IS* A SMALL NUMBER. My first fix compared the delta to the
    # wrap period and printed a precise figure when it looked small -- so this
    # one-hour gap printed a confident "20.86 s". No arithmetic recovers the truth
    # from one sample, so the output must STATE THE AMBIGUITY rather than bound it.
    check('AMBIGUOUS' in out,
          "an hour-long gap is reported as AMBIGUOUS, not as a confident figure")
    check('%.2f' % wrap in out or '25.5' in out,
          "and the wrap period is named so the reader knows the modulus")
    check('do not read it as a stall duration' in out,
          "and the reader is told not to use it as a stall duration")
    import re as _re
    bare = _re.search(r'completion: (\d+\.\d+) s\s*$', out, _re.M)
    check(not bare,
          "and no bare '<n> s' age is printed anywhere (got %r)"
          % (bare.group(1) if bare else None))


def arm_i():
    # MCU SIDE, checked structurally rather than by substring greps that are always true.
    check(C_SRC is not None,
          "display_dma.c was FOUND (searched: %s) -- if this fails the whole MCU half of"
          " this gate is inert, which is how it ran overlay-only while reporting 12/12"
          % (", ".join(DMA_C_SEARCHED),))
    if C_SRC is None:
        return
    body = C_SRC.split('command_display_dma_read(uint32_t *args)', 1)
    check(len(body) == 2, "display_dma.c defines command_display_dma_read")
    if len(body) == 2:
        b = body[1].split('\nDECL_COMMAND', 1)[0]
        # ⚠ strip // comments first: an earlier version of this check matched the
        # word "while" inside a comment and reported a spin that was not there.
        code = "\n".join(l.split('//')[0] for l in b.splitlines())
        check('while (' not in code and 'while(' not in code,
              "the read NEVER spins -- reusing display_dma_wait would let a cosmetic"
              " health check shutdown() the MCU and abort a print")
        check('shutdown' not in b, "and cannot shutdown()")
        check('d->count || d->running' in b or 'd->running || d->count' in b,
              "it declines while the engine is busy (both terms, so a mid-flush read"
              " cannot collide with the DMA engine on the same peripheral)")
        # ⚠ All of these run on `code`, NOT on `b`. Commenting a call out leaves its
        # text in the file, so an un-stripped check cannot see it removed -- verified
        # by mutation: `//spi_prepare(d->fast);` survived the un-stripped version.
        check('spi_prepare(d->slow)' in code
              and code.index('spi_prepare(d->slow)') < code.index('spi_transfer'),
              "the READ CLOCK is selected before any transfer (42 MHz writes exceed the"
              " ILI9488's ~6.6 MHz read max -- a read at write speed returns garbage)")
        check('spi_prepare(d->fast)' in code
              and code.rindex('spi_prepare(d->fast)') > code.rindex('spi_transfer'),
              "and the WRITE clock is restored after, or the engine inherits 4 MHz")
    # the slow config must actually be a different, lower rate
    mrate = re.search(r'#define DD_READ_RATE\s+(\d+)', C_SRC)
    check(mrate is not None, "DD_READ_RATE is defined")
    if mrate:
        check(int(mrate.group(1)) <= 6600000,
              "and is <= 6.6 MHz (%s)" % mrate.group(1))
    # FORMAT STRINGS MUST MATCH LITERALLY -- the old gate grepped, so swapping two
    # sendf arguments passed. Compare the C format against the host's expectation.
    if C_SRC is None:
        return
    cr = joined_literal(C_SRC, 'display_dma_rdata')
    hr = joined_literal(HOST_SRC, 'display_dma_rdata')
    check(cr is not None and hr is not None, "both sides name display_dma_rdata")
    if cr and hr:
        check(" ".join(cr.split()) == " ".join(hr.split()),
              "the C sendf format and the host response format are IDENTICAL\n"
              "        C   : %s\n        host: %s" % (cr, hr))
    # ⚠ The host writes these as MULTI-LINE implicit concatenations, so a single
    # "([^"]*)" capture takes only the first fragment -- an earlier version of this
    # check compared a truncated host string and reported a mismatch that was not
    # there. Join adjacent literals before comparing.
    cs = joined_literal(C_SRC, 'display_dma_state')
    hs = joined_literal(HOST_SRC, 'display_dma_state')
    check(cs is not None and hs is not None, "both sides name display_dma_state")
    if cs and hs:
        check(" ".join(cs.split()) == " ".join(hs.split()),
              "and so are the display_dma_state formats\n"
              "        C   : %s\n        host: %s" % (cs, hs))
    # 🔑 POSITIONAL ARGUMENT VALIDATION. Swapping two sendf arguments leaves the
    # format string untouched, so every substring check passes while the host
    # decodes dmaerr as tc. A reviewer confirmed that mutation survived the previous
    # version of this gate. Match each argument expression against the field name it
    # is supposed to fill.
    def sendf_args(src, prefix):
        i = src.find('sendf("' + prefix)
        if i < 0:
            return None, None
        fmt = joined_literal(src[i:], prefix)
        # argument list: after the format's last closing quote, up to the matching )
        j = src.index(fmt.split()[-1], i)
        j = src.index('"', j) + 1
        depth, k = 1, src.index('(', i) + 1
        k = j
        end = j
        depth = 1
        while k < len(src):
            if src[k] == '(':
                depth += 1
            elif src[k] == ')':
                depth -= 1
                if depth == 0:
                    end = k
                    break
            k += 1
        argstr = src[j:end].lstrip().lstrip(',')
        # split on top-level commas only
        args, depth, cur = [], 0, ''
        for ch in argstr:
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            if ch == ',' and depth == 0:
                args.append(cur.strip()); cur = ''
            else:
                cur += ch
        if cur.strip():
            args.append(cur.strip())
        # ⚠ ARITY: Klipper's `%*s` consumes TWO arguments (length, then buffer).
        # Counting one per field reported a spurious 4-vs-5 mismatch on a correct
        # sendf -- the check was wrong, not the code.
        fields = [(n, 2 if conv == '*s' else 1)
                  for n, conv in re.findall(r'(\w+)=%(\*s|\w+)', fmt)]
        return fields, args

    ALIAS = {'oid': ('args[0]', 'oid'), 'now': ('timer_read_time()', 'now')}
    for prefix in ('display_dma_state', 'display_dma_rdata'):
        fields, args = sendf_args(C_SRC, prefix)
        check(fields is not None and args is not None,
              "%s: format and argument list both parsed" % prefix)
        if not fields:
            continue
        want = sum(a for _n, a in fields)
        check(want == len(args),
              "%s: %d argument slots (%%*s counts 2) vs %d sendf arguments"
              % (prefix, want, len(args)))
        if want != len(args):
            continue
        bad, k = [], 0
        for fld, arity in fields:
            slot = args[k:k + arity]
            k += arity
            # scalar: the one arg must name the field. %*s: the BUFFER (last arg)
            # carries the payload; the length arg before it is free-form.
            cand = slot[-1] if arity == 2 else slot[0]
            ok = (fld in cand or cand in ALIAS.get(fld, '')
                  or cand.strip().isdigit()         # a literal constant is fine
                  or (arity == 2 and fld == 'data'))
            if not ok:
                bad.append("%s <- %s" % (fld, cand))
        check(not bad,
              "%s: every argument fills the field it is named for (offenders: %s)"
              % (prefix, "; ".join(bad) if bad else "none"))

    # the host must actually register the poll, or it can never run
    check("register_event_handler('klippy:ready', self._hq_ready)" in HOST_SRC,
          "the host registers the ready handler, so the poll can actually start")
    # tc must be incremented in the COMPLETION branch, not the error branch
    tcbranch = C_SRC.split('if (DMA2->HISR & DMA_HISR_TCIF5)', 1)
    check(len(tcbranch) == 2 and 'd->tc++;' in tcbranch[1].split('} else {', 1)[0],
          "d->tc++ is in the transfer-COMPLETE branch, not the error branch")
    errbranch = C_SRC.split('} else {', 1)
    check(len(errbranch) == 2 and 'd->dmaerr++;' in errbranch[1],
          "and d->dmaerr++ is in the error branch")
    check('d->tc = d->dmaerr = d->pushes = d->last_tc = 0;' in C_SRC,
          "the counters are zeroed at config (checked exactly, not as a disjunction"
          " that is always true -- the old form of this check could not fail)")
    rdbody = C_SRC.split('command_display_dma_read', 1)[1] if len(body) == 2 else ""
    check('irq_disable' in C_SRC.split('command_display_dma_query', 1)[1]
          .split('DECL_COMMAND', 1)[0],
          "the counter query snapshots under irq_disable (the DMA IRQ mutates them)")


def arm_j():
    """🚨 THE CRASH THIS GATE MISSED. First deploy took klippy down ~6 ms after ready.

    TRACED: klippy:ready handlers run inside `reactor.assert_no_pause()`
    (klippy.py:161). A query command blocks: send() -> raw_send_wait_ack() ->
    ReactorCompletion.wait(), which APPENDS the greenlet to completion.waiting
    (reactor.py:37) BEFORE calling pause() (:38) and only removes it after (:39) --
    no try/finally. pause() raises ReactorError, the greenlet stays in `waiting`,
    and the command has ALREADY been transmitted. This module's broad `except`
    logged the reactor's own diagnostic as a retryable blip, so ready completed,
    the connect greenlet was retired (`g_old.timer = None`, reactor.py:263), and
    the ack arriving milliseconds later ran complete() over the stale waiter:
    update_timer(None, ...) -> AttributeError inside the reactor -> shutdown.

    🔑 TWO INDEPENDENT PROPERTIES, pinned separately, because either alone is
    enough to crash and either alone could be reintroduced:
      (a) _hq_ready() must SEND NOTHING -- it may only arm the timer;
      (b) no send may be attempted while the reactor forbids pausing, whoever calls.
    """
    # (a) the ready handler must not touch the MCU
    d = Dut(reads=[rd(BASE)], engine=[eng()])
    d._hq_ready()
    check(d.mcu.rq.n == 0 and d.mcu.eq.n == 0,
          "_hq_ready() performs ZERO sends (got rd=%d eng=%d) -- it may only arm the"
          " timer" % (d.mcu.rq.n, d.mcu.eq.n))
    check(d._hq_timer is not None, "and it DOES arm the timer (or the poll never runs)")
    check(d._rd_base is None, "and takes no baseline there (that is the poll's job)")
    # (b) inside a no-pause section nothing may be sent, by any caller
    d2 = Dut(reads=[rd(BASE)], engine=[eng()], no_pause=True)
    check(d2._panel_read(0x0a, 2) is None,
          "_panel_read returns None inside a no-pause section")
    check(d2.mcu.rq.n == 0,
          "and sends NOTHING -- the command must not reach the MCU, because the"
          " poison is the transmitted-then-orphaned ack (got %d sends)" % d2.mcu.rq.n)
    check(d2._engine_counters() is None and d2.mcu.eq.n == 0,
          "_engine_counters likewise returns None without sending")
    # and the guard must not be a no-op on a reactor that permits pausing
    d3 = Dut(reads=[rd(BASE)], engine=[eng()])
    check(d3._panel_read(0x0a, 2) is not None and d3.mcu.rq.n == 1,
          "while a normal reactor still reads (the guard is not a blanket off-switch)")
    # a reactor with no verify_can_pause at all must still work
    d4 = Dut(reads=[rd(BASE)])
    del d4.reactor.__class__.verify_can_pause
    try:
        ok = d4._panel_read(0x0a, 2) is not None
    finally:
        Reactor.verify_can_pause = _saved_vcp
    check(ok, "and a reactor predating no-pause sections is not locked out")


def arm_k():
    # A None baseline with the command PRESENT must retry, not disarm for the session.
    # The baseline now runs while the screen is live, so busy=1 is expected and says
    # nothing about SDO -- disarming on it would turn a timing blip into a permanent off.
    d = Dut(reads=[rd([], busy=1), rd(BASE), rd(BASE)], engine=[eng()])
    d._hq_baseline()
    check(d._rd_base is None, "a busy first read leaves the baseline unset")
    check(not d._rd_unavailable,
          "and does NOT disarm -- busy is not evidence about SDO")
    d._hq_baseline()
    check(d._rd_base == bytes(bytearray(BASE)), "the next attempt adopts the baseline")
    # old firmware still disarms, as before
    d2 = Dut(lookup_err="Unknown command 'display_dma_read'")
    d2._hq_baseline()
    check(d2._rd_unavailable, "whereas missing firmware support DOES disarm")


_saved_vcp = Reactor.verify_can_pause

def arm_l():
    """DISPLAY_WEDGE_TEST: the only thing that can prove the CATCH on real hardware.

    🔑 WHY. Post-validation the detector was armed, stable and decoding correctly, but
    no real wedge had ever been CAUGHT -- that rested on this gate and an offline
    repro, both against a SIMULATED fault. SLPIN makes the panel itself drop RDDPM's
    sleep-out and display-on bits, which is the pattern a reset or wedged controller
    lands in. So the stimulus is real, and the command drives the REAL _hq_event
    rather than a copy of its logic.

    ⚠ It blanks the LCD on purpose, so the guards matter as much as the sequence:
    CONFIRM required, refuses mid-print, refuses when not armed or unavailable.
    """
    SLPIN, SLPOUT, DISPON = 0x10, 0x11, 0x29
    # guard: CONFIRM is mandatory
    d = _wire_segs(Dut(reads=[rd(BASE)])); d._rd_base = bytes(bytearray(BASE))
    g = _Gcmd()
    try:
        d.cmd_DISPLAY_WEDGE_TEST(g); refused = False
    except Exception:
        refused = True
    check(refused and not d.segs,
          "without CONFIRM=1 it refuses and sends NOTHING to the panel")
    # guard: refuses mid-print
    for state in ('printing', 'paused'):
        d = _wire_segs(Dut(reads=[rd(BASE)])); d._rd_base = bytes(bytearray(BASE))
        d.print_state = state
        try:
            d.cmd_DISPLAY_WEDGE_TEST(_Gcmd(CONFIRM=1)); ref = False
        except Exception:
            ref = True
        check(ref and not d.segs,
              "refuses with print_stats=%s and sends nothing -- a blank panel"
              " mid-print is never worth a diagnostic" % state)
    # guard: refuses when the detector is not armed / unavailable
    d = _wire_segs(Dut(reads=[rd(BASE)]))          # _rd_base stays None
    try:
        d.cmd_DISPLAY_WEDGE_TEST(_Gcmd(CONFIRM=1)); ref = False
    except Exception:
        ref = True
    check(ref and not d.segs, "refuses when no baseline is armed (nothing to catch)")
    d = _wire_segs(Dut(reads=[rd(BASE)])); d._rd_unavailable = True
    try:
        d.cmd_DISPLAY_WEDGE_TEST(_Gcmd(CONFIRM=1)); ref = False
    except Exception:
        ref = True
    check(ref and not d.segs, "and when read-back is unavailable on the board")
    # HAPPY PATH: asleep read differs -> caught; awake read matches -> cleared
    ASLEEP = [0x00, 0x08, 0x00]      # sleep-in + display-off, the real SLPIN profile
    d = _wire_segs(Dut(reads=[rd(ASLEEP), rd(BASE)], engine=[eng(), eng()]))
    d._rd_base = bytes(bytearray(BASE))
    g = _Gcmd(CONFIRM=1)
    _, recs = cap_run(lambda: d.cmd_DISPLAY_WEDGE_TEST(g))
    sent = [c for cs, dc, data in d.segs for c in data]
    check(SLPIN in sent, "the happy path sends SLPIN (0x%02x)" % SLPIN)
    check(SLPOUT in sent, "and SLPOUT (0x%02x)" % SLPOUT)
    check(DISPON in sent,
          "and DISPON, so the panel is unlikely to need a FIRMWARE_RESTART")
    check(sent.index(SLPIN) < sent.index(SLPOUT),
          "in that order (sleep before wake)")
    out = "\n".join(g.msgs)
    check('PASS' in out,
          "the verdict is PASS when the fault was caught AND recovery cleared it")
    check('00 08 00' in out and '00 9c 00' in out,
          "and the report quotes the asleep and awake readings verbatim")
    check(any('PANEL LIVENESS CHANGED' in m for lv, m in recs if lv >= 30),
          "the REAL warning path fired (not a copy of its logic)")
    check(d._panel_bad == 0, "and the flag is cleared again after recovery")
    check('FIRMWARE_RESTART' in out,
          "and the response names the recovery step for a blank screen")
    # and a FAILED catch must report FAIL, not PASS
    d2 = _wire_segs(Dut(reads=[rd(BASE), rd(BASE)], engine=[eng(), eng()]))
    d2._rd_base = bytes(bytearray(BASE))
    g2 = _Gcmd(CONFIRM=1)
    cap_run(lambda: d2.cmd_DISPLAY_WEDGE_TEST(g2))
    check('FAIL' in "\n".join(g2.msgs),
          "a panel that does NOT change under SLPIN reports FAIL -- the test cannot"
          " pass itself by construction")


arm("ARM A -- a panel leaving its baseline is detected and warned once:", arm_a)
arm("ARM B -- COUNTERWEIGHT: an unwired SDO DISARMS, it does not alarm:", arm_b)
arm("ARM C -- a matching baseline stays silent:", arm_c)
arm("ARM D -- a busy engine declines without a verdict, and re-arms:", arm_d)
arm("ARM E -- one transient failure must not kill the poll:", arm_e)
arm("ARM F -- a format mismatch is drift, not 'old firmware':", arm_f)
arm("ARM G -- DISPLAY_HEALTH runs, and verdicts from the FRESH read:", arm_g)
arm("ARM H -- the 25.57 s tick wrap is reported as a bound:", arm_h)
arm("ARM I -- MCU side: no spin, read clock, restored clock, formats match:", arm_i)
arm("ARM J -- THE CRASH: ready sends nothing, and no-pause blocks every send:", arm_j)
arm("ARM K -- a busy/None baseline retries instead of disarming:", arm_k)
arm("ARM L -- DISPLAY_WEDGE_TEST: guards, real sequence, and it can FAIL:", arm_l)

print()
if fails:
    print("DISPLAY-HEALTH GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("DISPLAY-HEALTH GATE: PASS -- the panel's own register is read back to detect a wedged"
      " panel, a board that does not drive SDO disarms the check instead of alarming, transient"
      " failures and a busy engine do not produce verdicts or kill the poll, DISPLAY_HEALTH"
      " reports from a fresh read, the tick wrap is bounded, and the MCU read cannot spin,"
      " shutdown, or leave the engine on the slow clock")
