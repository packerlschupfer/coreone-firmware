"""Gate: the reactor-stall high-water mark must SURVIVE GC_TUNE_STATUS RESET=1.

    python3 gc_session_highwater_test.py

🚨 WHY THIS EXISTS -- a measurement that was structurally incapable of showing the thing it
existed to show. PRINT_START calls `GC_TUNE_STATUS RESET=1`, so every stall field was PER-PRINT.
On 2026-11-10, after 4 days 6 h of continuous klippy uptime spanning several prints, `late_max_ms`
read 11.343 ms -- LOWER than the 13.733 ms read three days earlier on the same uninterrupted
session. Nothing anywhere held the worst stall of the session, which is exactly the figure that
would reveal a slow degradation.

🔑 THE TELL, and ARM A is built on it: a worst-case that FALLS while uptime GROWS means a counter
was reset. A high-water mark cannot do that, so ARM A asserts precisely that it does not.

⚠ ARM D is the counterweight. A session mark that merely ran in parallel and disagreed with the
per-print one would be a second wrong number, not an instrument: with no reset in between, the two
MUST agree. And ARM F pins that NO gcode path can clear the session fields -- a high-water mark a
macro can zero is not a high-water mark, and PRINT_START would zero it at the top of every print,
which is how the hole arose.
"""
import sys, os, importlib.util, inspect

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

GC_TUNE_PY = os.path.join(os.path.dirname(golden.PHASE_EXEC_PY), 'gc_tune.py')
SRC = open(GC_TUNE_PY).read()
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))


spec = importlib.util.spec_from_file_location('gct_hw', GC_TUNE_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
GCTune = None
for n in dir(mod):
    o = getattr(mod, n)
    if isinstance(o, type) and hasattr(o, '_wd_report'):
        GCTune = o
        break


class FakeReactor(object):
    NEVER = 9e99

    def __init__(self):
        self.t = 1000.0
        self.timers = {}

    def monotonic(self):
        return self.t

    def register_timer(self, cb, waketime=NEVER):
        self.timers[cb] = waketime
        return cb

    def update_timer(self, handle, waketime):
        self.timers[handle] = waketime

    def get_gc_stats(self):
        return [0.0, 0.0, 0.0]


class FakeGcode(object):
    def __init__(self):
        self.cmds = {}
        self.responses = []

    def register_command(self, name, cb, desc=None):
        self.cmds[name] = cb

    def respond_info(self, msg, log=True):
        self.responses.append(msg)


class FakePrinter(object):
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.objects = {'gcode': self.gcode}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default='__raise__'):
        if name in self.objects:
            return self.objects[name]
        if default == '__raise__':
            raise Exception("Unknown config object '%s'" % (name,))
        return default

    def register_event_handler(self, name, cb):
        pass


class FakeConfig(object):
    def __init__(self, printer, **vals):
        self.printer = printer
        self.vals = vals

    def get_printer(self):
        return self.printer

    def getboolean(self, name, default):
        return self.vals.get(name, default)

    def getint(self, name, default, minval=None, maxval=None):
        return self.vals.get(name, default)

    def getfloat(self, name, default, minval=None, maxval=None):
        return self.vals.get(name, default)


class FakeGcmd(object):
    def __init__(self, gcode, **params):
        self.gcode = gcode
        self.params = params

    def get_int(self, name, default, minval=None, maxval=None):
        return self.params.get(name, default)

    def respond_info(self, msg):
        self.gcode.respond_info(msg)


def build(**cfg):
    p = FakePrinter()
    g = GCTune(FakeConfig(p, **cfg))
    g._arm_watchdog()
    return p, g


def stall(p, g, late_ms):
    """Make the watchdog fire `late_ms` after the deadline it asked for."""
    p.reactor.t = g._wd_due + late_ms / 1e3
    return g._watchdog_event(p.reactor.t)


def do_reset(p, g):
    """Exercise the REAL gcode path PRINT_START uses, not _wd_reset directly."""
    cb = p.gcode.cmds.get('GC_TUNE_STATUS')
    if cb is None:
        raise RuntimeError("GC_TUNE_STATUS not registered")
    cb(FakeGcmd(p.gcode, RESET=1))


def arm_a():
    # THE REAL CASE, reproduced: a big stall, then PRINT_START's RESET, then a smaller stall.
    # Per-print must fall (that is correct and wanted); the SESSION mark must not.
    p, g = build()
    stall(p, g, 13.733)
    check(abs(g._wd_max - 13.733) < 1e-6, "per-print max takes the 13.733 ms stall")
    check(abs(g._wd_smax - 13.733) < 1e-6, "and so does the session mark")
    do_reset(p, g)
    check(g._wd_max == 0.0, "GC_TUNE_STATUS RESET=1 zeroes the PER-PRINT max (as designed)")
    check(abs(g._wd_smax - 13.733) < 1e-6,
          "but the SESSION mark still reads 13.733 -- the 2026-11-10 hole, closed")
    stall(p, g, 11.343)
    check(abs(g._wd_max - 11.343) < 1e-6, "the next print's max reads its own 11.343 ms")
    check(abs(g._wd_smax - 13.733) < 1e-6,
          "and the session mark STILL reads 13.733, so the worst stall is no longer lost")
    # the exact symptom that started this: per-print falling while uptime grows
    check(g._wd_max < g._wd_smax,
          "per-print < session is now REPRESENTABLE -- previously the only number fell silently")


def arm_b():
    # The session counters must accumulate ACROSS resets, not restart with each print.
    p, g = build()
    for _ in range(3):
        stall(p, g, 12.0)
    n_before = g._wd_sn
    do_reset(p, g)
    check(g._wd_n == 0, "per-print fire count zeroed by RESET")
    check(g._wd_sn == n_before == 3,
          "session fire count survives at %d" % (g._wd_sn,))
    check(sum(g._wd_over) == 0, "per-print buckets zeroed")
    check(sum(g._wd_sover) > 0, "session buckets survive")
    for _ in range(2):
        stall(p, g, 12.0)
    check(g._wd_sn == 5, "and they keep accumulating after the reset (3 + 2 = %d)" % g._wd_sn)


def arm_c():
    # A RE-ARM must not clear the session mark either -- _arm_watchdog calls _wd_reset.
    p, g = build()
    stall(p, g, 300.0)
    g._arm_watchdog()
    check(g._wd_max == 0.0, "re-arm zeroes the per-print max")
    check(abs(g._wd_smax - 300.0) < 1e-6,
          "but NOT the session mark (re-arm goes through _wd_reset, which must not reach it)")


def arm_d():
    # COUNTERWEIGHT: with no reset in between the two MUST agree, or the session figure is
    # just a second, differently-wrong number.
    p, g = build()
    for ms in (5.0, 40.0, 12.0, 7.5):
        stall(p, g, ms)
    check(abs(g._wd_max - g._wd_smax) < 1e-9,
          "no reset -> per-print max == session max (%.3f == %.3f)" % (g._wd_max, g._wd_smax))
    check(g._wd_n == g._wd_sn, "and the fire counts agree (%d == %d)" % (g._wd_n, g._wd_sn))
    check(g._wd_over == g._wd_sover, "and the buckets agree")
    check(abs(g._wd_smax - 40.0) < 1e-6, "the mark is the WORST, not the latest (40.0)")


def arm_e():
    # It has to be READABLE -- a high-water mark nobody can query is not an instrument.
    p, g = build()
    stall(p, g, 77.0)
    do_reset(p, g)
    st = g.get_status(p.reactor.t)
    check('late_max_session_ms' in st, "get_status exposes late_max_session_ms")
    check(abs(st.get('late_max_session_ms', 0) - 77.0) < 1e-6,
          "and it reads 77.0 after a reset, while late_max_ms reads %.1f"
          % st.get('late_max_ms', -1))
    check(st.get('late_max_ms') == 0.0, "(the per-print field correctly reads 0.0)")
    for k in ('late_session_fires', 'late_session_over_100ms',
              'late_session_over_250ms', 'late_max_session_context'):
        check(k in st, "get_status exposes %s" % k)
    rep = g._wd_report()
    check('SESSION' in rep, "the console report names the SESSION figure")
    check('77.0' in rep, "and prints it (%s)" % ('77.0' in rep))
    check('RESET' in rep,
          "and labels the per-print one as reset-able, so the two cannot be confused")


def arm_f():
    # No gcode path may clear the session fields. Checked against the SOURCE, because the
    # hazard is a FUTURE edit adding a RESET= branch that reaches them.
    flat = " ".join(SRC.split())
    check('_wd_session_init' in SRC, "there is a dedicated session initialiser")
    # _wd_reset must not mention any session field
    body = SRC.split('def _wd_reset(self):', 1)[1].split('\n    def ', 1)[0]
    leaked = [tok for tok in ('_wd_smax', '_wd_sn', '_wd_ssum', '_wd_sover', '_wd_sstart')
              if tok in body]
    check(not leaked,
          "_wd_reset() touches NO session field (leaked: %r)" % (leaked,))
    # ⚠ THIS CHECK USED TO ASSERT THE BUG. It required a `hasattr(self, '_wd_smax')` guard in
    # _arm_watchdog -- i.e. it pinned the LAZY INIT as though that were the safety property,
    # when the lazy init was itself the defect (get_status raised for `watchdog: False`; see
    # ARM H). A source assertion that encodes the current implementation rather than the
    # required property will defend a bug as readily as a feature.
    # The real invariant: initialise in __init__, and never re-initialise on a re-arm.
    # The BEHAVIOUR is ARM C's job; this is the structural half.
    init = SRC.split('def __init__', 1)[1].split('\n    def ', 1)[0]
    check('_wd_session_init()' in init,
          "the session fields are initialised in __init__ (not lazily at arm time)")
    armbody = SRC.split('def _arm_watchdog', 1)[1].split('\n    def ', 1)[0]
    check('_wd_session_init' not in armbody,
          "and _arm_watchdog does NOT re-initialise them, so a re-arm cannot clear the mark")
    check('late_max_session_ms' in SRC, "and the field is exported")


def arm_g():
    """The invariant, and the WARNING the first version of this patch destroyed.

    🚨 My first attempt anchored its replacement on the three lines that set _wd_max and
    added an `elif`, which SWALLOWED the warning block that followed them -- moving
    "THE HOST STOPPED FOR x ms" into a branch that can never run. That is the single most
    important log line in gc_tune.py: it is what reported the 188 ms stall that killed a
    print. Silently disabled, with the patch reading fine. gc_watchdog_test.py caught it.
    """
    p, g = build()
    # INVARIANT: session >= per-print, always. Both take the max of the same stream and only
    # the per-print one is ever reset -- which is ALSO why the `elif` was dead code: the
    # session mark cannot move without the per-print mark moving too.
    import random
    random.seed(7)
    for i in range(60):
        stall(p, g, random.choice([0.5, 5.0, 60.0, 300.0]))
        check_quiet = g._wd_smax >= g._wd_max - 1e-9
        if not check_quiet:
            break
        if i % 13 == 12:
            do_reset(p, g)
    check(g._wd_smax >= g._wd_max - 1e-9,
          "session >= per-print held across 60 samples and 4 resets (%.1f >= %.1f)"
          % (g._wd_smax, g._wd_max))
    # The warning must still live in the per-print branch and still fire.
    import logging as _lg

    class Cap(_lg.Handler):
        def __init__(self):
            _lg.Handler.__init__(self); self.msgs = []

        def emit(self, r):
            self.msgs.append((r.levelno, r.getMessage()))

    p2, g2 = build()
    cap = Cap(); root = _lg.getLogger(); root.addHandler(cap); lv = root.level
    root.setLevel(_lg.DEBUG)
    try:
        stall(p2, g2, 300.0)
    finally:
        root.removeHandler(cap); root.setLevel(lv)
    warn = [m for l, m in cap.msgs if l >= _lg.WARNING]
    check(len(warn) == 1, "a 300 ms stall still logs exactly one warning (got %d)" % len(warn))
    check(warn and 'THE HOST STOPPED' in warn[0],
          "and it is still THE HOST STOPPED -- the line a bad patch of mine once moved into"
          " an unreachable branch")
    # source-level: the warning must not sit in an elif of the session update
    body = SRC.split('def _watchdog_event', 1)[1].split('\n    def ', 1)[0]
    i_warn = body.find('THE HOST STOPPED')
    i_sess = body.find('if late_ms > self._wd_smax:')
    check(i_warn >= 0 and i_sess >= 0 and i_warn < i_sess,
          "the warning is emitted BEFORE the session update, not nested inside it")
    check('elif late_ms > self._wd_smax' not in body,
          "and the session update is not an elif of the per-print branch (it is unreachable"
          " there, and it swallowed the warning)")


def arm_h():
    """get_status must never raise -- with the watchdog OFF, and before anything is armed.

    🚨 FOUND BY AN INDEPENDENT REVIEW, not by me. My first version created the session fields
    in _arm_watchdog(), which does NOT run when `watchdog: False`. get_status() then raised
    AttributeError, and webhooks._do_query calls get_status UNGUARDED from a reactor timer, so
    klippy turns it into "Unhandled exception during run" -> invoke_shutdown: a Mainsail
    subscribe would have shut the printer down. Before the feature, __init__'s _wd_reset()
    covered every field get_status read; the lazy init silently broke that invariant.

    🔑 Built on the REAL constructor, not the armed helper. build() above always arms the
    watchdog, which is exactly why every other arm here was blind to this.
    """
    for wd in (False, True):
        p = FakePrinter()
        g = GCTune(FakeConfig(p, watchdog=wd))   # no _arm_watchdog()
        try:
            st = g.get_status(p.reactor.monotonic())
            ok, err = True, None
        except Exception as e:
            ok, err = False, "%s: %s" % (type(e).__name__, e)
        check(ok, "get_status() does not raise with watchdog=%s and nothing armed (%s)"
                  % (wd, err))
        if ok:
            check('late_max_session_ms' in st,
                  "and still reports the session field with watchdog=%s" % wd)
        # the report must not raise either -- PRINT_END calls it
        try:
            g._wd_report()
            rok = True
        except Exception as e:
            rok = False
        check(rok, "_wd_report() does not raise with watchdog=%s either" % wd)
    # and the fields must come from __init__, not from a lazy arm-time guard
    init = SRC.split('def __init__', 1)[1].split('\n    def ', 1)[0]
    check('_wd_session_init()' in init,
          "the session fields are initialised in __init__ (a lazy arm-time init is the bug)")
    armbody = SRC.split('def _arm_watchdog', 1)[1].split('\n    def ', 1)[0]
    check("hasattr(self, '_wd_smax')" not in armbody,
          "and _arm_watchdog no longer guards on hasattr -- that guard WAS the lazy init")


arm("ARM A -- RESET=1 must not take the session high-water with it:", arm_a)
arm("ARM B -- session counters accumulate ACROSS resets:", arm_b)
arm("ARM C -- a re-arm does not clear it either:", arm_c)
arm("ARM D -- counterweight: with no reset the two MUST agree:", arm_d)
arm("ARM E -- it is readable from get_status and the report:", arm_e)
arm("ARM F -- no path clears it (source-level, against future edits):", arm_f)
arm("ARM G -- the invariant, and the warning a bad patch of mine destroyed:", arm_g)
arm("ARM H -- get_status must not raise with the watchdog OFF (review finding):", arm_h)

print()
if fails:
    print("GC-SESSION-HIGHWATER GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("GC-SESSION-HIGHWATER GATE: PASS -- the worst reactor stall of the session survives"
      " PRINT_START's RESET=1 and a re-arm, accumulates across prints, agrees with the"
      " per-print figure when no reset intervened, and is readable from get_status")
