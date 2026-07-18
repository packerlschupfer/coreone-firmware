"""Gate: periodic re-freeze keeps the level-2 pass cost FLAT, and cannot itself stall the host.

    python3 gc_refreeze_test.py

WHY THIS EXISTS. The at-ready freeze fixes the STARTUP graph only. Everything allocated after
ready stays under the collector, so the level-2 pass cost grows back as a print runs -- which is
the open n=3/n=4 of 2026-10-31, where a 188 ms stall killed a print at 52 min. Periodic
re-freezing moves post-ready survivors into the permanent generation too.

🔑 ARM B IS THE ONE THAT MATTERS. The re-freeze timer must NEVER call gc.collect(). Its entire
safety argument is that it piggybacks on a level-2 pass klippy already decided to run (reactor
_check_gc, which only escalates when it believes the reactor is idle) and pays nothing itself.
A re-freeze that collected would BE the 258-269 ms stall it exists to prevent -- the module
would become its own failure mode. That is the regression this gate exists to catch.

🔑 ARM G IS THE COST SIDE, and it must stay in the suite even though it "passes trivially": it
pins that the leak is REAL and that the reclaim path works. The leak is NOT "frozen-then-dead
objects" in general -- ordinary dead objects are freed by refcounting whether frozen or not. It
is specifically frozen-then-dead REFERENCE CYCLES, which are unreachable AND uncollectable.
If a future revision ever makes _reclaim a no-op, the leak becomes unbounded and silent.

⚠ ARM F measures real wall-clock GC timings, so it asserts a RATIO with generous margin rather
than absolute milliseconds. Absolute figures are machine- and load-dependent; the shape (grows
vs flat) is the claim.

Real class under test: GCTune, loaded from the real file.
"""
import sys, os, importlib.util, gc, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

GC_TUNE_PY = os.path.join(os.path.dirname(golden.PHASE_EXEC_PY), 'gc_tune.py')
spec = importlib.util.spec_from_file_location('gct_rf', GC_TUNE_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

cls = None
for name in dir(mod):
    o = getattr(mod, name)
    if isinstance(o, type) and hasattr(o, 'get_status') and hasattr(o, '_wd_reset'):
        cls = o
        break

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm(title, fn):
    """Run one arm. An exception inside it is a FAILURE, never a crash.

    ⚠ WHY: against an unpatched module the re-freeze surface is simply absent, so a bare
    attribute access raises and every later arm is lost -- the gate would report one crash
    instead of the full picture. A gate that crashes tells you less than one that fails.
    (Same defect shipped in gc_counters_test.py first; fixed there too.)
    """
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))


class FakeReactor(object):
    NEVER = 9e99

    def __init__(self, l2=0.):
        self.l2 = l2
        self.timers = []
        self._mono = 1000.0

    def monotonic(self):
        return self._mono

    def get_gc_stats(self):
        return (0., 0., self.l2)

    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return cb

    def update_timer(self, h, w):
        pass


class NoStatsReactor(FakeReactor):
    def get_gc_stats(self):
        raise AttributeError("no get_gc_stats on this klipper")


class FakePrintStats(object):
    def __init__(self, state):
        self.state = state


class FakePrinter(object):
    def __init__(self, reactor, state=None):
        self._reactor = reactor
        self._objs = {}
        if state is not None:
            self._objs['print_stats'] = FakePrintStats(state)

    def get_reactor(self):
        return self._reactor

    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)


class Dut(cls):
    """Only the re-freeze surface; no config, no gcode plumbing."""

    def __init__(self, reactor=None, state=None, refreeze=True, enabled=True):
        self.printer = FakePrinter(reactor or FakeReactor(), state)
        self.enabled = enabled
        self.watchdog = False
        self.measure = False
        self.refreeze = refreeze
        self.wd_period = 1.0
        self.rf_period = 2.0
        self.last_report = "n/a"
        self._stats_at = None
        self._stats_cache = ([], 0)
        self._rf_timer = None
        self._l2_seen = None
        self._rf_n = 0
        self._rf_frozen_added = 0
        self._rf_last_ms = 0.
        self._rf_unavailable = False
        self.BUCKETS = getattr(cls, 'BUCKETS', (10., 25., 50., 100., 250.))
        self._wd_reset()


class FakeGcmd(object):
    class error(Exception):
        pass

    def __init__(self, **params):
        self.params = params
        self.said = []

    def get_int(self, name, default=0):
        return int(self.params.get(name, default))

    def respond_info(self, m):
        self.said.append(m)


def arm_a():
    check(cls is not None, "found the gc_tune class")
    for m in ('_do_refreeze', '_refreeze_event', '_arm_refreeze', '_reclaim', '_rf_report'):
        check(hasattr(cls, m), "has %s" % m)
    check(hasattr(cls, 'cmd_GC_RECLAIM'), "exposes a GC_RECLAIM command")


def arm_b():
    real_collect = gc.collect
    real_freeze = gc.freeze
    ncol = []
    nfrz = []
    gc.collect = lambda *a, **k: (ncol.append(a), 0)[1]
    gc.freeze = lambda: nfrz.append(1)
    try:
        r = FakeReactor(l2=0.)
        d = Dut(reactor=r)
        d._arm_refreeze()
        ev = 1000.0
        for i in range(40):
            r.l2 += 1.0                      # a level-2 pass every single tick: worst case
            ev += d.rf_period
            d._refreeze_event(ev)
    finally:
        gc.collect = real_collect
        gc.freeze = real_freeze
    check(len(ncol) == 0,
          "40 ticks with a level-2 pass on EVERY tick called gc.collect() zero times (got %d)"
          % len(ncol))
    # 39, not 40: the FIRST tick adopts the current level-2 timestamp without freezing, because
    # the at-ready freeze has already covered any pass that predates us. Pinning 39 pins that
    # adopt-on-first-look behaviour -- a 40 here would mean the first tick froze redundantly.
    check(len(nfrz) == 39,
          "but did freeze once per SUBSEQUENT pass, adopting the first (got %d of 40 ticks)"
          % len(nfrz))


def arm_c():
    real_freeze = gc.freeze          # saved HERE: a restore in a finally is useless if the
    nfrz2 = []                       # name it restores from lives in another function.
    gc.freeze = lambda: nfrz2.append(1)
    try:
        r = FakeReactor(l2=5.0)
        d = Dut(reactor=r)
        d._arm_refreeze()
        ev = 1000.0
        for i in range(10):                  # l2 never changes
            ev += d.rf_period
            d._refreeze_event(ev)
        quiet = len(nfrz2)
        r.l2 = 6.0                           # one pass happens
        ev += d.rf_period
        d._refreeze_event(ev)
        after_one = len(nfrz2)
    finally:
        gc.freeze = real_freeze
    check(quiet == 0, "10 ticks with no new pass froze NOTHING (got %d)" % quiet)
    check(after_one == 1, "and exactly one pass -> exactly one freeze (got %d)" % after_one)


def arm_d():
    import logging as _logging
    warns = []
    _orig_warn = _logging.warning
    _logging.warning = lambda *a, **k: warns.append(a[0] if a else '')
    try:
        d = Dut(reactor=NoStatsReactor())
        d._arm_refreeze()
        ret = d._refreeze_event(1000.0)
    finally:
        _logging.warning = _orig_warn
    check(d._rf_unavailable, "it records that re-freeze is unavailable")
    check(ret == NoStatsReactor.NEVER,
          "and stops polling by returning reactor.NEVER (got %r)" % (ret,))
    check(any('get_gc_stats' in str(w) for w in warns),
          "and WARNS -- a silent no-op looks identical to working (got %d warnings)" % len(warns))
    check('UNAVAILABLE' in d._rf_report(),
          "and the report says UNAVAILABLE rather than implying it is active")


def arm_e():
    d = Dut(reactor=FakeReactor(), state='printing')
    g = FakeGcmd()
    refused = False
    try:
        d.cmd_GC_RECLAIM(g)
    except Exception as e:
        refused = True
        msg = str(e)
    check(refused, "it raises while print_stats.state == 'printing'")
    check(refused and 'Timer too close' in msg,
          "and the refusal names the actual mechanism, not just 'unsafe'")
    d2 = Dut(reactor=FakeReactor(), state='standby')
    g2 = FakeGcmd()
    ok = True
    try:
        d2.cmd_GC_RECLAIM(g2)
    except Exception as e:
        ok = False
    check(ok and g2.said, "but runs when idle (standby)")
    d3 = Dut(reactor=FakeReactor(), state='printing')
    g3 = FakeGcmd(FORCE=1)
    forced = True
    try:
        d3.cmd_GC_RECLAIM(g3)
    except Exception:
        forced = False
    check(forced, "and FORCE=1 overrides the refusal")


def arm_f():


    def pass_cost_ms(n=3):
        best = None
        for _ in range(n):
            t = time.monotonic()
            gc.collect(2)
            d_ = (time.monotonic() - t) * 1e3
            best = d_ if best is None else min(best, d_)
        return best


    def run_arm(refreeze):
        gc.unfreeze()
        gc.collect(2)
        live = []
        gc.freeze()                   # the at-ready freeze both arms get
        costs = []
        for rnd in range(5):
            for _ in range(25000):
                live.append({'k': [rnd], 't': (rnd,)})
            costs.append(pass_cost_ms())
            if refreeze:
                gc.freeze()
        gc.unfreeze()
        del live
        gc.collect(2)
        return costs


    was_enabled = gc.isenabled()
    gc.disable()                      # as klippy does
    try:
        grow = run_arm(False)
        flat = run_arm(True)
    finally:
        if was_enabled:
            gc.enable()
    print("     no re-freeze : %s" % " ".join("%.2f" % c for c in grow))
    print("     re-freeze    : %s" % " ".join("%.2f" % c for c in flat))
    grow_ratio = grow[-1] / grow[0] if grow[0] > 0 else 0.
    flat_ratio = flat[-1] / flat[0] if flat[0] > 0 else 0.
    check(grow_ratio > 2.0,
          "WITHOUT re-freeze the pass cost grows with the live set (last/first = %.1fx)"
          % grow_ratio)
    check(flat_ratio < 2.0,
          "WITH re-freeze it stays flat (last/first = %.1fx)" % flat_ratio)
    check(flat[-1] < grow[-1],
          "and the final pass is cheaper with re-freeze (%.2f ms vs %.2f ms)"
          % (flat[-1], grow[-1]))


def arm_g():
    was_enabled = gc.isenabled()
    gc.disable()
    try:
        gc.unfreeze()
        gc.collect(2)
        cyc = []
        for i in range(50000):
            a = {}
            b = {'back': a, 'pad': [0] * 8}
            a['fwd'] = b
            cyc.append(a)
        gc.freeze()
        frozen_before = gc.get_freeze_count()
        del cyc
        freed_while_frozen = gc.collect(2)
        # now the reclaim path
        d = Dut(reactor=FakeReactor())
        pre, post, ms, freed = d._reclaim()
    finally:
        gc.unfreeze()
        gc.collect(2)
        if was_enabled:
            gc.enable()
    check(freed_while_frozen == 0,
          "frozen cyclic garbage is NOT reclaimed by a collection (freed %d)"
          % freed_while_frozen)
    check(frozen_before > 50000,
          "the cycles really were frozen (frozen=%d)" % frozen_before)
    check(freed > 0,
          "_reclaim DOES give them back -- unfreeze+collect freed %d objects" % freed)
    check(ms >= 0.0, "and it reports what that cost (%.1f ms) so callers can gate on it" % ms)


def arm_h():
    src = open(GC_TUNE_PY).read()
    i = src.find("'gc_counts'")
    seg = src[max(0, i - 2000):i + 200] if i >= 0 else ''
    # Normalise whitespace first: the phrase legitimately wraps across comment lines, and a doc
    # assertion that depends on where the line breaks fall is brittle, not strict.
    # Strip the leading '#' of each comment line BEFORE joining -- otherwise the flattened text
    # reads "COLLECTION # counters" and the assertion fails on punctuation, not on content.
    segflat = " ".join(" ".join(l.lstrip().lstrip('#').strip() for l in seg.splitlines()).split())
    check('COLLECTION counters' in segflat,
          "the gc_counts comment says [1]/[2] are COLLECTION counters, not object counts")
    check('refcount' in src.lower(),
          "the leak is documented as cyclic-only, naming refcounting for the rest")
    check('O(1)' in src or '0.002' in src or '0.004' in src,
          "freeze() being cheap is documented with the measured figure, not asserted")
    rf = src[src.find('def _refreeze_event'):]
    check('collect' not in rf.split('def _arm_refreeze')[0],
          "and _refreeze_event's body contains no 'collect' at all")


arm("ARM A -- the re-freeze surface exists:", arm_a)
arm("ARM B -- THE KEY ONE: the re-freeze timer must NEVER collect:", arm_b)
arm("ARM C -- it freezes only when the level-2 timestamp ADVANCES, not every tick:", arm_c)
arm("ARM D -- a klipper without reactor.get_gc_stats must FAIL LOUD, not silently no-op:", arm_d)
arm("ARM E -- GC_RECLAIM must refuse mid-print (its cost IS the documented failure):", arm_e)
arm("ARM F -- THE MEASURED CLAIM: pass cost grows without re-freeze, stays flat with it:", arm_f)
arm("ARM G -- the LEAK is real and bounded ONLY by the reclaim path:", arm_g)
arm("ARM H -- the docs must not repeat the two errors already made here:", arm_h)

print()
if fails:
    print("GC-REFREEZE GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("GC-REFREEZE GATE: PASS -- re-freeze follows klippy's own level-2 passes without ever"
      " collecting, freezes only when one actually happened, fails loud on a klipper that"
      " cannot support it, refuses the reclaim mid-print, holds the pass cost flat where the"
      " unfrozen arm grows, and the cyclic leak it creates is given back by _reclaim")
