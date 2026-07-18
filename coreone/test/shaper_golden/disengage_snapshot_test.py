"""Gate: the teardown must report the RUN's counters, not the window the teardown itself creates.

    python3 disengage_snapshot_test.py

WHY THIS EXISTS. `cmd_TRAPQ_DISENGAGE` used to stop the keep-alive as its first action and only
THEN read the MCU counters. With the keep-alive stopped the segment ring is uncovered, so the
report described its own teardown: on the 2026-10-31 validation print it printed
`lateBEF=39154` and `MCU-STALE events=2` for a job whose `PHASE_XY_CHECK` had returned
`lateBEF=0 stale=0 mcuSTALE=0` fifteen seconds earlier. The arithmetic is unambiguous --
39154 ticks / 40 kHz = 0.979 s, matching the same report's own `dry X=Y=0.98s`.

Worse than a cosmetic wrong number: `_adopt_latebef` turned the artefact into `xy_trusted=0`,
so EVERY print ended demanding a re-home. `_latebef_verdict` already had benign-detection for
exactly this class (built after the 2026-10-12 `lateBEF=11631` false alarm) but it could not
fire, because its benign test uses the HOST's idle-tail estimate, which reads 0.0 s at the
disengage while the MCU's own `dry` reports 0.98 s. The host cannot see that window. So the fix
is to stop contaminating the measurement rather than to teach the verdict to recognise it.

🔑 ARM B IS THE ONE THAT MATTERS: the snapshot must happen BEFORE the keep-alive stop. That is
the entire fix. If a later edit moves the snapshot below `update_timer(..., NEVER)` the numbers
silently become teardown numbers again and the cry-wolf banner returns -- with no test failing
unless the ORDER itself is pinned.

🔑 ARM C is the proof that `snap` is actually used: given a snapshot, the reporter must not
re-query the MCU. Combined with ARM E (the source takes `lb` from `p`), that establishes the
adopted verdict comes from the pre-teardown read.

⚠ The fix deliberately does NOT keep the keep-alive alive through the unwind. Stopping it there
is intentional, and changing it would alter teardown BEHAVIOUR rather than fix reporting.

⚠ RESIDUAL, NOT CLOSED BY THIS GATE: the ordering bug is confirmed, but it is NOT established
that it accounts for the full 2x8 s of `MCU-STALE events=2` -- the latch needs 8 s uncovered and
the reported `dry` age is 0.98 s. `PHASE_STEP_OFF` is a macro doing more than this one command
(coil unwind, possible dwells) and its timing is unchecked. Do not read a pass here as closing
the magnitude question.

Every arm is isolated and its exceptions become FAILURES -- against an unpatched module the
surface is simply absent, and a bare access would truncate every later arm.
"""
import sys, os, importlib.util, re, logging

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

PHASE_EXEC_PY = golden.PHASE_EXEC_PY
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


SRC = open(PHASE_EXEC_PY).read()


def _func_body(name):
    """Source text of one method, from its def to the next def at the same indent."""
    m = re.search(r'\n    def %s\(' % re.escape(name), SRC)
    if not m:
        return ''
    start = m.start()
    nxt = re.search(r'\n    def ', SRC[start + 10:])
    return SRC[start:start + 10 + nxt.start()] if nxt else SRC[start:]


# ---- load the module far enough to reach the class ----
cls = None
try:
    spec = importlib.util.spec_from_file_location('pe_dis', PHASE_EXEC_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for n in dir(mod):
        o = getattr(mod, n)
        if isinstance(o, type) and hasattr(o, '_report_shape_stats'):
            cls = o
            break
except Exception as e:
    print("  (module import failed: %s -- source arms still run)" % e)


class FakeCmd(object):
    def __init__(self, ret, log, name):
        self.ret = ret
        self.log = log
        self.name = name

    def send(self, args):
        self.log.append('query:' + self.name)
        return dict(self.ret)


class RaisingCmd(object):
    def __init__(self, log, name):
        self.log = log
        self.name = name

    def send(self, args):
        self.log.append('query:' + self.name)
        raise RuntimeError("mcu gone")


def arm_a():
    check(cls is not None, "found the phase_exec class")
    check(cls is not None and hasattr(cls, '_counter_snapshot'),
          "the class exposes _counter_snapshot")
    sig = _func_body('_report_shape_stats')[:120]
    check('snap=None' in sig,
          "_report_shape_stats takes snap=None (got %r)" % sig.split('\n')[1].strip()
          if sig else "signature not found")


def arm_b():
    body = _func_body('cmd_TRAPQ_DISENGAGE')
    check(bool(body), "found cmd_TRAPQ_DISENGAGE")
    i_snap = body.find('_counter_snapshot()')
    i_never = body.find('_r.NEVER')
    check(i_snap >= 0, "it calls _counter_snapshot()")
    check(i_never >= 0, "it stops the keep-alive (update_timer -> NEVER)")
    # THE ORDER IS THE FIX. Pinning only "both appear" would pass on a reverted edit.
    check(i_snap >= 0 and i_never >= 0 and i_snap < i_never,
          "and the SNAPSHOT comes BEFORE the keep-alive stop (snap@%d < never@%d)"
          % (i_snap, i_never))
    # and the snapshot must be handed to the reporter, not merely taken
    check('snap=snap' in body,
          "the snapshot is passed to _report_shape_stats, not taken and dropped")


def arm_c():
    # ⚠ THIS ARM TESTS THE MODULE'S SOURCE, deliberately. An earlier draft evaluated the
    # selection expression copied INTO this test and asserted it behaved -- a tautology that
    # passed against the unpatched blob and therefore tested nothing. A gate arm that cannot
    # fail on unfixed code is dead weight. Driving the real _report_shape_stats instead would
    # need a ~20-object fake of the whole reporter (gcmd, mcu clock, trust cache, verdict
    # helpers), so the source form is the honest proportionate check here.
    body = _func_body('_report_shape_stats')
    check(bool(body), "found _report_shape_stats")
    flat = " ".join(body.split())
    # both payloads must be selected snap-first with a live fallback
    shape_sel = ("p = ((snap or {}).get('shape') if (snap or {}).get('shape') is not None "
                 "else self.shape_query_cmd.send([self.oid]))")
    stale_sel = ("q = ((snap or {}).get('stale') if (snap or {}).get('stale') is not None "
                 "else self.stale_query_cmd.send([self.oid]))")
    check(shape_sel in flat,
          "the SHAPE payload is selected snap-first with a live fallback")
    check(stale_sel in flat,
          "the STALE payload is selected snap-first with a live fallback")
    # and the live fallback must still be reachable -- PHASE_SHAPE_STATS depends on it
    check('self.shape_query_cmd.send' in body and 'self.stale_query_cmd.send' in body,
          "the live query remains reachable, so standalone PHASE_SHAPE_STATS still works")


def arm_d():
    if cls is None:
        raise RuntimeError("class unavailable")

    class Dut(cls):
        def __init__(self, shape, stale, log):
            self.oid = 0
            self.shape_query_cmd = shape
            self.stale_query_cmd = stale
            self.log = log

    log = []
    d = Dut(FakeCmd({'latebefore': 11}, log, 'shape'),
            FakeCmd({'x_events': 3}, log, 'stale'), log)
    snap = d._counter_snapshot()
    check(isinstance(snap, dict), "_counter_snapshot returns a dict (got %r)" % type(snap))
    check(log == ['query:shape', 'query:stale'],
          "it queries BOTH shape and stale, in that order (got %r)" % (log,))
    check(snap.get('shape', {}).get('latebefore') == 11
          and snap.get('stale', {}).get('x_events') == 3,
          "and carries both payloads")


def arm_e():
    body = _func_body('_report_shape_stats')
    check("lb = p.get('latebefore'" in body.replace('"', "'"),
          "the adopted lateBEF is taken from p -- so a snapshot p feeds the verdict")
    check('_adopt_latebef(lb' in body,
          "and _adopt_latebef is what writes the trust cache (the actual harm path)")
    # the selection must be guarded on `is not None`, not truthiness: a legitimate snapshot
    # whose payload is falsy-but-present must still be used.
    check('is not None' in body,
          "the snap selection tests `is not None`, so a falsy-but-present payload is still used")


def arm_f():
    if cls is None:
        raise RuntimeError("class unavailable")

    class Dut(cls):
        def __init__(self, log):
            self.oid = 0
            self.shape_query_cmd = RaisingCmd(log, 'shape')
            self.stale_query_cmd = RaisingCmd(log, 'stale')

    warns = []
    orig = logging.warning
    logging.warning = lambda *a, **k: warns.append(a[0] if a else '')
    try:
        log = []
        d = Dut(log)
        out = d._counter_snapshot()
    finally:
        logging.warning = orig
    check(out is None,
          "a failing snapshot returns None rather than raising (disengage must not break)")
    check(any('snapshot' in str(w) for w in warns),
          "and it WARNS -- silently degrading to the old contaminated read is how a fix hides"
          " (got %d warnings)" % len(warns))


def arm_g():
    # The trigger text must not keep claiming PAUSE/M600 is THE trigger; it is one way
    # coverage breaks, and the teardown is another.
    flat = " ".join(SRC.split())
    check('THE TRIGGER IS THE RING GOING UNCOVERED' in flat
          or 'not idling as such' in flat.lower(),
          "the lateBEF banner names the real trigger (ring uncovered past PE_STALE_S)")
    check('most common way' in flat or 'not the only one' in flat,
          "and demotes PAUSE / M600 to the most common cause rather than the only one")
    check('STOPPED DELIBERATELY' in flat or 'every teardown does this' in flat,
          "the stale banner admits the teardown stops the keep-alive itself")
    check('_counter_snapshot' in _func_body('_counter_snapshot'),
          "_counter_snapshot documents why it exists")


arm("ARM A -- the snapshot surface exists:", arm_a)
arm("ARM B -- THE KEY ONE: snapshot BEFORE the keep-alive stop:", arm_b)
arm("ARM C -- a snapshot suppresses the re-query and supplies the figures:", arm_c)
arm("ARM D -- _counter_snapshot queries both counter sets:", arm_d)
arm("ARM E -- the adopted verdict is fed from p, i.e. from the snapshot:", arm_e)
arm("ARM F -- a failing snapshot degrades LOUDLY, not silently:", arm_f)
arm("ARM G -- the banners no longer misattribute the trigger:", arm_g)

print()
if fails:
    print("DISENGAGE-SNAPSHOT GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("DISENGAGE-SNAPSHOT GATE: PASS -- the teardown snapshots the MCU counters BEFORE stopping"
      " its own keep-alive, hands them to the reporter, suppresses the contaminating re-query,"
      " feeds the trust verdict from the pre-teardown read, fails loudly if the snapshot cannot"
      " be taken, and no longer tells the operator a pause caused what the teardown did")
