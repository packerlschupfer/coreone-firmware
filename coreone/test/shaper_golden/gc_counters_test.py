"""Gate: gc_tune exports allocation counters that are readable mid-print WITHOUT triggering a pass.

    python3 gc_counters_test.py

WHY THIS EXISTS. The mechanism behind the 2026-10-31 shutdown is the COST of a gen-2 pass, and that
cost can only be measured by RUNNING a pass (`GC_TUNE_STATUS COLLECT=1`, 93-240 ms on this machine).
Doing that during a print would make the instrument the fault -- so the only thing measurable live
was `late_max_ms`, which is the CONSEQUENCE, not the mechanism. `gc.get_count()` and
`gc.get_stats()` are the risk-free half: neither traverses nor collects. This gate pins them into
`get_status` and pins the two properties that make them safe.

🔑 THE ARM THAT MATTERS IS ARM C: a status query must NEVER collect. `get_status` is called several
times a second; if reading it could trigger a gen-2 pass, the observability patch would itself
become a source of the stall it is meant to observe. That is the regression this gate exists to
prevent, not the presence of the fields.

🔑 ARM D pins the THROTTLE, and it is not a micro-optimisation. `gc.get_stats()` builds three dicts
per call. Called unthrottled from a hot status path it would add steady allocation pressure to the
exact hot path whose allocation this module exists to measure -- a self-defeating instrument.
`gc.get_count()` is a plain tuple read and is deliberately NOT cached, so the pressure figure stays
instantaneous.

⚠ `gc_counts` is allocation PRESSURE (objects allocated-minus-freed per generation since that
generation's last collection), NOT heap size. The report must not call it a heap -- that conflation
is how two sessions spent five messages comparing startup baselines to runtime behaviour.

Real class under test: GCTune, loaded from the real file.
"""
import sys, os, importlib.util, gc
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

GC_TUNE_PY = os.path.join(os.path.dirname(golden.PHASE_EXEC_PY), 'gc_tune.py')
spec = importlib.util.spec_from_file_location('gct', GC_TUNE_PY)
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


class Dut(cls):
    """Only the status surface; no printer, no reactor, no config."""

    def __init__(self):
        self.enabled = True
        self.watchdog = True
        self.measure = True
        self.wd_period = 1.0
        self._stats_at = None
        self._stats_cache = ([], 0)
        self.BUCKETS = getattr(cls, 'BUCKETS', [1., 10., 50., 100., 250.])
        self._wd_reset()


print("ARM A -- the class loaded and exposes a status surface:")
check(cls is not None, "found the gc_tune class (got %r)" % (cls.__name__ if cls else None,))
d = Dut()
st = d.get_status(100.0)
check(isinstance(st, dict) and 'late_max_ms' in st, "get_status returns the existing fields")

print("\nARM B -- the new counters are present and well-typed:")
check('gc_counts' in st, "gc_counts exported")
check(isinstance(st.get('gc_counts'), list) and len(st['gc_counts']) == 3
      and all(isinstance(v, int) for v in st['gc_counts']),
      "gc_counts is a list of 3 ints (got %r)" % (st.get('gc_counts'),))
check('gc_collections' in st, "gc_collections exported")
check(isinstance(st.get('gc_collections'), list)
      and all(isinstance(v, int) for v in st['gc_collections']),
      "gc_collections is a list of ints (got %r)" % (st.get('gc_collections'),))
check('gc_uncollectable' in st, "gc_uncollectable exported (the leak canary)")
check(isinstance(st.get('gc_uncollectable'), int),
      "gc_uncollectable is an int (got %r)" % (st.get('gc_uncollectable'),))

print("\nARM C -- THE KEY ONE: a status query must NEVER collect:")
real_collect = gc.collect
calls = []
gc.collect = lambda *a, **k: (calls.append(a), 0)[1]
try:
    d2 = Dut()
    for i in range(50):
        d2.get_status(200.0 + i * 0.01)
finally:
    gc.collect = real_collect
check(len(calls) == 0,
      "50 status queries triggered gc.collect() ZERO times (got %d)" % len(calls))

print("\nARM D -- get_stats is THROTTLED; get_count is not (pressure stays instantaneous):")
real_stats = gc.get_stats
real_count = gc.get_count
ns = []
nc = []
gc.get_stats = lambda: (ns.append(1), real_stats())[1]
gc.get_count = lambda: (nc.append(1), real_count())[1]
try:
    d3 = Dut()
    for _ in range(20):
        d3.get_status(300.0)          # same eventtime -> inside the TTL
finally:
    gc.get_stats = real_stats
    gc.get_count = real_count
check(len(ns) == 1,
      "20 queries at one instant called gc.get_stats() ONCE, not 20x (got %d)" % len(ns))
check(len(nc) == 20,
      "but gc.get_count() every time, so pressure is not stale (got %d)" % len(nc))

# getattr with a default, NOT d4._STATS_TTL: against an unpatched module the attribute does not
# exist, and a bare access raises AttributeError -- which CRASHES the gate and truncates every arm
# after this one. A gate that crashes reports less than one that fails. Found by the teeth run.
check(hasattr(cls, '_STATS_TTL'), "the module defines a _STATS_TTL for the throttle")
ttl = getattr(cls, '_STATS_TTL', 1.0)
ns2 = []
gc.get_stats = lambda: (ns2.append(1), real_stats())[1]
try:
    d4 = Dut()
    d4.get_status(400.0)
    d4.get_status(400.0 + ttl + 0.01)             # past the TTL
finally:
    gc.get_stats = real_stats
check(len(ns2) == 2, "and it DOES refresh once the TTL elapses (got %d)" % len(ns2))

print("\nARM E -- instrumentation must never break a status query:")
gc.get_stats = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
try:
    d5 = Dut()
    ok = True
    try:
        s5 = d5.get_status(500.0)
    except Exception as e:
        ok = False
        s5 = None
finally:
    gc.get_stats = real_stats
check(ok, "a raising gc.get_stats does not propagate out of get_status")
check(s5 is not None and 'late_max_ms' in s5,
      "and the rest of the status is still served")

print("\nARM F -- the counters are NOT described as a heap anywhere in the module:")
src = open(GC_TUNE_PY).read()
i = src.find("'gc_counts'")
seg = src[max(0, i - 1200):i + 400] if i >= 0 else ''
check(i >= 0, "found the gc_counts export site")
check('pressure' in seg.lower(), "its comment calls it allocation PRESSURE")
check('not' in seg.lower() and 'heap' in seg.lower(),
      "and explicitly warns it is NOT a heap size -- the conflation that cost five messages")

print()
if fails:
    print("GC-COUNTERS GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("GC-COUNTERS GATE: PASS -- allocation pressure and collection counts are readable from"
      " get_status, a status query never collects, get_stats is throttled so the instrument does"
      " not add the pressure it measures, and a failure in it cannot break a status query")
