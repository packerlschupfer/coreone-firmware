"""Unit checks for the reactor-latency watchdog in extras/gc_tune.py.

    python3 gc_watchdog_test.py

WHY THIS GATE EXISTS. The watchdog's whole value is that its number can be quoted as
evidence -- "the host never stopped for more than N ms across this print". A latency monitor
that silently under-reports is worse than none, because it converts an unknown into a false
all-clear. The two failure modes that would do that are both easy to write by accident:

  1. MEASURING AGAINST THE PREVIOUS FIRE instead of against the deadline we asked for. Then a
     stall shifts the yardstick with it and the lateness reads ~0 -- the monitor is blind to
     exactly the event it exists for.
  2. RE-ARMING FROM THE MISSED DEADLINE. The reactor would then owe a burst of catch-up fires,
     piling work onto a host that just proved it is struggling, and the catch-up fires
     themselves read as late -- one stall inflates into a storm of fake ones.

Also pinned here: the watchdog must stay armed when `enabled: False`, so the no-freeze arm is
still instrumented (memory: defences_must_be_tested_independently -- a second defence that
quietly repairs the baseline arm stops the suite detecting either regression).

Fakes only the reactor/printer/config seams; the class under test is the real one, loaded from
the real file, same approach as keepalive_unit_test.py and latebef_verdict_test.py.
"""
import sys, os, importlib.util, logging
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# gc_tune.py sits beside phase_exec.py in BOTH layouts (klipper-port/extras in the overlay,
# <root>/klippy/extras in the release fork), so take the extras directory from the path the
# fold already rewrites instead of deriving '../../extras' here, which would resolve to
# <root>/coreone/extras in the fork layout. This needs no new export from golden.py.
import golden
GC_TUNE_PY = os.path.join(os.path.dirname(golden.PHASE_EXEC_PY), 'gc_tune.py')

spec = importlib.util.spec_from_file_location('gct', GC_TUNE_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
GCTune = mod.GCTune

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


# ----------------------------------------------------------------------------- the seams
class FakeReactor(object):
    NEVER = 9e99

    def __init__(self):
        self.t = 1000.0
        self.timers = {}
        self.updates = []

    def monotonic(self):
        return self.t

    def register_timer(self, cb, waketime=NEVER):
        self.timers[cb] = waketime
        return cb

    def update_timer(self, handle, waketime):
        self.timers[handle] = waketime
        self.updates.append(waketime)


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
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default='__raise__'):
        if name in self.objects:
            return self.objects[name]
        if default == '__raise__':
            raise Exception("Unknown config object '%s'" % (name,))
        return default

    def register_event_handler(self, name, cb):
        self.handlers[name] = cb


class FakeConfig(object):
    def __init__(self, printer, **vals):
        self.printer = printer
        self.vals = vals

    def get_printer(self):
        return self.printer

    def getboolean(self, name, default):
        return self.vals.get(name, default)

    def getfloat(self, name, default, minval=None, maxval=None):
        v = self.vals.get(name, default)
        if minval is not None and v < minval:
            raise Exception("%s below minval" % name)
        if maxval is not None and v > maxval:
            raise Exception("%s above maxval" % name)
        return v


class FakeGcmd(object):
    def __init__(self, gcode, **params):
        self.gcode = gcode
        self.params = params

    def get_int(self, name, default, minval=None, maxval=None):
        return self.params.get(name, default)

    def respond_info(self, msg):
        self.gcode.respond_info(msg)


def build(**cfg):
    """A GCTune with the watchdog armed and the reactor parked at a known instant."""
    p = FakePrinter()
    g = GCTune(FakeConfig(p, **cfg))
    return p, g


def fire(p, g, advance):
    """Advance the fake clock by `advance` and dispatch the watchdog once.

    Returns the waketime the callback asked for next.
    """
    p.reactor.t += advance
    return g._watchdog_event(p.reactor.t)


# --------------------------------------------------------- arming, and the enabled/watchdog split
print("arming:")
p, g = build()
g._arm_watchdog()
check(g._wd_timer is not None, "a timer is registered")
check(abs(g._wd_due - (1000.0 + g.wd_period)) < 1e-9,
      "first deadline is now + period (%.1f s)" % g.wd_period)

print("\nthe watchdog is independent of the freeze (so `enabled: False` stays instrumented):")
p, g = build(enabled=False)
p.reactor.t = 2000.0
g.printer.handlers['klippy:ready']()
check(g.enabled is False, "freeze is disabled")
check(g.watchdog is True and g._wd_timer is not None,
      "but the watchdog IS armed -- the A/B arm can still be measured")

print("\nwatchdog: False turns the measurement off without touching the freeze:")
p, g = build(watchdog=False)
p.reactor.t = 2000.0
g.printer.handlers['klippy:ready']()
check(g._wd_timer is None, "no timer registered")
check("OFF" in g._wd_report(), "the report SAYS it is off rather than printing a silent 0.0")

# ------------------------------------------------------------------------- on-time accounting
print("\nan on-time fire is not late:")
p, g = build()
g._arm_watchdog()
fire(p, g, 1.0)
check(g._wd_n == 1, "one sample recorded")
check(g._wd_max == 0.0, "worst lateness 0.0 ms")
check(sum(g._wd_over) == 0, "no bucket incremented")

print("\nan EARLY fire clamps to zero instead of reporting negative lateness:")
p, g = build()
g._arm_watchdog()
fire(p, g, 0.98)
check(g._wd_max == 0.0, "20 ms early -> 0.0 ms, not -20.0 ms")

# ------------------------------------------------------------- the headline: a real stall
print("\na 270 ms stall (the size that shut the MCU down twice):")
p, g = build()
g._arm_watchdog()
nxt = fire(p, g, 1.270)
check(abs(g._wd_max - 270.0) < 1e-6, "worst lateness reads 270.0 ms")
check(g._wd_over[-1] == 1, "the >=250 ms bucket caught it")
check(g._wd_over[0] == 1, "and it also counts in >=10 ms (buckets are cumulative)")

print("\nTRAP 1 -- lateness must be measured against the DEADLINE, not the previous fire:")
# Two consecutive 270 ms-late fires. If the yardstick were the previous fire, the second would
# read ~0 and a sustained stall storm would look clean after its first event.
nxt2 = fire(p, g, 1.270)
check(abs(g._wd_max - 270.0) < 1e-6, "the second stall still reads 270 ms, not ~0")
check(g._wd_over[-1] == 2, "both stalls counted in the >=250 ms bucket")

print("\nTRAP 2 -- re-arm from NOW, so a stall cannot owe a burst of catch-up fires:")
p, g = build()
g._arm_watchdog()
t_before = p.reactor.t
nxt = fire(p, g, 5.0)                  # a 4 s stall
check(nxt > p.reactor.t, "the next deadline is in the FUTURE (%.3f > %.3f)"
      % (nxt, p.reactor.t))
check(abs(nxt - (p.reactor.t + g.wd_period)) < 1e-9,
      "and it is exactly now + period, not deadline + period")
# The buggy version would return t_before + 2*period, which is already in the past.
check(nxt > t_before + 2 * g.wd_period,
      "a deadline-relative re-arm would have been in the past; this one is not")

# ------------------------------------------------------------------------ buckets and mean
print("\nbucket shape is preserved (a mean alone would hide it):")
p, g = build()
g._arm_watchdog()
for _ in range(100):
    fire(p, g, 1.012)                  # 12 ms late, a hundred times
fire(p, g, 1.300)                      # one 300 ms stall
check(g._wd_over[0] == 101, ">=10ms: 101")
check(g._wd_over[1] == 1, ">=25ms: 1 (only the big one)")
check(g._wd_over[-1] == 1, ">=250ms: 1")
check(abs(g._wd_max - 300.0) < 1e-6, "worst is the 300 ms one, not the mean")
rep = g._wd_report()
check("300.0 ms late" in rep, "the report leads with the worst case: %r"
      % rep.splitlines()[0].strip())
check("250ms:1" in rep, "and shows the bucket counts")

# ------------------------------------------------------------------------- context capture
print("\nthe worst case records WHERE it happened:")


class FakePrintStats(object):
    state = 'printing'


class FakeSD(object):
    file_position = 1234567


class FakeToolhead(object):
    print_time = 4321.5


p, g = build()
p.objects['print_stats'] = FakePrintStats()
p.objects['virtual_sdcard'] = FakeSD()
p.objects['toolhead'] = FakeToolhead()
g._arm_watchdog()
fire(p, g, 1.270)
check("print=printing" in g._wd_max_ctx, "records the print state")
check("sd_pos=1234567" in g._wd_max_ctx, "records the file position while printing")
check("print_time=4321.500" in g._wd_max_ctx, "records print_time")

print("\nsd_pos is SUPPRESSED when not printing (it is stale then -- the 2026-10-14 trap):")
p, g = build()
ps = FakePrintStats(); ps.state = 'complete'
p.objects['print_stats'] = ps
p.objects['virtual_sdcard'] = FakeSD()      # still holding 1234567 from the last print
g._arm_watchdog()
fire(p, g, 1.270)
check("print=complete" in g._wd_max_ctx, "records the state")
check("sd_pos" not in g._wd_max_ctx,
      "and does NOT quote the stale position: %r" % g._wd_max_ctx)

print("\ncontext lookup failures must not kill the watchdog:")


class Exploding(object):
    @property
    def state(self):
        raise RuntimeError("boom")


p, g = build()
p.objects['print_stats'] = Exploding()
g._arm_watchdog()
try:
    fire(p, g, 1.270)
    survived = True
except Exception as e:
    survived = False
check(survived, "a raising object is caught")
check(abs(g._wd_max - 270.0) < 1e-6, "and the measurement is still recorded")

# ------------------------------------------------------------------------------ warn once
print("\nthe log warning fires ONCE, not once per stall (a stall storm must not flood):")
p, g = build()
g._arm_watchdog()
warnings = []
_real = logging.warning
logging.warning = lambda *a, **k: warnings.append(a)
try:
    for i in range(5):
        # Strictly growing, so every fire sets a NEW max and every one is >= WARN_MS. That is
        # the case that matters: suppression must key off "already warned", not off "not a new
        # max", or a worsening stall storm would warn on every single fire.
        fire(p, g, 1.0 + 0.200 + 0.100 * i)
finally:
    logging.warning = _real
check(len(warnings) == 1, "exactly one warning for 5 growing stalls (got %d)" % len(warnings))
check("THE HOST STOPPED" in warnings[0][0], "and it says the host stopped")
check("NOT that GC did it" in warnings[0][0],
      "and it does NOT attribute the stall to GC -- lateness names a stall, not its cause")

# ----------------------------------------------------------------------------------- reset
print("\nRESET=1 zeroes the stats and does NOT run a collection:")
p, g = build()
g._arm_watchdog()
fire(p, g, 1.270)
check(g._wd_max > 0, "a stall is on the books")
calls = []
g._timed_collect = lambda gen=2: (calls.append(gen), (0.0, 0))[1]
g.cmd_GC_TUNE_STATUS(FakeGcmd(p.gcode, RESET=1))
check(g._wd_max == 0.0 and g._wd_n == 0, "stats zeroed")
check(not calls, "no level-2 collection was run (safe to call from PRINT_START)")
check("reset" in p.gcode.responses[-1], "and it says so")

print("\nGC_TUNE_STATUS without RESET does time a collection and includes the watchdog:")
p, g = build()
g._arm_watchdog()
fire(p, g, 1.050)
g._timed_collect = lambda gen=2: (0.4, 7)
g.cmd_GC_TUNE_STATUS(FakeGcmd(p.gcode))
out = p.gcode.responses[-1]
check("level-2 pass now: 0.4 ms" in out, "reports the timed collection")
check("worst 50.0 ms late" in out, "reports the worst stall")
check("within budget" in out, "and the verdict is scoped to the collection it measured")

# ---------------------------------------------------------------------------- get_status
print("\nCOLLECT=0 reports the stall figure WITHOUT timing a collection (safe from PRINT_END):")
p, g = build()
g._arm_watchdog()
fire(p, g, 1.270)
calls = []
g._timed_collect = lambda gen=2: (calls.append(gen), (999.0, 0))[1]
g.cmd_GC_TUNE_STATUS(FakeGcmd(p.gcode, COLLECT=0))
out = p.gcode.responses[-1]
check(not calls, "no collection was run -- a diagnostic cannot cause the fault it reports")
check("not timed (COLLECT=0)" in out, "and it says the pass was not timed")
check("worst 270.0 ms late" in out, "the watchdog figure is still reported")
check("within budget" not in out and "stalls the reactor" not in out,
      "and NO collection verdict is printed, since no collection was measured")

print("\nget_status exposes the headline for Moonraker / a PRINT_END macro:")
p, g = build()
g._arm_watchdog()
fire(p, g, 1.270)
st = g.get_status(p.reactor.t)
check(abs(st['late_max_ms'] - 270.0) < 1e-3, "late_max_ms = 270.0")
check(st['late_over_250ms'] == 1, "late_over_250ms = 1")
check(st['late_fires'] == 1, "late_fires = 1")
check(st['watchdog'] == 1, "watchdog = 1")

print()
if fails:
    print("GC-WATCHDOG GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("GC-WATCHDOG GATE: PASS -- lateness is measured against the deadline, the re-arm cannot"
      " burst, the stale sd_pos is suppressed and the warning does not attribute the stall to GC")
