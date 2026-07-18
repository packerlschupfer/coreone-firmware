"""Gate: a lateBEF verdict may NOT credit anything as benign once the staleness latch has fired.

    python3 latebef_latch_veto_test.py

🚨 WHY THIS EXISTS -- a hole in a fix of mine, caught by reading the output of the print that
"validated" it. ebeb0d7f2 credits lateBEF against the MCU's CURRENT dry episode. On 2026-11-01's
print 4 it reported

    lateBEF=32275 ... accrued entirely inside the MCU's own 0.81s DRY episode ... nothing was lost

for a run where the park was GENUINELY LOST -- the head was found mid-bed, not at X242 Y211. The
ticks and the loss belonged to an EARLIER >=8 s episode that had already closed, and `dry` only ever
reports the CURRENT one. The fail-closed companion (`_last_motion_wall <= now - dry_s`) passed too,
because the park flush landed ~1.0 s before a 0.81 s episode.

That is the same attribute-a-measurement-to-the-wrong-window shape as the five wrong diagnoses that
preceded it -- this time inside the repair written for it. Hence a veto rather than a tighter sum:
`events > 0` PROVES a long uncovered episode happened, whether or not it is the one still open.

🔑 ARM A IS PRINT 4 ITSELF, with its verbatim numbers. It must come out NOT benign. On the pre-veto
code it comes out benign, which is the defect.

🔑 ARM C: UNKNOWN IS NOT ZERO. `stale_events=None` (query failed, or firmware without a latch) must
veto as well. A fix that treated an unreadable counter as "no events" would turn a failed query into
an all-clear -- the fail-OPEN direction, and the exact shape of the 2026-09-26 stale_segs=0 bug.

⚠ The veto is deliberately coarse: `events` is cumulative since engage, so a latch that fired early
and recovered still vetoes a genuinely-parked tail at teardown. That is the designed asymmetry --
a false untrusted costs a re-home, a false trusted costs the frame (phase_exec.py's own words).
ARM D pins that the benign paths DO still work when the latch never fired, so the veto cannot be
"fixed" later by simply making it unreachable.
"""
import sys, os, importlib.util

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

PHASE_EXEC_PY = golden.PHASE_EXEC_PY
SRC = open(PHASE_EXEC_PY).read()
TICK_HZ = 40000.0
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


spec = importlib.util.spec_from_file_location('pe_veto', PHASE_EXEC_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
cls = None
for n in dir(mod):
    o = getattr(mod, n)
    if isinstance(o, type) and hasattr(o, '_latebef_verdict'):
        cls = o
        break


def dut(last_motion_wall=0.0, still_sum=0.0):
    class Dut(cls):
        def __init__(self):
            self.stepper_name = 'stepper_x'
            self._interval = 1
            self._inv_clock_s = 1.0 / TICK_HZ
            self._last_motion_wall = last_motion_wall
            self._latebef_still_sum = still_sum
            self._stale_segs = 0
            self._late_before = 0
            self._mcu_stale = 0
            self._group_slave = None
    return Dut()


# ⚠ ADAPT THE CALL to whichever signature is present. Passing stale_events unconditionally made
# every arm die with TypeError against the pre-veto module -- five identical "ARM DID NOT RUN"
# lines and NOT ONE arm demonstrating the defect. A gate for a fail-open must exercise the old
# code's actual behaviour and show it is wrong, not merely fail to invoke it.
import inspect
ACCEPTS_EVENTS = 'stale_events' in inspect.signature(cls._latebef_verdict).parameters


def call(lb, now=1000.0, dry_s=None, events=None, lmw=None, still=0.0):
    d = dut(last_motion_wall=(now - 1.0) if lmw is None else lmw, still_sum=still)
    if ACCEPTS_EVENTS:
        return d._latebef_verdict(lb, now=now, dry_s=dry_s, stale_events=events)
    # pre-veto: the latch count is simply not consulted -- which IS the defect
    return d._latebef_verdict(lb, now=now, dry_s=dry_s)


def arm_a():
    check(ACCEPTS_EVENTS,
          "_latebef_verdict takes stale_events (absent = the latch count is never consulted,"
          " which is the fail-open itself)")
    # PRINT 4, VERBATIM. lb=32275 (0.807 s at 40 kHz), dry=0.81 s, park flush ~1.0 s earlier,
    # and the latch HAD fired (events=2, one episode across both axes). The park was lost.
    benign, text = call(32275, dry_s=0.81, events=2)
    check(not benign,
          "PRINT 4's verbatim case is NOT benign (lb=32275, dry=0.81s, events=2) -- the run"
          " whose park was physically lost")
    check('RE-HOME' in text,
          "and the text tells the operator to RE-HOME")
    check('already closed' in text or 'earlier episode' in text,
          "and explains WHY: the ticks can belong to an earlier, already-closed episode")
    # the pre-veto behaviour, reproduced: without the events argument it WOULD have credited
    benign_old, text_old = call(32275, dry_s=0.81, events=0)
    check(benign_old,
          "whereas the SAME ticks with events=0 are benign -- so the veto is what changes the"
          " verdict, not a tightened threshold")
    check('DRY episode' in text_old, "(and that path still names the dry episode)")


def arm_b():
    # Any nonzero event count vetoes, not just 2. And it vetoes the DORMANCY path too, not only
    # the dry path -- both are credits, and the latch having fired undermines both.
    for ev in (1, 2, 5, 65535):
        check(not call(32275, dry_s=0.81, events=ev)[0],
              "events=%d vetoes the dry path" % ev)
    # dormancy path: huge banked still-time would otherwise credit it easily
    benign, _ = call(32275, dry_s=None, events=3, lmw=1000.0 - 50.0, still=40.0)
    check(not benign,
          "and 50 s of dormancy + 40 s banked still-time does NOT credit it either while the"
          " latch has fired -- the veto covers BOTH benign paths")


def arm_c():
    # UNKNOWN IS NOT ZERO. A failed stale query must not read as an all-clear.
    benign, text = call(32275, dry_s=0.81, events=None)
    check(not benign,
          "stale_events=None (query failed / no latch in fw) VETOES -- unknown is not zero")
    check('unknown is not zero' in text or 'could not be read' in text,
          "and the text says so, rather than implying the latch was clean")
    # and with a long dormancy too, so it is not just the dry path being unavailable
    check(not call(32275, dry_s=None, events=None, lmw=1000.0 - 50.0, still=40.0)[0],
          "the None veto also covers the dormancy path")


def arm_d():
    # The veto must NOT make the benign paths unreachable -- otherwise the 'XY IS WRONG at every
    # PRINT_END' false alarm comes straight back, which is what ebeb0d7f2 existed to remove.
    b1, t1 = call(32275, dry_s=0.81, events=0)
    check(b1, "with events=0 the DRY path still credits a genuine parked tail")
    b2, t2 = call(32275, dry_s=None, events=0, lmw=1000.0 - 50.0, still=40.0)
    check(b2, "and with events=0 the DORMANCY path still credits one")
    check('nothing was lost' in t1 and 'nothing was lost' in t2,
          "both still say nothing was lost, so the false-alarm fix is intact")
    # lb == 0 is benign regardless -- nothing accrued, nothing to attribute
    b3, t3 = call(0, dry_s=None, events=99)
    check(b3 and t3 == "",
          "and lb=0 stays benign with empty text even with events=99 -- nothing accrued")


def arm_e():
    # Fail-closed properties that must survive the veto.
    sat = getattr(mod, 'PE_CNT_MAX', 0xFFFF)
    check(not call(sat, dry_s=100.0, events=0)[0],
          "a SATURATED lb is still never creditable (a bound, not a count) -- the 11631 lesson")
    # real loss with a clean latch: more ticks than the dry window can hold
    check(not call(60000, dry_s=0.81, events=0)[0],
          "and ticks exceeding the dry window are still flagged with events=0")
    flat = " ".join(SRC.split())
    check('unknown is not zero' in flat.lower(),
          "the source states the unknown-is-not-zero rule where the veto lives")
    check('stale_events' in SRC and '_latebef_verdict(lb, dry_s=dry_s, stale_events=' in flat,
          "and the events count is actually PLUMBED to the verdict, not defaulted away")


arm("ARM A -- PRINT 4's verbatim case must NOT be benign:", arm_a)
arm("ARM B -- any nonzero event count vetoes BOTH benign paths:", arm_b)
arm("ARM C -- UNKNOWN IS NOT ZERO: a failed query vetoes:", arm_c)
arm("ARM D -- but the benign paths still work when the latch never fired:", arm_d)
arm("ARM E -- the pre-existing fail-closed properties survive:", arm_e)

print()
if fails:
    print("LATEBEF-LATCH-VETO GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("LATEBEF-LATCH-VETO GATE: PASS -- once the staleness latch has fired, no lateBEF is credited"
      " as benign by either path; an unreadable event count vetoes too; and with a clean latch the"
      " parked-tail credit still works, so the false-alarm fix is not undone")
