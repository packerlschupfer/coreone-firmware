"""Gate: the lateBEF benign test must credit the window the ticks ACTUALLY accrued in, so its
verdict cannot depend on when it is asked.

⚠ TWO FIXES LIVE HERE, in the order they were found. The accumulator (ARMS B/C/E/F) removes a real
fail-open -- any motion used to erase the dormancy that excused the ticks. But it does NOT fix the
observed PRINT_END case, and ARM H is the arm that proves it: lateBEF accrues at tick_hz for
exactly as long as the ring is dry, so accrual == dormancy by construction, and subtracting a 1 s
guard then demands `dormancy >= dormancy + 1 s`. The dormancy path is UNSATISFIABLE for a parked
tail -- the very case it was written for. The real fix is to credit the MCU's own dry episode
(x_age/y_age), which is measured in the same clock the ticks are counted in and needs no
flush-lead correction.

    python3 latebef_stilltime_test.py

WHY THIS EXISTS. `_latebef_verdict` decides whether MCU lateBEF ticks are harmless (they accrued
while the gantry stood, so holding a stationary position lost nothing) or real (motion was
commanded and not executed). It used to credit only the CURRENT dormancy,
`now - _last_motion_wall`, which ANY motion resets.

That produced a false "XY IS WRONG / position desynced -- RE-HOME" at the end of EVERY print, on
2026-10-31 and again on 11-01 after a first fix aimed at the wrong window:

  14:20:10  PHASE_XY_CHECK: lateBEF=0 stale=0 mcuSTALE=0 -> xy_trusted=1
  14:20:25  Phase-stepping OFF. Re-home before printing (position desynced).
            lateBEF=39135  (39135 / 40 kHz = 0.978 s)

The ticks accrued while the gantry stood between the last print move and PRINT_END's park. Then
the PARK MOVE stamped `_last_motion_wall` ~1 s before the teardown, so `idle_s` read 0.0 and could
credit nothing. The same counter was ruled BENIGN by the same code 51 s later, once nothing had
flushed. 🔑 The park move both ENDED the still-period in which the ticks accrued AND RESET the
measure used to excuse them.

🔑 ARM B IS THE BUG ITSELF: ticks accrue while still, a motion ends the still-period, the verdict
is asked immediately -> must be BENIGN.

🔑 ARM C IS THE PROPERTY WHOSE ABSENCE WAS THE DEFECT: the same counter, same state, asked at two
different moments, must give the SAME verdict. A time-of-asking-dependent verdict is not
detectable as wrong from a log -- the real-world refutation only landed because a peer happened to
run the live query 51 s later instead of 2 s later. Had the delay been short, the false warning
would have reproduced and "confirmed" the wrong diagnosis. That is why this arm exists and why it
is worth more than ARM B.

⚠ FAIL-CLOSED IS PRESERVED, and ARM D pins it. A SUM is the correct bound, not a max: during a
still period of duration T at most T*tick_hz ticks can accrue harmlessly, so the total harmless
count is bounded by the sum of the still periods; anything beyond that accrued while motion WAS
commanded, i.e. real loss. And the sum cannot buy a fail-open beyond what one 1.7 s pause already
does, because `readable` excludes the saturating ceiling: a readable lb is < 65535 ticks = 1.638 s
at 40 kHz. A saturated counter stays un-creditable -- the lateBEF=11631 lesson.

Real class under test: loaded from the real file.
"""
import sys, os, importlib.util

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

PHASE_EXEC_PY = golden.PHASE_EXEC_PY
SRC = open(PHASE_EXEC_PY).read()
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)

# ⚠ CLEAN LATCH IS NOW A PRECONDITION OF "BENIGN" (2026-11-02, pfb latch veto).
# _latebef_verdict vetoes BOTH benign paths when the MCU staleness latch has fired, and treats an
# UNREADABLE count as fired ("unknown is not zero"). These arms exercise the parked-tail credit,
# which means a clean latch -- so they must say stale_events=0 explicitly. That precondition was
# previously IMPLICIT and WRONGLY ASSUMED, which is the fail-open the veto closes: on 2026-11-01's
# print 4 the dry-episode path credited a real lost park as benign because nothing consulted the
# latch. Making it explicit here is the faithful update, not a relaxation.
# See latebef_latch_veto_test.py for the veto's own arms.
def _vcall(d, lb, **kw):
    import inspect
    if 'stale_events' in inspect.signature(d._latebef_verdict).parameters:
        kw.setdefault('stale_events', 0)
    return d._latebef_verdict(lb, **kw)



def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))


cls = None
mod = None
try:
    spec = importlib.util.spec_from_file_location('pe_still', PHASE_EXEC_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for n in dir(mod):
        o = getattr(mod, n)
        if isinstance(o, type) and hasattr(o, '_latebef_verdict'):
            cls = o
            break
except Exception as e:
    print("  (import failed: %s -- source arms still run)" % e)

TICK_HZ = 40000.0


def make_dut(last_motion_wall=0.0, still_sum=0.0):
    class Dut(cls):
        def __init__(self):
            self.stepper_name = 'stepper_x'
            self._interval = 1
            self._inv_clock_s = 1.0 / TICK_HZ      # tick_hz = 40 kHz
            self._last_motion_wall = last_motion_wall
            self._latebef_still_sum = still_sum
            self._stale_segs = 0
            self._late_before = 0
            self._mcu_stale = 0
            self._group_slave = None
    return Dut()


def arm_a():
    check(cls is not None, "found the class exposing _latebef_verdict")
    check('_latebef_still_sum' in SRC, "the still-time accumulator exists")
    d = make_dut()
    b, t = _vcall(d, 0, now=100.0)
    check(b and t == "", "lb=0 is benign with empty text")
    check(d._latebef_still_sum == 0.0, "and lb=0 RESETS the accumulator (scopes it to the run)")


def arm_b():
    # THE BUG. 10 s of stillness accrued 0.978 s worth of ticks (39135 @ 40 kHz), then a park
    # move stamped _last_motion_wall 1 s ago. Old code: idle_s = max(0, 1 - 1 guard) = 0 -> NOT
    # benign -> "XY IS WRONG". New code: the 10 s still-period was banked.
    LB = 39135
    d = make_dut(last_motion_wall=999.0, still_sum=10.0 - 1.0)   # 9 s banked after the guard
    benign, text = _vcall(d, LB, now=1000.0)            # park was 1 s ago
    check(benign,
          "ticks accrued while still, then a motion ended the still-period, asked 1 s later"
          " -> BENIGN (the PRINT_END park scenario)")
    check('PARKED' in text and 'nothing was lost' in text,
          "and the text says PARKED / nothing was lost, not XY IS WRONG")
    # the same state WITHOUT the accumulator is the old behaviour, and must fail
    d_old = make_dut(last_motion_wall=999.0, still_sum=0.0)
    benign_old, text_old = _vcall(d_old, LB, now=1000.0)
    check(not benign_old,
          "and with NO banked still-time the same input is still flagged -- so the accumulator"
          " is what changes the verdict, not a loosened threshold")
    check('XY IS WRONG' in text_old, "(old-shape input still produces the XY IS WRONG text)")


def arm_c():
    # THE PROPERTY. Same counter, same banked history, asked 1 s after the park and 51 s after.
    # The old code flipped between these two. They must now agree.
    LB = 39135
    v = []
    for delay in (1.0, 3.0, 10.0, 51.0, 300.0):
        d = make_dut(last_motion_wall=1000.0, still_sum=9.0)
        v.append(_vcall(d, LB, now=1000.0 + delay)[0])
    check(len(set(v)) == 1,
          "the verdict is identical at +1/+3/+10/+51/+300 s after the motion (got %r)" % (v,))
    check(all(v), "and that single verdict is BENIGN")
    # and the pathological case the old code had: benign ONLY because of a long wait
    d1 = make_dut(last_motion_wall=1000.0, still_sum=0.0)
    d2 = make_dut(last_motion_wall=1000.0, still_sum=0.0)
    early = _vcall(d1, LB, now=1001.0)[0]
    late = _vcall(d2, LB, now=1051.0)[0]
    check(early != late,
          "WITHOUT banked still-time the verdict still flips on delay alone (%r vs %r) -- which"
          " is exactly the defect, and is why the accumulator is the fix" % (early, late))


def arm_d():
    # FAIL-CLOSED. Real loss = more ticks than the still-time can account for.
    d = make_dut(last_motion_wall=1000.0, still_sum=0.5)   # only 0.5 s of stillness banked
    benign, text = _vcall(d, 39135, now=1000.5)   # 0.978 s of ticks
    check(not benign,
          "ticks EXCEEDING the total still-time are still flagged (real loss stays detected)")
    check('XY IS WRONG' in text, "and the text still says XY IS WRONG")
    # a SATURATED counter is a bound, never creditable, however long the stillness
    sat = getattr(mod, 'PE_CNT_MAX', 0xFFFF)
    d2 = make_dut(last_motion_wall=0.0, still_sum=100000.0)  # absurd credit
    benign2, _ = _vcall(d2, sat, now=1e6)
    check(not benign2,
          "a SATURATED lb (%d) is NEVER benign even with 100000 s banked -- it is a bound, not"
          " a count (the 11631 lesson)" % sat)
    check(not _vcall(d2, sat + 7 * 65536, now=1e6)[0],
          "and the same holds above the ceiling")


def arm_e():
    # The guard must apply PER banked period, so sub-guard pauses credit nothing.
    flat = " ".join(SRC.split())
    inc = SRC.find('_latebef_still_sum +=')
    check(inc > 0, "the accumulator is incremented somewhere")
    # Guarded slice, not a blind split()[1]: against unpatched code the token is absent and
    # the index raises, which the arm harness converts to a failure but reports as an
    # IndexError instead of the actual missing property. Same lesson as the _STATS_TTL crash.
    after = SRC[inc:inc + 300] if inc > 0 else ''
    check('_LATEBEF_IDLE_GUARD_S' in after,
          "and the per-period guard is applied when banking (short pauses credit 0)")
    check('_latebef_still_sum = 0.' in SRC,
          "it is reset (lb==0 and engage)")
    check(SRC.count('_latebef_still_sum = 0.') >= 2,
          "reset happens at BOTH points, not just one (got %d)"
          % SRC.count('_latebef_still_sum = 0.'))


def arm_f():
    # Banking must happen BEFORE _last_motion_wall is overwritten, or the period is lost.
    i = SRC.find('_latebef_still_sum +=')
    j = SRC.find('self._last_motion_wall = _nw')
    check(i > 0 and j > 0, "found both the bank and the clock update")
    check(i > 0 and j > 0 and i < j,
          "the still-period is BANKED BEFORE the clock is overwritten (bank@%d < stamp@%d)"
          % (i, j))
    # slave mirror: an unmirrored accumulator is the same fail-open as an unmirrored clock
    check('sl._latebef_still_sum = self._latebef_still_sum' in SRC,
          "the accumulator is MIRRORED to the slave (unmirrored = fail-OPEN, one step back"
          " from the unmirrored dormancy clock)")


def arm_g():
    # The banner must report the composite window, not just the current dormancy -- otherwise a
    # reader cannot tell which window was actually credited.
    flat = " ".join(SRC.split())
    check('earlier still-time since lateBEF' in flat or 'earlier still-time' in flat,
          "the benign banner names the earlier still-time it credited")
    check('total' in flat and 'now +' in flat,
          "and shows the split (total = now + earlier), so the credit is auditable")
    check('still_s' in SRC, "the composite window is what the test uses")



def arm_h():
    # 🔑 THE REAL OBSERVED CASE, from print 3's teardown on 2026-11-01. Numbers verbatim:
    #   lateBEF=39135 (= 0.978 s at 40 kHz), dry X/Y = 0.98 s, park flush ~1.0 s earlier,
    #   and NO earlier still-period to bank. The accumulator alone leaves this broken.
    d = make_dut(last_motion_wall=1000.0 - 1.0, still_sum=0.0)
    benign, text = _vcall(d, 39135, now=1000.0, dry_s=0.98)
    check(benign,
          "THE OBSERVED PRINT_END CASE is benign: lb=39135, dry=0.98s, flush 1.0s ago, nothing"
          " banked (this is what the accumulator alone did NOT fix)")
    check('DRY episode' in text and 'nothing was lost' in text,
          "and the text attributes it to the MCU's dry episode")
    # and prove the dormancy path alone cannot do it -- the structural point
    d2 = make_dut(last_motion_wall=1000.0 - 1.0, still_sum=0.0)
    check(not _vcall(d2, 39135, now=1000.0)[0],
          "while WITHOUT the dry age the same input is still flagged -- so the dormancy path is"
          " genuinely unsatisfiable here, not merely tight")


def arm_i():
    # The dry path's boundaries. Each case is built so ONLY the dry path can decide it
    # (dormancy 1.0 s - 1.0 s guard = 0 credit), otherwise the dormancy path masks the result.
    # ⚠ An earlier draft of this arm used a 5 s dormancy and read a correct dormancy-path
    # credit as a fail-open. Construct the case so the path under test is the deciding one.
    NOW = 1000.0
    cases = [
        ("ticks EXCEED the dry window (60000 > 0.98*40000)", NOW - 1.0, 60000, 0.98, False),
        ("ticks fit the dry window exactly (39200)",          NOW - 1.0, 39200, 0.98, True),
        ("one tick over the dry window (39201)",              NOW - 1.0, 39201, 0.98, False),
        ("flush INSIDE the dry episode -> real loss",         NOW - 0.5, 39135, 0.98, False),
        ("flush exactly at the episode start",                NOW - 0.98, 39135, 0.98, True),
        ("SATURATED counter, 100 s dry window",               NOW - 5.0, 0xFFFF, 100.0, False),
        ("dry_s=0 (ring covered) -> dry path must not apply", NOW - 1.0, 39135, 0.0, False),
    ]
    for label, lmw, lb, dry, expect in cases:
        got = _vcall(make_dut(last_motion_wall=lmw), lb, now=NOW, dry_s=dry)[0]
        check(got == expect, "%s -> %s" % (label, "benign" if expect else "flagged"))


def arm_j():
    # dry_s must actually REACH the verdict, resolved from the snapshot or a live query.
    flat = " ".join(SRC.split())
    # ⚠ UPDATED 2026-11-02: the production call grew stale_events for the latch veto, and this
    # arm failed -- which is the arm WORKING. A source assertion on an exact call should fail
    # when the call changes. Updated to the new form and extended, not loosened.
    check('_latebef_verdict(lb, dry_s=dry_s, stale_events=stale_ev)' in flat,
          "the verdict is called WITH the dry age AND the latch count")
    check('stale_ev = None' in SRC,
          "and stale_ev starts as None, so an unresolved count VETOES rather than reads clean")
    i = SRC.find('dry_s = None')
    j = SRC.find('benign, latewarn = self._latebef_verdict(lb, dry_s=dry_s, stale_events=stale_ev)')
    check(i > 0 and j > i, "and it is resolved BEFORE the verdict call")
    seg = SRC[i:j] if (i > 0 and j > i) else ''
    check("(snap or {}).get('stale')" in seg,
          "resolved from the pre-teardown SNAPSHOT when present (no extra round-trip there)")
    check('self.stale_query_cmd.send' in seg,
          "and from a live query otherwise, so standalone PHASE_SHAPE_STATS still works")
    check('x_age' in seg and 'y_age' in seg, "using the MCU's x_age/y_age")
    check('except Exception' in seg,
          "a resolve failure falls back rather than breaking the report")

arm("ARM A -- surface and the reset-on-zero:", arm_a)
arm("ARM B -- THE BUG: still -> motion ends it -> asked immediately -> BENIGN:", arm_b)
arm("ARM C -- THE PROPERTY: the verdict must not depend on WHEN it is asked:", arm_c)
arm("ARM D -- fail-closed preserved: real loss flagged, saturation never credited:", arm_d)
arm("ARM E -- the per-period guard and both reset points:", arm_e)
arm("ARM F -- bank BEFORE stamping, and mirror to the slave:", arm_f)
arm("ARM G -- the banner reports the composite window:", arm_g)
arm("ARM H -- THE REAL OBSERVED CASE from print 3 (dry-window path):", arm_h)
arm("ARM I -- the dry path is fail-closed at every boundary:", arm_i)
arm("ARM J -- the dry age is plumbed to the verdict:", arm_j)

print()
if fails:
    print("LATEBEF-STILLTIME GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("LATEBEF-STILLTIME GATE: PASS -- lateBEF is credited against TOTAL still-time, so a motion"
      " that ends a still-period no longer erases the window that excuses it; the verdict is"
      " identical whenever it is asked; real loss and saturated counters are still flagged; the"
      " period is banked before the clock is overwritten and mirrored to the slave")
