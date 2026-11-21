"""Gate: the PAUSE_VALIDATE harness for the engaged-pause acceptance condition.

    python3 pause_validate_test.py

WHAT IT GUARDS. The standing rule is "print with PHASE_STEP=0 when the job needs a pause",
because an engaged pause has lost XY on this machine repeatedly: 10-09, 10-12, 10-13, 10-26,
10-27, 10-30, 10-31, 11-01. A pre-registered condition lifts it -- 5 independent engaged runs,
>=2 short (<60 s) and >=2 long (>600 s), every one clean -- and PAUSE_VALIDATE drives it.

🔑 THE PROPERTY THAT MATTERS MOST (ARM B): the run is VOID unless phase stepping is ENGAGED.
A PHASE_STEP=0 pause is the known-good path and passes every criterion trivially, so counting
one toward the five would be precisely the self-deception the pre-registered condition exists
to prevent. The macro raises rather than warns.

⚠ WHY THERE IS NO INTERNAL DWELL (ARM D). A G4 long enough for the >600 s arm would hold the
GCODE MUTEX for ten minutes, blocking the console AND the LCD menu -- and an LCD item is
queue_gcode, so it could not even abort. The operator holds the pause; the macro brackets it.

⚠ WHY THE READOUT IS A SUB-MACRO (ARM C). A {} expression in a gcode_macro renders ONCE at
invocation, before the body runs, so a parent that calls PHASE_XY_CHECK and then reads
xy_trusted in the same template reads the STALE cache. phase_exec.py says so at its get_status.

⚠ AND WHY THE VERDICT IS EXPLICITLY INCOMPLETE (ARM E). get_status cannot see the per-axis
BEFORE counter, and the teardown verdict exists only at disengage. The 2026-10-27 failure was
INVISIBLE to PHASE_XY_CHECK and was found only in the teardown MCU-SHAPE report -- which is why
a sweep of xy_trusted alone once read as "no failures since 10-15". A harness that implied its
own verdict was sufficient would rebuild that blind spot.

ARMS
  A  all six macros exist and every Jinja template COMPILES (a broken one stops klippy)
  B  the run is void unless engaged -- and it RAISES, not warns
  C  every runtime read happens in a sub-macro, not in the parent
  D  no internal dwell (no G4 holding the mutex)
  E  the mid-run verdict says it is incomplete, naming bef and the teardown
  F  a failing run RESETS the tally to zero (the pre-registered no-renegotiation rule)
"""
import sys, os, re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

EXTRAS = os.path.dirname(golden.PHASE_EXEC_PY)
CFG = os.path.join(os.path.dirname(EXTRAS), 'printer.cfg')

fails = []
skips = []
checks = 0


def check(ok, what):
    global checks
    checks += 1
    print("  %-4s %s" % ("ok" if ok else "FAIL", what))
    if not ok:
        fails.append(what)


def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))


NAMES = ['PAUSE_VALIDATE', '_PV_START', 'PAUSE_VALIDATE_RESUME', '_PV_READ',
         'PAUSE_VALIDATE_STATUS', '_PV_STATUS']

SRC = open(CFG).read() if os.path.exists(CFG) else None


def blocks():
    """name -> gcode template text, for our macros only."""
    out = {}
    if SRC is None:
        return out
    for name, body in re.findall(
            r'\n\[gcode_macro (PAUSE_VALIDATE\w*|_PV_\w+)\]\n(.*?)(?=\n\[)', SRC, re.S):
        m = re.search(r'(?:^|\n)\s*gcode:\n(.*)$', body, re.S)
        if m:
            out[name] = "\n".join(l[4:] if l.startswith('    ') else l
                                  for l in m.group(1).splitlines())
    return out


B = blocks()


def need():
    if SRC is None:
        skips.append("printer.cfg not present (fork layout) -- PAUSE_VALIDATE lives in the "
                     "overlay's printer.cfg, which the fork does not carry")
        print("  SKIP %s" % skips[-1])
        return False
    return True


def arm_a():
    if not need():
        return
    for n in NAMES:
        check(n in B, "macro %s exists with a gcode: block" % n)
    try:
        import jinja2
    except ImportError:
        skips.append("jinja2 unavailable -- template compilation not checked")
        print("  SKIP %s" % skips[-1])
        return
    env = jinja2.Environment()
    for n, tmpl in sorted(B.items()):
        try:
            env.parse(tmpl)
            check(True, "%s: Jinja compiles" % n)
        except Exception as e:
            check(False, "%s: JINJA ERROR (would stop klippy): %s" % (n, e))


def arm_b():
    if SRC is None:
        return
    t = B.get('_PV_START', '')
    check('engaged' in t, "_PV_START reads phase_exec's engaged flag")
    check('action_raise_error' in t,
          "a non-engaged run RAISES (a warning would let a trivial pass be counted)")
    # the raise must be the not-engaged branch, not only the wrong-state branch
    m = re.search(r'if not eng\b.*?action_raise_error', t, re.S)
    check(m is not None, "the raise is on the NOT-ENGAGED branch specifically")
    check('print_stats.state' in t or "print_stats" in t,
          "_PV_START also requires an actually-printing state")


def nocomments(text):
    """Strip whole-line # comments. A presence check that reads comment prose is not a
    check -- ARM C's first version failed because 'xy_trusted' appears in the parent's
    explanatory comment, which is exactly the trap this project has hit before."""
    return "\n".join(l for l in text.splitlines() if not l.lstrip().startswith('#'))


def arm_c():
    if SRC is None:
        return
    parent = nocomments(B.get('PAUSE_VALIDATE_RESUME', ''))
    child = nocomments(B.get('_PV_READ', ''))
    check('PHASE_XY_CHECK' in parent, "the parent refreshes via PHASE_XY_CHECK")
    check('_PV_READ' in parent, "the parent then calls the _PV_READ sub-macro")
    check('xy_trusted' not in parent,
          "the parent does NOT read xy_trusted itself (renders-once trap)")
    check('xy_trusted' in child, "the sub-macro is where xy_trusted is read")
    # fail-closed sentinels
    check('-1' in child, "unreadable counters default to a -1 sentinel, not 0")
    m = re.search(r'ok\s*=\s*\(?\s*xt\s*==\s*1', child)
    check(m is not None, "the verdict requires xy_trusted == 1 explicitly")
    for f in ('lb == 0', 'ms == 0', 'ss == 0'):
        check(f in child, "the verdict requires %s" % f)
    # 🔴 gap_moves_kept must NOT gate the verdict. get_status says it "is deliberately NOT a
    # distrust signal: since the fix it counts motion the host RECOVERED from a skipped span".
    # An engaged pause PARKS, so those moves land in the gap the bridge recovers and gk is
    # non-zero on a CORRECT machine -- gating on gk==0 fails every run, and a FAIL zeroes the
    # tally, so the harness could never accumulate. This arm exists because the gate's first
    # version asserted the bug.
    m2 = re.search(r'set ok = \([^)]*gk\s*==\s*0', child)
    check(m2 is None, "gap_moves_kept is NOT part of the PASS test (it is the fix working)")
    check('neither a fault' in child,
          "the report labels gap_skips AND kept as not-faults (neither gates the verdict)")
    # 🔴 The positive control is gap_skips, NOT gap_moves_kept. gap_skips counts >1s flush
    # windows while engaged (the mechanism's precondition); gap_moves_kept counts only the
    # sub-case where a move landed inside one. PROOF gk is wrong for this: the FAILING
    # 2026-10-27 run recorded "GAPSKIP=3 (kept 0)" -- a counter that reads 0 in the known
    # failure cannot evidence that a run reached the failure path.
    check('gap_skips' in child, "the readout reads gap_skips, not only gap_moves_kept")
    # Stream-derived counters only ever increment on the CoreXY MASTER (one flush cb, on the
    # engaged object: phase_exec.py:3002). stepper_y's are structurally 0, so polling both axes
    # and gating on "both > 0" would make the campaign unpassable.
    check("phase_exec stepper_x" in child and "phase_exec stepper_y" not in child,
          "the readout polls stepper_x ONLY (stream counters are master-only)")
    # NB: `child` is comment-STRIPPED (nocomments), so the rationale must be checked against
    # the RAW macro body or this arm can never pass.
    check('DO NOT "FIX" THIS TO POLL BOTH AXES' in B.get('_PV_READ', ''),
          "the master-only constraint is documented at the read site (raw body)")
    # 🔑 The control must be a DELTA across the pause. gap_skips is cumulative per ENGAGE, so
    # an absolute reading cannot attribute a gap to the pause -- it may have opened 20 layers
    # earlier during ordinary printing.
    # 🔴 gap_skips does NOT gate. The 2026-09-23 scrap test exonerated the gap-skip path
    # (GAPSKIP=2, reanch=0, travel still dropped via lateBEF=65045); the mechanism is the MCU
    # shaper's ~10.6ms lookback ring draining over an engaged idle, which EVERY pause does by
    # 3-4 orders of magnitude. So the tally counts clean runs, not exercised-by-gap_skips runs.
    check('INFORMATIONAL ONLY' in child and 'DO count it' in child,
          "a gap_skips delta of 0 is INFORMATIONAL -- the run still counts")
    check('pv_short' not in child and 'pv_long' not in child,
          "the duration ARMS no longer gate the tally (hold duration is non-causal)")
    check('pv_clean' in child, "the tally is a single clean-run counter")

    # 🔴 THE CLASS OF BUG THAT COST RUN 1 AND measure-B: a MUX gcode command called without its
    # required key. `PHASE_XY_CHECK` is register_mux_command(..., 'STEPPER', ...), so a bare call
    # raises "missing STEPPER" and aborts the CALLING macro -- here, one line before _PV_READ, so
    # the readout and tally vanished AND the late_before/mcu_stale caches went unrefreshed.
    # Compiling the Jinja (ARM A) cannot catch this: it is valid Jinja and an invalid gcode call.
    MUX_CMDS = {'PHASE_XY_CHECK': 'STEPPER', 'PHASE_DMA_STATS': 'STEPPER',
                'SET_PHASE_KEEPALIVE': 'STEPPER', 'PHASE_TRAPQ_ENGAGE': 'STEPPER'}
    for mname, mbody in B.items():
        for line in mbody.splitlines():
            bare = line.split('#')[0].strip()
            for cmd, key in MUX_CMDS.items():
                if bare == cmd or bare.startswith(cmd + ' '):
                    check(key + '=' in bare,
                          "%s calls %s WITH %s= (mux command; a bare call aborts the macro)"
                          % (mname, cmd, key))

    # 🔴 ORDERING: RESUME queues the restore travel and returns without waiting, so the refresh
    # must be fenced behind M400 or PHASE_XY_CHECK queries a PRE-TRAVEL state -- reporting
    # lateBEF=0 for a run whose travel was about to vanish. Instrument before event.
    res = B.get('PAUSE_VALIDATE_RESUME', '')
    order = [l.split('#')[0].strip() for l in res.splitlines() if l.split('#')[0].strip()]
    check(order[:2] == ['RESUME', 'M400'],
          "PAUSE_VALIDATE_RESUME fences the refresh behind RESUME then M400")
    check(order.index('M400') < order.index('PHASE_XY_CHECK STEPPER=stepper_x'),
          "M400 precedes the cache refresh (travel completes before the MCU is queried)")
    check(order.index('PHASE_XY_CHECK STEPPER=stepper_x') < order.index('_PV_READ'),
          "the refresh precedes the readout")
    check('lateBEF=65045' in B.get('_PV_READ', ''),
          "the exoneration of the gap-skip path is cited at the decision site")
    check('pv_gs0' in child, "_PV_READ reads the pv_gs0 snapshot to form the delta")

    # And the PRODUCER must actually save it -- testing only the consumer is the bound-method
    # identity trap again (a reader that defaults a never-written variable reads -1 forever).
    start = B.get('_PV_START', '')
    check('SAVE_VARIABLE VARIABLE=pv_gs0' in start,
          "_PV_START SAVES pv_gs0 (producer, not just the consumer)")
    check('gap_skips' in start,
          "_PV_START's snapshot reads gap_skips off stepper_x at pause time")

    # The keep-alive witnesses decide whether PERIOD is even the right injection lever, so they
    # are snapshotted by the producer and reported EVERY run, pass or fail.
    for v in ('pv_kaf0', 'pv_kaml0'):
        check('SAVE_VARIABLE VARIABLE=%s' % v in start,
              "_PV_START snapshots %s (keep-alive witness)" % v)
    check('ka_maxlag' in child and 'ka_period' in child,
          "the readout reports ka_maxlag and ka_period")
    check('PERIOD is NOT the lever' in B.get('_PV_READ', ''),
          "a maxlag past GAP_MAX with a zero delta is called out as UNINTERPRETABLE for tuning")
    check(re.search(r'gk == 0', child) is not None and 'expected shape' in child,
          "gk==0 with gs>0 is reported as EXPECTED, not as a problem")


def arm_d():
    if SRC is None:
        return
    joined = "\n".join(B.values())
    body = "\n".join(l for l in joined.splitlines() if not l.lstrip().startswith('#'))
    check(not re.search(r'\bG4\b', body),
          "no G4 dwell anywhere in the harness (would hold the gcode mutex)")


def arm_e():
    if SRC is None:
        return
    t = B.get('_PV_READ', '')
    low = t.lower()
    # ⚠ "mid-run" alone is a WEAK check: it also appears in the per-run PASS/FAIL line, so a
    # mutation that gutted the incompleteness notice while leaving that line would pass it.
    # Pin the actual CLAIM instead -- that the run is not complete.
    check('not complete' in low, "the verdict states the run is NOT COMPLETE")
    check(low.count('mid-run') >= 2, "both the verdict line and the notice say MID-RUN")
    check('bef' in low, "it names bef, which get_status cannot see")
    check('teardown' in low, "it names the teardown report as the missing half")
    check('65535' in t, "it gives the bef saturation value to look for")


def arm_f():
    if SRC is None:
        return
    t = B.get('_PV_READ', '')
    check(re.search(r'if not ok', t) is not None,
          "a FAILING run resets its tally to 0 (pre-registered rule)")
    for v in ('pv_clean', 'pv_mid_clean'):
        check('%s VALUE=0' % v in t, "a FAIL can reset %s" % v)
        check('%s VALUE={ cl + 1 }' % v in t, "a PASS can increment %s" % v)
    # 🔴 TWO CAMPAIGNS, TWO TALLIES. between-layer (target 5, met) vs mid-extrusion (target 2,
    # the M600/runout class). A fail in one must NOT erase the other -- they are independent
    # evidence about different pause classes.
    check('target = 2 if mode' in t and "else 5" in t,
          "the target is per-mode: 2 for mid-extrusion, 5 for between-layer")
    check("OTHER" in t and "untouched" in t,
          "a FAIL states that the other campaign's tally is untouched")
    # the reset must be INSIDE the mode branch, not a blanket reset of both
    reset_blk = re.search(r'if not ok.*?\{%\s*endif\s*%\}', t, re.S).group(0)
    check(reset_blk.count('VALUE=0') == 2 and "if mode == 'mid'" in reset_blk,
          "the reset is mode-branched (exactly one counter zeroed per fail)")
    start = B.get('_PV_START', '')
    check("MODE must be 'layer' or 'mid'" in start,
          "_PV_START REJECTS an unknown MODE rather than silently defaulting")
    check('SAVE_VARIABLE VARIABLE=pv_mode' in start,
          "_PV_START persists the mode so _PV_READ credits the right tally")
    check('SET BY HELMUT IN ADVANCE' in start,
          "the provenance of the target-2 pre-registration is recorded at the decision site")
    check('60' in t and '600' in t, "the arm thresholds 60 s / 600 s are present")


arm("ARM A -- macros exist and templates compile", arm_a)
arm("ARM B -- a non-engaged run is VOID and raises", arm_b)
arm("ARM C -- runtime reads live in a sub-macro, fail-closed", arm_c)
arm("ARM D -- no internal dwell (mutex)", arm_d)
arm("ARM E -- the verdict declares itself incomplete", arm_e)
arm("ARM F -- a failure resets the tally", arm_f)

print("\n" + "=" * 50)
for s in skips:
    print("SKIPPED: %s" % s)
if fails:
    print("pause_validate gate: %d/%d checks FAILED" % (len(fails), checks))
    for f in fails:
        print("  - " + f)
    sys.exit(1)
print("pause_validate gate: %d/%d checks passed" % (checks, checks))
