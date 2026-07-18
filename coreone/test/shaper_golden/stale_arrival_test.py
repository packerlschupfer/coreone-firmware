"""Reproduction for the STALE-ARRIVAL motion loss (built 2026-10-09).

    python3 stale_arrival_test.py

WHY THIS EXISTS. gap_bridge_test.py closed the BEFORE storm (a ring HOLE across a long pause:
3,151,364 PE_EVAL_BEFORE -> 0 once bridged) but says plainly in its own docstring that it does
NOT reproduce the motion loss seen on the printer, because "the harness feeds the whole ring up
front and ticks monotonically, so it cannot model segments ARRIVING after their clock window has
already elapsed". That is the separate condition the host already DETECTS and cannot fix:
extras/phase_exec.py counts it as `_stale_segs` and warns "segment emitted N ms INTO THE PAST
while engaged -- commanded motion cannot be played and XY will be silently offset".

The harness protocol needed no change: tokens are processed in STREAM ORDER, so a SEG placed
between two TICKs is an arrival at that moment. golden.run_harness_timeline exposes that.

METHOD -- the two arms differ ONLY in arrival order, nothing else:
  * identical pulses (the machine's live shapers, mzv 63.6 / 49.4), identical PROJ, retire on
  * identical segment list, identical tick list, identical start/end measurement points
  * ON TIME : the travel segments are pushed BEFORE the ticks reach their start_clock
  * LATE    : the SAME segments are pushed only AFTER the ticks have passed start_clock+duration
So any difference in commanded travel is attributable to arrival order alone.
⚠⚠ READ THIS BEFORE TRUSTING THE "OK" COLUMN -- THE PASS CRITERION IS NOT SAFETY.
"OK" here means only "the 40 mm of displacement was commanded". ABOVE the band it is commanded
by the past-the-head branch FREEZING AT THE SEGMENT ENDPOINT -- i.e. as a 40 mm INSTANTANEOUS
JUMP, which at the gantry is a lurch, the class of event that rams a rail. So BOTH outcomes of
a late-arriving segment are bad:
    inside the band  -> the move is LOST          (silent XY desync)
    outside the band -> the move is JUMPED        (lurch)
MEASURED 2026-10-09, loss band = [12.783 s, 25.567 s) = [2^31, 2^32) ticks @168 MHz:
    on time   +40.000   5.0s +40.000   15.0s +0.000   20.0s +0.000   30.0s +40.000
A FIX ATTEMPT THAT FAILED, recorded so it is not repeated: guarding the backward walk against
an implausible negative delta (PE_BACKSTEP_MAX_S) moved the loss from the walk (backscan=3) to
the BEFORE branch (before=13551, backclamp=13551) -- which freezes at start_pos, so the move
was STILL lost, 0.000 mm. Reverted. A wrapped negative delta is misread as "tau precedes this
segment" by the walk AND by the BEFORE branch; patching one just hands it to the other.
CONCLUSION: this is NOT an evaluator bug to patch. A segment whose window has already elapsed
must never be ACTED ON at all -- the host should not emit it. extras/phase_exec.py already
DETECTS the condition (`_stale_segs`, "segment emitted N ms INTO THE PAST while engaged") and
emits anyway; that is where the fix belongs, by re-anchoring and resuming from the present so
the lost motion is VISIBLE (xy_trusted=0) instead of silent.
"""
import sys, os; sys.path.insert(0, '.')
import golden
golden.build_harness()
CLOCK = golden.CLOCK; SPM = 200.0; M32 = 0xffffffff
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)
PAUSE = 180.0
BRIDGE_MAX = 8.0

EV_BEFORE = 1 << 2
EV_PAST   = 1 << 1
EV_EMPTY  = 1 << 0
PR_HELD   = 1 << 0
PR_DRY    = 1 << 1


def build():
    """Historical segments (pre-pause + bridge + lead-in) and the travel segments, split."""
    S = int(1.0 * CLOCK)
    hist_x, hist_y, trav_x, trav_y = [], [], [], []
    def mk(t_s, dur_s, pos_mm, v_mms, ra=0):
        return ((int(round(t_s)) & M32, int(round(dur_s)), pos_mm*SPM, v_mms*SPM, 0.0, ra),
                (int(round(t_s)) & M32, int(round(dur_s)), 0.0, 0.0, 0.0, ra))
    def add_hist(*a, **k):
        x, y = mk(*a, **k); hist_x.append(x); hist_y.append(y)
    def add_trav(*a, **k):
        x, y = mk(*a, **k); trav_x.append(x); trav_y.append(y)

    add_hist(S, 0.25*CLOCK, 0.0, 40.0)                 # pre-pause move
    add_hist(S + 0.25*CLOCK, 0.05*CLOCK, 10.0, 0.0)    # trailing hold
    t = S + 0.30*CLOCK
    resume = S + (0.30 + PAUSE) * CLOCK
    while t < resume - 1:                              # BRIDGED: no ring hole in either arm
        n = min(resume, t + BRIDGE_MAX*CLOCK)
        add_hist(t, n - t, 10.0, 0.0)
        t = n
    add_trav(resume, 0.10*CLOCK, 10.0, 0.0, ra=1)            # lead-in hold, carries reanchor
    add_trav(resume + 0.10*CLOCK, 0.10*CLOCK, 10.0, 400.0)   # THE TRAVEL 10 -> 50 mm
    add_trav(resume + 0.20*CLOCK, 0.30*CLOCK, 50.0, 0.0)     # settle hold
    return (hist_x, hist_y), (trav_x, trav_y), S, resume


def run(late_s):
    """late_s = seconds AFTER the travel window has fully elapsed before the segments arrive.

    The first version of this test used 0.2 s and did NOT reproduce the loss: the evaluator's
    past-the-head branch freezes at the segment ENDPOINT, so the full displacement still gets
    commanded (as a jump). That is a real finding -- arrival order alone is not enough.
    The ingredient it was missing is the SIGNED-32 CLOCK WINDOW that phase_shaper.c:122 and
    _bridge_gap both warn about: `int32_t trel = (int32_t)(tau - sg->start_clock)` is only
    meaningful within +-2^31 ticks = 12.783 s at 168 MHz. A real pause is MINUTES. Past that,
    a segment arriving stamped in the past has its delta WRAP NEGATIVE, so the evaluator reads
    it as BEFORE and freezes at start_pos instead of the endpoint -- and the displacement is
    gone. 0 < late_s < 12.783 should complete; late_s > 12.783 should lose it.
    """
    (hx, hy), (tx, ty), S, resume = build()
    step = int(CLOCK/40000)
    # push_after: when the travel batch enters the stream. late_s=None means "on time".
    if late_s is None:
        push_after = resume + 0.05*CLOCK      # ahead of start_clock -> normal arrival
        end = int(resume + 0.45*CLOCK)
    else:
        push_after = resume + (0.30 + late_s)*CLOCK   # window fully elapsed by late_s
        end = int(push_after + 0.15*CLOCK)
    ticks = []
    t = int(S + 0.10*CLOCK)
    while t < end:
        ticks.append(t); t += step
    events = []
    for sg in hx: events.append(('SEG', 0, sg))
    for sg in hy: events.append(('SEG', 1, sg))
    pushed = False
    for tk in ticks:
        if not pushed and tk > push_after:
            for sg in tx: events.append(('SEG', 0, sg))
            for sg in ty: events.append(('SEG', 1, sg))
            pushed = True
        events.append(('TICK', tk & M32))
    if not pushed:                       # safety: never silently drop the travel
        raise SystemExit('stale_arrival: travel segments were never pushed')
    # Pin the C STALENESS LATCH out of the way (PE_STALE_S = 8 s). This vector documents the
    # PRE-LATCH evaluator -- the signed-32 wrap the latch exists to stop -- so with the latch
    # live the arm below simply HOLDS and proves nothing about stale ARRIVAL ordering.
    # The latch has its own gate: stale_latch_test.py.
    got, err = golden.run_harness_timeline(px_dt, px_q[0], py_dt, py_q[0],
                                           events, proj, retire=True, stale=600.0)
    ref = int(resume + 0.05*CLOCK) & M32
    pre = [g for g in got if g['now'] == ref]
    a_pre = pre[0]['posA'] if pre else got[0]['posA']
    a_end = got[-1]['posA']
    st = lambda bit: sum(1 for g in got if (g['stx']|g['sty']) & bit)
    pr = lambda bit: sum(1 for g in got if (g['pa']|g['pb']) & bit)
    return {'travel': (a_end - a_pre)/SPM, 'before': st(EV_BEFORE), 'past': st(EV_PAST),
            'empty': st(EV_EMPTY), 'lagged': st(1 << 3), 'held': pr(PR_HELD),
            'dry': pr(PR_DRY), 'nseg': len(hx) + len(tx), 'nt': len(ticks),
            'counters': err.strip()}


print("%-28s %-15s %-8s %-8s %-8s %-8s %-7s"
      % ("arrival", "motorA travel", "BEFORE", "PAST", "LAGGED", "EMPTY", "HELD"))
print("-" * 92)
rows = {}
# The signed-32 delta is NEGATIVE only for a lateness in [2^31, 2^32) ticks =
# [12.783 s, 25.567 s). Below that it reads PAST (freeze at the endpoint -> displacement still
# delivered); ABOVE 25.567 s it wraps a SECOND time back to positive and reads PAST again.
# So the loss window is a BAND, not a threshold -- 30 s overshot it on the first attempt.
ARMS = ((None, "ON TIME (normal)"), (5.0, "LATE  5.0s (below band)"),
        (15.0, "LATE 15.0s (IN the band)"), (20.0, "LATE 20.0s (IN the band)"),
        (30.0, "LATE 30.0s (above band)"))
for late, name in ARMS:
    r = run(late); rows[late] = r
    ok = "OK" if abs(r['travel'] - 40.0) < 0.5 else "*** LOST %.3f mm ***" % (40.0 - r['travel'])
    print("%-28s %+-15.3f %-8d %-8d %-8d %-8d %-7d %s"
          % (name, r['travel'], r['before'], r['past'], r['lagged'], r['empty'],
             r['held'], ok))
    for ln in r['counters'].splitlines():
        print("      harness: %s" % ln)
print("\n(expected motorA travel = +40.000 mm; ticks per run = %d, segments = %d)"
      % (rows[None]['nt'], rows[None]['nseg']))

on, lt = rows[None], rows[15.0]
print()
if abs(on['travel'] - 40.0) >= 0.5:
    print("INCONCLUSIVE: the ON-TIME arm already fails (%.3f mm) -- the vector is wrong,"
          " not the code." % on['travel'])
    raise SystemExit(2)
if abs(lt['travel'] - 40.0) < 0.5:
    print("NOT REPRODUCED: late arrival still travelled %.3f mm. Arrival order alone does not"
          " lose the motion in this vector -- the hardware loss needs some other ingredient."
          % lt['travel'])
    raise SystemExit(3)
print("STALE-ARRIVAL REPRODUCED: on-time %.3f mm vs late %.3f mm (lost %.3f mm) with"
      " identical segments, ticks and pulses -- arrival order alone."
      % (on['travel'], lt['travel'], on['travel'] - lt['travel']))
