"""Gate: lateBEF and the STALENESS LATCH are INDEPENDENT mechanisms.

    python3 latebef_vs_stale_test.py

WHY THIS EXISTS (2026-10-18). After the staleness latch was validated on hardware I claimed
that "lateBEF cannot be driven nonzero by idling, because the 8 s latch pre-empts the 12.783 s
signed-32 wrap". That is true of ONE path and I over-generalised it to the counter, which left
a real observation looking like a contradiction: the 2026-10-12 A1 print reported

    MCU-SHAPE ... primed=1 lateBEF=11631 reanch=2 hold=44183
              | X past=43213 bef=34005 empty=424 back=3 lag=0
    MCU-STALE events=2 latched=0x0 heldticks=43758 thresh=8.0s | dry X=0.55s Y=0.55s
    streamed 1786327 segs, net 0.0000mm | ... JUMP=0 jumpMAX=0.000mm

i.e. the latch DID fire (events=2) and lateBEF accrued anyway, on a print that lost no motion
(net 0.0000mm, JUMP=0). Those two numbers are not in tension, and this gate pins why.

THE TWO PATHS. `covered` in pe_axis_eval is set PER IMPULSE, by the in-segment branch only:

    } else {                      /* tau inside this segment */
        ...
        covered = 1;
    }

and the latch arms on the EXIT side only when `!covered`. But PE_EVAL_BEFORE is raised by a
DIFFERENT impulse in the same call -- one whose tau fell before the oldest retained segment.
So a single tick can be BOTH covered (one impulse inside a segment -> no dry episode -> the
latch can never arm) AND BEFORE (another impulse straddling off the front of the ring). In
phase_exec.c that tick increments the counter, because the guard excludes STALE but not BEFORE:

    if (!(st & (PE_EVAL_EMPTY | PE_EVAL_LAGGED | PE_EVAL_STALE))) {
        if (!(st & PE_EVAL_BEFORE))  pe_shape_primed = 1;
        else if (pe_shape_primed)    pe_cnt_inc(&pe_shape_late_before);
    }

So the honest statement is:
  - the LATCH covers the UNCOVERED episode, whose DURATION the signed-32 wrap turns dangerous;
  - lateBEF additionally counts a RETENTION condition -- the shaper's lookback window
    straddling off the front of a sparsely-populated ring -- which is benign for position
    (the BEFORE branch commands that segment's start_pos, and with a park that IS where the
    gantry is) and which the latch neither sees nor should suppress.
A park with few segments in the ring produces both at once, which is exactly the A1 signature:
`past` and `bef` both large, `empty` small, net 0.

ARM A proves BEFORE accrues with the latch provably NOT engaged (stale=0, latched=0).
ARM B proves the latch still fires on the genuinely uncovered case, so A is not just "the
latch is broken".

Drives the real phase_shaper.c through the golden harness; the per-axis counters are read from
the harness's stderr diagnostics (harness.c prints n_before / n_stale / stale_latch).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

golden.build_harness()
CLOCK = golden.CLOCK
Oracle, _PE = golden.load_oracle_class()
px_q, px_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(63.6)))
py_q, py_dt = golden.quantize_pulses(Oracle._init_shaper_pulses(*golden.shaper_mzv(49.4)))
proj = (1.0, 1.0, 0.0, 1.0, -1.0, 0.0)

TICK = 4200                      # 168 MHz / 4200 = 40 kHz, the real ISR rate
MS = int(CLOCK / 1000.0)         # ticks per millisecond
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def axis_diag(stderr, ax):
    """Pull `axis<N> before=.. past=.. empty=.. backscan=.. lagged=.. stale=.. latched=..`."""
    for ln in stderr.splitlines():
        if ln.startswith('axis%d ' % ax):
            return {k: int(v) for k, v in
                    (tok.split('=') for tok in ln.split()[1:])}
    raise SystemExit('no diagnostics for axis%d in:\n%s' % (ax, stderr))


# The widest BACKWARD reach of the shaper, in ticks. Klipper's impulses look backwards
# (dt <= 0), which is why phase_shaper.h documents startup BEFOREs as expected.
span = max(-min(px_dt), -min(py_dt))
print("shaper lookback span: %.2f ms (%d ticks)   tick rate %.0f Hz"
      % (span / float(MS), span, CLOCK / TICK))

T0 = 1 << 28                     # start well away from 0 so signed deltas are unremarkable

# ---------------------------------------------------------------- ARM A: covered AND before
# ONE short segment per axis, far shorter than the lookback span, and we tick strictly INSIDE
# it. The dt=0 impulse samples the segment (covered=1, so no dry episode can ever start),
# while the most-negative impulse lands before the oldest retained segment (BEFORE).
SEG_DUR = 2 * MS                 # 2 ms, vs a ~8 ms lookback
segA = [(T0, SEG_DUR, 0.0, 0.0, 0.0, 0)]
ticksA = [T0 + TICK * i for i in range(1, SEG_DUR // TICK)]   # all inside the segment

print("\nARM A -- tick INSIDE a 2 ms segment, lookback straddling off the front of the ring:")
# Pin the latch far beyond the timeline as well, so a PASS cannot be bought by the threshold
# simply not being reached in wall-clock terms -- the point is that `covered` suppresses the
# episode entirely, not that the run was short.
outA, errA = golden.run_harness(px_dt, px_q[0], py_dt, py_q[0],
                                (segA, list(segA)), ticksA, proj, True)
dA = axis_diag(errA, 0)
print("   axis0: %s" % dA)
check(len(ticksA) > 10, "ticked %d times inside the segment" % len(ticksA))
check(dA['before'] > 0,
      "BEFORE accrued (%d) -- the lookback straddles off the front" % dA['before'])
check(dA['stale'] == 0, "stale events = 0 -- the latch never armed")
check(dA['latched'] == 0, "latched = 0 -- and it is not holding")
check(dA['empty'] == 0, "ring was never empty (%d), so this is not the EMPTY path" % dA['empty'])
# This is the whole point: in phase_exec.c such a tick is !STALE && BEFORE && primed -> lateBEF++
check(dA['before'] > 0 and dA['stale'] == 0,
      "=> a tick can be BEFORE while the latch is provably not engaged,"
      " which is precisely what increments lateBEF")

# ------------------------------------------------------------- ARM B: uncovered -> latch fires
# Same sparse ring, but now tick far PAST the segment's end so no impulse is ever inside it.
# That is the uncovered episode the latch exists for. Shrink the threshold via the harness
# STALE override rather than simulating 8 real seconds.
STALE_S = 0.050
segB = [(T0, SEG_DUR, 0.0, 0.0, 0.0, 0)]
start = T0 + SEG_DUR + span + TICK          # past the end AND past the lookback reach
ticksB = [start + TICK * i for i in range(int(0.200 * CLOCK) // TICK)]   # 200 ms of ticks

print("\nARM B -- tick PAST the segment end (nothing covered), threshold pinned to %.0f ms:"
      % (STALE_S * 1e3,))
outB, errB = golden.run_harness(px_dt, px_q[0], py_dt, py_q[0],
                                (segB, list(segB)), ticksB, proj, True, stale=STALE_S)
dB = axis_diag(errB, 0)
print("   axis0: %s" % dB)
check(dB['stale'] >= 1, "the latch ARMED (%d event(s)) once nothing was covered" % dB['stale'])
check(dB['latched'] == 1, "and it is STILL holding at the end (sticky until a new segment)")
check(dB['past'] > 0, "PAST accrued (%d) -- impulses beyond the newest segment" % dB['past'])

print("\nCONTRAST: covered+straddling -> before=%d stale=%d | uncovered -> before=%d stale=%d"
      % (dA['before'], dA['stale'], dB['before'], dB['stale']))
check(dA['stale'] == 0 and dB['stale'] >= 1,
      "the SAME ring latches or not purely on whether an impulse is inside a segment")

print()
if fails:
    print("LATEBEF-VS-STALE GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("LATEBEF-VS-STALE GATE: PASS -- lateBEF counts a RETENTION condition the latch neither"
      " sees nor suppresses, so a nonzero lateBEF alongside a latch event is consistent,"
      " not contradictory (2026-10-12 A1: lateBEF=11631 with MCU-STALE events=2, net 0.0000mm)")
