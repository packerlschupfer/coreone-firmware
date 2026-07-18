"""Gate: the estall telemetry histogram must RESOLVE a print, not collapse it into one bin.

    python3 estall_telemetry_test.py

WHY THIS EXISTS. On 2026-10-26 I quoted `ESTALL_STATS` percentiles across three prints as if they
were measurements -- `p50=4375`, `p99.9` "creeping" 30625 -> 56875 -> 83125 -- and used them to argue
about distribution shape. They were BIN CENTRES. The histogram spans [-0.5*thr, 2.5*thr] = 3*thr, so
at the old `TELE_NBINS = 80` the bin width was 3*700000/80 = **26250 raw counts**, and those four
figures are each exactly one bin apart (bins 13/14/15/16). `p50 == p90 == p99 == 4375` across a whole
print was not a tight distribution; it was ONE BIN, because >90% of a print lands in a single bin at
that width. The fix raises the count to 2100 (bin width 1000).

WHAT THIS GATE PINS:
  * the resolution itself -- two values that are 26250 apart must land in DIFFERENT bins now. At 80
    bins they could not, which is the whole defect, so this arm fails on any revert of TELE_NBINS.
  * the UNBOUNDED END BINS. `_tele_add` clamps everything outside [lo, hi) into bin 0 / bin N-1, so a
    bin-centre estimate there reports a CONSTANT however far the real tail runs. The exactly-tracked
    extreme is reported instead, so a saturated tail shows up as `p1 == min` rather than as a
    stable-looking number.
  * that `min`/`mean`/`max`/`would-trip` stay EXACT. They are tracked outside the histogram, which is
    why they -- and not the percentiles -- are the figures that stay comparable across a bin-count
    change. Every load-bearing conclusion on 2026-10-26 rested on `max` and `would-trip`; those were
    sound. The percentile argument was not.
  * that the report echoes `bw=`, so a reader can tell the resolution of the numbers beside it
    instead of having to know TELE_NBINS.

⚠ A bin-centre estimate shifts by up to half a bin when the count changes (+-13125 between 80 and
2100), so DIFFING A NEW p50 AGAINST A PRE-PATCH LOG reads quantisation as a real change. That is what
`bw=` is for.

Real class under test: EStallDetect, loaded from the real file. The module path is derived from
golden.PHASE_EXEC_PY's directory so this works in BOTH layouts (overlay `klipper-port/extras/`,
fork `klippy/extras/`) without adding a second layout seam to golden.py.
"""
import sys, os, importlib.util
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

ESTALL_PY = os.path.join(os.path.dirname(golden.PHASE_EXEC_PY), 'estall_detect.py')
spec = importlib.util.spec_from_file_location('es_tele', ESTALL_PY)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
EStallDetect = mod.EStallDetect
TELE_NBINS = mod.TELE_NBINS

THR = 700000.0
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


class Rec(object):
    def __init__(self):
        self.lines = []

    def respond_info(self, m):
        self.lines.append(m)


class Dut(EStallDetect):
    """Only the telemetry accumulator; no printer, no sensor, no gcode plumbing."""

    def __init__(self, thr=THR):
        self.threshold = thr
        self.gcode = Rec()
        self._tele_reset()

    def feed(self, *vals):
        for v in vals:
            self._tele_add(v)
        return self


print("ARM A -- the resolution that was the whole defect:")
check(TELE_NBINS == 2100, "TELE_NBINS is 2100 (got %r)" % (TELE_NBINS,))
d = Dut()
bw = d._tele_bw
check(abs(bw - 1000.0) < 1e-6,
      "bin width is 1000 raw counts at the default thr=700000 (got %.1f)" % bw)
check(abs((d._tele_hi - d._tele_lo) - 3 * THR) < 1e-6,
      "the span is still 3*thr = [-0.5*thr, 2.5*thr]")


def bin_of(dut, v):
    return int((v - dut._tele_lo) / dut._tele_bw)


print("\nARM B -- THE KEY ONE: values 26250 apart must now be DISTINGUISHABLE:")
# 4375 / 30625 / 56875 / 83125 are what the Pi reported on 2026-10-26. At 80 bins they are
# consecutive bin CENTRES (bins 13/14/15/16) -- i.e. adjacent, unresolvable steps.
OBSERVED = (4375.0, 30625.0, 56875.0, 83125.0)
bins = [bin_of(d, v) for v in OBSERVED]
check(len(set(bins)) == 4, "all four land in DISTINCT bins (got %r)" % (bins,))
spread = max(bins) - min(bins)
check(spread >= 78,
      "and they are now ~26 bins apart each, not 1 -- so a real change between runs is visible"
      " (span %d bins)" % spread)
old_bw = 3 * THR / 80
check(abs((OBSERVED[1] - OBSERVED[0]) - old_bw) < 1e-6,
      "confirming the premise: those observed values were exactly ONE old bin (%.0f) apart"
      % old_bw)

print("\nARM C -- the UNBOUNDED end bins report an OBSERVED extreme, not a constant:")
# Everything below lo clamps into bin 0. A bin-centre estimate there is a constant whatever the
# real tail is, which is how a saturated tail passes as a stable-looking number.
d2 = Dut().feed(-5e6, -4e6, 0.0, 0.0, 0.0)
p1 = d2._tele_pct(0.01)
check(p1 == d2._tele_min,
      "p1 falling in bin 0 returns the exact tracked MIN (-5e6), not the bin centre (got %r)"
      % (p1,))
check(p1 != d2._tele_lo + 0.5 * d2._tele_bw,
      "and specifically NOT the bin-0 centre constant %.0f" % (d2._tele_lo + 0.5 * d2._tele_bw,))
d3 = Dut().feed(0.0, 0.0, 0.0, 9e6)
p999 = d3._tele_pct(0.999)
check(p999 == d3._tele_max,
      "p99.9 falling in the LAST bin returns the exact tracked MAX (9e6) (got %r)" % (p999,))

print("\nARM D -- a mid-range percentile is still a bin-centre estimate (unchanged behaviour):")
d4 = Dut().feed(*([12345.0] * 100))
p50 = d4._tele_pct(0.50)
b = bin_of(d4, 12345.0)
expect = d4._tele_lo + (b + 0.5) * d4._tele_bw
check(p50 == expect,
      "p50 is the centre of the containing bin (%.0f), within half a bin of the truth" % expect)
check(abs(p50 - 12345.0) <= d4._tele_bw / 2.0 + 1e-6,
      "and the error is bounded by half a bin = %.0f counts" % (d4._tele_bw / 2.0,))

print("\nARM E -- min/mean/max/would-trip are EXACT, which is why they are the comparables:")
d5 = Dut().feed(-123456.0, 1.0, 2.0, 999999.0, 800000.0)
check(d5._tele_min == -123456.0, "min is exact, not quantised")
check(d5._tele_max == 999999.0, "max is exact, not quantised")
check(abs(d5._tele_sum / d5._tele_n - (-123456.0 + 1 + 2 + 999999.0 + 800000.0) / 5.0) < 1e-6,
      "mean is exact")
check(d5._tele_over == 2,
      "would-trip counts samples strictly above thr: 999999 and 800000 (got %d)" % d5._tele_over)

print("\nARM F -- the report must EXPOSE the resolution and the new low percentiles:")
d6 = Dut().feed(*([4000.0] * 50 + [900000.0]))
d6._tele_report('gate')
line = d6.gcode.lines[-1] if d6.gcode.lines else ''
for tok in ('bw=', 'p1=', 'p10=', 'p50=', 'p99.9=', 'max=', 'would-trip='):
    check(tok in line, "report contains %r" % tok)
check('bw=1000' in line.replace(' ', ''),
      "and bw= reports the ACTUAL bin width, so percentiles beside it are self-describing")

print("\nARM G -- the end-bin test must follow the ACTUAL histogram, not the module constant:")
# WHY: _tele_pct special-cases the unbounded TOP bin so it reports the exactly-tracked max instead
# of a constant bin centre. That test read `i == TELE_NBINS - 1`. It agreed with reality only
# because _tele_reset allocates [0] * TELE_NBINS -- correct BY COINCIDENCE. If the two ever diverge
# the constant silently stops matching and the top bin falls through to the bin-centre formula,
# i.e. straight back to the constant this branch exists to avoid. Pinned with len().
d7 = Dut()
d7._tele_hist = [0] * 10          # deliberately DIVERGED from TELE_NBINS (2100)
d7._tele_n = 4
d7._tele_hist[9] = 4              # everything in the unbounded TOP bin of THIS histogram
d7._tele_min, d7._tele_max = -111.0, 4242.0
top = d7._tele_pct(0.50)
check(top == 4242.0,
      "the top bin of a 10-bin histogram still returns the tracked MAX (got %r)" % (top,))
check(top != d7._tele_lo + 9.5 * d7._tele_bw,
      "and NOT the bin-centre formula (%.0f), which is what the constant would have fallen through"
      " to" % (d7._tele_lo + 9.5 * d7._tele_bw,))
# bin 0 was already length-independent; assert it stays so alongside.
d8 = Dut()
d8._tele_hist = [0] * 10
d8._tele_n = 4
d8._tele_hist[0] = 4
d8._tele_min, d8._tele_max = -999.0, 7.0
check(d8._tele_pct(0.50) == -999.0, "bin 0 still returns the tracked MIN, unaffected")

print()
if fails:
    print("ESTALL-TELEMETRY GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("ESTALL-TELEMETRY GATE: PASS -- the histogram resolves a print instead of collapsing it into"
      " one bin, the unbounded end bins report an observed extreme instead of a constant, the exact"
      " statistics stay exact, and the report echoes its own bin width")
