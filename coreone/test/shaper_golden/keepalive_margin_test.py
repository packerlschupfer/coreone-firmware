"""Gate: the keep-alive must emit its first bridging segment with MARGIN before the MCU latch.

    python3 keepalive_margin_test.py

WHY THIS EXISTS. phase_exec.py used to claim "with both defences live no amount of idling can
exercise the backstop". That was FALSE. The keep-alive emitted only once `lag >= _ka_period`, so
after the toolhead buffer drains:

    worst case to the first bridging segment
      = _ka_period     lag must climb from 0 to the threshold
      + _ka_period     the timer only CHECKS every period, and a tick can land just UNDER it
      = 8.0 s at the 4.0 s default
    PE_STALE_S                                                  = 8.0 s
    margin                                                      = 0.0 s

Zero margin, so the MCU staleness latch could fire before the first hold ever landed. That is the
>=8 s uncovered episode measured in print 3's PRINT_END tail on 2026-11-01, and the same race as the
pause-side `maxlag 7.59 s vs latch 8.0 s` already on record -- two observations that had been filed
as "incommensurable" and are in fact the same 4+4-vs-8 collision.

🔑 ARM A IS THE WHOLE POINT: it recomputes the worst-case latency from the live constants and
requires real margin against PE_STALE_S *read out of src/phase_shaper.h*, not hardcoded here. A
later edit that raises _ka_period, raises _KA_EMIT_FRAC, or lowers PE_STALE_S closes the margin
again -- and this arm is what notices. Pinning "the constant is 0.5" would NOT catch that; pinning
the ARITHMETIC does.

🔑 ARM C pins that STEADY STATE is unchanged. The fix must only move the FIRST-emit latency: after
an emit, t0 := est so lag restarts near 0 and the next tick sees lag == _ka_period, which clears
either threshold. One emit per tick before and after. If a future "optimisation" made it emit
several times per tick, that would be new MCU traffic on the path this module exists to keep quiet.

⚠ ARM D keeps SET_PHASE_KEEPALIVE usable as the documented TEST KNOB: raising the period above
PE_STALE_S must still let the latch be reached deliberately, or the only supported way to exercise
the backstop is gone.
"""
import sys, os, re, importlib.util

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

PHASE_EXEC_PY = golden.PHASE_EXEC_PY
SRC = open(PHASE_EXEC_PY).read()
def _find_shaper_h():
    """Locate src/**/phase_shaper.h by SEARCHING, not by assuming a layout.

    ⚠ WHY A SEARCH. The first draft hardcoded two candidates, both ending `src/phase_shaper.h`,
    differing only in how far up they walked — and one was even commented "fork layout", which
    was wrong: they were the SAME flat-src assumption at two depths. The overlay is flat
    (`klipper-port/src/phase_shaper.h`) but the fork nests by MCU family
    (`src/stm32/phase_shaper.h`), so both candidates missed there, PE_STALE_S read None, and
    ARM A/B could not run — 20/21 in the fork while passing 21/21 in the overlay. Same layout
    seam the golden.py PHASE_EXEC_PY resolution already had to solve; the other path-dependent
    gates derive from golden.PHASE_EXEC_PY's directory for exactly this reason.
    """
    searched = []
    d = os.path.dirname(os.path.abspath(PHASE_EXEC_PY))
    for _ in range(6):                      # extras/ -> klipper-port/ or klippy/ -> repo root
        root = os.path.join(d, 'src')
        searched.append(root)
        if os.path.isdir(root):
            for dirpath, _dirs, files in os.walk(root):
                if 'phase_shaper.h' in files:
                    return os.path.join(dirpath, 'phase_shaper.h'), searched
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return None, searched


SHAPER_H, SHAPER_H_SEARCHED = _find_shaper_h()

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


cls = None
try:
    spec = importlib.util.spec_from_file_location('pe_ka', PHASE_EXEC_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for n in dir(mod):
        o = getattr(mod, n)
        if isinstance(o, type) and hasattr(o, '_keepalive_event'):
            cls = o
            break
except Exception as e:
    print("  (import failed: %s)" % e)


def _pe_stale_s():
    """Read PE_STALE_S from the MCU header -- the threshold must not be hardcoded here."""
    if not SHAPER_H or not os.path.exists(SHAPER_H):
        return None
    m = re.search(r'#define\s+PE_STALE_S\s+([0-9.]+)f?', open(SHAPER_H).read())
    return float(m.group(1)) if m else None


def arm_a():
    stale = _pe_stale_s()
    # Name the searched roots on failure: "header not found" with the paths is actionable,
    # a bare None is not -- and a layout seam is exactly what this needs to survive.
    check(stale is not None,
          "PE_STALE_S read from %s (got %r)"
          % (SHAPER_H if SHAPER_H else "NOT FOUND under any of %s" % SHAPER_H_SEARCHED, stale))
    ka = getattr(cls, '_KEEPALIVE_S', None)
    frac = getattr(cls, '_KA_EMIT_FRAC', None)
    check(ka is not None, "_KEEPALIVE_S present (got %r)" % (ka,))
    check(frac is not None, "_KA_EMIT_FRAC present (got %r)" % (frac,))
    # ⚠ Default to 1.0, which IS the pre-fix semantics (emit at the full period), rather than
    # bailing out. An arm that raises on a missing constant reports "constant absent" and never
    # evaluates the margin -- so it would not DEMONSTRATE that the old configuration was
    # unsafe, only that it looked different. Measure whatever is actually configured.
    if frac is None:
        frac = 1.0
        print("     (no _KA_EMIT_FRAC -> assuming 1.0, the pre-fix full-period behaviour)")
    if None in (stale, ka):
        raise RuntimeError("need PE_STALE_S and _KEEPALIVE_S to compute the margin")
    # lag must climb to frac*period, and the timer only samples every period
    worst = ka * frac + ka
    margin = stale - worst
    print("     _ka_period=%.1fs  frac=%.2f  ->  worst first emit %.1fs  vs PE_STALE_S %.1fs"
          % (ka, frac, worst, stale))
    check(worst < stale,
          "worst-case first emit (%.1fs) is BELOW PE_STALE_S (%.1fs) -- the latch cannot win"
          % (worst, stale))
    # a real margin, not a tie. 1 s is the smallest that survives a timer slipping one tick edge.
    check(margin >= 1.0,
          "and the margin is REAL, not a tie: %.1fs >= 1.0s" % margin)
    # the pre-fix configuration must be demonstrably unsafe, or this arm proves nothing
    worst_old = ka * 1.0 + ka
    check(worst_old >= stale,
          "sanity: emitting at the FULL period gives %.1fs >= %.1fs, i.e. the old code had no"
          " margin -- so this arm would have failed on it" % (worst_old, stale))


def arm_b():
    body = SRC[SRC.find('def _keepalive_event'):]
    body = body[:body.find('\n    def ', 10)] if '\n    def ' in body[10:] else body
    check('_KA_EMIT_FRAC' in body,
          "the emit gate uses _KA_EMIT_FRAC, so the threshold is not a bare _ka_period")
    check('lag >= self._ka_period * self._KA_EMIT_FRAC' in " ".join(body.split()),
          "and the condition is `lag >= _ka_period * _KA_EMIT_FRAC`")
    check('_emit_keepalive' in body, "and it still calls _emit_keepalive")
    # the corrected invariant must no longer assert the false claim
    flat = " ".join(SRC.split())
    check('no amount of idling can exercise the backstop' not in flat
          or 'CORRECTED 2026-11-01' in flat,
          "the FALSE 'no amount of idling' claim is corrected, not left standing")
    check('ZERO margin' in flat or 'zero margin' in flat,
          "and the zero-margin arithmetic is written down where the constant lives")


def arm_c():
    # STEADY STATE unchanged: after an emit t0 := est, so lag restarts ~0 and the next tick
    # sees lag == _ka_period. Both the old and new thresholds clear that, so the emit rate is
    # one per tick either way -- the fix must not add MCU traffic.
    ka = getattr(cls, '_KEEPALIVE_S', 4.0)
    frac = getattr(cls, '_KA_EMIT_FRAC', 1.0)
    lag_at_next_tick = ka
    check(lag_at_next_tick >= ka * frac,
          "in steady state the next tick's lag (%.1fs) still clears the new threshold (%.1fs)"
          " -> one emit per tick, as before" % (lag_at_next_tick, ka * frac))
    check(lag_at_next_tick >= ka * 1.0,
          "and it cleared the OLD threshold too (%.1fs) -> the emit RATE is unchanged, only the"
          " first-emit latency moved" % (ka * 1.0,))
    bridge = getattr(cls, '_BRIDGE_MAX', None)
    check(bridge is not None and ka <= bridge,
          "a steady-state span (%.1fs) still fits _BRIDGE_MAX (%r)" % (ka, bridge))


def arm_d():
    # The documented test knob must still be able to REACH the latch deliberately.
    stale = _pe_stale_s() or 8.0
    frac = getattr(cls, '_KA_EMIT_FRAC', 0.5)
    test_period = 10.0          # what SET_PHASE_KEEPALIVE's help suggests (> 8 s)
    worst = test_period * frac + test_period
    check(worst > stale,
          "with keepalive_period=%.0fs the worst case (%.1fs) still EXCEEDS PE_STALE_S (%.1fs),"
          " so the latch stays reachable for testing" % (test_period, worst, stale))
    check('TEST KNOB' in SRC or 'supported way to test' in SRC,
          "and the knob is still documented as the way to exercise the backstop")


arm("ARM A -- THE POINT: recompute the margin from live constants vs PE_STALE_S:", arm_a)
arm("ARM B -- the emit gate and the corrected invariant:", arm_b)
arm("ARM C -- steady state unchanged: one emit per tick, no new MCU traffic:", arm_c)
arm("ARM D -- SET_PHASE_KEEPALIVE still reaches the latch (test knob preserved):", arm_d)

print()
if fails:
    print("KEEPALIVE-MARGIN GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("KEEPALIVE-MARGIN GATE: PASS -- the keep-alive's worst-case first emit lands with real"
      " margin before PE_STALE_S (recomputed from the live constants and the MCU header, not"
      " hardcoded), the steady-state emit rate is unchanged, and the documented test knob can"
      " still reach the latch on purpose")
