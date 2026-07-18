# Golden-compare gate — MCU-side input shaping (Phase A)

```
python3 golden.py        # -v for per-vector diagnostics
```

No printer, no MCU, no motion. Needs `gcc` and `python3`; nothing else.

## What it asserts

`src/phase_shaper.c` — compiled **as shipped**, via `#include` from `harness.c` — must
reproduce the Python oracle in `extras/phase_exec.py` (`_shaped_motor` / `_eval_axis`,
themselves the verbatim replica of `chelper/kin_shaper.c`) on the same raw axis trajectory,
for both motor **position** and motor **velocity**.

The oracle is imported from the real `extras/phase_exec.py`, not copied, so the two cannot
drift. Those two methods are pinned with DO-NOT-DELETE comments for this reason.

Three things are checked per vector:

1. **Numeric agreement** with the oracle, position and velocity.
2. **Retire-equivalence** — every vector runs twice, once with the ring tail pinned and once
   with `pe_axis_retire` advancing it after each tick. The results must be bit-identical. This
   is the direct test that the lookback window is right: if retirement eats history an impulse
   still needs, the retiring run diverges.
3. **The identity pulse** `([1.0],[0.0])` reduces the evaluator exactly to un-shaped segment
   evaluation — the path the extruder, `PHASE_SEG_TEST`, the oscillator and the cogging sweeps
   keep using.

## Tolerance

The MCU evaluates in float32, the oracle in float64. At print scale (250 mm x 200 su/mm =
50 000 su) one float32 ULP is already ~0.006 su, so a flat 1e-4 su bound would be
arithmetically impossible and therefore a fake gate. The tolerance is ULP-scaled to the
magnitude under test (`8 * ULP`, floor 1e-4 su / 1e-3 su/s) and the report prints the observed
error in ULP.

Measured across the vector set: **0.7 – 3.0 ULP**. A logic regression shows up at tens of
thousands of ULP, so there is no ambiguity between "float32 noise" and "wrong".

This is the same precision the existing motor-space path already runs at — `phase_seg.start_pos`
has always been float32 in step-units — so it is a status-quo bound, not a cost introduced by
moving the shaping onto the MCU.

## Vectors

| vector | what it covers |
|---|---|
| `cruise` | constant velocity; the shaper identity `input_shaper(v*T) == v*T` |
| `ramp` | accel / cruise / decel trapezoid |
| `corner` | velocity zero-crossing (the reversal apex) |
| `diag` | simultaneous X and Y with **different** shapers — the case that forces axis space |
| `dwell` | explicit zero-velocity hold segment, as `_trapq_flush_cb` emits for an idle span |
| `printscale` | same trapezoid translated to ~50 000 su — the float32 headroom probe |
| `longhold` | multi-second hold; `(float)duration` precision in the frozen branch |
| `xsign_flip`, `ysign_flip` | the CoreXY projection sign conventions |
| `edges` | sampling before the first segment and past the last one |
| `jitter`, `jitter_step` | non-monotonic `now` — exercises the bounded backward walk |
| `reanchor`, `reanchor_mid` | segments carrying the halt-resume flag in `duration`'s MSB |
| `identity` | trivial pulses reduce to the un-shaped path |

## Mutation coverage

The gate was verified to have teeth by deliberately breaking `phase_shaper.c`. 9 of 10
mutants are caught:

| mutation | result |
|---|---|
| drop the `2x` on `half_accel` in the velocity term | caught (15 vectors) |
| frozen branch returns the wrong endpoint | caught (15) |
| impulse weight `a` -> `1.0` | caught (15) |
| pulse `dt` sign flipped | caught (15) |
| `pe_axis_retire` takes the max cursor instead of the min | caught (16) |
| backward walk disabled | caught (3 — the `jitter` vectors) |
| `trel` off by one tick (6 ns) | caught (2) |
| `duration` read without the `PE_SEG_DUR` reanchor mask | caught (2) |
| before-tail hold returns the wrong value | caught (2) |
| forward advance `>` -> `>=` | **not caught — genuinely equivalent** |

The last one differs only when `tau` lands exactly on a segment boundary, where a contiguous
chain gives the same position *and* velocity from either side. It is not a coverage gap.

Two earlier iterations of this gate were themselves wrong and are worth not re-introducing:
the pass criterion originally ignored velocity error entirely (a mutant with a 1e4 su/s
velocity error "passed"), and the first `jitter` vector used an offset smaller than one
refresh interval, so `now` never actually went backwards and the backward walk was never
executed.

## Not covered here

Everything downstream of the evaluator: the projection wiring into `pe_update_angle`, the
engage / MSCNT `phase_offset` handoff, startup pre-fill of the lookback window, the halt/
disengage tail flush, and the dry-coast policy (which is deliberately the caller's job, applied
post-projection so a simultaneous X and Y coast cannot sum past the half-electrical-period
clamp). Those are Phase C.
