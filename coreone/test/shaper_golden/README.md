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

## Sibling gates

`golden.py` is the oracle comparison. The files beside it are separate gates aimed at specific
defects, each runnable on its own (`python3 <file>`). Most drive the same harness; the last group
does not touch it at all (it gates host-side extras), which is why "the shaper golden tests" is a
misleading name for the directory as it now stands.

⚠ **This table is CHECKED, not maintained by hand.** `run_all.sh` fails when a `*_test.py` gate
has no row here, and when a row names a gate that no longer exists. It had drifted to covering
6 of 22 before the check existed, so do not trust a hand-kept list of gates again -- including
this one, unless `./run_all.sh` is green.

### Evaluator and segment emission

| gate | asserts |
|---|---|
| `gap_bridge_test.py` | the trapq-gap BEFORE storm is bridged (hole 3,151,364 -> 0) |
| `stale_arrival_test.py` | **reproduces an OPEN bug**: a segment arriving after its clock window loses the motion (40 mm -> 0 mm) on arrival order alone |
| `stale_neutralise_test.py` | a dead-window segment is sent as a **hold** (ring stays contiguous, nothing commanded), the next live segment re-anchors so the loss cannot become a lurch, and the loss is counted rather than silent |
| `seg_duration_cap_test.py` | no emitted segment reaches the MCU's 2^31 duration ceiling, where bit 31 is the re-anchor flag. Long holds **and** long moves split, pieces telescope and sum to the original span, every boundary sits on the un-split trajectory, and only the first piece carries the re-anchor. ARM A is the 14.20 s `PRINT_END` lift that lost a park |

### Keep-alive and the staleness latch

| gate | asserts |
|---|---|
| `idle_keepalive_test.py` | the host keep-alive holds position where the shipped path drives off, with the MCU latch pinned out of the way |
| `keepalive_unit_test.py` | `_ka_chunk` stays inside the 2^31 window even when the period is raised far past it |
| `keepalive_margin_test.py` | the keep-alive's **worst-case first** emit lands with real margin before `PE_STALE_S`, recomputed from the live constants and the MCU header rather than hardcoded; steady-state emit rate unchanged; the documented knob can still reach the latch on purpose |
| `stale_latch_test.py` | the MCU staleness latch holds stickily through the 2^32 wrap, and does NOT fire while the keep-alive is running |

### Counters and the XY trust verdict

| gate | asserts |
|---|---|
| `counter_saturation_test.py` | the uint16 diagnostic counters are exact below 65535 and **saturate** there instead of wrapping back through 0 — the `xy_trusted` hazard |
| `latebef_verdict_test.py` | the parked tail is excused; a real BEFORE storm and an **unreadable** counter are not |
| `latebef_stilltime_test.py` | lateBEF is credited against *total* still-time, so a motion ending a still-period no longer erases the window that excuses it; the verdict is identical whenever it is asked; the period is banked before the clock is overwritten, and mirrored to the slave |
| `latebef_vs_stale_test.py` | lateBEF and the latch are **independent** mechanisms — lateBEF counts a retention condition the latch neither sees nor suppresses, so a nonzero lateBEF alongside a latch event is consistent, not contradictory (2026-10-12 A1: `lateBEF=11631` with `events=2`, net 0.0000 mm) |
| `latebef_latch_veto_test.py` | once the latch has fired, **no** lateBEF is benign by either path, and an unreadable event count vetoes too (unknown is not zero); with a clean latch the parked-tail credit still works, so the false-alarm fix is not undone. ARM A is print 4's verbatim numbers, which the pre-veto code called benign for a physically lost park |
| `disengage_snapshot_test.py` | the teardown reports the **run's** counters, not the window the teardown itself creates: snapshot before stopping its own keep-alive, suppress the contaminating re-query, feed the trust verdict from the pre-teardown read, and fail loudly if the snapshot cannot be taken |

### Engage, disengage and the TMC bus

| gate | asserts |
|---|---|
| `engage_unwind_test.py` | a failed engage unwinds its own damage **and only its own** — clears `direct_mode`, restores the checks, unwinds a write that raised, and the re-entry guard cannot reach into a running phase-stepped print |
| `disengage_clear_homing_test.py` | the disengage restore path clears XY homing, **independently** of the re-base, and logs each failure distinguishably instead of swallowing it |
| `tmc_enable_race_test.py` | a queued Klipper enable callback can never see the register shadow without the checks — checks are shadowed no later than the registers at every point, and a failed engage restores them rather than leaving the driver unmonitored |
| `tmc_shadow_test.py` | `pe_tmc_shadow_get` is register-aware: GSTAT reads 0 so the error check cannot trip, DRV_STATUS keeps `cs_actual=0x1F`, and the one-constant shadow that caused the shutdown still fails here |

### Host-side extras (these do NOT drive the harness)

| gate | asserts |
|---|---|
| `gc_counters_test.py` | `gc_tune` exports allocation counters readable mid-print **without** triggering a pass; `get_stats` is throttled so the instrument does not add the pressure it measures, and a failure in it cannot break a status query |
| `gc_refreeze_test.py` | periodic re-freeze follows klippy's own level-2 passes without ever collecting, freezes only when one actually happened, refuses the reclaim mid-print, holds pass cost **flat** where the unfrozen arm grows, and the cyclic leak it creates is given back by `_reclaim` |
| `gc_watchdog_test.py` | reactor lateness is measured against the **deadline**, the re-arm cannot burst, the stale `sd_pos` is suppressed, and the warning does not attribute the stall to GC |
| `estall_telemetry_test.py` | the e-stall histogram **resolves** a print instead of collapsing it into one bin, the unbounded end bins report an observed extreme rather than a constant, the exact statistics stay exact, and the report echoes its own bin width |

The phase_exec gates each take the OTHER defences out of the way on purpose (`stale=` pins the
latch, the keep-alive arms are explicit), so no fix can silently repair another's regression
vector. That is not optional politeness: a second fix for the same fault quietly repairs the
baseline arm, and then neither defence is being tested any more.

Several gates own their monkeypatch saves per arm, in a `finally`, for the same reason — a leaked
patch in one arm once faked a PASS two arms later.
