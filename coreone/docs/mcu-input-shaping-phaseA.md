# MCU-side input shaping — Phase A implementation plan

> **Implementation record (2026-07-29) — see §7 at the bottom.** Steps 1–3 of §3 are DONE
> and the golden gate PASSES; only step 4 (the on-hardware cycle benchmark) is open.
> §7 also records where the implementation deviated from this plan and what measurement
> replaced an estimate. Branch `feature/mcu-input-shaping`.

**Status:** plan written 2026-07-29 against the overlay at
`~/git/pfb-klipper/klipper-port/` (`src/phase_exec.c` 1149 lines, `extras/phase_exec.py` 1751 lines).

**Goal (whole project):** move the input-shaping convolution off the Pi and onto the F427, so the
host sends raw un-shaped **axis** segments (~1/move) instead of shaped, chain-fit **motor** segments
(~3–6/move + a slew control loop), and removes the phase-stepping "Timer too close" class.
Prusa parity (Prusa shapes on the Buddy MCU).

> **Measured outcome (2026-08-22), replacing this plan's "~10x host CPU" estimate.** The estimate was
> too optimistic about CPU and too pessimistic about what the change actually buys. Dry-run motion,
> `DRYRUN_assembly1_L2plus.gcode`, all four host/shaping combinations:
>
> | | Pi 4 MCU | Pi 4 host | Pi 3 MCU | Pi 3 host |
> |---|---|---|---|---|
> | axis/motor segments streamed | 90,608 | 319,593 | 90,608 | 319,742 |
> | MCU starved of trajectory (`dry`) | 2 | 113 | 1 | 248 |
> | klippy CPU (one core) | 3.0% | 9.7% | 6.8% | 15.3% |
> | `Timer too close` | 0 | 0 | 0 | 0 |
>
> **CPU: ~3.2x on a Pi 4, not 10x** (and 1.85x on a real extruding print, where pressure advance and
> extruder kinematics roughly double klippy's motion work regardless of shaping). The honest headline
> is **feed margin, not CPU**: host-shaping costs ~56x the starvation events on a Pi 4 and ~248x on a
> Pi 3. Note the shape of that — **host-side shaping is expensive on any host; a slow host compounds
> it rather than causing it.** Consequence: a Pi 3 is sufficient for this printer *with* MCU-side
> shaping, in an F427-only configuration (it cannot host both MCUs — `dwc_otg` FIQ load stalls PID1
> past the 60s systemd watchdog).
>
> Caveats: the Pi 4 cells carried both MCUs and a webcam where the Pi 3 cells had one MCU and no
> camera (conservative — the Pi 4 wins while handicapped); the CPU figures use different sampling
> methods across hosts and are approximate.

**Phasing:** **A)** MCU axis rings + per-tick convolution + projection → **B)** host raw-trapezoid
emitter + ship pulse tables → **C)** engage / MSCNT handoff / startup prefill / halt flush →
**D)** validate (golden-compare, resonance re-test, `Assembly1` Pi-4 reprint).

**This document covers Phase A only.** Check in at each phase boundary.

---

## 1. Verification of the scope memory against current code

Done before designing. `mcu_side_input_shaping_scope.md` was a few days old.

### Confirmed as cited

| Symbol | Location | Note |
|---|---|---|
| `struct phase_seg` | `src/phase_exec.c:83` | start_clock/duration/start_pos/start_v/half_accel — 20 bytes |
| `pe_update_angle` | `src/phase_exec.c:461` | |
| TIM8 ISR `TIM8_UP_TIM13_IRQHandler` | `src/phase_exec.c:614` | prio 0, BASEPRI-unmaskable |
| `_shaped_motor` | `extras/phase_exec.py:847` | the golden oracle |
| `_load_shaper_pulses` | `extras/phase_exec.py:755` | |
| `_init_shaper_pulses` | `extras/phase_exec.py:736` | still the verbatim `kin_shaper.c` replica |

### Four corrections — all load-bearing

1. **`_shaped_states_forward` does not exist.** The forward-cursor builder is **`_build_st_forward`**
   (`extras/phase_exec.py:869`). Same semantics as the memory describes; only the name was wrong.

2. **The ISR is round-robin, not both-motors-per-tick.** `pe_dma_refresh` (`src/phase_exec.c:587`)
   refreshes *exactly one* motor per tick, alternating A/B — a deliberate fix for the
   layer-registration drift (each motor's hold is 2 ticks; one-refresh-period lead is the ZOH centre).
   The scope's "shaper eval computed once/tick, **shared by both motors**" is therefore wrong — but
   favourably: it is **one axis-pair eval + one projection per tick**, not one eval + two projections.
   No cross-motor caching is needed, and the cycle budget improves slightly.

3. **The lookback window is not `max_pulse_delay`.** After `_init_shaper_pulses`' shift-to-zero-mean
   (`t[i] -= sum(a*t)`), the pulse offsets **straddle zero**. For MZV@43.4 Hz they span roughly
   **−11.5 ms … +11.5 ms**, not 0 … 23 ms. The *future* side is already covered host-side by
   `_pulse_margin` (`phase_exec.py:781-786`, the flush horizon lags by max positive `t`). The MCU
   needs **history back to `now + min(t)`** ≈ 11.5 ms, and retirement must be driven by the
   **oldest impulse cursor**, not by `seg_end + max_pulse_delay`.
   At the `min_seg_dt` = 0.8 ms floor that is ~15 segments of history — trivial.

4. **`phase_seg.duration`'s MSB carries the reanchor flag** (`src/phase_exec.c:89`, unpacked at
   `:436`), and reanchor rewrites the **per-motor** `phase_offset` from `start_pos`. In axis space
   that has to move to *after* the projection. Not a Phase A problem, but it constrains the struct:
   axis segments keep the same 20-byte layout and the flag stays where it is.

### RAM note

`SEG_RING` (1024) × 20 B = **20 KB per executor**, inline in `struct phase_exec` via `oid_alloc`.
Two *additional* axis rings would be +40 KB against the `alloc_chunk` cliff the code already
comments on (`src/phase_exec.c:40-50`, `:90`). Avoided by ring reuse — see A1.

---

## 2. Phase A plan

### A0. Factor the eval into a dependency-free translation unit — do this first

New `src/phase_shaper.h` / `src/phase_shaper.c`: pure `float` / `int32_t`, **no** `sched.h`,
`command.h`, CMSIS, or `internal.h`. `phase_exec.c` includes it; a plain `gcc` harness on the host
compiles the identical source.

This is the load-bearing decision of Phase A — it is what makes the golden-compare a real numeric
test of *shipped code* rather than of a re-implementation. Nothing else in Phase A should be written
before this shape is settled.

### A1. Data structures

```c
#define PE_MAX_PULSE 5
struct pe_pulse { float a; int32_t dt; };      // dt in MCU clock ticks, signed

struct pe_axis {
    struct phase_seg *ring;                    // BORROWED — see below
    uint16_t mask, head;
    uint16_t cur[PE_MAX_PULSE];                // forward cursor per impulse
    uint8_t  npulse;
    struct pe_pulse p[PE_MAX_PULSE];
    float    last_pos, last_v;                 // per-axis dry/coast state
    uint32_t last_end_clock;
};
```

**Ring reuse, not new rings.** Motor A's existing `seg_ring` becomes the X axis ring, motor B's
becomes Y. Both `pe_axis` structs hold borrowed pointers. Zero extra RAM, no `alloc_chunk` risk,
and it is honest — in shaped mode the per-motor rings are dead anyway. Two module-global
`struct pe_axis pe_axes[2]`, bound at engage.

Pulse offsets are stored as **integer clock ticks**, pre-converted host-side, so the hot path has no
seconds↔ticks conversion.

### A2. The eval — direct transcription of `_build_st_forward`

```c
void pe_axis_eval(struct pe_axis *ax, uint32_t now, float *pos, float *vel);
```

Per impulse `i`: `tau = now + p[i].dt`; advance `cur[i]` forward while the *next* segment's
`start_clock <= tau` (signed 32-bit delta compare, never backward); then the same three branches as
the Python:

- `trel < 0` → static at `start_pos`, no velocity contribution
- `trel > duration` → frozen endpoint `start_pos + (start_v + ha·d)·d`, vel = accel = 0
- else → the quadratic, accumulating `a·pos` and `a·vel`

**One deliberate divergence from the Python.** `_build_st_forward`'s bounded containment back-scan
(`phase_exec.py:890-901`) exists to survive trapq *shadow/overlap* entries. MCU segments arrive
time-contiguous by construction (the telescoping-clock invariant, `phase_exec.py:1148-1158`), so the
back-scan is dropped; a non-containing cursor falls into the frozen branch, which is what the
Python's `not found` path does anyway.

→ **Precondition, must be stated in the header and honoured by the golden test:** segments fed to
`pe_axis_eval` are time-contiguous and non-overlapping.

### A3. Retirement and dry-ahead

Per-axis tail = `min(cur[i])` over the impulses; the producer's overflow check uses that. If the
newest segment ends before `tau`, reuse the existing dry policy (Fix 2 coast on `last_v`,
`PE_MAX_DRY_SEC`).

**Apply the distance clamp post-projection, not per-axis.** Per-axis clamping would let a
simultaneous X and Y coast sum to 2 × `PE_DRY_MAX_STEPS` in motor space and break the
"< ½ electrical period" invariant that Fix 3 (`src/phase_exec.c:55-60`) exists to protect.
Post-projection clamping preserves today's exact 24-su semantics.

### A4. Projection and per-motor mapping

Add to `struct phase_exec`:

```c
uint8_t shape;          // 0 = today's motor-space path (DEFAULT)
float   cx, cy;         // xs*sign, ±ys*sign  (host folds the signs in)
float   pos_ref;        // engage reference, subtracted post-projection
```

`pos = cx·Xs + cy·Ys − pos_ref`; `vel` likewise (the cogging fwd/bwd blend at `src/phase_exec.c:531`
needs it). Everything downstream — `phase_offset`, cogging LUTs, `swap`, `lead_ticks`,
`pack_xdirect`, the DMA path — is untouched, because the ISR's *output* is still a per-motor angle.

Two new commands:

```
phase_exec_shape  oid=%c enable=%c cx=%u cy=%u pos_ref=%u
phase_exec_pulses oid=%c axis=%c n=%c a=%*s dt=%*s
```

Both land in Phase A. `shape` **defaults 0**, so on hardware this is dead code until Phase C
explicitly turns it on.

### A5. ISR integration

A single branch at the top of the `if (m->analytic)` block in `pe_update_angle`:

```c
if (m->shape) pe_shaped_pos(m, tnow, &pos, &vel);
else          { /* existing pe_seg_advance + cur_seg path, unchanged */ }
```

Round-robin means one motor per tick → one axis-pair eval per tick, no cross-motor cache.

### A6. The gate — offline golden-compare (before any flash)

`test/shaper_golden/`:

- **`harness.c`** — `#include "../../src/phase_shaper.c"`; reads pulse tables + axis segments + a
  tick list on stdin; emits per-tick `Xs, Ys, posA, posB`.
- **`golden.py`** — builds synthetic trapq move lists, derives the 1:1 equivalent raw axis segments,
  runs the *real* `_init_shaper_pulses` and `_shaped_motor` / `_eval_axis` lifted from
  `extras/phase_exec.py`, asserts `max|Δ| < 1e-4` step-units per motor.

**Vector set:** cruise; accel/decel ramp; corner reversal (velocity zero-crossing — the case the host
needed a dedicated breakpoint insertion for, `phase_exec.py:1065-1087`); dwell/gap freeze;
pre-first-move (`trel < 0`); end-of-buffer; and **print-scale absolute positions** (~50 000 su, not
near zero) to exercise float32 headroom.

**Pulses:** the live EI@55.4 X / MZV@43.4 Y, plus the identity case `([1.0], [0.0])` which must
reproduce the un-shaped path bit-for-bit.

Float32 headroom check: 200 su/mm × 250 mm ≈ 50 000 su; float32 gives ~7 significant digits →
~0.006 su ≈ 0.03 µm. Comfortable, but the test must confirm it at print scale rather than assume it.

### A7. Cycle-cost measurement (still no shaping enabled)

DWT-CYCCNT bracket around `pe_axis_eval`; min/max reported through `phase_exec_seg_query`.
**Gate: < ~15 % of the 21 000-cycle tick at 8 kHz.** Measured on hardware with `shape=0` driving a
synthetic ring, so it is a benchmark, not a live-motion change.

---

## 3. Order of work

1. `phase_shaper.h` / `.c` + eval — no ISR touch
2. Golden harness + vectors — **must pass before anything else proceeds**
3. Commands + `struct phase_exec` fields, `shape` defaults 0
4. Cycle instrumentation + on-hardware benchmark
5. **Checkpoint — review before Phase B**

## 4. Explicitly NOT in Phase A

- Host raw-trapezoid emitter
- Deleting `_shaped_motor` / `_build_st_forward` / the slew chain-fit machinery
- Engage and MSCNT `phase_offset` handoff
- Startup pre-fill of the lookback window
- Halt / disengage tail flush
- Setting `shape=1` on any real motor

## 5. Decisions taken (flagged for review, not assumed silently)

- **Ring reuse** (A1) — motor A's ring = X, motor B's ring = Y. Keeps this off the `alloc_chunk`
  cliff. Alternative was two dedicated 20 KB axis rings.
- **Post-projection dry clamp** (A3) — protects the Fix-3 "< ½ electrical period" invariant.
  Alternative was per-axis clamping, which can sum to 2× in motor space.

## 6. Constraints carried from the session

- Validation gate is the offline golden-compare — pure numeric, **before any hardware**.
- No flashing or printing unless the user is at the machine.
- Host is a Pi 5 now; the Pi-4 `Assembly1` reprint (the one that threw "Timer too close") stays the
  end-to-end validation target for Phase D.
- Refresh rate (8 kHz) is **not** a lever for host load — do not conflate them.
- Multi-session effort; check in at every phase boundary. This is the hottest, most
  safety-relevant path in the firmware.

---

## 7. Implementation record (2026-07-29)

Branch `feature/mcu-input-shaping`, cut from `aa13e50d3`.

| commit | what |
|---|---|
| `72323f308` | pre-work brief §1.2 (`seg_primed` -> `PE_PRIME_DEPTH` 16) + §1.3 (golden-oracle pins) |
| `50d175c97` | `src/phase_shaper.{c,h}` + `test/shaper_golden/` |

### Done

- **§3 step 1** — `src/phase_shaper.{c,h}`. Dependency-free (`stdint.h` only); `gcc -Wall
  -Wextra -Werror` clean standalone. `struct phase_seg` moved into the header so the
  evaluator and `phase_exec.c` cannot disagree on layout; **still 20 bytes**, so ring sizing
  and the `alloc_chunk` budget are unmoved.
- **§3 step 2** — the golden gate. **PASSES at 0.7–3.0 ULP of float32**, 15 vectors.

- **§3 step 3** — `b06ec6e35`. `struct phase_exec` gains `shape`/`cx`/`cy`/`pos_ref`;
  `pe_shaped_pos()` evaluates both axis rings, projects, and retires; commands
  `phase_exec_shape` (bind + coefficients + enable), `phase_exec_pulse` (staged, committed
  whole), `phase_exec_shape_query`. Ring reuse is live: motor A's `seg_ring` binds as X,
  motor B's as Y. **Dead code at `shape = 0`** — the entire diff is additive apart from one
  moved local declaration, so every existing path is byte-for-byte unchanged. Builds clean
  (59 400 B, +2080); all three commands present in the dictionary.

### Open (rest of Phase A)

- **§3 step 4** — the DWT-CYCCNT bracket is written and reported through
  `phase_exec_shape_query` (`cyc_max`/`cyc_last`, budget 21 000 cycles/tick @ 8 kHz), but the
  actual on-hardware benchmark needs the machine.

### Not covered by the gate

The golden gate validates `phase_shaper.c`. The ~45 lines of glue in `pe_shaped_pos()` —
projection, hold-on-EMPTY/LAGGED, retire wiring, the depth diagnostic — are reviewed but not
machine-tested; they cannot be unit-tested without dragging `phase_exec.c`'s MCU dependencies
into the harness. Treat them as unproven until the hardware bring-up in Phase C.

### Deviations from the plan above

1. **Two bounds the plan did not have.**
   - `PE_ADVANCE_MAX` (64) caps the forward cursor walk. Unbounded it is the same prio-0 ISR
     hazard as brief §1.1 — but unlike the retire loop, deferring is *not* safe here: a
     lagging cursor evaluates a **stale segment**, i.e. commands a wrong angle. So saturation
     raises `PE_EVAL_LAGGED` and the caller must hold rather than trust the value.
   - `PE_BACKSCAN_MAX` (4) bounds a backward walk. §A2 said the Python containment back-scan
     could be dropped because MCU segments are contiguous — true, but contiguity does not
     protect against `now` stepping *backwards*, which a live `SET_PHASE_LEAD` change or a
     re-engage can do. A forward-only cursor would silently commutate off a stale segment.
     Honest scope correction: the original justification (per-motor `lead_ticks` skew under
     round-robin) does **not** hold — that skew is microseconds and cannot reverse a 125 us
     tick. The walk is defensive against the re-seed cases, and it is now tested.
2. **The dry-coast policy is explicitly NOT in the evaluator.** `pe_axis_eval` freezes at the
   last segment's endpoint exactly as the oracle does, and reports `PE_EVAL_PAST_HEAD`. Plan
   §A3's post-projection clamp becomes the caller's job in step 3. This is what keeps the
   golden compare meaningful — a safety policy inside the evaluator would have no oracle to
   compare against.
3. **§A6's `max|Δ| < 1e-4` su gate was not achievable and has been replaced.** At print scale
   one float32 ULP is ~0.006 su, so a flat 1e-4 su bound is arithmetically impossible — it
   would have been a fake gate. Tolerance is now ULP-scaled (`8 * ULP`, floors 1e-4 su /
   1e-3 su/s) and the report prints error in ULP. This is the precision the existing
   motor-space path already runs at (`phase_seg.start_pos` has always been float32 in
   step-units), so it is a status-quo bound, not a new cost.
4. **The gate asserts velocity too**, which §A6 did not call for. Velocity drives the fwd/bwd
   cogging blend, so a wrong derivative shows up as a torque transient at reversals rather
   than a position error. An early mutation run passed on position alone while carrying a
   1e4 su/s velocity error.
5. **Retire-equivalence added** as a third assertion: every vector runs twice, tail pinned vs
   `pe_axis_retire` advancing. That is the direct test of correction #3 (the lookback window).

### Measurements that replace estimates

- **The shaper window is ASYMMETRIC.** Measured from the live shapers (EI@55.4 X, MZV@43.4 Y):
  **lookback 10.56 ms, lookahead 7.58 ms**. §1 correction 3 guessed ~11.5 ms either side. The
  ring must retain 10.56 ms of history; the host's `_pulse_margin` covers the 7.58 ms.
- `phase_shaper.o` = **900 B text, 0 data, 0 bss**. `struct pe_axis` = 96 B (x2 = 192 B).
  Nothing here threatens the `alloc_chunk` cliff.
- Firmware builds clean: `./build.sh noboot`, 57 320 B. **Not flashed.**

### Gate has teeth

9 of 10 deliberate mutations of `phase_shaper.c` are caught (table in
`test/shaper_golden/README.md`). The one that is not — forward advance `>` vs `>=` — is
genuinely equivalent on a contiguous chain. Two vectors (`jitter*`, `reanchor*`) exist
because earlier versions of the gate were themselves wrong and let real mutants through.

---

## 8. Phase B implementation record (2026-07-29)

Commit `f1457ab33`. Host side: raw axis-trapezoid emitter + pulse-table shipping.

### What landed

- **`PhaseExec._raw_axis_segments()`** — a `@staticmethod` with the clock conversion injected.
  One trapq move → exactly one X segment and one Y segment (both axes are already
  constant-accel within a move). Replaces the entire shaped-fitting path.
- **`_trapq_flush_raw_cb()`** — the Phase B flush callback. Horizon is `flush_time`, **not**
  `flush_time - _pulse_margin`: the margin existed because the host evaluated shaped samples
  at `T+t_i`; it evaluates nothing now, so the lag disappears.
- **`_ship_shaper_pulses()` / `_send_shape_binding()`** — `_load_shaper_pulses` /
  `_init_shaper_pulses` are reused *unchanged*; the result is shipped instead of convolved.
  Binding sends a bind pass then an enable pass, because the MCU refuses `enable` until both
  axis rings are bound (a single pass would leave the master at `shape=0`).
- **`ENGAGE MCU_SHAPE=1`** — engage order is analytic ON → pulses → bind+enable → start.
  Enabling before `start_cmd` is deliberate: rings are empty, so the first ticks evaluate
  EMPTY and hold at `last_pos = 0`, i.e. angle = `phase_offset` = MSCNT.

### Deviation from the plan — deliberate

The plan said Phase B **deletes** the shaped fitter. **It does not.** `MCU_SHAPE` defaults
OFF and the host-shaped path remains the default. Deleting a proven path before its
replacement has ever run on hardware would leave the machine unprintable if anything is
wrong, and none of this can be tested on hardware yet. Phase C flips the default after
bring-up; Phase D deletes. (`_shaped_motor` / `_eval_axis` stay regardless — they are the
oracle.)

### Known gap — Phase C blocker

**The reanchor flag is inert in shaped mode.** `pe_seg_advance` is never called, so nothing
consumes it; the emitter sets it after a gap-skip and the MCU masks it out of `duration`.
Resume-after-halt phase seeding is unimplemented. This must be closed before a print.

### Gate extension

Every vector now runs twice — synthetic 1:1 segments, then the **real emitter** at
`spm = 200`. (`spm = 1` would let an emitter that dropped the mm → step-unit conversion pass.)
6 of 7 emitter mutations are caught numerically.

Added a **structural contiguity assertion**, because numeric comparison provably cannot see
the defect that matters most here: a sub-tick gap at every segment boundary is 6 ns, far under
the float32 floor, yet gaplessness is `pe_axis_eval`'s documented precondition. Verified it
fires on an injected 1-tick hole.

That assertion also established something worth not "fixing" later: deriving `duration` by
rounding `move_t` independently of the rounded start is **harmless in this design**, because
`tc = ec` chains regardless. Telescoping is enforced structurally, not by the arithmetic.

New vector `gap`: a genuine hole in the move list (not `dwell`'s explicit hold move). The
oracle freezes across it; the emitter fills it with a hold segment; both must agree.
