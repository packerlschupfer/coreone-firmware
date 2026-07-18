# MCU-side input shaping — hardware bring-up (Phase C)

Everything up to here is code-complete and offline-validated but **has never run on hardware**.
This is the ordered procedure for the parts that need the machine. Plan + implementation
records: `mcu-input-shaping-phaseA.md` (§7 Phase A, §8 Phase B).

**Prerequisite: flash a firmware built from `feature/mcu-input-shaping`.** Nothing below works
on the currently-installed firmware — it has no `phase_exec_shape` / `phase_exec_pulse`.
`ENGAGE MCU_SHAPE=1` fails cleanly with a message on an older MCU rather than misbehaving.

```
./flash.sh f427-katapult-usb        # run this WHERE THE BOARD IS
```

That profile builds the **`katapult` variant (0x08008000)** and USB-flashes it. It runs
`katapult-preflight.sh --reboot` first, which reboots the board into the bootloader and
**refuses to flash if Katapult does not appear** — that check is the difference between a safe
flash and erasing the open bootloader. Do not substitute `f427-noboot-swd`: that links at
0x08000000 and would overwrite Katapult.

⚠ **`build.sh noboot` is a compile check only, not a flashable artifact for this board.**
Every "builds clean" recorded in the Phase A/B/C notes used it in that sense.

**Default is unchanged.** `MCU_SHAPE` defaults 0 and `shape` defaults 0 on the MCU, so a
flashed-but-unused firmware behaves exactly as today. Every step below is opt-in.

---

## 0. Regression check first — the default path

Before touching shaping at all, confirm the new firmware is a no-op for normal use.

```
PHASE_TRAPQ_ENGAGE STEPPER=stepper_x        # no MCU_SHAPE -> host-shaped path
<a short normal print or a few travels>
PHASE_TRAPQ_DISENGAGE STEPPER=stepper_x
```

**Gate:** `PHASE_SEG_STATS` numbers in family with previous runs — `overflow=0`, `JUMP=0`,
`snapMAX` unchanged. If anything moved here, stop: the Phase A/B additions were supposed to be
additive (the whole `phase_exec.c` diff has one removed line) and something is wrong.

One expected difference: `minrep` now primes at ring depth 16 instead of 512
(`PE_PRIME_DEPTH`), so the reported margin will be a real number earlier in the run.

## 1. Cycle benchmark — the A7 gate

⚠ **CORRECTED 2026-07-30 — the original version of this step measured NOTHING.** "Motor
stationary, ring idling" means `pe_axis_eval` returns `PE_EVAL_EMPTY` on its early exit, so the
reported cost (384 cycles, measured) is the empty-ring path, not the convolution. The real cost
only appears with a POPULATED ring. Measure during/just after motion:

```
PHASE_TRAPQ_ENGAGE ... MCU_SHAPE=1
G1 X130 F300                                # something short and slow
PHASE_SHAPE_STATS STEPPER=stepper_x         # ring still populated -> real cost
```

**Gate:** `cyc max` < **~630** at RATE=40000, or < ~3150 at RATE=8000.

📊 **MEASURED 2026-07-30: `cyc max=4205`, `last=3163`.** That is ~15–20 % of the 8 kHz budget
(workable) but **EXCEEDS the entire 4200-cycle tick at 40 kHz** — MCU-side shaping cannot run at
this machine's default rate as currently built. Root cause in §Known gaps.

⚠ Corrected 2026-07-30 after step 0: **this machine's `PHASE_STEP_ON` defaults to RATE=40000**,
not the 8 kHz the plan assumed. The tick budget is therefore 168e6/40000 = **4200 cycles**, and
15 % of that is 630. The original 300–600 cycle estimate sits right at that limit, so this gate
is genuinely tight rather than the formality it looked like at 8 kHz. If it fails, engage with
`PHASE_STEP_ON RATE=8000` (21 000-cycle budget) and re-measure before concluding anything about
the cost model.

Also check `bound=0x3` (both axis rings bound). `bound=0x1` or `0x2` means only one motor got
its binding and shaping will not have enabled.

**Do not proceed if the cycle gate fails.**

## 2. Static engage — no motion

Still with `MCU_SHAPE=1`, motor engaged, toolhead idle.

**Gate:** no audible change, no lurch, no torque step at the moment of engage. The first ticks
evaluate EMPTY and hold at `last_pos = 0`, i.e. angle = `phase_offset` = MSCNT, so this should
be indistinguishable from a normal engage.

`PHASE_SHAPE_STATS` — `hold` will be non-zero (the ring is empty until the first flush), and
`primed=0` until the lookback fills. Both expected.

## 3. First motion — slow, single axis, hand on the power

⚠ **This is the first time a commutation angle has ever been driven from the shaped path.**
Give a beat before commanding it and be ready to cut power.

```
G1 X100 F600        # slow, single axis
```

**Gate:** motion is smooth, no rumble, no slip. Then `PHASE_SHAPE_STATS`:

| field | expected | if not |
|---|---|---|
| `primed` | 1 | the ring never held the ~10.6 ms lookback — feed problem |
| `lateBEF` | **0** | history retired too early — a retention bug, stop |
| `X/Y lag` | 0 | forward cursor walk saturating — segments arriving far behind |
| `X/Y empty` | small, early only | ring starving |
| `hold` | small | same |
| `cyc max` | as step 1 | |

`past` non-zero at the end of a move is normal (ring dry ahead → froze at the endpoint).

## 3b. OPERATIONAL: reboot the MCU before homing after a phase-stepping session

Learned the hard way 2026-07-30. After several engage/disengage cycles, `G28` degraded:
first the pre-existing `MSCNT_HOME X phase off 496/512` warnings, then a hard
`MSCNT_HOME Y ABORTED: drifted 32mm toward the far frame — StallGuard too noisy` with three
`travel=0.00 (premature trip)` retries. The machine was COLD, so the error's own "home cold,
not hot" explanation did not apply, and there were ZERO such aborts in any log from 07-25 to
07-29 — it was new that day.

**A `FIRMWARE_RESTART` cleared it completely**: the very next `G28` homed both axes first try,
phase-locked, `SNAP 0.0000mm`, with no `phase off` warning at all. So it is accumulated
MSCNT/rotor state from repeated phase-stepping exits, not a firmware defect.

⚠ `homed_axes` read `"y"` through the failure — a premature StallGuard trip is
indistinguishable from a real home in the data. Do not trust position after one.

**Rule: `FIRMWARE_RESTART` between a phase-stepping session and the next `G28`.** Do not chase a
premature trip with repeated re-home attempts.

Trigger a queue-drain halt and resume: a `PAUSE` / long `TEMPERATURE_WAIT` mid-move, then
resume. This is the only way to exercise `PE_EVAL_REANCHOR` on hardware.

**Gate:** `reanch` ≥ 1 in `PHASE_SHAPE_STATS`, and no positional step at the resume.

The angle arithmetic itself IS now covered offline (see Known gaps). What is not covered is the
interaction: a gap-skip leaves the ring non-contiguous at exactly the moment the reanchor fires,
and no offline vector reproduces that pairing. **Treat a resume glitch here as expected-ish and
diagnosable, not as a surprise** — and if one occurs, suspect the hole, not the angle math.

## 5. Resonance re-test — does the shaping still work

The point of all this is that the shaper still cancels resonance. Re-run the ringing test
(`belt_tune` / the input-shaper test print) with `MCU_SHAPE=1` and compare ringing amplitude
against a host-shaped run of the same geometry.

**Gate:** ringing amplitude unchanged vs host-shaped. If ringing is *worse*, the convolution is
reaching the motor differently than the oracle predicts, and the golden gate did not catch it —
that would be a genuinely interesting finding and the pulse-table shipping is the first suspect.

### ✅ PASSED 2026-08-01

X-axis, band-passed to the shaper target (55–72 Hz, live `mzv 63.6`), RMS of the accelerometer
in the first 30 ms after an abrupt stop:

| condition | 3–30 ms | 30–60 ms | 60–120 ms | vs unshaped |
|---|---|---|---|---|
| unshaped (`IS=0`) | **2062.8** | 392.9 | 412.5 | 100 % |
| host-shaped (reference) | 376.8 | 125.7 | 74.7 | **18 %** |
| **MCU-shaped** | 399.9 | 112.3 | 75.9 | **19 %** |

Shaping gives **5.5x suppression**, and MCU-side reproduces host-side within 6 % — inside
single-run scatter. Segment count for identical motion: **24 (MCU) vs 127 (host) vs 153
(unshaped)**, i.e. ~5x fewer host->MCU messages, which is the project's actual objective.

### How to run it so it MEASURES something

Three things all had to be right; getting any one wrong makes the test read "inconclusive":

1. **Excite the shaper's frequency.** Residual vibration at `f` peaks when the deceleration ramp
   is HALF a period. For 63.6 Hz that is 7.86 ms, so with `ACCEL=10000` use
   `v = 10000 x 0.00786 = 78.3 mm/s` -> **`G1 X160 F4700`**. A "fast" move (F18000, 30 ms ramp
   = 2 periods) barely excites it at all — the first attempt showed only 84 % RMS vs unshaped
   and looked like the shaper did nothing.
2. **Use a window matched to the DAMPING, not to your patience.** At zeta=0.1 a 63.6 Hz mode has a
   ~25 ms time constant. Averaging over 250 ms dilutes it ~10x into the broadband floor — that
   alone turned a 5.5x effect into "no measurable difference". Use **3–30 ms** and band-pass.
3. **Never move right after `ACCELEROMETER_MEASURE` (stop).** See §3c.

⚠ Raw RMS across the whole decay is USELESS here: it is dominated by 252–260 Hz and 16–20 Hz
content that is identical in all three conditions and has nothing to do with the shaper.

### 3c. Never move immediately after the accelerometer readout

`ACCELEROMETER_MEASURE` (stop) makes klippy read out the buffer and write an ~80 KB CSV. That
host burst starves the phase-stepping segment feed. With a move placed right after it, all three
runs of the first attempt juddered audibly and the disengage stats showed **JUMP=12/16** (field
teleports) and **minrep=0** (ring hit empty) — while **nothing** appeared in klippy.log, no
"Timer too close". Inserting `G4 P2500` between the stop and the next move gave **JUMP=0** on all
three runs.

## 6. The actual objective — Pi-4 `Assembly1` reprint

Reprint the `Assembly1` model that threw "Timer too close" on the Pi 4, with `MCU_SHAPE=1`.

**Gate:** completes; host motion CPU and segment rate both visibly down (sends drop from ~6–12
per move to 2). This is the whole reason the project exists.

### ✅ PASSED 2026-08-02 as a DRY RUN

The full `Assembly1` is 5 h 32 m, and the original failure happened inside the first ~20 layers,
so it was cut down to a **motion-only dry run**: layers 2–4 (skipping the slow first layer, which
is not the stress case), **no heat, no extrusion, Z lifted +2 mm**, generated by stripping `E`,
`M104/109/140/190`, and offsetting `Z`. **39 125 moves at ~31 moves/s over 21 min** — the same
dense full-speed geometry, at 1/16 the time and zero filament.

```
streamed 92017 segs | overflow=0 dry=2 minrep=5 | refills=0 snapSUM=0 snapMAX=0
| settleEXC=0 maxCOAST=0 | JUMP=0 jumpMAX=0 | chainDIV=0 | segERR=0.0su
mid-run: cyc max=1295 (31% of the 40kHz tick) | primed=1 lateBEF=0 hold=3
         past=0 bef=0 back=0 lag=0 | stack 608/2048 (30%)
Timer too close: 0        errors: none
```

- **92 017 segments / 39 125 moves = 2.35 per move** — the raw-emitter target (X+Y per move plus
  holds). The legacy fitter measured 5.3x more on identical motion.
- **`dry=2`** in ~50 million ticks, `hold=3`. The ring was kept fed essentially perfectly.
- Every tripwire zero: overflow, JUMP, snap, coast, chainDIV, segERR.
- `minrep=5` — a real margin figure, only visible because of the `PE_PRIME_DEPTH` fix.

⚠ **A dry run validates the MOTION path only** — which is what MCU-side shaping affects, and what
"Timer too close" is about. It does **not** validate print quality (extrusion, first layer,
surface finish). A real print is still required before calling the feature done end to end.

## 7. Flip the default — ✅ DONE 2026-08-01 (`100ccb6cd`)

`MCU_SHAPE` now defaults **on**, after steps 0–5 passed on hardware. ⚠ Note this was flipped
BEFORE step 6 (the `Assembly1` reprint) rather than after — deliberately, because the print is
then itself the test of the default. **If step 6 fails, revert this first.**

The default is **auto (-1)**, not a bare 1, so it cannot hard-fail the cases it does not serve.
Verified on hardware:

| engage | result |
|---|---|
| default | `SHAPE=MCU` |
| `IS=0` (unshaped control) | `SHAPE=host` — silent fallback, no error |
| explicit `MCU_SHAPE=1` + `IS=0` | clean error (user asked, so say why) |

Non-CoreXY, and MCUs built without `src/phase_shaper.c`, fall back the same way. The engage
banner reports `SHAPE=MCU|host` so the log is never ambiguous about which path ran.

Phase D then deletes the host-shaped fitting machinery (`_shaped_motor` and `_eval_axis` stay —
they are the golden gate's oracle and are pinned DO-NOT-DELETE).

---

## Abort / revert

`archive/shaper-on-pi-20260729` @ `aa13e50d3` is the pre-project state. Reverting the firmware
is enough on its own: with an old MCU, `MCU_SHAPE=1` is refused and the host-shaped path runs.

## ✅ RESOLVED: hard-float adopted (`hardfloat.patch`)

Measured on hardware 2026-07-30, same moves, both rates:

| | soft-float | hard-float |
|---|---|---|
| `cyc max` | 4205 | **1250** |
| fraction of the 8 kHz tick (21 000) | 20 % | **6 %** |
| fraction of the 40 kHz tick (4 200) | **>100 % — did not fit** | **30 % — fits** |
| firmware size | 59 816 B | **58 040 B** (smaller) |

3.4x faster and smaller. Motion correct at both rates, positions exact, `past=0 lag=0 back=0`,
no faults. Note 30 % at 40 kHz is **double the ~15 % design target** — it fits with real headroom,
but it is not free, and anything else added to this ISR has to be costed against the remaining 70 %.

The original blocker analysis is kept below because it explains why the patch exists.

## (historical) BLOCKER FOUND AT BRING-UP: the firmware was SOFT-FLOAT

The plan costed the convolution at "300–600 cycles **with the FPU**". That assumption is false:
Klipper's STM32F4 target compiles with `-mcpu=cortex-m4` and **no** `-mfpu` / `-mfloat-abi=hard`,
so despite `__FPU_PRESENT 1` on the F427 every float operation is a library call.

Evidence: `phase_shaper.o` contains **zero** VFP instructions and 32 `__aeabi_f*` calls
(12 `fadd`, 16 `fmul`, plus conversions); the whole `klipper.elf` contains zero `v*.f32`.
That is ~10x the per-op cost assumed, and it matches the measured 3163 cycles almost exactly.

Options, in rough order of value:
1. **Build the F427 with hardware float** (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`). Restores the
   original estimate (~300–400 cycles), makes 40 kHz comfortable, and speeds every other float
   consumer too (`pe_update_angle`'s own trajectory math is 39 soft-float calls). Needs care:
   it changes codegen firmware-wide and FP state in ISRs relies on Cortex-M4 lazy stacking.
   Upstream Klipper does not do this, so understand WHY before adopting it.
2. **Cap MCU-side shaping at RATE=8000**, where it already fits at ~15–20 %.
3. **Reduce float work in the eval** (precompute per-segment coefficients, or fixed-point).

Until one of these lands, do NOT enable `MCU_SHAPE=1` at RATE=40000.

## Known gaps going in

Two gaps listed here originally were **closed** in `ea90d1301` by moving the arithmetic into
`phase_shaper.c`, where the golden harness can reach it:

- ~~the glue in `pe_shaped_pos()` is not machine-tested~~ → projection, hold policy and the
  reanchor now live in `pe_motor_step()` / `pe_reanchor_edge()` and are covered.
- ~~the reanchor **angle** bookkeeping is unvalidated~~ → the harness now emits the commanded
  angle and the gate asserts the invariant directly: **on the tick a motor re-anchors, the
  commanded angle must not move.** 4 of 4 reanchor mutations are caught.

What remains genuinely unproven on hardware:

- The bookkeeping still left in `pe_shaped_pos()` — counters, retire wiring, the depth
  diagnostic, the cycle bracket. Not safety-critical, but not covered either.
- **After a gap-skip the ring is deliberately not time-contiguous** (the host jumps `t0` to
  `t1 - 0.25`). The evaluator handles it — the cursor freezes at the old endpoint until `tau`
  reaches the new segment — but it is the one place the documented contiguity precondition is
  knowingly violated, and it coincides exactly with the reanchor edge. **This is now the single
  least-validated interaction in the project** and is what §4 is really testing.
- Everything timing-related: the cycle cost (§1), and whether the real trapq feed keeps the
  ring ahead of the 7.58 ms lookahead under load.
