# CHANGES — modifications to Klipper (GPLv3 §5 disclosure)

`coreone-firmware` is a **modified version of Klipper**
([github.com/Klipper3d/klipper](https://github.com/Klipper3d/klipper), GPLv3), the open-firmware
port for the **Prusa Core One+** (STM32F427 xBuddy + STM32H503 xBuddy-extension). It is based on
upstream Klipper **v0.13.0-699** (`c707dd19214709dc23684b254a68e3bf69e4cfb3`).

Per GPLv3 §5, the significant modifications this fork carries on top of upstream are disclosed
below, with the dates they were made. The full **git commit history** in this repository is the
authoritative, dated record; the summary below is the human-readable index. Upstream Klipper's
copyright notices and per-file license headers are **retained unchanged**; the license
([COPYING](COPYING), GPLv3) is unchanged.

## Platform / HAL
- STM32F427 (xBuddy) + STM32H503 (xBuddy-extension) board support; vendored STM32H5 HAL/CMSIS. — 2026-06
- APB2 full-speed (84 MHz) clock tree, for Prusa hardware parity. — 2026-06-27
- BASEPRI critical sections; a priority-0 IRQ tier reserved for phase-stepping (serial/CAN bumped to 1). — 2026-06/07

## Motion / homing
- StallGuard (sensorless) homing. — 2026-06
- MSCNT rotor-phase, temperature-robust *validated* homing (Prusa-style: coarse StallGuard →
  travel-validate/retry → phase-snap), and native `G28.1` home under a `[gcode_macro G28]` override. — 2026-06 … 2026-06-27
- Phase-stepping executor: **open-loop** TMC2130 XDIRECT commutation on the F427, plus cogging
  compensation. — 2026-06
- Phase-stepping **per-layer registration fix**: round-robin commutation, position-continuous
  segment chaining (POS-FIT), forward-cursor shaped-trajectory build, D1 host back-pressure,
  D7 teleport tripwire (JUMP counter), firmware min-duration merge. — 2026-07-05
- Phase-stepping **MCU-side input shaping**: the input-shaping convolution moved from the host onto
  the F427 (`phase_shaper.c`); the host now streams raw un-shaped axis trapezoids and the MCU
  convolves + projects to CoreXY in the 40 kHz ISR (`MCU_SHAPE`, default-on/auto, host-shaped path
  retained as fallback). Requires the F427 built **hard-float** (`-mfpu`/`-mfloat-abi=hard`,
  `MACH_STM32F427` only; CPACR enabled early in `armcm_boot.c`; `STACK_SIZE`→2048). Offline
  equivalence gate in `coreone/test/shaper_golden/`. — 2026-08
- Phase-stepping engage: **clear the TIM8 NVIC pending latch on re-arm** (Prusa BFW-8383). Clearing
  `TIM8->SR` at re-arm kills the peripheral flag but not the NVIC pending bit (and `armcm_enable_irq` is
  SetPriority+EnableIRQ only, no ClearPending), so a `UIF` latched just before disengage could fire one
  tick early at the next engage → a single mistimed commutation frame. Two `NVIC_ClearPendingIRQ` in
  `phase_exec.c` (`pe_tim8_start`/`pe_tim8_stop`); justified by code inspection, re-engage exercised
  clean on hardware. — 2026-08-30
- Phase-stepping slave-stats diagnostic (host-side, `phase_exec.py`): dropped `seg_dry` from the
  starvation warn condition. An engaged-but-idle ring is dry by definition, so it false-alarmed
  ("position ratchet risk") on healthy manual engages; the honest starved signals
  (`overflow`/`jump`/`refills>0`/`minrep==0`) remain and `dry` is still printed as data. — 2026-08-30
- Phase-stepping trapq **gap-skip / pause-resume XY correctness** (host-side, `phase_exec.py`), after a
  bench move under phase-stepping finished ~65 mm off its mark with every slip counter reading zero:
  - **gap-skip fix** — on a queue-drain resume, find the first *real* move in the drained span and resume
    there instead of past it, so the move is kept rather than dropped. *Hardware-proven* (the head now
    returns on the mark); also offline-gated. — 2026-09-25
  - **`_bridge_gap`** — a multi-minute engaged idle left a ring hole longer than the signed-32 clock window
    (`phase_shaper.c:122`), so the evaluator raised `PE_EVAL_BEFORE` continuously (`lateBEF=65045`, a field
    doc'd "Must be 0"); the hole is now bridged with bounded hold segments. Offline regression gate
    `coreone/test/shaper_golden/gap_bridge_test.py` (180 s hole: 3.15M BEFORE → 0). Fixes the BEFORE storm,
    **not** the motion loss above. — 2026-09-25
  - **STALE late-arrival detector** + a `get_status()` (`gap_skips`/`stale_segs`/`xy_trusted`) that surfaces
    a segment arriving after its clock window as a loud stop-stat rather than a silent XY offset;
    reviewed-and-reasoned (has not yet fired). The `[gcode_macro]` park guards (PRINT_END / PAUSE) now gate
    on `stale_segs` — a deliberate relaxation of the prior unconditional refuse, on operator instruction. — 2026-09-26
  - **park-guard fail-open closed (`PHASE_XY_CHECK`).** Gating the park guard on `stale_segs` alone left a
    hole: a real M600 pause lost XY the *other* way — the MCU-side shaper ran out of history and held while
    the host position advanced (`lateBEF` high, `stale_segs` still 0), so `xy_trusted` read 1 and PRINT_END
    drove an absolute park to a position the machine did not have. Now a `PHASE_XY_CHECK STEPPER=<n>` mux
    command refreshes an on-demand `late_before` cache from the MCU (get_status is Moonraker-polled, too hot
    for a per-read round trip) and `xy_trusted = 0 if (stale_segs or late_before)` — both loss paths feed one
    verdict. printer.cfg calls `PHASE_XY_CHECK` then reads the verdict in the `_PRINT_END_PARK`/`_PAUSE_PARK`
    sub-macros (a gcode_macro's Jinja renders once, so the refresh must precede the read). The failure it
    closes is hardware-observed; the fix itself is reviewed-and-reasoned, not yet exercised by a real paused
    print. — 2026-09-26
  engaged (shared SPI3). — 2026-06 … 2026-07-03
  - optional **shadow telemetry** (`CRASH_DETECT SHADOW=1`): MCU-side accumulation of the gated-in
    `sg_result` distribution + a would-trip counter, reported to the host on disarm instead of
    shutting down — for tuning `sg_floor`/`crash_sgt` from real prints. Off by default. — 2026-07-26

## Sensing
- HX717 load cell as loadcell **Z-probe**; extruder **filament-presence** sensor on the same HX717
  (channel B), with a 12:1 channel **interleave** so both run during a print. — 2026-06 … 2026-06-28
- Extruder filament sensor **armed for runout** (not insert-only), per Prusa's design of running
  runout on both the hotend and spool sensors independently, with an **edge gate** in
  `_eval_presence` (`hx71x_filament_sensor.py`; port of `filament_sensor.cpp:19-34`): no edge event
  is emitted across a non-working window (out-of-range / uncalibrated / the probe holding HX717
  channel A) — the new state is adopted silently, so a first-after-resume sample cannot fire a false
  runout mid-print. — 2026-09-26
- Loadcell-based extruder **clog / stuck-filament detection** (`estall_detect`; 5-tap FIR over the
  loadcell force stream). — 2026-06-28
- Per-tool **filament / material profiles** (`[filaments]`): a port of Prusa's `config_store` filament
  model — the preset table from `src/common/filament.cpp`, `M865`, and the mismatch guard from
  `marlin_print_preview.cpp` (`check_correct_filament_type`). Per-tool throughout (for INDX). Runtime
  state lives host-side in `save_variables`; the module is inert unless `[filaments]` is configured. — 2026-08-30
- Loadcell **pressure-advance bench probe** (`loadcell_pa_probe.py`): a calibration tool (enable temporarily
  in `printer.cfg`) that drives a slow→fast extrude step in air across a list of PA values and captures the
  HX717 nozzle-loadcell force, writing one CSV + a summary line per PA, to pick an optimal pressure-advance K.
  Reuses the estall HX717 ch-A client and suppresses estall around the sweep. Bench/diagnostic, **not a
  print-path module**; honours `soft_cancel` via `_check_abort`. — 2026-09-26

## Peripherals
- ILI9488 colour TFT display type (SPI6 TX-DMA). — 2026-06
- PuppyBus RS-485 master, used to *flash* the H503 enclosure MCU (at runtime the H503 is a
  second Klipper MCU on its own USB); TCA6408A I/O expander. — 2026-06

## Build / tooling
- `coreone/` reproducible build + flash tooling (SWD / DFU / BBF / RS-485); Kconfig/Makefile wiring. — 2026-06
- `coreone/soft-abort.sh` — out-of-band **soft abort**: calls `soft_cancel.py`'s `soft_cancel/abort` webhook
  directly over klippy's UDS. It's the only way to interrupt a macro that holds the gcode mutex — the
  `SOFT_ABORT` gcode *command* itself queues behind the running macro and never fires. `soft_cancel` registered
  and documented that webhook but nothing in-tree ever called it, so the out-of-band path did not exist for
  any user. Trigger tested end-to-end (`{"status":"aborting"}`); the per-module bail-out (`_check_abort`) has
  not yet been fired in anger. — 2026-09-26

## Deliberately NOT included
Printer *configuration* (`printer.cfg`, `boards/xbuddy.cfg`, `h5/extension.cfg`) — it carries real
device serials and lives host-side (`coreone-host`), so this repository stays secret-free.
