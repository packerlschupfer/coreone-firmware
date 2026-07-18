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
- StallGuard **crash detection** (X/Y), with a guard refusing to arm while phase-stepping is
  engaged (shared SPI3). — 2026-06 … 2026-07-03

## Sensing
- HX717 load cell as loadcell **Z-probe**; extruder **filament-presence** sensor on the same HX717
  (channel B), with a 12:1 channel **interleave** so both run during a print. — 2026-06 … 2026-06-28
- Loadcell-based extruder **clog / stuck-filament detection** (`estall_detect`; 5-tap FIR over the
  loadcell force stream). — 2026-06-28

## Peripherals
- ILI9488 colour TFT display type (SPI6 TX-DMA). — 2026-06
- PuppyBus RS-485 master, used to *flash* the H503 enclosure MCU (at runtime the H503 is a
  second Klipper MCU on its own USB); TCA6408A I/O expander. — 2026-06

## Build / tooling
- `coreone/` reproducible build + flash tooling (SWD / DFU / BBF / RS-485); Kconfig/Makefile wiring. — 2026-06

## Deliberately NOT included
Printer *configuration* (`printer.cfg`, `boards/xbuddy.cfg`, `h5/extension.cfg`) — it carries real
device serials and lives host-side (`coreone-host`), so this repository stays secret-free.
