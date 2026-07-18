# coreone-firmware — a Klipper fork for the Prusa Core One+

This repository is a **fork of [Klipper3d/klipper](https://github.com/Klipper3d/klipper)**
carrying the open-firmware port for the **Prusa Core One+** (STM32F427 xBuddy + STM32H503
xBuddy-extension). It is the single source of record for the port — the host (klippy) extras
**and** the MCU firmware live here together, committed as real history over a pinned upstream base.

## What's in the fork
- **Base:** vanilla Klipper **v0.13.0-699** (`c707dd19214709dc23684b254a68e3bf69e4cfb3`).
- **Port commits** (logical, on top of the base):
  - vendored STM32H5 HAL/CMSIS for the H503
  - STM32F427 + STM32H503 platform support
  - HX717 load cell as Z-probe + extruder filament sensor
  - ILI9488 colour TFT display (SPI6 TX-DMA)
  - StallGuard sensorless homing, MSCNT homing + crash detection
  - phase-stepping (open-loop TMC2130 XDIRECT phase execution + cogging compensation)
  - PuppyBus RS-485 link to the H503 (used to *flash* it; at runtime the H503 is a
    second Klipper MCU on its own USB) + TCA6408A I/O expander
  - belt tuning, build sheets, soft cancel, mesh progress
  - build wiring (Kconfig/Makefile) + `coreone/` build & flash tooling
- **Deliberately NOT in the fork:** printer *configuration* (`printer.cfg`, `boards/xbuddy.cfg`,
  `h5/extension.cfg`). Those carry the real serials and live host-side in **`coreone-host`**
  (ansible), so this fork stays secret-free.

## Build & flash
The `coreone/` directory holds the reproducible build/flash tooling:
- F427: `coreone/.config` → `make`; pack with `coreone/pack-bbf.sh` (needs a venv — see
  `coreone/requirements.txt`).
- H503: `coreone/h5/build-h5.sh` / `coreone/h5/build-autoboot.sh` (needs the Prusa puppy
  bootloader source + CMSIS H5).
- Flashing: `coreone/flash.sh <profile>` (SWD / DFU / BBF / RS-485). See
  `coreone/docs/OWNERS_GUIDE.md`.

Prebuilt H503 images are **not committed** (they embed a version string and would drift from
source); build them with the scripts above, or download them from the matching **GitHub Release**.

## Deployment
The Pi clones this fork at `/home/pi/klipper` and tracks it via Moonraker
`[update_manager klipper]` (`git_repo`, `primary_branch: main`). Because it's a real fork with a
clean tree, Mainsail shows it up-to-date — no more "dirty". Pi provisioning (clone + any deploy
key) lives in `coreone-host`.

## Maintaining the fork
- **Changes to the port** go here (edit `main`, commit, push). The Pi picks them up via
  `update_manager`. Do **not** re-introduce an overlay/apply-script — that's the divergence this
  fork was created to end.
- **Upgrading to newer upstream Klipper** is a normal `git rebase` onto the new tag, then
  rebuild + reflash both MCUs from the new base (host↔MCU versions must agree), then let the Pi
  pull. Sequence the reflash and the Pi pull in one printer-idle window to avoid version skew.
