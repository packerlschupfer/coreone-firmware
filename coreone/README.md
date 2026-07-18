# coreone-firmware — open firmware for the Prusa Core One+ (a two-MCU Klipper port)

This overlay runs **Klipper** on the Prusa Core One+ instead of stock Buddy firmware.
The Core One+ is a **two-MCU machine**, and this port targets both:

> **Flashing / configurations / restore-to-stock → [docs/OWNERS_GUIDE.md](docs/OWNERS_GUIDE.md)** (driven by `flash.sh <profile>`).

```
            ┌─ USB ──────► xBuddy main board   (STM32F427ZI)  ── motion, heaters,
   host     │               "Klipper MCU"          TMC2130/SPI, loadcell probe,
 (klippy) ──┤                                       PART-COOLING + HOTEND fans
            └─ USB-C ────► xBuddy extension     (STM32H503CBU7) ── ENCLOSURE fans
                            "Klipper MCU #2"         (chamber+filtration), RGBW LED,
                                                     chamber temp, filament sensor
```
Note the fan split: the **toolhead** fans (part-cooling + hotend heatbreak) are on the
**F427 main board** (PE11/PE9) — so cooling works without the H503. The extension only
adds the **enclosure** fans (chamber circulation + carbon filtration).

Klipper is split: a thin **firmware** on each MCU + the **klippy** brain on a host
computer (a PC, Pi, or VM running Moonraker/Fluidd). The host drives both MCUs over
USB — there is **no RS-485 between them** in this design (the H503 talks to the host
directly over its own USB-C, sidestepping Prusa's PuppyBus entirely).

> This is a **self-contained fork** of Klipper: the full Klipper tree is vendored in this
> repo (the fork root), with every Core One change committed on top of the upstream base
> recorded in `KLIPPER_VERSION`. Build directly **in-tree** — no separate clone or overlay
> step. All Core One tooling, board configs, `printer.cfg`, and docs live under `coreone/`.

## Status

Both MCUs are flashed and **hardware-validated on a real Core One+**.

| | xBuddy main (F427) | xBuddy extension (H503) |
|---|---|---|
| Klipper target | `MACH_STM32F427` (F429 sibling) | **`MACH_STM32H5`** (from-scratch family port) |
| Host link | USB-CDC | USB-C (front port, J13) |
| Flash method | SWD @ `0x08000000`, or coexist BBF via the Prusa bootloader @ `0x08020200` | SWD, USB-DFU, or RS-485 PuppyBus reflash |
| **Validated** | CoreXY motion, StallGuard/sensorless homing, loadcell Z-probe + bed mesh, bed+hotend PID, extruder, ILI9488 display (SPI6 DMA), input shaper, phase-stepping (XDIRECT) + cogging, filament sensor | chamber thermistor + chamber/filtration fans, RGBW + white LEDs, filament spool sensor — all live over USB-C |

The F427 runs either a self-contained no-bootloader build or a "coexist" build that
chainloads from the restored Prusa bootloader (BBF over a USB stick). The H503 runs the
enclosure subsystem and is re-flashable without an ST-Link (USB-DFU or over RS-485).

## Layout

| Path | Purpose |
|---|---|
| `KLIPPER_VERSION` | upstream Klipper base SHA this fork branched from (provenance) |
| `build.sh [noboot\|coexist]`, `flash-noboot.sh` | F427 in-tree build + SWD flash @ `0x08000000` |
| `boards/xbuddy.cfg` | **F427 hardware config** (motion, TMC, heaters, loadcell probe, homing) |
| `printer.cfg` | **consolidated F427 config** = include board cfg + bed_mesh + macros |
| `test/*.cfg` | focused F427 bring-up configs (homing, motion, tmc, …) + `dump_tmc.py` |
| `h5/build-h5.sh`, `h5/klipper-h5.patch`, `h5/stm32h5.c`, `h5/.config-h503` | **H503 port** (build + patch + clock) |
| `h5/extension.cfg`, `h5/tca6408a.py` | H503 board config (2nd MCU) + I2C fan-enable module |
| `out/` | built artifacts |
| `identify.py` | host-side klippy `IDENTIFY` check |

## Build & flash — F427 main board
```
./build.sh noboot             # in-tree build -> coreone/out/klipper-0x08000000.bin (vectors @ 0x08000000)
./flash-noboot.sh             # SWD: one write + SRAM clear + reset
./identify.py                 # confirm USB-CDC IDENTIFY
```
Then run klippy with `printer.cfg` (fix the `[mcu]` serial path in `boards/xbuddy.cfg`).
Bring-up beyond boot: see `../docs/klipper_phase2_progress.md` and, for the probe,
`../docs/klipper_loadcell_probe.md`.

## Build & flash — H503 extension board
```
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_h5.git ~/git/cmsis_device_h5
h5/build-h5.sh                # vendors CMSIS, applies klipper-h5.patch, builds out/klipper.bin
```
Flashing is **SWD only**, on the extension's `J9` Tag-Connect / `TP2`(NRST) `TP3`(SWCLK)
`TP4`(SWDIO) pads (chip needs power — easiest with the F427 running our Klipper +
`PUPPY_PRECHARGE`, which powers the extension without managing it). **First SWD action:
read RDP + back up the H503 flash** (`~/git/backup/h503_ext_stock.bin`) — its puppy
bootloader is irreplaceable. Then flash + connect the front USB-C. Plan + pin map:
`../docs/klipper_h5_port_plan.md`, `../docs/klipper_h5_pinmap.md`.

> The H503 can also be reflashed **over the F427** via the RS-485 PuppyBus bootloader
> (122 KB upload + salted-fingerprint validate + boot) — no ST-Link needed. USB (SWD/DFU)
> is the default dev path; RS-485 is the in-place field-reflash route.

## Hard rules
- **SWD is the standard dev flash method.** F427 = no-bootloader build at `0x08000000`.
- **One write, then wait** for the board to confirm-boot. No back-to-back writes.
- **Never mass-erase.** F427 recovery to stock = SWD-restore `~/git/backup/full_flash_stock.bin`.
- H503: **back up before any write** (no backup exists yet; check RDP first).

## Docs index (`../docs/`)
`klipper_build.md`, `klipper_pinmap.md`, `klipper_phase1_result.md`,
`klipper_phase2_progress.md`, `klipper_flash_procedure.md`,
**`klipper_f427_bringup.md`** (ordered first-flash runbook),
`klipper_loadcell_probe.md` (F427 probe) · `klipper_h5_port_plan.md`,
`klipper_h5_pinmap.md` (H503) · `klipper_puppybus.md` (abandoned RS-485 approach) ·
**`codegraph.md`** (structural code-search indexes for both trees) ·
**`klipper_limitations.md`** (what Klipper can't replicate vs Prusa + TODOs) ·
`ref/MK4-xBuddyExtension-06.pdf` (extension schematic).
</content>
