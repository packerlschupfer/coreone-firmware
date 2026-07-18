# Prusa Core One+ — Klipper firmware: owner's guide

Flashing, configurations, and returning to stock for the Core One+ Klipper port.
There are two **independent** MCUs: the **F427** "main" board and the **H503**
"extension" board. You pick one image for each; any F427 image works with any H503 image.

> Everything here is driven by one script: **`klipper-port/flash.sh <profile>`**.
> Run it with no arguments to list profiles. It prints what each profile does / needs /
> risks, and asks you to type `yes` before any write.

> ⚠️ **F427 prerequisite — break the appendix seal first.** On a factory Core One+ the
> F427 only accepts signed Prusa firmware **and its SWD is blocked**, because the
> "appendix" (a breakaway PCB tab on the xBuddy, tied to **PA13 / SWDIO**) is intact —
> while attached it loads the SWD line, so an ST-Link can't connect. **Snap it off** to
> enable *both* unsigned firmware and SWD; this is required for **every** F427 path below
> (USB-BBF and SWD). One-way mod — follow Prusa's guide:
> <https://help.prusa3d.com/article/zoiw36imrs-flashing-custom-firmware>. The H503 needs
> no such step (it ships product-state Open).

> 🔧 **SWD tooling.** Any SWD path needs a **genuine STLINK-V3** *and* the **STM32CubeIDE build of
> OpenOCD** (`0.12.0+dev.stcubeide…`, the STMicroelectronics fork — `apt` package `openocd` from ST,
> or STM32CubeIDE's bundled copy). **Mainline OpenOCD has no `stm32h5x` target**, so the H503 will
> not connect with it. And on the **F427** the SWD/JTAG debug header (**J21**) is **unpopulated from
> the factory** — you must solder a 1.27 mm 2×5 Cortex-debug connector (e.g. **Samtec
> FTSH-105-01-L-DV-K-A-P-TR**) to it to attach the probe. The no-tools paths (F427 USB-BBF, H503
> RS-485) avoid SWD entirely.

---

## 1. The configurations

### F427 "main" — choose by boot mode
| Image | What it is | How you flash it | Recovery |
|---|---|---|---|
| **coexist** | Klipper behind the stock Prusa bootloader (@0x08020200) | USB stick (no tools) **or** SWD | USB-BBF — easy |
| **noboot** | Self-contained Klipper (@0x08000000), no Prusa bootloader | SWD only | SWD only |

The RS-485 master is compiled into **both** F427 images but stays inert unless
`[puppybus]` is in your `printer.cfg`. So "USB extension" vs "RS-485 extension" is a
**config** choice, not a different firmware.

### H503 "extension" — all run the enclosure over USB; choose by how you flash + update it
| Image | What it is | Flash / update | Runtime |
|---|---|---|---|
| **autoboot** *(recommended)* | the **auto-chainload** puppy bootloader (@0x08000000) + klipper-puppy app (@0x08002000) | **SWD once** (combined image), then **RS-485, no ST-Link** for every update (`PUPPY_REFLASH`) | `[mcu extension]` over USB, **auto-boots** — set-and-forget *and* no-ST-Link updates |
| **usb** | self-contained klipper-puppy (@0x08000000), no bootloader — auto-boots | **SWD every time** | `[mcu extension]` over USB — set-and-forget, but SWD-only updates |
| **rs485** | klipper-puppy behind the *stock* puppy bootloader (@0x08002000) | **RS-485, no ST-Link** onto the factory bootloader | `[mcu extension]` over USB, **but waits for `PUPPY_START` each boot** (no auto-boot) |

All three run the enclosure (chamber fan/temp, RGBW/white LEDs, spool sensor) as a Klipper
`[mcu extension]` over the H503's own USB. **autoboot** is the best of both — one SWD flash of the
modified bootloader, then it auto-boots like noboot AND updates over RS-485 like rs485. The modified
bootloader auto-jumps to a descriptor-valid app after ~3 s of bus idle (**proven on hardware**:
auto-boot + a full RS-485 `PUPPY_REFLASH` round-trip). A runtime-over-RS-485 bridge (which would drop
the H503's USB cable) was evaluated and **deferred**: Klipper's clock-sync fits a polled half-duplex
link poorly, for a one-cable gain. RS-485 stays the **flash/update** path.

---

## 2. Which do I want?

- **Most people:** `f427-coexist-bbf` (USB stick) + `h503-autoboot-swd`. The F427 is no-tools; the
  H503 needs an ST-Link **once** to write the auto-chainload bootloader, after which it auto-boots
  the enclosure over USB AND updates over RS-485 with no ST-Link (`PUPPY_REFLASH`). (The F427's
  **appendix seal must be broken**; both MCUs run over USB, so the H503 needs its USB cable.)
- **No custom bootloader (simplest layout):** `h503-usb` (noboot) — auto-boots too, but every
  firmware update is an SWD flash.
- **Stock bootloader only:** `h503-rs485` — no-tools `PUPPY_FLASH` onto the factory bootloader,
  but it waits for `PUPPY_START` each boot (no auto-boot).

**Updating the H503 over RS-485 (autoboot image, no ST-Link):** copy `h5/puppybus-flash.cfg` over
`printer.cfg`, `FIRMWARE_RESTART` (drops PG2 → H503 powers off), then `PUPPY_REFLASH
FILE=/tmp/klipper-h503-puppy.bin` (it powers the H503 up, catches the bootloader in its idle window,
streams the app, and the H503 auto-chainloads), then restore your normal `printer.cfg`.

---

## 3. Flashing (no-tools paths first)

**F427 coexist via USB stick (recommended)** — `flash.sh f427-coexist-bbf`
Builds + packs `out/klipper.bbf`. Copy it to a FAT32 stick as `firmware.bbf`, insert,
press reset, accept the "custom firmware / verification failed" prompt (unsigned is
expected). No tools.

**F427 via SWD** — `flash.sh f427-coexist-swd` or `flash.sh f427-noboot-swd`
Needs a genuine STLINK-V3 with NRST wired (running Klipper hijacks the SWD pins, so the
flasher connects under reset). coexist writes @0x08020000 (never the bootloader); noboot
writes @0x08000000.

**H503 USB via SWD** — `flash.sh h503-usb-swd`
Needs an STLINK-V3 + the **ST-fork OpenOCD** (the STMicroelectronics build — mainline has no
STM32H5 driver) on the H503 (J9 Tag-Connect / TP2-4) and the H503 powered (the F427's
PG2 rail — do not restart the F427 mid-flash). **An RDP gate runs first** (see Safety).

**H503 over RS-485 (no tools)** — `flash.sh h503-rs485`
Only if your F427 already runs Klipper with `[puppybus]`. Builds the puppy image; you then
run `PUPPY_FLASH` + `PUPPY_START` in the Klipper console — works on the factory bootloader
(verified). The H503 then runs the enclosure as an `[mcu extension]` over its own USB.

---

## 4. Return to stock Prusa firmware

Try the no-tools path first.
- **F427, no tools** — `flash.sh restore-f427-bbf`: copy the stock BBF to a stick, reset.
- **F427, SWD** — `flash.sh restore-f427-swd`: full restore (bootloader + firmware).
- **H503, SWD** — `flash.sh restore-h503-swd`: the only H503 restore (RDP gate first).

---

## 5. If something goes wrong
- No Klipper flash ever touches the Prusa bootloader, so the **USB-BBF restore almost
  always works** — try that first.
- Use a full SWD restore only if the flash is genuinely corrupted.
- **H503: if its product state is locked, there is NO recovery** from a bad write — which
  is exactly why the RDP gate refuses to flash a non-Open H503.

---

## Safety (the tooling enforces these — don't bypass them)
- **Never mass-erase** either MCU — both have closed, irreplaceable bootloaders.
- F427 coexist needs a valid 512-byte descriptor; the tooling builds it (a raw image to
  0x08020200 is rejected by the bootloader as error #31608).
- **One flash, then confirm it boots** before any second flash (back-to-back SWD writes
  trap the chip mid-reset; only a power-cycle recovers).
- **The H503 product state must be Open (0xED)** or an SWD write mass-erases the puppy
  bootloader forever. `h5/check-rdp.sh` gates this (reads it via OpenOCD + an STLINK-V3).
- The H503 is powered only by the F427's PG2 rail — don't reset the F427 mid-H503-flash.

---

## Verification status — both former unknowns now RESOLVED on hardware
1. **H503 product state — RESOLVED: ships Open.** Confirmed `PRODUCT_STATE 0xED` on two units (a
   pristine factory board via SWD + an updated one via DFU), so stock H503s ship Open and the SWD/DFU
   paths work. `check-rdp.sh` still gates each board in case a unit ever differs.
2. **Stock puppy bootloader accepts our UNSIGNED app over RS-485 — CONFIRMED.** Tested on a pristine
   factory **v297** bootloader: the full 122880 B `PUPPY_FLASH` + the salted-fingerprint `PUPPY_START`
   were both accepted (`fingerprint_match=1`, then it jumped to the app). So the RS-485 H503 path is
   **no-tools for every owner** — no v302 swap required.
