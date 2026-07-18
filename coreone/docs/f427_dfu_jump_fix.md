# STM32F4 software-USB-DFU: the jump-fix, and why the Core One still can't use it

*Investigation 2026-06-07, Prusa Core One xBuddy (STM32F427). Status: jump bug root-caused +
fixed; F427 USB-DFU nonetheless blocked on this board by a hardware (VBUS routing) limitation.
This documents the finding only — there is no upstream PR.*

## TL;DR

- Klipper's "reboot into the STM32 ROM DFU bootloader" path (`src/stm32/dfu_reboot.c`) **does not
  work on the STM32F427**: a 1200-baud touch (or SWD boot-flag) reboots the chip but it boots the
  app instead of the DFU bootloader.
- The bug is the **jump itself**, not the flag. The bare `mov sp / bx` into the ROM **faults** on
  the F4. Fix: before the jump, **`__disable_irq()` + enable SYSCFG clock + `SYSCFG->MEMRMP = 1`**
  (remap system memory to `0x0`). With that the F427 *enters the ROM*. This is a real, generic F4
  fix worth upstreaming for boards that wire VBUS to PA9.
- **But the Core One still can't DFU over USB**: its USB-C VBUS is sensed by an **FUSB302B** PD
  controller, not by the F427's **PA9** — which is the pin the F4 OTG_FS ROM bootloader monitors to
  detect "USB plugged in". The ROM therefore enters, sees no VBUS, and never enumerates. Driving
  PA9 high in software to fake VBUS does **not** take (the ROM reconfigures the pin).
- This also kills the **BOOT0-strap** route (same ROM, same dead PA9). So **F427-over-USB DFU is a
  hardware dead-end on the Core One.** The F427 stays on SWD; the **H503 DFU** is the no-tools path
  (see `docs/h503_dfu_flashing.md`).

## Background

Goal: let users flash the F427 Klipper port with no SWD probe and no BOOT0 strap — just the USB
cable, via the standard `make flash` / 1200-baud-touch path. Klipper's mechanism:

1. Host opens the USB CDC at **1200 baud with DTR deasserted** (`src/generic/usb_cdc.c:456`).
2. Firmware calls `bootloader_request()` → `dfu_reboot()`: writes a flag to RAM
   (`USB_BOOT_FLAG = 0x55534220424f4f54`, "USB BOOT", at `RAM_START+RAM_SIZE-1024` = `0x2001FC00`
   on the F427) and `NVIC_SystemReset()`.
3. On reboot, `dfu_reboot_check()` (first line of `armcm_main` in `src/stm32/stm32f4.c`) re-reads
   the flag; if set it clears it and jumps to the ROM bootloader at
   `CONFIG_STM32_DFU_ROM_ADDRESS = 0x1fff0000`.

Observed: step 1-2 reboot the F427, but it boots Klipper again — never DFU.

## Root-cause chain

Debugging this on the F427 is hard: the only SWD probe is a flaky ST-Link clone that trains only
at `mode=UR freq=400` and intermittently. So the key technique was **hang-on-condition
instrumentation** — patch `dfu_reboot_check` to spin (`for(;;)`) under a chosen condition, then
read the outcome from the **Pi/klippy side** (does the F427 stay on USB as Klipper, or vanish?).
That sidesteps the flaky probe entirely for the *result*; the probe is only needed for the reflash.

Layers, in the order they were eliminated:

1. **Flag survival + CPU read — FINE.** A distinctive marker SWD-written to `0x2001FC00`, then a
   warm reset: `dfu_reboot_check` matched it and the F427 hung. So SRAM is retained across the
   reset and the CPU reads it correctly at early boot. (An earlier detour relocating the flag to
   **CCM-RAM** `0x10000000` was a *red herring* — the CCM is readable by the SWD debug-AP but the
   **CPU** can't read it at that early-boot point, so it made things worse, not better. Plain SRAM
   was always correct.)

2. **The JUMP is the bug.** With the real flag set, `dfu_reboot_check` matches + clears it, then
   the bare
   ```c
   asm volatile("mov sp, %0\n bx %1" : : "r"(sysbase[0]), "r"(sysbase[1]));
   ```
   **faults** → the chip resets → boots the app (flag already cleared → no retry). The ROM target
   is valid (read `0x1fff0000` = SP `0x20002D40`, a sane RAM address; the ROM is intact). So it is
   purely a *bad entry state* fault, the classic STM32F4 "software jump to system bootloader"
   gotcha.

## The fix

Give the ROM a clean entry: mask interrupts and remap system memory to `0x00000000` before the
jump. In `dfu_reboot_check()`, right before the `mov sp / bx`:

```c
    uint32_t *sysbase = (uint32_t*)CONFIG_STM32_DFU_ROM_ADDRESS;
#if CONFIG_MACH_STM32F4
    // The bare mov-sp/bx into the F4 system bootloader faults on some boards (e.g.
    // STM32F427); the ROM needs a clean entry: IRQs masked + system memory at 0x0.
    __disable_irq();
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->MEMRMP = 0x1;
#endif
    asm volatile("mov sp, %0\n bx %1"
                 : : "r"(sysbase[0]), "r"(sysbase[1]));
```

**Verified:** with this, a flag-triggered reset no longer boots the app — the F427 leaves the USB
bus entirely (it is now executing in the ROM bootloader). That is the fix proven, end-to-end.

This is generic and harmless for any F4 that reaches the ROM via this path; it would let
software-USB-DFU work on F4 boards that route VBUS to PA9 — hence "upstreamable", but see the
deploy caveat below.

## Why the Core One still can't DFU (the hardware wall)

Entering the ROM is necessary but not sufficient: the ROM's **USB DFU never enumerates** on the
Core One. The OTG_FS ROM bootloader senses **VBUS on PA9** to decide a host is connected. On the
Core One the USB-C VBUS is handled by an **FUSB302B** USB-C PD controller over I²C
(`src/buddy/usb_device.cpp`, `usb_vbus_state`), and **PA9 is not driven** by the connector. So the
ROM sees "no VBUS" and stays silent — the chip is in the ROM but invisible on USB ("STUCK").

Attempted workaround: drive **PA9 high in software** before the jump (output, and output+pull-up)
to fake VBUS-present. **It does not take** — the ROM reconfigures PA9 when it sets up VBUS sensing,
discarding the GPIO config. There is no software path to assert VBUS on a pin the ROM owns.

Two consequences:
- The **BOOT0 strap is equally dead** — BOOT0-high+reset reaches the *same* ROM with the *same*
  un-driven PA9. So there is no hardware-strap escape hatch either.
- **F427 USB-DFU is a hardware/ROM dead-end on this board.** The only fixes are PCB-level (route
  USB VBUS to PA9), which is out of scope.

## Deploy caveat

**Do not ship the jump-fix in the Core One's deployed firmware.** Without working USB it converts a
DFU attempt from a harmless "boots Klipper" into a "STUCK in ROM, no USB" that needs an SWD reset
to recover. It was reverted from the flashed firmware here; it lives only in this writeup (and the
[[f427-usb-power-and-swd]] memory) for a future F4 upstream contribution.

## What to use instead

- **F427:** flash over **SWD** (`klipper-port/flash-noboot.sh`; the probe is marginal, retry the
  `mode=UR freq=400` connect — it trains first-try once the contact lands).
- **H503 (extension):** **USB-DFU works** and is the genuine no-tools path —
  `docs/h503_dfu_flashing.md` (boot-flag at `0x20007C00` + `dfu-util`).

## References

- `src/stm32/dfu_reboot.c` — `dfu_reboot()` / `dfu_reboot_check()`; flag `0x55534220424f4f54`,
  addr `RAM_END-1024` (`0x2001FC00` on F427), ROM `0x1fff0000`.
- `src/stm32/stm32f4.c:armcm_main` — calls `dfu_reboot_check()` first (before `SystemInit`).
- `src/generic/usb_cdc.c:456` — 1200-baud-touch trigger (`dwDTERate==1200 && !DTR`).
- Prusa: `src/buddy/usb_device.cpp` (FUSB302B VBUS), `src/hw/FUSB302B.*`.
- STM32F4 OTG_FS VBUS sense pin = PA9; ROM monitors it via `OTG_FS GCCFG.VBUSBSEN`.
