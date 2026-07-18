# H503 (xBuddy extension) — inline DFU flashing ritual

> **Advanced / developer doc.** For normal flashing + restore see
> [OWNERS_GUIDE.md](OWNERS_GUIDE.md) and `klipper-port/flash.sh`. (Software-DFU on the
> F427 is a hardware dead-end; the owner-guide H503 paths are SWD + RS-485.)

Flash the extension's STM32H503 over USB-DFU **without removing it from the printer
and without SWD**. Requires Klipper on the F427 (it controls the extension's power
rail, PG2). Verified pin/power facts come from MK4-xBuddyExtension rev06
(docs/ref/, sheets 1/3/4/8).

Key hardware facts:
- The extension is powered ONLY from the main board: J5 `PWR_VIN` 24V → TPS1663
  eFuse → AP63200 buck → 5V → TLV740 LDO → 3V3. **USB-C J13 cannot power it**
  (the USB sheet only *sources* VBUS out via an EN/OC switch = the `usb_power` reg).
- `TP1` = BOOT0 with a 10k pulldown (R5). BOOT0 is sampled **only at reset/power-on**.
- `J9 pad 1` = VTref = the 3V3 net (handy strap target right next to TP1).
- Green LED **D16** = the 3V3 rail indicator (your power/discharge gauge).
- The ROM bootloader lives in system memory (region base 0x0BF80000; Kconfig
  default `STM32_DFU_ROM_ADDRESS=0x0bf87000` — verify vs AN2606 on first use).

## Phase 0 — prep
1. F427 runs Klipper, klippy ready (the ritual depends on PG2 control).
2. Build the H503 image: `cd coreone/h5 && ./build-h5.sh`;
   `cp ../../out/klipper.bin /tmp/klipper-h503.bin`.
   NB: the in-tree build writes the H5 `.config` at the fork root — rebuild the F427
   target with `coreone/build.sh` (its own `.config`) before flashing the main board.
3. Add to the live printer.cfg + FIRMWARE_RESTART:
   ```ini
   [output_pin ext_pwr]    # xBuddy extension power rail (PG2)
   pin: PG2
   value: 0
   ```
4. `dfu-util` on the desktop (and ideally STM32CubeProgrammer for option bytes).
5. Locate TP1, J9 pad 1, J13 USB-C. Plug the USB cable into J13 (harmless anytime —
   the board takes no power from it).

## Phase 1 — power sanity (~10 s)
- `SET_PIN PIN=ext_pwr VALUE=1` → green LED D16 lights (fans may hum — normal for
  an unmanaged extension). Then `SET_PIN PIN=ext_pwr VALUE=0`.

## Phase 2 — enter DFU (the only timed part)
1. Extension OFF. **Wait until D16 is fully dark, then +3 s** (~5 s total) — the
   rails must truly discharge or the next power-up isn't a real power-on reset.
2. **Strap TP1 ↔ J9-pad-1** with a clip/wire (the 3V3 net is dead now — correct;
   BOOT0 rises with the rail). Must hold solid for the next ~5 s.
3. `SET_PIN PIN=ext_pwr VALUE=1` → H503 samples BOOT0=1 → ROM bootloader (<100 ms).
4. **Wait 2 s → remove the strap** (only sampled at reset; later resets boot flash).
5. From now until Phase 7: **do NOT FIRMWARE_RESTART / power-cycle the printer** —
   a klippy restart resets PG2=0 and kills the H503 mid-session.

## Phase 3 — verify
`dfu-util -l` → `Found DFU: [0483:df11]` with `@Internal Flash` + `@Option Bytes` alts.

## Phase 4 — gates: READ THE OPTION BYTES FIRST
`STM32_Programmer_CLI -c port=usb1 -ob displ`
(or `dfu-util -a <OB-alt#> -U ob.bin && xxd ob.bin`)
- **RDP**: 0xAA/level-0 → dump allowed. Level 1 → NO dump (DFU or SWD). Dump-less
  is survivable: only **Prusa-Bootloader-Puppy** (open source,
  github.com/prusa3d/Prusa-Bootloader-Puppy) needs restoring — Buddy re-bootstraps
  the puppy APP from the main BBF automatically.
- **TZEN**: must be disabled for our flat image. Enabled → STOP, plan TZ-disable.

## Phase 5 — dump (~30 s, if RDP0)
```
dfu-util -a 0 -s 0x08000000:131072 -U h503_stock_full.bin
xxd h503_stock_full.bin | head -2   # word0 ~ 20xxxxxx (SP), word1 ~ 0800xxxx (reset)
cp h503_stock_full.bin ~/git/backup/
```

## Phase 6 — flash Klipper (~10 s)
```
dfu-util -a 0 -s 0x08000000:leave -D /tmp/klipper-h503.bin
```

## Phase 7 — first boot
`:leave` boots it directly (strap already off). Else: `VALUE=0` → dark+3 s → `VALUE=1`.
`lsusb | grep 1d50` → **1d50:614e**. If no enum: flip USBSEL to HSI48 (0), check CRS
SYNCSRC = USB-SOF, TrustZone state — then it's TC2030/SWD debugging time.

## Phase 8 — updates forever after (zero touches)
`make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32h503xx_*`
(Klipper sends bootloader_request → ROM-DFU → dfu-util → reboot.)
Bricked image? The TP1 ritual always works.

## Troubleshooting: no 0483:df11
1. Rails not fully discharged before power-on (D16 dark +3 s!).
2. Strap not solid through the power-on instant.
3. H503 held in reset by PG8's float state: add
   `[output_pin ext_reset] pin: PG8, value: 0` (BOM>=37: low=run);
   still dead → try value: 1 (older BOM inverted polarity).
4. Charge-only USB cable, or the cable isn't actually in J13 (confirm which
   physical USB-C the extension's PA11/PA12 reach).
5. Worst case: bench-power 5V at TP23 and repeat outside the printer.
