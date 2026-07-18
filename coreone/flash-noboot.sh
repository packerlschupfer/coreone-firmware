#!/usr/bin/env bash
# Phase-1 SWD flash: self-contained Klipper at 0x08000000 (NO Prusa bootloader).
# ONE write, then RAM-state clear, then a single software reset. Do NOT run this
# twice back-to-back (chip-mid-reset trap) — wait for the printer to confirm-boot.
#
# Recovery if Klipper doesn't come up (bootloader is already absent, so SWD only):
#   st-flash --connect-under-reset --reset write ~/git/backup/full_flash_stock.bin 0x08000000
#   (restores bootloader+stock fw), then BBF-install a known-good image.
#
# Writes ~43 KiB => erases flash sectors 0-2 only. Never mass-erases. The old
# firmware in higher sectors becomes dead code (Klipper's reset vector wins).
set -euo pipefail

BIN="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/out/klipper-0x08000000.bin}"
echo "Flashing: $BIN -> 0x08000000"
[ -f "$BIN" ] || { echo "missing $BIN"; exit 1; }

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "init" \
  -c "reset halt" \
  -c "flash write_image erase \"$BIN\" 0x08000000" \
  -c "verify_image \"$BIN\" 0x08000000" \
  -c "mwb 0x20000000 0x00; mwb 0x20000001 0x01; mwb 0x20000002 0x00; mwb 0x20000003 0x01" \
  -c "mmw 0x40023830 0x00040000 0; mmw 0x40007000 0x00000100 0" \
  -c "mww 0x40024000 0x00000000; mww 0x40024004 0x00000000; mww 0x40024008 0x00000000; mww 0x4002400C 0x00000000" \
  -c "mww 0xE000ED0C 0x05FA0004" \
  -c "shutdown"

echo "Done. Wait for the printer to come up, then verify on the host:"
echo "  ls /dev/serial/by-id/usb-Klipper_stm32f427xx_*-if00"
echo "  ~/git/klipper/klippy/console.py /dev/serial/by-id/usb-Klipper_stm32f427xx_*-if00"
