#!/usr/bin/env bash
# check-rdp.sh — READ-ONLY safety gate for any SWD/DFU write to the H503.
#
# The H503 carries an IRREPLACEABLE Prusa puppy bootloader. On STM32H5, writing the
# flash while the device is NOT in the "Open" product state triggers a regression /
# mass-erase that destroys that bootloader with no recovery. This confirms the product
# state is Open (0xED) BEFORE any write. It NEVER writes anything (only `mdw`).
#
#   exit 0 = Open (0xED), safe to flash
#   exit 1 = NOT Open, or unreadable -> ABORT, do not flash
#   exit 2 = no probe / openocd missing -> ABORT (fail closed)
#
# Product state lives in FLASH_OPTSR_CUR (0x40022050), bits[15:8]. Verified on a
# pristine factory + an updated H503: both 0xED (Open). Uses the H5-capable ST-fork
# OpenOCD + a genuine STLINK-V3 (same toolchain as swd-flash.sh) — no STM32CubeProgrammer
# needed. (An intact F427 appendix is irrelevant here; the H503 ships Open.)
set -euo pipefail

OPTSR_ADDR=0x40022050   # FLASH_OPTSR_CUR; PRODUCT_STATE = bits[15:8]
OPEN_STATE=0xED

command -v openocd >/dev/null 2>&1 || { echo "check-rdp: openocd not found in PATH" >&2; exit 2; }
openocd --version 2>&1 | grep -qiE 'STMicroelectronics|stcubeide' || {
  echo "check-rdp: needs the STM32CubeIDE build of OpenOCD (ST fork with STM32H5 support);" >&2
  echo "  mainline OpenOCD has no stm32h5x target. ABORT." >&2; exit 2; }
serial="$(for d in /sys/bus/usb/devices/*; do
  [ "$(cat "$d/idVendor" 2>/dev/null)" = 0483 ] && [ "$(cat "$d/idProduct" 2>/dev/null)" = 3754 ] \
    && { cat "$d/serial" 2>/dev/null; break; }; done)"
[ -n "$serial" ] || { echo "check-rdp: no genuine STLINK-V3 (0483:3754) on USB" >&2; exit 2; }

echo "check-rdp: reading H503 product state via OpenOCD + STLINK-V3 $serial (read-only)..."
# Must `halt` before the read: against a RUNNING app (e.g. the H503 Klipper) a bare `mdw` of
# FLASH_OPTSR_CUR returns 0x00000000 (looks falsely NOT-Open); halting first yields the true
# value. `reset run` leaves the chip running afterwards (this gate writes nothing).
out="$(timeout 25 openocd -f interface/stlink-dap.cfg -c "adapter serial $serial" \
  -f target/stm32h5x.cfg -c init -c halt -c "mdw $OPTSR_ADDR 1" -c "reset run" -c shutdown 2>&1 || true)"

optsr="$(printf '%s\n' "$out" \
  | grep -oiE "${OPTSR_ADDR}: *[0-9a-f]{8}" | grep -oiE '[0-9a-f]{8}$' | head -1)"
if [ -z "$optsr" ]; then
  echo "check-rdp: could NOT read FLASH_OPTSR_CUR -- ABORT (never write blind)." >&2
  printf '%s\n' "$out" | grep -iE 'voltage|error|too low|unable' | tail -4 >&2
  exit 1
fi

ps=$(( (0x$optsr >> 8) & 0xFF ))
printf 'check-rdp: FLASH_OPTSR_CUR=0x%s  PRODUCT_STATE=0x%02X\n' "$optsr" "$ps"
if [ "$ps" -eq $((OPEN_STATE)) ]; then
  echo "  -> Open (0x${OPEN_STATE#0x}) -- safe to write the H503."
  exit 0
fi
echo "  -> NOT Open (0x${OPEN_STATE#0x}) -- a write would MASS-ERASE the puppy bootloader. ABORT." >&2
exit 1
