#!/usr/bin/env bash
# swd-flash.sh — flash a firmware image to the Core One's F427 (main) or H503
# (xBuddy extension) over SWD, using the H5-capable ST-fork OpenOCD + a genuine
# STLINK-V3. Auto-detects the V3 probe (USB 0483:3754).
#
# Usage:
#   swd-flash.sh stock f427          # restore Prusa stock F427 (full_flash_stock.bin)
#   swd-flash.sh stock h503          # restore Prusa stock H503 (h503_stock_full.bin)
#   swd-flash.sh f427  <image.bin>   # flash any image to the F427 @0x08000000
#   swd-flash.sh h503  <image.bin>   # flash any image to the H503 @0x08000000
#
# Notes / safety:
#  - The F427 runs Klipper which HIJACKS its SWD pins, so we connect-under-reset
#    (NRST wired to probe) + `reset halt` NOT plain `halt` (a plain halt of running
#    Klipper times out; reset-halt catches it at the reset vector). H503 = plain halt.
#  - `program ... verify` does a sector-TARGETED erase+write+verify — NOT a
#    mass-erase. (Never use st-flash erase / mass_erase: it wipes the restore target.)
#  - Move the single V3 between the F427 and H503 SWD between runs.
set -euo pipefail
BACKUP="${BACKUP:-$HOME/git/backup}"

detect_v3() {  # genuine STLINK-V3MINIE = 0483:3754
  for d in /sys/bus/usb/devices/*; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "0483" ] && \
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "3754" ] && { cat "$d/serial" 2>/dev/null; return; }
  done
}

mode="${1:?usage: swd-flash.sh <stock|f427|f427-coexist|h503> ...}"
sha=""; addr=0x08000000
if [ "$mode" = stock ]; then
  case "${2:?usage: swd-flash.sh stock <f427|h503>}" in
    f427) target=f427; img="$BACKUP/full_flash_stock.bin"; sha=585c7b38f383 ;;
    h503) target=h503; img="$BACKUP/h503_stock_full.bin";  sha=811d4674d418 ;;
    *) echo "usage: swd-flash.sh stock <f427|h503>"; exit 1 ;;
  esac
elif [ "$mode" = f427-coexist ]; then
  # Klipper-coexist image = [512B descriptor][firmware vectors@0x200], installed
  # BEHIND the Prusa bootloader. Writes @0x08020000 — NEVER 0x08000000.
  target=f427; img="${2:?need an image path}"; addr=0x08020000
else
  target="$mode"; img="${2:?need an image path}"
fi

case "$target" in
  f427) cfg="target/stm32f4x.cfg"; reset_cfg="reset_config srst_only srst_nogate connect_assert_srst"; halt_cmd="reset halt" ;;
  h503) cfg="target/stm32h5x.cfg"; reset_cfg=""; halt_cmd="halt" ;;
  *) echo "target must be f427 or h503"; exit 1 ;;
esac

# The H503 (stm32h5x) needs the STM32CubeIDE OpenOCD build (ST fork); mainline has no H5 target.
if [ "$target" = h503 ] && ! openocd --version 2>&1 | grep -qiE 'STMicroelectronics|stcubeide'; then
  echo "ERROR: H503 SWD needs the STM32CubeIDE build of OpenOCD (ST fork with STM32H5). ABORT." >&2
  exit 2
fi

[ -f "$img" ] || { echo "MISSING image: $img"; exit 1; }
if [ -n "$sha" ]; then
  got=$(sha256sum "$img" | cut -c1-12)
  [ "$got" = "$sha" ] || { echo "SHA MISMATCH: $img = $got (expected $sha) — ABORT"; exit 1; }
fi
# Safety: confirm a real vector table sits where this address expects it. A coexist
# descriptor image written to 0x08000000 would overwrite the irreplaceable bootloader.
voff=0; [ "$addr" = 0x08020000 ] && voff=512
python3 - "$img" "$voff" "$addr" <<'PYGUARD'
import sys, struct
img, voff, addr = sys.argv[1], int(sys.argv[2]), sys.argv[3]
d = open(img, 'rb').read()
if len(d) < voff + 8:
    sys.exit(f"SAFETY ABORT: {img} too small for a {addr} image")
msp = struct.unpack_from('<I', d, voff)[0]
if not (0x20000000 <= msp < 0x20040000):
    sys.exit(f"SAFETY ABORT: {img} stack word @+{voff} = 0x{msp:08X} is not valid RAM; "
             f"does not look like a vector table for {addr}. Wrong image/address. Refusing to flash.")
PYGUARD
serial="$(detect_v3)"; [ -n "$serial" ] || { echo "no STLINK-V3 (0483:3754) found on USB"; exit 1; }
echo ">> $target  <-  $img  ($(stat -c%s "$img") B${sha:+, sha $sha})  via STLINK-V3 $serial"

args=(-f interface/stlink-dap.cfg -c "adapter serial $serial" -f "$cfg")
[ -n "$reset_cfg" ] && args+=(-c "$reset_cfg")
args+=(-c "init" -c "$halt_cmd" -c "program $img $addr verify" -c "reset run" -c "shutdown")
openocd "${args[@]}"
