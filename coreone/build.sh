#!/usr/bin/env bash
# Reproducibly build a Klipper variant for the xBuddy (STM32F427) from this
# self-contained fork (in-tree). Verifies the vector base afterwards.
#
# Usage: ./build.sh [noboot|coexist]
#   noboot  (default) -> vectors @ 0x08000000, self-contained (no Prusa bootloader)
#                        -> out artifact: out/klipper-0x08000000.bin
#   coexist           -> vectors @ 0x08020200, needs Prusa bootloader at 0x08000000
#                        -> out artifact: out/klipper.bin   (+ pack-bbf.sh for BBF)
set -euo pipefail

VARIANT="${1:-noboot}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUDDY="$(cd "$HERE/.." && pwd)"          # fork root = the Klipper tree itself
KLIPPER_DIR="${KLIPPER_DIR:-$BUDDY}"     # build in-tree (override to build elsewhere)
TOOLCHAIN="$BUDDY/.dependencies/gcc-arm-none-eabi-13.3.1/bin"

case "$VARIANT" in
  noboot)  CFG="$HERE/.config-noboot"; WANT_BASE="08000000"; ART="klipper-0x08000000.bin" ;;
  coexist) CFG="$HERE/.config";        WANT_BASE="08020200"; ART="klipper.bin" ;;
  *) echo "unknown variant '$VARIANT' (use: noboot | coexist)"; exit 1 ;;
esac
[ -f "$CFG" ] || { echo "missing config $CFG"; exit 1; }
[ -d "$KLIPPER_DIR/.git" ] || { echo "KLIPPER_DIR=$KLIPPER_DIR is not a git checkout"; exit 1; }

# Build in-tree: this fork already contains every Core One source file.
[ -d "$TOOLCHAIN" ] && export PATH="$TOOLCHAIN:$PATH"

cd "$KLIPPER_DIR"
cp "$CFG" .config
make olddefconfig >/dev/null
make clean >/dev/null 2>&1 || true
make -j"$(nproc)"

# Verify the vector table landed where the chosen variant expects.
BASE="$(arm-none-eabi-objdump -h out/klipper.elf | awk '/\.text/{print $4; exit}')"
echo "built $VARIANT: .text base = 0x$BASE (want 0x$WANT_BASE)"
[ "$BASE" = "$WANT_BASE" ] || { echo "VECTOR BASE MISMATCH"; exit 1; }

mkdir -p "$HERE/out"
cp out/klipper.bin "$HERE/out/$ART"
echo "OK -> coreone/out/$ART  ($(stat -c%s out/klipper.bin) bytes)"
