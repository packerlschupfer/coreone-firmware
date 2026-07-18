#!/usr/bin/env bash
# Reproducibly build a Klipper variant for the xBuddy (STM32F427) from this
# self-contained fork (in-tree). Verifies the vector base afterwards.
#
# Usage: ./build.sh [noboot|coexist|katapult]
#   noboot  (default) -> vectors @ 0x08000000, self-contained (no Prusa bootloader)
#                        -> out artifact: out/klipper-0x08000000.bin
#   coexist           -> vectors @ 0x08020200, needs Prusa bootloader at 0x08000000
#                        -> out artifact: out/klipper.bin   (+ pack-bbf.sh for BBF)
#   katapult          -> vectors @ 0x08008000, needs a Katapult bootloader at 0x08000000
#                        (dev machines flashed with Katapult; flash the .bin via flashtool.py)
#                        -> out artifact: out/klipper-katapult-0x08008000.bin
set -euo pipefail

VARIANT="${1:-noboot}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUDDY="$(cd "$HERE/.." && pwd)"          # fork root = the Klipper tree itself
KLIPPER_DIR="${KLIPPER_DIR:-$BUDDY}"     # build in-tree (override to build elsewhere)
TOOLCHAIN="$BUDDY/.dependencies/gcc-arm-none-eabi-13.3.1/bin"

case "$VARIANT" in
  noboot)   CFG="$HERE/.config-noboot";   WANT_BASE="08000000"; ART="klipper-0x08000000.bin" ;;
  coexist)  CFG="$HERE/.config";          WANT_BASE="08020200"; ART="klipper.bin" ;;
  katapult) CFG="$HERE/.config-katapult"; WANT_BASE="08008000"; ART="klipper-katapult-0x08008000.bin" ;;
  *) echo "unknown variant '$VARIANT' (use: noboot | coexist | katapult)"; exit 1 ;;
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

# HARD-FLOAT / CPACR ORDERING GUARD (F427 is built hard-float; see hardfloat rationale in
# src/generic/armcm_boot.c + src/stm32/Makefile). The invariant that matters is not "a CPACR
# write exists" but "the FPU is enabled BEFORE the first FP instruction executes" -- the first
# FP instruction on a reset-disabled FPU hard-faults. Compare ADDRESSES: lowest CP10/CP11
# enable (orr with 0xf00000) must precede the lowest single-precision FP instruction. A
# hard-float image has TWO enables (explicit in armcm_boot + inlined SystemInit), so matching
# "an enable exists" is not enough -- only the address ordering cannot be fooled.
# NB: nothing here may exit a pipeline early -- under `set -o pipefail` an early-exiting reader
# (grep -q / awk exit) makes objdump take SIGPIPE and the script dies 141.
# This script builds ONLY the F427 (xBuddy), which MUST be hard-float -- MCU input-shaping
# needs it. A silently soft-float image (e.g. the -mfpu/-mfloat-abi flags dropped from a stale
# Makefile) would skip an "if hard-float" guard entirely and ship wrong; its only symptom is a
# ~1.7 KB larger binary. So assert hard-float FIRST, then check the ordering. (FW/klipper 2bde139b4.)
_HF=$(arm-none-eabi-readelf -A out/klipper.elf | grep -c "Tag_ABI_VFP_args" || true)
if [ "$_HF" = "0" ]; then
  echo "FATAL: F427 image is SOFT-FLOAT -- the -mfpu/-mfloat-abi flags were dropped." >&2
  echo "       This build must be hard-float (MCU input-shaping); a soft-float image is" >&2
  echo "       silently wrong (only symptom: a larger binary). Refusing to ship it." >&2
  exit 1
fi
if [ "$_HF" != "0" ]; then
  _DIS=$(arm-none-eabi-objdump -d out/klipper.elf)
  _EN=$(printf '%s\n' "$_DIS" | grep -E "orr.*15728640" | head -1 | sed 's/^ *//;s/:.*//')
  _FP=$(printf '%s\n' "$_DIS" | grep -E "\sv[a-z]+(\.f32|\.f64|\.s32)" | head -1 | sed 's/^ *//;s/:.*//')
  if [ -z "$_EN" ]; then
    echo "FATAL: hard-float ABI but NO CPACR enable in the image (first FP would hard-fault)." >&2
    exit 1
  fi
  if [ -n "$_FP" ] && [ "$((0x$_EN))" -ge "$((0x$_FP))" ]; then
    echo "FATAL: FPU enabled at 0x$_EN but first FP instruction at 0x$_FP (FP before enable)." >&2
    exit 1
  fi
  echo "hard-float: FPU enabled at 0x$_EN, first FP use at 0x${_FP:-none} (ordering OK)"
fi

mkdir -p "$HERE/out"
cp out/klipper.bin "$HERE/out/$ART"
echo "OK -> coreone/out/$ART  ($(stat -c%s out/klipper.bin) bytes)"
