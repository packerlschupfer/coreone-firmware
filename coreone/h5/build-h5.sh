#!/usr/bin/env bash
# Build the STM32H503 (xBuddy extension) Klipper port in-tree from this fork. Adds a
# from-scratch MACH_STM32H5 target (the H5 sources are committed here; the patches below
# are kept for provenance and no-op when already applied). Vendors the external STM32H5
# CMSIS device headers at build time. Hardware-validated on a real Core One+ enclosure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUDDY="$(cd "$HERE/../.." && pwd)"               # fork root = the Klipper tree itself
KLIPPER_DIR="${KLIPPER_DIR:-$BUDDY}"             # build in-tree (override to build elsewhere)
TOOLCHAIN="$BUDDY/.dependencies/gcc-arm-none-eabi-13.3.1/bin"
CMSIS="${CMSIS_H5:-$HOME/git/cmsis_device_h5}"   # github.com/STMicroelectronics/cmsis_device_h5

# Build variant: 'noboot' (default) = standalone app @0x08000000 for SWD flashing;
# 'boot' = puppy-app @0x08002000 chain-loaded by the Prusa-Bootloader-Puppy, padded
# to the 122,880 B RS-485 install slot (backlog 9b).
VARIANT="${1:-noboot}"
case "$VARIANT" in
    noboot) CONFIG_SRC="$HERE/.config-h503" ;;
    boot)   CONFIG_SRC="$HERE/.config-h503-boot" ;;
    *) echo "usage: build-h5.sh [noboot|boot]"; exit 1 ;;
esac
echo "=== build variant: $VARIANT (config: $(basename "$CONFIG_SRC")) ==="

[ -d "$KLIPPER_DIR/.git" ] || { echo "KLIPPER_DIR=$KLIPPER_DIR is not a git checkout"; exit 1; }
[ -d "$CMSIS/Include" ] || { echo "missing CMSIS H5 headers at $CMSIS (clone cmsis_device_h5)"; exit 1; }

# 1. Vendor the STM32H5 CMSIS device headers into the Klipper tree
mkdir -p "$KLIPPER_DIR/lib/stm32h5/include"
cp "$CMSIS"/Include/*.h "$KLIPPER_DIR/lib/stm32h5/include/"
# core_cm33.h (Cortex-M33) ships with Klipper's cmsis-core already; copy if absent
[ -f "$KLIPPER_DIR/lib/cmsis-core/core_cm33.h" ] \
    || cp "$HOME/git/STM32CubeH5/Drivers/CMSIS/Core/Include/core_cm33.h" \
          "$KLIPPER_DIR/lib/cmsis-core/" 2>/dev/null || true

# 2. Apply the MACH_STM32H5 source patch (internal.h/Kconfig/Makefile/usbfs.c + new
#    stm32h5.c), idempotently.
cd "$KLIPPER_DIR"
if git apply --check "$HERE/klipper-h5.patch" 2>/dev/null; then
    git apply "$HERE/klipper-h5.patch"; echo "applied klipper-h5.patch"
elif git apply --reverse --check "$HERE/klipper-h5.patch" 2>/dev/null; then
    echo "klipper-h5.patch already applied (skipping)"
else
    echo "WARNING: klipper-h5.patch does not apply cleanly against this Klipper SHA"
fi

# 2a. Apply the H5 ADC DEEPPWD-exit fix to src/stm32/stm32h7_adc.c, idempotently.
#     (The ADC *clock-enable* fix lives in h5/stm32h5.c, copied below.)
if git apply --check "$HERE/adc-h5-deeppwd.patch" 2>/dev/null; then
    git apply "$HERE/adc-h5-deeppwd.patch"; echo "applied adc-h5-deeppwd.patch"
elif git apply --reverse --check "$HERE/adc-h5-deeppwd.patch" 2>/dev/null; then
    echo "adc-h5-deeppwd.patch already applied (skipping)"
fi

# 2a2. Apply the H5 I2C2 bus defs (xBuddy ext TCA6408A on PB10/PB13, AF4) to the v2
#      driver src/stm32/stm32f0_i2c.c, idempotently.
if git apply --check "$HERE/i2c-h5.patch" 2>/dev/null; then
    git apply "$HERE/i2c-h5.patch"; echo "applied i2c-h5.patch"
elif git apply --reverse --check "$HERE/i2c-h5.patch" 2>/dev/null; then
    echo "i2c-h5.patch already applied (skipping)"
fi

# 2a3. Allow the 8 KiB-bootloader offset (CONFIG_STM32_FLASH_START_2000) for H5 — adds
#      MACH_STM32H5 to the Kconfig gate so the 'boot' variant can place the app at
#      0x08002000. Harmless for the noboot build (default stays FLASH_START_0000).
if git apply --check "$HERE/flash-start-2000.patch" 2>/dev/null; then
    git apply "$HERE/flash-start-2000.patch"; echo "applied flash-start-2000.patch"
elif git apply --reverse --check "$HERE/flash-start-2000.patch" 2>/dev/null; then
    echo "flash-start-2000.patch already applied (skipping)"
fi

# 2b. The standalone h5/stm32h5.c is the SOURCE OF TRUTH for the clock/boot code —
#     the patch only carries the initial snapshot (it went stale once already: the
#     USBSEL fix lived here but not in the patch). Always overwrite the patch's copy.
cp "$HERE/stm32h5.c" "$KLIPPER_DIR/src/stm32/stm32h5.c"
echo "installed src/stm32/stm32h5.c (overlay copy overrides the patch snapshot)"

# 3. Configure + build. NB: rm -rf out -- a plain `make` does NOT regenerate the
#    command dict when only DECL_ENUMERATION/DECL_COMMAND change (stale
#    compile_time_request.o), so always start clean for this small overlay build.
cp "$CONFIG_SRC" .config
[ -d "$TOOLCHAIN" ] && export PATH="$TOOLCHAIN:$PATH"
rm -rf out
make olddefconfig >/dev/null
make -j"$(nproc)"
echo "=== size ==="; arm-none-eabi-size out/klipper.elf
BASE="$(arm-none-eabi-objdump -h out/klipper.elf | awk '/\.text/{print $4; exit}')"
if [ "$VARIANT" = boot ]; then
    echo "built STM32H503 (puppy): .vector_table base = 0x$BASE (expect 08002000)"
    # Post-link pad: the Prusa-Bootloader-Puppy fingerprints a FIXED 122,880 B range
    # (0x08002000..0x08020000, incl the trailing 128 B FW_DESCRIPTOR), so the shipped
    # file MUST equal the flashed slot byte-for-byte. Pad the raw app .bin with zeros
    # up to the full slot; the top 128 B stay zero (the descriptor placeholder).
    PUPPY_BIN="out/klipper-h503-puppy.bin"
    python3 - "out/klipper.bin" "$PUPPY_BIN" <<'PYEOF'
import sys, struct, hashlib
src, dst = sys.argv[1], sys.argv[2]
TOTAL = 122880            # 0x1E000 = full H503 puppy app slot (incl 128 B descriptor)
DESC  = 128               # FW_DESCRIPTOR region at the tail (== FW_DESCRIPTOR_SIZE)
FW_TYPE = 12321           # FWDescriptor::StoredType::fw (crash_dump_shared.hpp)
data = open(src, "rb").read()
# Contract self-verify (fail loud before a bad image reaches the RS-485 install):
assert len(data) <= TOTAL - DESC, \
    f"app too big: {len(data)} B > {TOTAL-DESC} B usable (128 B reserved for FW_DESCRIPTOR)"
msp, rst = struct.unpack("<II", data[:8])
assert 0x20000000 <= msp <= 0x20008000, \
    f"bad initial MSP 0x{msp:08X} — vectors not at the chained-load base?"
assert 0x08002000 <= (rst & ~1) < 0x08020000, \
    f"reset vector 0x{rst:08X} outside puppy slot — wrong FLASH_START variant (need .config-h503-boot)"
# Pad to the full slot, then write a VALID FW_DESCRIPTOR into the last 128 B so the puppy
# bootloader's unsalted-fingerprint check passes and it AUTO-CHAINLOADS (no PUPPY_START).
# Mirrors Prusa-Firmware-Buddy/utils/gen_puppies_descriptor.py exactly: fingerprint =
# sha256(app excluding the 128 B descriptor); struct = stored_type(fw), dump_offset(0),
# fingerprint[32], dump_size(0), little-endian, zero-padded to 128.
padded = bytearray(data + b"\x00" * (TOTAL - len(data)))
assert len(padded) == TOTAL
fp = hashlib.sha256(bytes(padded[:TOTAL - DESC])).digest()
desc = struct.pack("<II", FW_TYPE, 0) + fp + struct.pack("<I", 0)
padded[TOTAL - DESC:] = desc + b"\x00" * (DESC - len(desc))
# self-check: the embedded fingerprint must equal sha256 over the final image body
assert bytes(padded[TOTAL-DESC+8 : TOTAL-DESC+40]) == hashlib.sha256(bytes(padded[:TOTAL-DESC])).digest(), \
    "FW_DESCRIPTOR fingerprint self-check failed"
open(dst, "wb").write(bytes(padded))
print(f"verified+padded: {len(data)} B app, vectors@0x08002000, descriptor written "
      f"(fp {fp.hex()[:16]}…) -> {len(padded)} B slot (headroom {TOTAL-DESC-len(data)} B)")
PYEOF
    # Copy to a stable path under the overlay (out/ gets wiped by the next build).
    STABLE="$HERE/klipper-h503-puppy.bin"
    cp "$PUPPY_BIN" "$STABLE"
    echo "puppy binary: $STABLE"
    echo "  -> hand this path to the Buddy build: -DXBUDDY_EXTENSION_BINARY_PATH=$STABLE"
else
    echo "built STM32H503: .text base = 0x$BASE (expect 08000000, no-bootloader)"
    [ "$BASE" = 08000000 ] || { echo "ERROR: .text base 0x$BASE != 0x08000000 — wrong config (need .config-h503)"; exit 1; }
    # Emit a stable, self-checked .bin for SWD flashing @0x08000000 (out/ is wiped each build).
    NOBOOT_BIN="out/klipper-h503-0x08000000.bin"
    python3 - "out/klipper.bin" "$NOBOOT_BIN" <<'PYEOF'
import sys, struct
src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()
msp, rst = struct.unpack("<II", data[:8])
assert 0x20000000 <= msp <= 0x20008000, \
    f"bad initial MSP 0x{msp:08X} — vectors not at the no-bootloader base?"
assert 0x08000000 <= (rst & ~1) < 0x08020000, \
    f"reset vector 0x{rst:08X} outside the no-bootloader region — wrong FLASH_START (need .config-h503)"
open(dst, "wb").write(data)
print(f"verified: {len(data)} B app, vectors@0x08000000 (MSP 0x{msp:08X}, reset 0x{rst:08X})")
PYEOF
    # Copy to a stable path under the overlay (out/ gets wiped by the next build).
    STABLE="$HERE/klipper-h503-noboot.bin"
    cp "$NOBOOT_BIN" "$STABLE"
    echo "noboot binary: $STABLE"
    echo "NOT yet flashed — SWD to the H503 (J9/TP2-4) + RDP check + backup first."
fi
