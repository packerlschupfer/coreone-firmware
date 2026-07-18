#!/usr/bin/env bash
# build-autoboot.sh — build the AUTO-CHAINLOAD H503 image: the modified Prusa puppy
# bootloader (idle-timeout auto-jump) @0x08000000 + the klipper-puppy app @0x08002000,
# combined into one 128 KB image for a single SWD write.
#
# The bootloader change (BOOTLOADER_AUTOBOOT_MS, xbuddy_extension only) makes it
# auto-chainload a descriptor-valid app after ~3 s of RS-485 idle, so this image:
#   - auto-boots the enclosure over USB (no PUPPY_START), like noboot, AND
#   - is re-flashable over RS-485 with no ST-Link (PUPPY_REFLASH; see h5/puppybus-flash.cfg).
#
# Output: out/klipper-h503-autoboot-combined.bin  (+ stable h5/klipper-h503-autoboot-combined.bin)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BL_REPO="${BL_REPO:-$HOME/git/Prusa-Bootloader-Puppy}"
BL_BIN="$BL_REPO/build/xbuddy_extension/bootloader-v302-xbuddy_extension-1.0.bin"

[ -d "$BL_REPO" ] || { echo "ERROR: bootloader repo not found: $BL_REPO (set BL_REPO=)"; exit 1; }
# Preflight: the bootloader source MUST carry the auto-chainload change, else the image
# would never auto-boot (it would wait forever for PUPPY_START).
grep -q 'BOOTLOADER_AUTOBOOT_MS' "$BL_REPO/CMakeLists.txt" \
  || { echo "ERROR: $BL_REPO has no BOOTLOADER_AUTOBOOT_MS (un-patched bootloader)."; exit 1; }

echo "=== 1/3 build the modified xbuddy_extension bootloader ==="
( cd "$BL_REPO" && cmake --preset xbuddy_extension >/dev/null && cmake --build --preset xbuddy_extension >/dev/null )
[ -f "$BL_BIN" ] || { echo "ERROR: bootloader binary not produced: $BL_BIN"; exit 1; }
echo "  bootloader: $(stat -c%s "$BL_BIN") B"

echo "=== 2/3 build the klipper-puppy app (valid FW_DESCRIPTOR) ==="
bash "$HERE/build-h5.sh" boot >/dev/null
APP_BIN="$HERE/klipper-h503-puppy.bin"
[ -f "$APP_BIN" ] || { echo "ERROR: app binary not produced: $APP_BIN"; exit 1; }

echo "=== 3/3 combine -> 128 KB image (8 KB bootloader + 120 KB app) ==="
OUT="$HERE/klipper-h503-autoboot-combined.bin"
python3 - "$BL_BIN" "$APP_BIN" "$OUT" <<'PYEOF'
import sys, struct, hashlib
boot = open(sys.argv[1],'rb').read(); app = open(sys.argv[2],'rb').read()
assert len(boot) <= 8192,   f"bootloader {len(boot)} B > 8192"
assert len(app)  == 122880, f"app {len(app)} B != 122880"
bmsp,brst = struct.unpack('<II', boot[:8]); amsp,arst = struct.unpack('<II', app[:8])
assert 0x20000000 <= bmsp <= 0x20008000 and 0x08000000 <= (brst & ~1) < 0x08002000, "bad bootloader vectors"
assert 0x20000000 <= amsp <= 0x20008000 and 0x08002000 <= (arst & ~1) < 0x08020000, "bad app vectors"
comb = boot + b'\x00'*(8192-len(boot)) + app
assert len(comb) == 131072
st, = struct.unpack('<I', comb[-128:][:4]); fp = comb[-128:][8:40]
assert st == 12321 and fp == hashlib.sha256(comb[8192:131072-128]).digest(), "descriptor invalid"
open(sys.argv[3],'wb').write(comb)
print(f"  combined OK: 131072 B, descriptor valid (fp {fp.hex()[:16]}...) -> auto-chainloads")
PYEOF
echo ""
echo "Flash to the H503 @0x08000000 (RDP-gated, Open required):"
echo "  flash.sh h503-autoboot-swd      # or: swd-flash.sh h503 $OUT"
echo "Then power-cycle -> the H503 auto-boots klipper-puppy over USB (no PUPPY_START)."
