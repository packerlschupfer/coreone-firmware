#!/usr/bin/env bash
# flash.sh — owner-facing dispatcher for Prusa Core One+ Klipper firmware.
#
# Picks a PROFILE, runs prerequisite + safety gates, then delegates the actual flash
# to the proven per-MCU scripts (build.sh, h5/build-h5.sh, swd-flash.sh, pack-bbf.sh,
# bbf-to-swd.py, h5/check-rdp.sh). It never invents flash logic. The F427 and H503
# images are independent — pick one of each. See docs/OWNERS_GUIDE.md.
#
# Each build script re-applies its own overlay/.config, so building one MCU then the
# other is safe (no manual "restore mode" step needed).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSUME_YES=0

usage() {
  cat <<'EOF'
usage: flash.sh <profile> [--yes]

  F427 "main" MCU:
    f427-coexist-bbf   PRIMARY, no tools. Build + pack a BBF; install from a USB stick.
    f427-coexist-swd   advanced. Build + SWD-write behind the bootloader @0x08020000.
    f427-noboot-swd    advanced. Build + SWD-write a self-contained image @0x08000000.

  H503 "extension" MCU:
    h503-autoboot-swd  RECOMMENDED. Build the auto-chainload bootloader + app, SWD-write the
                       combined image (RDP gate). Auto-boots the enclosure over USB AND is
                       re-flashable over RS-485 (no ST-Link) thereafter.
    h503-usb-swd       Build + SWD-write the self-contained noboot USB-Klipper image (RDP gate).
    h503-rs485         Build the puppy app for RS-485 flashing onto the STOCK bootloader
                       (needs PUPPY_START each boot; prefer h503-autoboot-swd for auto-boot).

  Restore to stock Prusa firmware:
    restore-f427-bbf   no tools. Install the stock BBF from a USB stick.
    restore-f427-swd   SWD full restore (bootloader + firmware).
    restore-h503-swd   SWD full restore (RDP gate). The only H503 restore path.

  --yes / -y   skip the confirmation prompt (for scripting).
EOF
}

confirm() {  # $1 = action description
  echo
  echo "ABOUT TO: $1"
  [ "$ASSUME_YES" = 1 ] && { echo "(--yes given, proceeding)"; return 0; }
  read -r -p "Type 'yes' to proceed: " ans
  [ "$ans" = yes ] || { echo "aborted."; exit 1; }
}

preflight() {
  # The RS-485 host module must carry PUPPY_FLASH/START at the right bootloader
  # address. Read the canonical klippy/extras/puppybus.py directly (single source
  # of record) so a stale duplicate can't exist to be flashed.
  local pb="$HERE/../klippy/extras/puppybus.py"
  if ! grep -q 'def cmd_PUPPY_START' "$pb" 2>/dev/null \
     || ! grep -q 'ADDR_XBE_BOOTLOADER = 0x11' "$pb" 2>/dev/null; then
    echo "PREFLIGHT FAIL: $pb is missing PUPPY_START / ADDR_XBE_BOOTLOADER=0x11." >&2
    echo "  The fork's klippy/extras/puppybus.py is the proven module — restore it." >&2
    exit 1
  fi
}

profile="${1:-}"; [ -z "$profile" ] && { usage; exit 1; }
shift || true
for a in "$@"; do case "$a" in --yes|-y) ASSUME_YES=1 ;; *) echo "unknown arg: $a"; usage; exit 1 ;; esac; done
preflight

case "$profile" in
  f427-coexist-bbf)
    cat <<'EOF'
== f427-coexist-bbf (PRIMARY, no tools) ==
  Prereq: the APPENDIX SEAL must be BROKEN (factory F427 rejects unsigned firmware until then).
  Does : build the coexist F427 image + pack an unsigned BBF.
  Needs: a FAT32 USB stick; the stock Prusa bootloader present (factory default).
  Risk : none to the bootloader — it installs the firmware the factory way.
EOF
    confirm "build coexist F427 + pack BBF"
    "$HERE/build.sh" coexist
    "$HERE/pack-bbf.sh" "$HERE/out/klipper.bin"
    cat <<EOF

NEXT (on the printer, no tools):
  1. Copy  $HERE/out/klipper.bbf  to a FAT32 USB stick, renamed  firmware.bbf
  2. Insert the stick, press the printer's reset button.
  3. Accept the "custom firmware / verification failed" prompt (unsigned is expected).
  Confirm Klipper enumerates over USB before doing anything else.
EOF
    ;;

  f427-coexist-swd)
    cat <<'EOF'
== f427-coexist-swd (advanced) ==
  Prereq: APPENDIX SEAL BROKEN — an intact appendix loads PA13/SWDIO and blocks SWD (+ unsigned fw).
  Does : build coexist F427 + SWD-write [descriptor][firmware] @0x08020000.
  Needs: a genuine STLINK-V3 with NRST wired to the F427.
  Risk : writes sector 5+ only; the guard refuses to touch the bootloader. One flash, then STOP.
EOF
    confirm "build coexist F427 + SWD-write @0x08020000"
    "$HERE/build.sh" coexist
    "$HERE/pack-bbf.sh" "$HERE/out/klipper.bin"
    python3 "$HERE/bbf-to-swd.py" "$HERE/out/klipper.bbf" "$HERE/out/klipper-coexist-0x08020000.bin"
    "$HERE/swd-flash.sh" f427-coexist "$HERE/out/klipper-coexist-0x08020000.bin"
    echo "Confirm Klipper enumerates, then STOP (no second flash)."
    ;;

  f427-noboot-swd)
    cat <<'EOF'
== f427-noboot-swd (advanced) ==
  Prereq: APPENDIX SEAL BROKEN — an intact appendix loads PA13/SWDIO and blocks SWD (+ unsigned fw).
  Does : build the self-contained F427 image + SWD-write @0x08000000.
  Needs: a genuine STLINK-V3 with NRST wired. NO Prusa bootloader after this (SWD-only recovery).
  Risk : overwrites flash from 0x08000000. One flash, then STOP.
EOF
    confirm "build noboot F427 + SWD-write @0x08000000"
    "$HERE/build.sh" noboot
    "$HERE/swd-flash.sh" f427 "$HERE/out/klipper-0x08000000.bin"
    echo "Confirm Klipper enumerates, then STOP."
    ;;

  h503-usb-swd)
    cat <<'EOF'
== h503-usb-swd ==
  Does : build the USB-Klipper H503 image + SWD-write @0x08000000.
  Needs: a genuine STLINK-V3 on the H503 (J9/TP2-4); H503 powered via the F427 (PG2).
  Risk : H503 product state MUST be Open or a write mass-erases the puppy bootloader -> RDP gate runs first.
         Do NOT FIRMWARE_RESTART the F427 mid-flash (it powers the H503).
EOF
    confirm "RDP-gate + build + SWD-write the H503 USB image"
    "$HERE/h5/check-rdp.sh"
    "$HERE/h5/build-h5.sh" noboot
    "$HERE/swd-flash.sh" h503 "$HERE/h5/klipper-h503-noboot.bin"
    echo "Confirm the H503 enumerates over USB, then STOP."
    ;;

  h503-autoboot-swd)
    cat <<'EOF'
== h503-autoboot-swd (RECOMMENDED H503 image) ==
  Does : build the auto-chainload bootloader + klipper-puppy app, combine, SWD-write @0x08000000.
  Needs: a genuine STLINK-V3 on the H503 (J9/TP2-4) + ST-fork OpenOCD; H503 powered via the F427 (PG2).
         The bootloader source must carry the BOOTLOADER_AUTOBOOT_MS change (build-autoboot.sh checks).
  Result: the H503 AUTO-BOOTS the enclosure over USB (no PUPPY_START) AND is re-flashable over RS-485
          with no ST-Link thereafter (see h5/puppybus-flash.cfg + PUPPY_REFLASH).
  Risk : H503 product state MUST be Open -> RDP gate runs first. ONE flash, then confirm-boot.
         Do NOT FIRMWARE_RESTART the F427 mid-flash (it powers the H503). [output_pin ext_pwr]
         must have shutdown_value:1 so a klippy shutdown can't cut PG2 mid-write.
EOF
    confirm "RDP-gate + build (bootloader+app) + SWD-write the H503 auto-chainload image @0x08000000"
    "$HERE/h5/check-rdp.sh"
    "$HERE/h5/build-autoboot.sh"
    "$HERE/swd-flash.sh" h503 "$HERE/h5/klipper-h503-autoboot-combined.bin"
    cat <<'EOF'
Confirm the H503 enumerates over USB (auto-boot, no PUPPY_START), then STOP.
Later RS-485 updates need NO ST-Link: copy h5/puppybus-flash.cfg over printer.cfg, FIRMWARE_RESTART
(drops PG2 -> H503 off), then  PUPPY_REFLASH FILE=/tmp/klipper-h503-puppy.bin , then restore your config.
EOF
    ;;

  h503-rs485)
    cat <<'EOF'
== h503-rs485 (no tools; F427 already running Klipper) ==
  Does : build the puppy H503 image; you flash it over RS-485 from the printer.
  Needs: F427 on Klipper with [puppybus] in printer.cfg; H503 in its puppy bootloader (addr 0x11).
  Note : verified -- the STOCK Prusa puppy bootloader (v297) accepts our unsigned app over RS-485
         (full PUPPY_FLASH + salted-fingerprint PUPPY_START); no v302 swap needed.
EOF
    confirm "build the puppy H503 image for RS-485 flashing"
    "$HERE/h5/build-h5.sh" boot
    cat <<EOF

NEXT (on the host running klippy):
  1. Copy  $HERE/h5/klipper-h503-puppy.bin  to the Pi (e.g. /tmp/).
  2. In the Klipper console:
       PUPPY_HWINFO                                  # confirm a bootloader answers at 0x11 (record bl_ver)
       PUPPY_FLASH FILE=/tmp/klipper-h503-puppy.bin
       PUPPY_START FILE=/tmp/klipper-h503-puppy.bin
     Expect fingerprint_match=1, then the bootloader stops answering (it jumped to the app).
EOF
    ;;

  restore-f427-bbf)
    cat <<'EOF'
== restore-f427-bbf (no tools) ==
  Does : return the F427 to stock Prusa firmware from a USB stick.
  Needs: the stock BBF + a FAT32 stick. The Prusa bootloader must be present.
EOF
    bbf="$HOME/git/firmware_stock_6.5.3.bbf"
    [ -f "$bbf" ] || { echo "missing stock BBF: $bbf" >&2; exit 1; }
    confirm "stage the stock F427 BBF restore"
    cat <<EOF
NEXT: copy  $bbf  to a FAT32 stick as  firmware.bbf , insert, press reset.
EOF
    ;;

  restore-f427-swd)
    cat <<'EOF'
== restore-f427-swd (full SWD restore) ==
  Does : SWD-write the full stock image (bootloader + firmware) @0x08000000.
  Needs: a genuine STLINK-V3 with NRST wired.
  Risk : one flash, then STOP.
EOF
    confirm "SWD full-restore the F427 to stock"
    "$HERE/swd-flash.sh" stock f427
    ;;

  restore-h503-swd)
    cat <<'EOF'
== restore-h503-swd (only full H503 restore) ==
  Does : SWD-write the full stock H503 image (puppy bootloader + app) @0x08000000.
  Needs: a genuine STLINK-V3 on the H503; H503 powered via the F427.
  Risk : RDP gate runs first (a non-Open write mass-erases the bootloader). One flash, then STOP.
EOF
    confirm "RDP-gate + SWD full-restore the H503 to stock"
    "$HERE/h5/check-rdp.sh"
    "$HERE/swd-flash.sh" stock h503
    ;;

  -h|--help|help) usage; exit 0 ;;
  *) echo "unknown profile: $profile" >&2; usage; exit 1 ;;
esac
