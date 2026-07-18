#!/usr/bin/env bash
# Pre-flight for a Katapult USB flash of the F427.
#
# WHY THIS EXISTS: which bootloader the board runs is the single fact that decides whether a
# flash is safe, and it is the one fact easy to get wrong from notes. Writing a 0x08008000
# image to a board that has no Katapult bricks it until SWD; writing a 0x08000000 image to a
# board that HAS Katapult silently erases the open bootloader. So look, don't assume.
#
# Prints the resolved device path on stdout (for the caller to use) and everything else on
# stderr. Exits non-zero if the board is not in a state we can safely USB-flash.
#
#   ./katapult-preflight.sh            # report + resolve
#   ./katapult-preflight.sh --reboot   # also ask a running Klipper to reboot into Katapult
#
# Exit codes:  0 = Katapult present, safe to flash (device path on stdout)
#              1 = do NOT flash (nothing found / wrong layout / no flashtool)
#              2 = ambiguous (several matching devices -- name the right one by hand)
#              3 = Klipper present but bootloader unconfirmed; re-run with --reboot
#              4 = TOOLING failure -- the check could NOT be performed (no pyserial, flashtool
#                  errored). ⚠ This is NOT a verdict about the board. Never read a 4 as "no
#                  Katapult" and never fall back to SWD/coexist on it -- that erases the
#                  bootloader if Katapult is in fact present.
set -euo pipefail

# BYID is overridable ONLY so the branch logic can be exercised without hardware, by pointing
# it at a directory of empty files named like real devices. Never override it for a real flash.
BYID="${BYID:-/dev/serial/by-id}"
KATAPULT_GLOB='usb-katapult_stm32f4*-if00'
KLIPPER_GLOB='usb-Klipper_stm32f427xx_*-if00'
FLASHTOOL="${FLASHTOOL:-$HOME/git/katapult/scripts/flashtool.py}"
DO_REBOOT=0
[ "${1:-}" = "--reboot" ] && DO_REBOOT=1

say() { echo "$@" >&2; }

find_one() {  # $1 = glob; echoes the single match or nothing
  local m=()
  # shellcheck disable=SC2231
  for p in $BYID/$1; do [ -e "$p" ] && m+=("$p"); done
  [ "${#m[@]}" -eq 1 ] && { echo "${m[0]}"; return 0; }
  [ "${#m[@]}" -gt 1 ] && { say "PREFLIGHT FAIL: $1 matched ${#m[@]} devices:"; \
                            printf '  %s\n' "${m[@]}" >&2; return 2; }
  return 1
}

[ -d "$BYID" ] || { say "PREFLIGHT FAIL: $BYID missing -- no USB serial devices at all."; exit 1; }
[ -f "$FLASHTOOL" ] || { say "PREFLIGHT FAIL: flashtool.py not found at $FLASHTOOL"; \
                         say "  clone https://github.com/Arksine/katapult or set FLASHTOOL="; exit 1; }

# Resolve an interpreter that actually HAS pyserial, and fail loudly here if none does.
# WHY THIS EXISTS: this used to be a bare `python3`, and on the printer host the SYSTEM python3
# has no pyserial. flashtool then died with "The pyserial python package was not found", the
# `|| true` below swallowed it, no katapult device ever appeared, and the script concluded
# "this board does NOT have Katapult" and advised the SWD/coexist path -- which is precisely
# what ERASES the open bootloader. A tooling error must never become a hardware verdict.
# Klipper's own venv is the reliable source of pyserial on a printer host.
PYBIN=""
for cand in ${PYTHON:-} "$HOME/klippy-env/bin/python" /home/pi/klippy-env/bin/python \
            "$HOME/klipper-env/bin/python" python3; do
    [ -n "$cand" ] || continue
    command -v "$cand" >/dev/null 2>&1 || [ -x "$cand" ] || continue
    if "$cand" -c 'import serial' >/dev/null 2>&1; then PYBIN="$cand"; break; fi
done
if [ -z "$PYBIN" ]; then
    say "PREFLIGHT FAIL (TOOLING, *NOT* a verdict about the board): no python with pyserial."
    say "  Tried: \$PYTHON, ~/klippy-env/bin/python, /home/pi/klippy-env/bin/python,"
    say "         ~/klipper-env/bin/python, python3"
    say "  flashtool.py cannot run without pyserial, so the bootloader check CANNOT be performed."
    say "  This says NOTHING about whether Katapult is present. Do NOT conclude it is absent,"
    say "  and do NOT fall back to an SWD/coexist image on the strength of this -- that would"
    say "  erase the bootloader if Katapult IS there. Fix the interpreter, then re-run:"
    say "     PYTHON=/home/pi/klippy-env/bin/python $0 ${1:-}"
    exit 4
fi
say "python for flashtool: $PYBIN"

# Ambiguity must exit 2 here and not fall through: with `|| true` a multi-match used to be
# swallowed and reported as "found neither", which is both wrong and confusing (two
# contradictory PREFLIGHT FAIL messages for one cause).
kat="$(find_one "$KATAPULT_GLOB")" || [ $? -eq 1 ] || exit 2
klip="$(find_one "$KLIPPER_GLOB")" || [ $? -eq 1 ] || exit 2

if [ -n "$kat" ]; then
  say "Board is ALREADY in Katapult (bootloader) mode:"
  say "  $kat"
  echo "$kat"
  exit 0
fi

if [ -z "$klip" ]; then
  say "PREFLIGHT FAIL: found neither Katapult nor Klipper on USB."
  say "  Present devices:"; ls -1 "$BYID" >&2 2>/dev/null || say "  (none)"
  say
  say "  Do NOT flash. Either the board is off, or it is not running a USB-Klipper build,"
  say "  or it is in the Prusa-bootloader (coexist) layout -- in which case a Katapult"
  say "  image is the WRONG variant and would need SWD to recover."
  exit 1
fi

say "Board is running Klipper (application) mode:"
say "  $klip"
say
say "This tells us a USB-Klipper build is live, but NOT by itself that Katapult sits at"
say "0x08000000. Rebooting into the bootloader is the actual proof: if Katapult appears,"
say "it is there; if nothing appears, it is not, and this is the wrong flash path."
if [ "$DO_REBOOT" != 1 ]; then
  say
  say "Re-run with --reboot to request the bootloader, or do it by hand:"
  say "  $PYBIN $FLASHTOOL -r -d $klip"   # resolved interpreter, NOT a bare python3 -- see above
  exit 3
fi

say
say "Requesting a reboot into Katapult..."
# ⚠ DO NOT restore `|| true` HERE. The whole point of this script is to decide whether Katapult
# exists, and it infers that from "did a katapult device appear?". If the request never RAN, that
# inference is invalid -- absence of the device then means nothing about the board. Swallowing the
# exit code is what turned a missing-pyserial error into "this board does NOT have Katapult, use
# SWD instead", i.e. into advice that erases the bootloader. Same anti-pattern build.sh documents
# in its own history. Keep the two failures distinguishable.
rc=0
"$PYBIN" "$FLASHTOOL" -r -d "$klip" >&2 || rc=$?
if [ "$rc" -ne 0 ]; then
    say
    say "PREFLIGHT FAIL (TOOLING, *NOT* a verdict about the board): the reboot request could not"
    say "  run -- $PYBIN $FLASHTOOL exited $rc."
    say "  The bootloader check was NOT performed. This says NOTHING about whether Katapult is"
    say "  present. Do NOT flash, and do NOT switch to an SWD/coexist image on the strength of"
    say "  this -- if Katapult IS there, that erases it. Fix the error above and re-run."
    exit 4
fi
for i in $(seq 1 20); do
  sleep 0.5
  kat="$(find_one "$KATAPULT_GLOB" || true)"
  [ -n "$kat" ] && break
  printf '.' >&2
done
echo >&2
if [ -z "$kat" ]; then
  say "PREFLIGHT FAIL: no Katapult device appeared after the reboot request."
  say "  (The request itself RAN and succeeded -- exit 0 above -- so this absence is a real"
  say "   observation about the board, not a tooling failure. That distinction is why the"
  say "   flashtool exit code is checked rather than swallowed; a tooling failure exits 4.)"
  say "  => this board does NOT have Katapult at 0x08000000. A 0x08008000 image would"
  say "     not be run by anything. Do NOT flash; use the SWD/coexist path instead."
  exit 1
fi
say "Katapult present after reboot -- layout CONFIRMED:"
say "  $kat"
echo "$kat"
