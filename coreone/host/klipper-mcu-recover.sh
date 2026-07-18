#!/usr/bin/env bash
# Auto-recover Klipper after a printer power-cycle on a separately-powered host.
#
# On this dev setup the Pi stays powered while the printer (F427 + H503) can be
# cold-booted on its own. Klipper does NOT auto-reconnect to an MCU that vanished
# -- it transitions to "shutdown" and waits for a FIRMWARE_RESTART. Triggered by
# udev when the F427 Klipper serial re-enumerates, this waits for BOTH MCUs and,
# only if klippy is stuck (shutdown/error), issues a FIRMWARE_RESTART. A healthy
# klippy (ready / fresh startup) is left untouched.
set -u
LOG=/tmp/klipper-mcu-recover.log
STAMP=/tmp/klipper-mcu-recover.last
MOON=http://localhost:7125
F427='/dev/serial/by-id/usb-Klipper_stm32f427xx_*'
H503='/dev/serial/by-id/usb-Klipper_stm32h503xx_*'

log(){ echo "$(date '+%F %T') $*" >>"$LOG" 2>/dev/null; }
state(){ curl -s "$MOON/printer/info" 2>/dev/null \
         | grep -o '"state":"[a-z]*"' | head -1 | cut -d'"' -f4; }

# Debounce: a FIRMWARE_RESTART re-enumerates the MCUs and re-fires udev, so skip
# if we already acted in the last 45s -- this is what stops a restart loop.
if [ -f "$STAMP" ] && [ $(( $(date +%s) - $(stat -c %Y "$STAMP") )) -lt 45 ]; then
    log "debounce: acted <45s ago, skip"; exit 0
fi

# Wait for BOTH MCUs (the H503 is powered off the F427 rail, comes up a bit later).
for _ in $(seq 1 20); do
    ls $F427 >/dev/null 2>&1 && ls $H503 >/dev/null 2>&1 && break
    sleep 1
done
ls $F427 >/dev/null 2>&1 && ls $H503 >/dev/null 2>&1 \
    || { log "both MCUs not present; skip"; exit 0; }

# Only act if klippy is actually stuck; poll a little, since the shutdown can lag
# the re-enumeration by a couple of seconds.
s=""
for _ in $(seq 1 12); do
    s=$(state)
    case "$s" in
        shutdown|error)
            log "klippy '$s' -> FIRMWARE_RESTART"
            touch "$STAMP"
            curl -s -X POST "$MOON/printer/firmware_restart" >/dev/null 2>&1
            sleep 10
            log "post-restart state: $(state)"
            exit 0 ;;
        ready)
            log "klippy ready; nothing to do"; exit 0 ;;
    esac
    sleep 2
done
log "klippy state never settled (last: '$s'); skip"
exit 0
