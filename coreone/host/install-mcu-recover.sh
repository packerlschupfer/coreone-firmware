#!/usr/bin/env bash
# Install the Klipper cold-boot auto-recovery (udev rule -> oneshot service ->
# recover script). Run ON THE PI. Idempotent.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo install -m 0755 "$HERE/klipper-mcu-recover.sh" \
    /usr/local/bin/klipper-mcu-recover.sh
sudo install -m 0644 "$HERE/klipper-mcu-recover.service" \
    /etc/systemd/system/klipper-mcu-recover.service
sudo install -m 0644 "$HERE/99-klipper-mcu-recover.rules" \
    /etc/udev/rules.d/99-klipper-mcu-recover.rules

sudo systemctl daemon-reload
sudo udevadm control --reload
echo "installed klipper-mcu-recover: udev rule + oneshot service + script"
echo "test:  udevadm trigger --action=add \$(readlink -f /dev/serial/by-id/usb-Klipper_stm32f427xx_*-if00)"
echo "log:   /tmp/klipper-mcu-recover.log"
