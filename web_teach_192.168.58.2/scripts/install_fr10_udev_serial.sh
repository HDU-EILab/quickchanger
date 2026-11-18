#!/usr/bin/env bash
# Install udev rules for stable /dev/fr10-*-serial symlinks (see udev/99-fr10-usb-serial.rules).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
RULE_SRC="$ROOT/udev/99-fr10-usb-serial.rules"
RULE_DST="/etc/udev/rules.d/99-fr10-usb-serial.rules"
test -f "$RULE_SRC"
sudo cp -a "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
echo "Installed $RULE_DST — use /dev/fr10-motor-serial and /dev/fr10-hand-serial"
