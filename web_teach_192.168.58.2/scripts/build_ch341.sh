#!/usr/bin/env bash
# Build Linux mainline ch341.c (v5.15) as out-of-tree module for CH340/CH341 USB-serial.
# Jetson L4T 5.15.148-tegra ships with CONFIG_USB_SERIAL_CH341 unset; this supplies the driver.
set -euo pipefail
KVER="$(uname -r)"
BUILD="${CH341_BUILD_DIR:-/tmp/ch341-oot}"
LINUX_TAG="${CH341_LINUX_TAG:-v5.15}"
SRC_URL="https://raw.githubusercontent.com/torvalds/linux/${LINUX_TAG}/drivers/usb/serial/ch341.c"

if [[ ! -d "/lib/modules/${KVER}/build" ]]; then
  echo "Missing /lib/modules/${KVER}/build. Install nvidia-l4t-kernel-headers." >&2
  exit 1
fi

mkdir -p "$BUILD"
curl -fsSL "$SRC_URL" -o "$BUILD/ch341.c"
cat >"$BUILD/Makefile" <<'EOF'
obj-m += ch341.o
EOF

make -C "/lib/modules/${KVER}/build" M="$BUILD" modules
echo "Built: $BUILD/ch341.ko"
echo "Load once:    sudo insmod $BUILD/ch341.ko"
echo "Then check:   ls -l /dev/ttyUSB* /dev/serial/by-id"
echo "Install boot: sudo mkdir -p /lib/modules/${KVER}/updates && sudo cp $BUILD/ch341.ko /lib/modules/${KVER}/updates/ && sudo depmod -a && echo ch341 | sudo tee /etc/modules-load.d/ch341.conf >/dev/null"
