#!/usr/bin/env bash
# Runs on KV260, as root, only after locally verified package transfer.
set -euo pipefail
cd "$(dirname "$0")"
[[ "$(uname -m)" == aarch64 && "$(id -u)" == 0 ]] || { echo 'Run on KV260 using sudo.' >&2; exit 2; }
sha256sum --check SHA256SUMS
[[ -d /proc/device-tree/fpga-full ]] || { echo 'Expected Kria FPGA region missing' >&2; exit 2; }
app=kv260-can
destination="/lib/firmware/xilinx/$app"
if [[ -e "$destination" ]]; then
 cp -a "$destination" "$destination.backup-$(date +%Y%m%d_%H%M%S)"
fi
install -d "$destination"
install -m 644 kv260-can.bit.bin kv260-can.dtbo shell.json "$destination/"
# dfx-mgr removes the old overlay before loading matching firmware and UIO node.
# The host already verified a healthy boot; no JTAG/PS initialization is used.
xmutil listapps
xmutil unloadapp
xmutil loadapp "$app"
[[ "$(cat /sys/class/fpga_manager/fpga0/state)" == operating ]]
gcc -O2 -Wall -Wextra -Werror -shared -fPIC mmio.c -o libkv260_mmio.so
/usr/bin/python3 canctl.py status | tee board_status_before.json
/usr/bin/python3 canctl.py selftest | tee board_selftest.json
# Grant only the named, newly loaded control device to the current board user.
# This is a session permission, not a permanent sudo/udev policy change.
for dev in /sys/class/uio/uio*; do
 if [[ "$(cat "$dev/name")" == kv260-can ]]; then
  chown ubuntu:ubuntu "/dev/$(basename "$dev")"
  chmod 600 "/dev/$(basename "$dev")"
 fi
done
printf '%s\n' 'PASS: firmware loaded; internal loopback completed; external CAN TX disabled.'
