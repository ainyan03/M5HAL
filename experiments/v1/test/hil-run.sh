#!/usr/bin/env bash
#
# hil-run.sh — run a HIL (hardware-in-the-loop) experiment-test end to end:
# flash the device firmware, build the host driver, then execute the host driver
# against the connected serial port.
#
#   experiments/v1/test/hil-run.sh <name> [port] [baud]
#     <name>  a HIL test under experiments/v1/test/<name>/ (e.g. uart_echo)
#     port    serial device (default: first /dev/cu.usbserial-* / ttyUSB* found)
#     baud    link baud (default: 115200)
#
# Materializes pio_envs/v1/hil.ini.cli -> hil.ini for the run and removes it
# afterwards. Requires the device + host envs v1_hil_<name>_device_esp32 and
# v1_hil_<name>_host (see pio_envs/v1/hil.ini.cli).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"  # experiments/v1/test -> repo root

name="${1:-}"
port="${2:-}"
baud="${3:-115200}"

if [ -z "$name" ]; then
    echo "usage: experiments/v1/test/hil-run.sh <name> [port] [baud]" >&2
    echo "  e.g. experiments/v1/test/hil-run.sh uart_echo /dev/cu.usbserial-XXXX 3000000" >&2
    exit 2
fi
if [ ! -d "$REPO/experiments/v1/test/$name/host" ]; then
    echo "no HIL test '$name' (expected $REPO/experiments/v1/test/$name/)" >&2
    exit 2
fi
if [ -z "$port" ]; then
    port="$(ls /dev/cu.usbserial-* /dev/cu.wchusbserial* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 || true)"
    [ -n "$port" ] || { echo "no serial port found; pass one explicitly" >&2; exit 2; }
fi

dev_env="v1_hil_${name}_device_esp32"
host_env="v1_hil_${name}_host"
echo "HIL '$name'  port=$port  baud=$baud"

cd "$REPO"
# Load the HIL envs from their *.ini.cli via M5HAL_PIO_EXTRA_CONFIG (the
# ${sysenv.*} hook in platformio.ini) — no materialize/copy.
export M5HAL_PIO_EXTRA_CONFIG="pio_envs/v1/hil.ini.cli"

echo "=== flash device ($dev_env) @ ${baud} baud ==="
PLATFORMIO_BUILD_FLAGS="-DM5HAL_HIL_ECHO_BAUD=$baud" \
    pio run -e "$dev_env" -t upload --upload-port "$port"

echo "=== build host ($host_env) ==="
pio run -e "$host_env"

echo "=== run host driver against $port @ ${baud} baud ==="
M5HAL_POSIX_UART_PORT="$port" M5HAL_POSIX_UART_BAUD="$baud" ".pio/build/$host_env/program"
