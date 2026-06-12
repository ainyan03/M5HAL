// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_PORTS_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_PORTS_HPP

// Host serial-port enumeration for the POSIX UART variant. Lists the
// device paths that plausibly carry a serial peer, best candidates
// first, so a host tool can probe them in order instead of demanding a
// path argument. Selection heuristics are name-based only; the caller
// makes the final call by actually talking to the port (e.g. the remote
// `hello` exchange, spec/design/remote.md).

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#include <stddef.h>
#include <stdint.h>

namespace m5::variants::frameworks::posix::hal::v1::uart {

constexpr size_t kSerialPortPathCapacity = 128;

struct SerialPortInfo {
    char path[kSerialPortPathCapacity] = {};  ///< e.g. "/dev/cu.usbserial-XXXX".
    uint8_t rank                       = 0;   ///< 0 = USB serial, 1 = other serial, 2 = likely noise.
};

/*!
  @brief Rank a /dev entry name as a serial-port candidate.

  Pure name heuristic (unit-testable without a device):
  - macOS: `cu.usbserial*` / `cu.usbmodem*` -> 0; other `cu.*` -> 1,
    except known noise (`cu.Bluetooth*`, `cu.debug-console`,
    `cu.wlan-debug`) -> 2. `tty.*` is not a candidate (-1): the dial-in
    device can block on carrier detect and every port also has a `cu.`.
  - Linux: `ttyUSB*` / `ttyACM*` -> 0 (the by-id scan in
    `listSerialPorts` covers stable names separately).
  @return rank, or -1 when the name is not a candidate.
 */
int rankSerialPortName(const char* dev_name);

/*!
  @brief Enumerate serial-port candidates, best first.

  Scans `/dev` (and `/dev/serial/by-id/` on Linux, preferred when
  present) and fills `out` ordered by (rank ascending, name ascending).
  Returns the number of entries written (<= capacity); extra candidates
  beyond `capacity` are dropped from the tail (the weakest ones).
 */
size_t listSerialPorts(SerialPortInfo* out, size_t capacity);

}  // namespace m5::variants::frameworks::posix::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
