// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_HPP

// Device-side TCP transport for the remote bus. The stream / listener
// bodies are the shared BSD-socket detail (_detail/bsd_tcp.hpp —
// rationale, stream semantics and socket options documented there);
// lwIP serves the BSD API through the ESP-IDF VFS, so the SAME
// implementation backs both the espidf and the arduino-esp32 framework
// — the framework difference is the WiFi bring-up, which is example
// responsibility (decisions/024). The host (PC) sibling lives in the
// posix variant; the device side is the server, so this header exposes
// listen / accept / attach and deliberately no client connect.
//
// WiFi credentials / network bring-up never enter the HAL.

#if defined(ESP_PLATFORM) && __has_include(<sys/socket.h>)

#include "../../../../../_detail/bsd_tcp.hpp"

namespace m5::variants::frameworks::espidf::hal::v1::tcp {

using namespace ::m5::hal::v1;

using TcpStream   = ::m5::hal::v1::detail::BsdTcpStream;
using TcpListener = ::m5::hal::v1::detail::BsdTcpListener;

}  // namespace m5::variants::frameworks::espidf::hal::v1::tcp

#endif  // ESP_PLATFORM && <sys/socket.h>

#endif
