// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_HPP

// Host TCP transport for the remote bus. The stream / listener bodies
// are the shared BSD-socket detail (_detail/bsd_tcp.hpp — rationale,
// stream semantics and socket options documented there); this variant
// adds the client-only `connect`, because the host side is the one
// that dials out (the device sibling in the espidf variant stays
// server-only).
//
// Gated like the rest of the posix variant: active on POSIX host builds
// only (frameworks/_checker.hpp, M5HAL_FRAMEWORK_HAS_POSIX).

#if M5HAL_FRAMEWORK_HAS_POSIX

#include "../../../../../_detail/bsd_tcp.hpp"

namespace m5::variants::frameworks::posix::hal::v1::tcp {

using namespace ::m5::hal::v1;

/*!
  @brief One TCP connection as a byte stream (client or accepted side).

  `::m5::hal::v1::detail::BsdTcpStream` plus the client `connect`.
 */
class TcpStream : public ::m5::hal::v1::detail::BsdTcpStream {
public:
    /*!
      @brief Connect to `host:port` (numeric address or DNS name, IPv4/IPv6).
      @param timeout_ms bound on the whole attempt (resolve excluded: getaddrinfo
             has no portable timeout; numeric addresses never block there).
      Replaces any previously held socket.
     */
    ::m5::hal::v1::error::error_t connect(const char* host, uint16_t port, uint32_t timeout_ms = 3000);
};

using TcpListener = ::m5::hal::v1::detail::BsdTcpListener;

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
