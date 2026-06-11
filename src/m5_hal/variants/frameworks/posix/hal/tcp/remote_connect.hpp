#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_REMOTE_CONNECT_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_REMOTE_CONNECT_HPP

// Remote-bus connection establishment over TCP: the same shape as the
// serial utility (uart/remote_connect.hpp) on a different transport.
// TCP already addresses one specific peer, so there is no candidate
// enumeration — connect, then let hello decide whether the peer speaks
// the remote protocol (spec/design/remote.md §接続ユーティリティ).

#if M5HAL_FRAMEWORK_HAS_POSIX

#include "../../../../../hal/v1/remote/remote.hpp"
#include "./tcp.hpp"

namespace m5::variants::frameworks::posix::hal::v1::tcp {

constexpr size_t kTcpEndpointCapacity = 128;  ///< max "host:port" length (incl. NUL).

/*!
  @brief Owning bundle for one TCP remote endpoint: the socket stream
         and the transport-agnostic `RemoteLink`.

  After `connectRemoteTcp` succeeds, `link.session()` is the established
  session and `peer()` names the endpoint.
 */
struct TcpRemoteEndpoint {
    TcpStream stream;
    ::m5::hal::v1::remote::RemoteLink link;

    explicit TcpRemoteEndpoint(const ::m5::hal::v1::remote::RemoteSession::Config& session_cfg =
                                   ::m5::hal::v1::remote::RemoteSession::Config{})
        : link{stream, stream, session_cfg}
    {
    }

    /*! @brief Endpoint of the last successful `connectRemoteTcp` ("" before that). */
    const char* peer() const
    {
        return _peer;
    }

    // Written by connectRemoteTcp.
    char _peer[kTcpEndpointCapacity] = {};
};

struct TcpConnectOptions {
    uint32_t connect_timeout_ms = 3000;
    /*! @brief Per-attempt hello timeout while probing (the session's configured timeout is restored after). */
    uint32_t hello_timeout_ms = 500;
    /*! @brief hello attempts once connected. No connect-time board reset
        exists on TCP (the device is already up and listening), so a couple
        of tries only bridges a serve loop that is momentarily busy. */
    int attempts = 2;
};

/*!
  @brief Establish a remote-bus session over TCP.

  `endpoint` is `"host:port"` — a DNS name, an IPv4 address, or a
  bracketed IPv6 address (`"[fe80::1]:5555"`). Connects (bounded by
  `opt.connect_timeout_ms`), resets the link, and attempts hello; a
  fresh socket carries no stale bytes, so unlike the serial utility no
  transport-level flush is involved.

  Returns the capabilities on success (also cached in the session).
  Errors: `INVALID_ARGUMENT` for a malformed endpoint; `IO_ERROR` when
  the endpoint could not be reached (refused / unreachable / connect
  timeout); `TIMEOUT_ERROR` when connected but nothing answered hello
  (the socket is closed again before returning).
 */
m5::stl::expected<::m5::hal::v1::remote::Capabilities, ::m5::hal::v1::error::error_t> connectRemoteTcp(
    TcpRemoteEndpoint& ep, const char* endpoint, const TcpConnectOptions& opt = TcpConnectOptions{});

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
