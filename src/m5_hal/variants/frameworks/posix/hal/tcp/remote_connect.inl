// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_REMOTE_CONNECT_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_REMOTE_CONNECT_INL

#include "./remote_connect.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace m5::variants::frameworks::posix::hal::v1::tcp {

namespace {

using error_t = ::m5::hal::v1::error::error_t;

// Split "host:port" / "[v6addr]:port" into a host copy and a port.
// The bracket form is how IPv6 literals keep their own colons apart
// from the port separator; bare forms split at the LAST colon.
bool parseEndpoint(const char* endpoint, char* host, size_t host_cap, uint16_t& out_port)
{
    if (endpoint == nullptr) {
        return false;
    }
    const char* host_begin = endpoint;
    const char* host_end   = nullptr;
    const char* port_begin = nullptr;
    if (endpoint[0] == '[') {
        const char* closing = ::strchr(endpoint, ']');
        if (closing == nullptr || closing[1] != ':') {
            return false;
        }
        host_begin = endpoint + 1;
        host_end   = closing;
        port_begin = closing + 2;
    } else {
        const char* colon = ::strrchr(endpoint, ':');
        if (colon == nullptr) {
            return false;
        }
        host_end   = colon;
        port_begin = colon + 1;
    }
    const size_t host_len = static_cast<size_t>(host_end - host_begin);
    if (host_len == 0 || host_len >= host_cap || port_begin[0] == '\0') {
        return false;
    }
    char* parse_end          = nullptr;
    const unsigned long port = ::strtoul(port_begin, &parse_end, 10);
    if (parse_end == port_begin || *parse_end != '\0' || port == 0 || port > 65535) {
        return false;
    }
    ::memcpy(host, host_begin, host_len);
    host[host_len] = '\0';
    out_port       = static_cast<uint16_t>(port);
    return true;
}

}  // namespace

result_t<::m5::hal::v1::remote::Capabilities> connectRemoteTcp(TcpRemoteEndpoint& ep, const char* endpoint,
                                                               const TcpConnectOptions& opt)
{
    char host[kTcpEndpointCapacity];
    uint16_t port = 0;
    if (!parseEndpoint(endpoint, host, sizeof(host), port)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    ep._peer[0] = '\0';

    // "Reached" vs "answered" stay two distinct buckets (the serial
    // utility's semantics): any connect failure folds into IO_ERROR.
    if (ep.stream.connect(host, port, opt.connect_timeout_ms) != error_t::OK) {
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }

    auto& session = ep.link.session();
    // Probe with the short hello timeout; the configured value comes
    // back once the connection is (or is not) established.
    const uint32_t saved_timeout = session.responseTimeoutMs();
    session.setResponseTimeout(opt.hello_timeout_ms);

    ::m5::hal::v1::remote::Capabilities caps;
    bool connected = false;
    for (int a = 0; a < opt.attempts && !connected; ++a) {
        ep.link.reset();
        auto answered = session.hello();
        if (answered.has_value()) {
            caps      = answered.value();
            connected = true;
        }
    }

    session.setResponseTimeout(saved_timeout);
    if (!connected) {
        ep.stream.close();
        return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
    }
    ::snprintf(ep._peer, sizeof(ep._peer), "%s", endpoint);
    return caps;
}

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
