// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_INL

#include "tcp.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX

#include "../../../../../_detail/bsd_tcp.inl"

#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

namespace m5::variants::frameworks::posix::hal::v1::tcp {

namespace {
using error_t = ::m5::hal::v1::error::error_t;
}  // namespace

::m5::hal::v1::error::error_t TcpStream::connect(const char* host, uint16_t port, uint32_t timeout_ms)
{
    using ::m5::hal::v1::detail::monotonicMs;
    using ::m5::hal::v1::detail::waitFd;

    close();
    if (host == nullptr || port == 0) {
        return error_t::INVALID_ARGUMENT;
    }

    char port_str[8];
    ::snprintf(port_str, sizeof(port_str), "%u", static_cast<unsigned>(port));
    struct addrinfo hints = {};
    hints.ai_family       = AF_UNSPEC;  // IPv4 and IPv6 alike
    hints.ai_socktype     = SOCK_STREAM;
    struct addrinfo* res  = nullptr;
    if (::getaddrinfo(host, port_str, &hints, &res) != 0 || res == nullptr) {
        return error_t::IO_ERROR;
    }

    const uint64_t deadline = monotonicMs() + timeout_ms;
    error_t result          = error_t::IO_ERROR;
    for (struct addrinfo* ai = res; ai != nullptr; ai = ai->ai_next) {
        int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (configureSocket(fd) != error_t::OK) {
            ::close(fd);
            continue;
        }
        int r;
        do {
            r = ::connect(fd, ai->ai_addr, static_cast<socklen_t>(ai->ai_addrlen));
        } while (r != 0 && errno == EINTR);
        if (r != 0 && errno == EINPROGRESS) {
            // Non-blocking connect: writability signals completion, then
            // SO_ERROR tells success from e.g. ECONNREFUSED.
            const uint64_t now      = monotonicMs();
            const uint32_t remained = now < deadline ? static_cast<uint32_t>(deadline - now) : 0;
            if (waitFd(fd, true, remained) > 0) {
                int err       = 0;
                socklen_t len = sizeof(err);
                if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0) {
                    r = 0;
                }
            } else {
                result = error_t::TIMEOUT_ERROR;
            }
        }
        if (r == 0) {
            _fd      = fd;
            _owns_fd = true;
            ::freeaddrinfo(res);
            return error_t::OK;
        }
        ::close(fd);
    }
    ::freeaddrinfo(res);
    return result;
}

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
