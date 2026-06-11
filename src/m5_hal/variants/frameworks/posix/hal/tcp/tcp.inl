#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_INL

#include "tcp.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace m5::variants::frameworks::posix::hal::v1::tcp {

namespace {

using error_t = ::m5::hal::v1::error::error_t;

// SIGPIPE suppression on write-after-hangup: Linux has the per-send
// flag, macOS/BSD have the per-socket option (set in configureSocket).
#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

// Block until `fd` is readable / writable for up to `timeout_ms`.
// Returns >0 when ready, 0 on timeout, <0 on error.
int waitFd(int fd, bool for_write, uint32_t timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(timeout_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
    int r;
    do {
        r = ::select(fd + 1, for_write ? nullptr : &fds, for_write ? &fds : nullptr, nullptr, &tv);
    } while (r < 0 && errno == EINTR);
    return r;
}

uint64_t monotonicMs()
{
    struct timespec ts;
    (void)::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000u + static_cast<uint64_t>(ts.tv_nsec) / 1000000u;
}

}  // namespace

::m5::hal::v1::error::error_t TcpStream::configureSocket(int fd)
{
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        return error_t::IO_ERROR;
    }
    // Best-effort socket options: a transport that cannot set them still
    // works, just slower (Nagle) — and SIGPIPE is also masked per send.
    int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
#if defined(SO_NOSIGPIPE)
    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
    return error_t::OK;
}

::m5::hal::v1::error::error_t TcpStream::connect(const char* host, uint16_t port, uint32_t timeout_ms)
{
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

::m5::hal::v1::error::error_t TcpStream::attach(int fd, bool take_ownership)
{
    close();
    if (fd < 0) {
        return error_t::INVALID_ARGUMENT;
    }
    auto configured = configureSocket(fd);
    if (configured != error_t::OK) {
        if (take_ownership) {
            ::close(fd);
        }
        return configured;
    }
    _fd      = fd;
    _owns_fd = take_ownership;
    return error_t::OK;
}

void TcpStream::close()
{
    if (_owns_fd && _fd >= 0) {
        ::close(_fd);
    }
    _fd      = -1;
    _owns_fd = false;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> TcpStream::read(data::DataSpan dst)
{
    if (_fd < 0) {
        return m5::stl::make_unexpected(error_t::CLOSED);
    }
    if (dst.data == nullptr || dst.size == 0) {
        return static_cast<size_t>(0);
    }
    int ready = waitFd(_fd, false, read_timeout_ms);
    if (ready < 0) {
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }
    if (ready == 0) {
        return static_cast<size_t>(0);  // nothing arrived in time
    }
    for (;;) {
        ssize_t n = ::recv(_fd, dst.data, dst.size, 0);
        if (n > 0) {
            return static_cast<size_t>(n);
        }
        if (n == 0) {
            // Orderly peer shutdown — a state a serial line can never
            // report. Latch it: every later call answers CLOSED too.
            close();
            return m5::stl::make_unexpected(error_t::CLOSED);
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return static_cast<size_t>(0);  // spurious readiness
        }
        close();
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> TcpStream::readableBytes(void)
{
    if (_fd < 0) {
        return m5::stl::make_unexpected(error_t::CLOSED);
    }
    int avail = 0;
    if (::ioctl(_fd, FIONREAD, &avail) != 0 || avail < 0) {
        return static_cast<size_t>(0);
    }
    return static_cast<size_t>(avail);
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> TcpStream::write(data::ConstDataSpan src)
{
    if (_fd < 0) {
        return m5::stl::make_unexpected(error_t::CLOSED);
    }
    size_t done = 0;
    while (done < src.size) {
        ssize_t n = ::send(_fd, src.data + done, src.size - done, kSendFlags);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (waitFd(_fd, true, write_timeout_ms) > 0) {
                    continue;
                }
                break;  // write timeout: report the short write
            }
            if (errno == EPIPE || errno == ECONNRESET) {
                close();
                return m5::stl::make_unexpected(error_t::CLOSED);
            }
            return m5::stl::make_unexpected(error_t::IO_ERROR);
        }
        done += static_cast<size_t>(n);
    }
    return done;
}

::m5::hal::v1::error::error_t TcpListener::listen(uint16_t port, const char* bind_addr, int backlog)
{
    close();
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return error_t::IO_ERROR;
    }
    int one = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {};
    addr.sin_family         = AF_INET;
    addr.sin_port           = htons(port);
    addr.sin_addr.s_addr    = htonl(INADDR_ANY);
    if (bind_addr != nullptr && ::inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1) {
        ::close(fd);
        return error_t::INVALID_ARGUMENT;
    }
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(fd, backlog) != 0) {
        ::close(fd);
        return error_t::IO_ERROR;
    }
    // Resolve a port-0 (ephemeral) request to the actual port.
    struct sockaddr_in bound = {};
    socklen_t len            = sizeof(bound);
    if (::getsockname(fd, reinterpret_cast<struct sockaddr*>(&bound), &len) != 0) {
        ::close(fd);
        return error_t::IO_ERROR;
    }
    // Non-blocking so accept() can be paced by select() alone.
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    _fd   = fd;
    _port = ntohs(bound.sin_port);
    return error_t::OK;
}

int TcpListener::accept(uint32_t timeout_ms)
{
    if (_fd < 0) {
        return -1;
    }
    if (waitFd(_fd, false, timeout_ms) <= 0) {
        return -1;
    }
    int fd;
    do {
        fd = ::accept(_fd, nullptr, nullptr);
    } while (fd < 0 && errno == EINTR);
    return fd;
}

void TcpListener::close()
{
    if (_fd >= 0) {
        ::close(_fd);
    }
    _fd   = -1;
    _port = 0;
}

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
