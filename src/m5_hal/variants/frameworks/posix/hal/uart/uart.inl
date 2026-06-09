#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_INL

#include "uart.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#if defined(__APPLE__)
// IOSSIOSPEED: set an arbitrary (non-B*) baud on macOS, whose termios only has
// B* constants up to B230400. Lets the variant reach 0.5/1/2/3 Mbaud etc.
#include <IOKit/serial/ioss.h>
#endif

namespace m5::variants::frameworks::posix::hal::v1::uart {

namespace {

// Map a numeric baud rate to a termios speed_t, returning false if this libc
// has no B* constant for it. Rates above the POSIX-standard set are #ifdef
// -guarded so the file compiles everywhere. Linux glibc/musl define
// B460800..B4000000, so those high rates take this path there. macOS termios
// stops at B230400; higher rates return false here and applyConfig() sets them
// via the IOSSIOSPEED ioctl instead (validated to 3 Mbaud against real hardware
// in LovyanAPI). Single source of truth for the table; Bus::baudToSpeed() is a
// thin public wrapper over it for unit tests.
bool baudConstant(uint32_t baud, speed_t& out)
{
    switch (baud) {
        case 1200:
            out = B1200;
            return true;
        case 2400:
            out = B2400;
            return true;
        case 4800:
            out = B4800;
            return true;
        case 9600:
            out = B9600;
            return true;
        case 19200:
            out = B19200;
            return true;
        case 38400:
            out = B38400;
            return true;
        case 57600:
            out = B57600;
            return true;
        case 115200:
            out = B115200;
            return true;
        case 230400:
            out = B230400;
            return true;
#ifdef B460800
        case 460800:
            out = B460800;
            return true;
#endif
#ifdef B500000
        case 500000:
            out = B500000;
            return true;
#endif
#ifdef B576000
        case 576000:
            out = B576000;
            return true;
#endif
#ifdef B921600
        case 921600:
            out = B921600;
            return true;
#endif
#ifdef B1000000
        case 1000000:
            out = B1000000;
            return true;
#endif
#ifdef B1152000
        case 1152000:
            out = B1152000;
            return true;
#endif
#ifdef B1500000
        case 1500000:
            out = B1500000;
            return true;
#endif
#ifdef B2000000
        case 2000000:
            out = B2000000;
            return true;
#endif
#ifdef B2500000
        case 2500000:
            out = B2500000;
            return true;
#endif
#ifdef B3000000
        case 3000000:
            out = B3000000;
            return true;
#endif
#ifdef B3500000
        case 3500000:
            out = B3500000;
            return true;
#endif
#ifdef B4000000
        case 4000000:
            out = B4000000;
            return true;
#endif
        default:
            return false;
    }
}

bool sameConfig(const ::m5::hal::v1::uart::UARTAccessConfig& lhs, const ::m5::hal::v1::uart::UARTAccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

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

}  // namespace

bool Bus::baudToSpeed(uint32_t baud, uint32_t& out_speed)
{
    speed_t s = 0;
    if (!baudConstant(baud, s)) {
        return false;
    }
    out_speed = static_cast<uint32_t>(s);
    return true;
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::init(const ::m5::hal::v1::bus::BusConfig& config)
{
    if (config.getBusKind() != ::m5::hal::v1::types::bus_kind_t::UART) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    (void)release();
    const auto& uart_config = static_cast<const BusConfig&>(config);
    _config                 = uart_config;
    _device_path            = uart_config.device_path;  // termios open is lazy (first write/read)
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::release(void)
{
    if (_owns_fd && _fd >= 0) {
        ::close(_fd);
    }
    _fd      = -1;
    _owns_fd = false;
    _begun   = false;
    return {};
}

::m5::hal::v1::error::error_t Bus::open(const char* device_path, uint32_t baud)
{
    (void)release();
    if (device_path == nullptr) {
        return ::m5::hal::v1::error::error_t::INVALID_ARGUMENT;
    }
    int fd = ::open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return ::m5::hal::v1::error::error_t::UNKNOWN_ERROR;
    }
    _fd      = fd;
    _owns_fd = true;
    _begun   = false;

    ::m5::hal::v1::uart::UARTAccessConfig cfg;
    cfg.baud_rate = baud;
    auto applied  = applyConfig(cfg);
    if (!applied.has_value()) {
        (void)release();
        return applied.error();
    }
    return ::m5::hal::v1::error::error_t::OK;
}

::m5::hal::v1::error::error_t Bus::attach(int fd)
{
    (void)release();
    _fd      = fd;
    _owns_fd = false;  // caller keeps ownership of the descriptor
    _begun   = false;

    // Configure the line to raw immediately (symmetric with open()), so a peer
    // that writes before our first read sees a raw — not canonical — slave and
    // the bytes are delivered rather than line-buffered. The real per-access
    // baud/format is re-applied on the first write/read if it differs.
    ::m5::hal::v1::uart::UARTAccessConfig cfg;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return applied.error();
    }
    return ::m5::hal::v1::error::error_t::OK;
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::applyConfig(
    const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    if (cfg.baud_rate == 0 || cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    // Lazily open the configured device when no fd has been adopted yet.
    if (_fd < 0) {
        if (_device_path == nullptr) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        int fd = ::open(_device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        _fd      = fd;
        _owns_fd = true;
        _begun   = false;
    }

    if (_begun && sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    speed_t speed          = 0;
    const bool have_bconst = baudConstant(cfg.baud_rate, speed);
#if !defined(__APPLE__)
    // Linux/other: only rates that have a termios B* constant are supported.
    // glibc/musl provide B460800..B4000000, covering the standard high rates.
    if (!have_bconst) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
#endif

    struct termios tio;
    if (::tcgetattr(_fd, &tio) != 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
    }
    ::cfmakeraw(&tio);
    // A rate without a B* constant (macOS high baud) gets a placeholder here; the
    // real integer speed is applied via IOSSIOSPEED after tcsetattr() below.
    const speed_t cfspeed = have_bconst ? speed : static_cast<speed_t>(B9600);
    ::cfsetispeed(&tio, cfspeed);
    ::cfsetospeed(&tio, cfspeed);

    tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    tio.c_cflag |= static_cast<tcflag_t>(CS8);
    if (cfg.stop_bits == 2) {
        tio.c_cflag |= static_cast<tcflag_t>(CSTOPB);
    } else {
        tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    }
    switch (cfg.parity) {
        case ::m5::hal::v1::uart::parity_t::none:
            tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);
            break;
        case ::m5::hal::v1::uart::parity_t::even:
            tio.c_cflag |= static_cast<tcflag_t>(PARENB);
            tio.c_cflag &= ~static_cast<tcflag_t>(PARODD);
            break;
        case ::m5::hal::v1::uart::parity_t::odd:
            tio.c_cflag |= static_cast<tcflag_t>(PARENB);
            tio.c_cflag |= static_cast<tcflag_t>(PARODD);
            break;
        default:
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    // Non-blocking semantics; timeouts are enforced by waitFd()/select().
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(_fd, TCSANOW, &tio) != 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
    }
#if defined(__APPLE__)
    // Set a baud that termios has no B* constant for (e.g. 0.5/1/2/3 Mbaud).
    // Must follow tcsetattr(), which would otherwise reset the line speed.
    if (!have_bconst) {
        speed_t real = static_cast<speed_t>(cfg.baud_rate);
        if (::ioctl(_fd, IOSSIOSPEED, &real) != 0) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
    }
#endif
    // Ensure the descriptor is non-blocking even when it was adopted via attach().
    int flags = ::fcntl(_fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(_fd, F_SETFL, flags | O_NONBLOCK);
    }
    // Deliberately no tcflush() here: discarding pending RX would drop bytes a
    // peer already sent before this (re)configure. Stale-input draining, if
    // ever wanted, belongs to the caller.

    _applied_cfg = cfg;
    _begun       = true;
    return {};
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::write(::m5::hal::v1::bus::Accessor* owner,
                                                                    const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                    ::m5::hal::v1::data::Source* tx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    size_t done = 0;
    while (tx != nullptr && !tx->eof() && done < len) {
        auto span = tx->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        ssize_t n = ::write(_fd, span.value().data, span.value().size);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (waitFd(_fd, true, cfg.write_timeout_ms) > 0) {
                    continue;
                }
                break;  // write timeout
            }
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        if (n == 0) {
            break;
        }
        auto advanced = tx->advance(static_cast<size_t>(n));
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += static_cast<size_t>(n);
    }
    // No tcdrain(): the bytes are handed to the OS write buffer here, and
    // draining the line discipline can block indefinitely on a pty (the
    // loopback used by tests). Flushing the physical UART FIFO is the OS
    // driver's job; callers that need a hard drain can add one out of band.
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::read(::m5::hal::v1::bus::Accessor* owner,
                                                                   const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                   ::m5::hal::v1::data::Sink* rx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    size_t done = 0;
    while (rx != nullptr && !rx->closed() && done < len) {
        // first byte vs. inter-byte gap timeout (mirrors the arduino variant).
        const uint32_t timeout = (done == 0) ? cfg.first_byte_timeout_ms : cfg.inter_byte_timeout_ms;
        int ready              = waitFd(_fd, false, timeout);
        if (ready < 0) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        if (ready == 0) {
            break;  // timed out waiting for (more) data
        }
        auto span = rx->reserve(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        ssize_t n = ::read(_fd, span.value().data, span.value().size);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        if (n == 0) {
            break;  // EOF / hang-up
        }
        auto committed = rx->commit(static_cast<size_t>(n));
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        done += static_cast<size_t>(n);
    }
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::readableBytes(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    int avail = 0;
    if (::ioctl(_fd, FIONREAD, &avail) != 0 || avail < 0) {
        return static_cast<size_t>(0);
    }
    return static_cast<size_t>(avail);
}

}  // namespace m5::variants::frameworks::posix::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
