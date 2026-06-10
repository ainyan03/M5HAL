#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_REMOTE_CONNECT_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_REMOTE_CONNECT_INL

#include "./remote_connect.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace m5::variants::frameworks::posix::hal::v1::uart {

namespace {

using error_t = ::m5::hal::v1::error::error_t;

// Deterministic run-mode reset over the modem lines (ESP32 auto-reset
// wiring: RTS -> EN, DTR -> IO0; both lines through a transistor pair).
// Releasing DTR keeps IO0 high (run mode), then an RTS pulse cycles EN:
// the board reboots into the application even if the open()-time
// DTR/RTS race had parked it in the ROM bootloader. Best-effort: a port
// without modem lines (pty, some adapters) just fails the ioctls.
void hardwareResetIntoRun(int fd)
{
    int dtr = TIOCM_DTR;
    int rts = TIOCM_RTS;
    (void)::ioctl(fd, TIOCMBIC, &dtr);  // IO0 high = run mode
    (void)::ioctl(fd, TIOCMBIS, &rts);  // EN low   = reset asserted
    ::usleep(100 * 1000);
    (void)::ioctl(fd, TIOCMBIC, &rts);  // EN high  = boot
}

// One candidate: open, optionally reset the board, then reset the
// session and try hello `attempts` times.
bool probeCandidate(SerialRemoteEndpoint& ep, const char* path, uint32_t baud, const ConnectOptions& opt, int attempts,
                    ::m5::hal::v1::remote::Capabilities& out_caps, bool& out_opened)
{
    if (opt.on_attempt != nullptr) {
        opt.on_attempt(opt.on_attempt_ctx, path);
    }
    out_opened = ep.bus.open(path, baud) == error_t::OK;
    if (!out_opened) {
        return false;
    }
    if (opt.hardware_reset) {
        hardwareResetIntoRun(ep.bus.nativeHandle());
    }
    auto& session = ep.link.session();
    for (int a = 0; a < attempts; ++a) {
        // Full flush before every attempt: the reset makes the peer
        // stream its ROM boot log, and stale bytes left in either the
        // OS queue or the Stream adapter would desynchronize the frame
        // reader right when the real hello_resp arrives.
        (void)::tcflush(ep.bus.nativeHandle(), TCIFLUSH);
        ep.link.reset();
        auto caps = session.hello();
        if (caps.has_value()) {
            out_caps = caps.value();
            return true;
        }
    }
    (void)ep.bus.release();
    return false;
}

}  // namespace

m5::stl::expected<::m5::hal::v1::remote::Capabilities, error_t> connectRemoteSerial(SerialRemoteEndpoint& ep,
                                                                                    const ConnectOptions& opt)
{
    const uint32_t baud = ep.rx.getConfig().baud_rate;
    auto& session       = ep.link.session();

    // Probe with the short hello timeout; the configured value comes
    // back once the connection is (or is not) established.
    const uint32_t saved_timeout = session.responseTimeoutMs();
    session.setResponseTimeout(opt.hello_timeout_ms);

    ::m5::hal::v1::remote::Capabilities caps;
    bool connected     = false;
    bool any_opened    = false;
    ep._device_path[0] = '\0';

    if (opt.path != nullptr) {
        bool opened = false;
        connected   = probeCandidate(ep, opt.path, baud, opt, opt.strong_attempts, caps, opened);
        any_opened  = opened;
        if (connected) {
            ::snprintf(ep._device_path, sizeof(ep._device_path), "%s", opt.path);
        }
    } else {
        SerialPortInfo ports[8];
        const size_t n = listSerialPorts(ports, sizeof(ports) / sizeof(ports[0]));
        for (size_t i = 0; i < n && !connected; ++i) {
            if (ports[i].rank > opt.max_rank) {
                continue;
            }
            const int attempts = ports[i].rank == 0 ? opt.strong_attempts : opt.weak_attempts;
            bool opened        = false;
            connected          = probeCandidate(ep, ports[i].path, baud, opt, attempts, caps, opened);
            any_opened |= opened;
            if (connected) {
                ::snprintf(ep._device_path, sizeof(ep._device_path), "%s", ports[i].path);
            }
        }
    }

    session.setResponseTimeout(saved_timeout);
    if (!connected) {
        return m5::stl::make_unexpected(any_opened ? error_t::TIMEOUT_ERROR : error_t::IO_ERROR);
    }
    return caps;
}

}  // namespace m5::variants::frameworks::posix::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
