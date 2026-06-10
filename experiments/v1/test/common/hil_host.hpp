// SPDX-License-Identifier: MIT
//
// experiments/v1/test/common/hil_host.hpp
//
// Shared host-side harness for HIL (hardware-in-the-loop) experiment-tests: a
// native host process drives a real device over a serial link through the
// M5HAL posix UART variant, and judges the device by what comes back.
//
// HIL tests live under experiments/v1/test/<name>/{device,host}/ and run via
// `pio run` + executing the built host binary (NOT `pio test`, whose discovery
// is locked to test_dir). See experiments/v1/test/README.md.
//
// Reusable pieces:
//   - env helpers:    hil::portEnv(), hil::baudEnv()
//   - link bring-up:  hil::openSynced() — open a port, drop DTR/RTS to avoid the
//                     USB-bridge auto-reset, and wait for the device to boot and
//                     echo a sync marker (so a reset-on-open + ROM boot log is
//                     absorbed before the real assertions run).
//   - stream helpers: hil::drain(), hil::readExact(), hil::ioConfig()
//
// New HIL host drivers reuse these so each driver stays thin.

#ifndef M5HAL_EXPERIMENTS_TEST_COMMON_HIL_HOST_HPP_
#define M5HAL_EXPERIMENTS_TEST_COMMON_HIL_HOST_HPP_

#include <M5HAL_v1.hpp>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace hil {

namespace uart  = ::m5::hal::v1::uart;
namespace data  = ::m5::hal::v1::data;
namespace error = ::m5::hal::v1::error;

// Serial device path for the device under test (e.g. /dev/cu.usbserial-XXXX).
// nullptr when unset — a host driver should skip in that case.
inline const char* portEnv()
{
    return ::getenv("M5HAL_POSIX_UART_PORT");
}

// Link baud; M5HAL_POSIX_UART_BAUD overrides, else `fallback`.
inline uint32_t baudEnv(uint32_t fallback = 115200)
{
    const char* b = ::getenv("M5HAL_POSIX_UART_BAUD");
    if (b == nullptr) {
        return fallback;
    }
    long v = ::strtol(b, nullptr, 10);
    return v > 0 ? static_cast<uint32_t>(v) : fallback;
}

// A timing profile generous enough to cover the echo round-trip latency.
inline uart::UARTAccessConfig ioConfig(uint32_t baud)
{
    uart::UARTAccessConfig cfg;
    cfg.baud_rate             = baud;
    cfg.first_byte_timeout_ms = 300;
    cfg.inter_byte_timeout_ms = 50;
    cfg.write_timeout_ms      = 500;
    return cfg;
}

// Accumulate exactly `len` bytes (the peer may deliver them in chunks). Returns
// the count actually read (== len on success).
inline size_t readExact(uart::UARTRxAccessor& dev, uint8_t* dst, size_t len, int max_reads)
{
    size_t got = 0;
    for (int i = 0; i < max_reads && got < len; ++i) {
        auto r = dev.read(dst + got, len - got);
        if (!r.has_value()) {
            break;
        }
        if (r.value() == 0) {
            continue;  // one timeout window elapsed; keep waiting
        }
        got += r.value();
    }
    return got;
}

// Read and discard everything pending until the line is idle for one timeout.
// Clears boot-ROM noise / stale bytes before a clean exchange.
inline void drain(uart::UARTRxAccessor& dev)
{
    uint8_t scratch[256];
    for (int i = 0; i < 64; ++i) {
        auto n = dev.read(scratch, sizeof(scratch));
        if (!n.has_value() || n.value() == 0) {
            break;
        }
    }
}

// Open `port` at `baud` and confirm the device is up: drop DTR/RTS (reduce the
// CP2104/CH340 auto-reset on open), then round-trip a distinctive sync marker,
// retrying for a few seconds. This absorbs a reset-on-open (ROM boot log at a
// different baud is read as garbage, drained, then the app's echo replies
// cleanly). Returns true once the device echoes the marker. `bus` must outlive
// the test that uses it.
inline bool openSynced(uart::Bus& bus, const char* port, uint32_t baud)
{
    if (bus.open(port, baud) != error::error_t::OK) {
        return false;
    }
    int fd = bus.nativeHandle();
    if (fd >= 0) {
        int bits = TIOCM_DTR | TIOCM_RTS;
        (void)::ioctl(fd, TIOCMBIC, &bits);
    }

    uart::UARTAccessConfig sync_cfg;
    sync_cfg.baud_rate             = baud;
    sync_cfg.first_byte_timeout_ms = 150;
    sync_cfg.inter_byte_timeout_ms = 30;
    sync_cfg.write_timeout_ms      = 300;
    uart::UARTTxAccessor tx{bus, sync_cfg};
    uart::UARTRxAccessor rx_dev{bus, sync_cfg};

    static const uint8_t marker[] = {0xA5, 0x5A, 0xC3, 0x3C};
    for (int attempt = 0; attempt < 24; ++attempt) {
        drain(rx_dev);
        if (!tx.write(data::ConstDataSpan{marker, sizeof(marker)}).has_value()) {
            continue;
        }
        uint8_t echoed[sizeof(marker)] = {};
        size_t got                     = readExact(rx_dev, echoed, sizeof(marker), 6);
        if (got == sizeof(marker) && ::memcmp(echoed, marker, sizeof(marker)) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace hil

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif  // M5HAL_EXPERIMENTS_TEST_COMMON_HIL_HOST_HPP_
