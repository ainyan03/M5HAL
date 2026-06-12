// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/uart/uart.hpp"

// Host POSIX UART variant: a real serial port driven through termios. It
// is offered (and flat-injected as the host UART) only on a POSIX host
// build; gated by M5HAL_FRAMEWORK_HAS_POSIX (frameworks/_checker.hpp,
// on for non-ESP/non-Arduino hosts) plus the M5HAL_CONFIG_POSIX_UART
// opt-out, which suppresses this UART kind only (the variant's runtime
// kind is unaffected).

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

namespace m5::hal::v1::uart {

// Variant-specific bus config: a host serial device path
// (e.g. "/dev/ttyUSB0", "/dev/tty.usbserial-XXXX") instead of MCU pins.
// Mirrors the arduino variant carrying a `HardwareSerial*`.
struct BusConfig_posix : public ::m5::hal::v1::uart::IBusConfig {
    const char* device_path = nullptr;

    /*! Coalesce writes in user space and flush them in batches of up to this
        many bytes (0 = write through immediately, the default). Each write()
        syscall on a USB serial device costs a USB scheduling round trip, so
        burst-heavy protocols (e.g. the remote bus stream writes, ~250 B per
        frame) gain substantial throughput from batching. Any read path
        (read / readableBytes) flushes pending bytes first, so a write-then-
        await-reply pattern can never deadlock on buffered output. */
    size_t tx_coalesce_bytes = 0;

    constexpr BusConfig_posix(void) : ::m5::hal::v1::uart::IBusConfig{}
    {
    }
};

class Bus_posix : public ::m5::hal::v1::uart::IBus {
public:
    ~Bus_posix() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig_posix. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_posix& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<size_t> write(::m5::hal::v1::bus::IAccessor* owner,
                                          const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Source* tx,
                                          size_t len) override;
    ::m5::hal::v1::result_t<size_t> read(::m5::hal::v1::bus::IAccessor* owner,
                                         const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Sink* rx,
                                         size_t len) override;
    ::m5::hal::v1::result_t<size_t> readableBytes(::m5::hal::v1::bus::IAccessor* owner,
                                                  const ::m5::hal::v1::uart::AccessConfig& cfg) override;

    // Open the named device (owning the fd), or adopt a caller-owned fd
    // (e.g. one end of an openpty() pair). Both leave termios setup to the
    // first write/read via the per-access AccessConfig.
    ::m5::hal::v1::error::error_t open(const char* device_path, uint32_t baud = 115200);
    ::m5::hal::v1::error::error_t attach(int fd);
    int nativeHandle() const
    {
        return _fd;
    }

    // Map a numeric baud to its termios B<rate> constant (returned in out_speed
    // as a speed_t cast to uint32_t). Returns false when this libc has no B*
    // constant for the rate — on macOS those are set via IOSSIOSPEED instead.
    // Static + public so the baud table can be unit-tested without a device.
    static bool baudToSpeed(uint32_t baud, uint32_t& out_speed);

private:
    ::m5::hal::v1::result_t<void> applyConfig(const ::m5::hal::v1::uart::AccessConfig& cfg);
    // Push one span to the fd, honouring EAGAIN + the write timeout.
    ::m5::hal::v1::result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms);
    // Drain the coalescing buffer (no-op when empty / coalescing disabled).
    ::m5::hal::v1::result_t<void> flushCoalesced(uint32_t timeout_ms);

    static constexpr size_t kCoalesceCapacity = 4096;

    const char* _device_path = nullptr;
    size_t _tx_coalesce      = 0;  // from BusConfig_posix::tx_coalesce_bytes (the base _config slices)
    int _fd                  = -1;
    bool _owns_fd            = false;
    bool _begun              = false;
    ::m5::hal::v1::uart::AccessConfig _applied_cfg;
    size_t _co_used = 0;
    uint8_t _co_buf[kCoalesceCapacity];
};

}  // namespace m5::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
