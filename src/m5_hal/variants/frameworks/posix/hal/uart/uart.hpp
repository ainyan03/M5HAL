// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

// Host POSIX UART variant: a real serial port driven through termios. It
// is offered (and flat-injected as the host UART) only on a POSIX host
// build; gated by M5HAL_FRAMEWORK_HAS_POSIX (frameworks/_checker.hpp,
// on for non-ESP/non-Arduino hosts) plus the M5HAL_CONFIG_POSIX_UART
// opt-out, which suppresses this UART kind only (the variant's runtime
// kind is unaffected).

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

namespace m5::hal::v2::uart {

// Variant-specific bus config: a host serial device path
// (e.g. "/dev/ttyUSB0", "/dev/tty.usbserial-XXXX") instead of MCU pins.
// Mirrors the arduino variant carrying a `HardwareSerial*`.
// Deliberately does NOT inherit the tag-pin constructors (Tx / Rx):
// this variant never reads the pin fields, so a one-line pin
// construction would only look complete while leaving `device_path`
// unset.
struct BusConfig_posix : public uart::IBusConfig {
    const char* device_path = nullptr;

    /*! Coalesce writes in user space and flush them in batches of up to this
        many bytes (0 = write through immediately, the default). Each write()
        syscall on a USB serial device costs a USB scheduling round trip, so
        burst-heavy protocols (e.g. the remote bus stream writes, ~250 B per
        frame) gain substantial throughput from batching. Any read path
        (read / readableBytes) flushes pending bytes first, so a write-then-
        await-reply pattern can never deadlock on buffered output. */
    size_t tx_coalesce_bytes = 0;

    constexpr BusConfig_posix(void) : uart::IBusConfig{}
    {
    }
};

class Bus_posix : public uart::Bus_streaming {
public:
    ~Bus_posix() override
    {
        (void)release();
    }

    result_t<void> init(const BusConfig_posix& config);
    result_t<void> release(void) override;

    // Override write for coalescing optimisation; read uses Bus_streaming::read.
    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                           size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override;

    // Open the named device (owning the fd), or adopt a caller-owned fd
    // (e.g. one end of an openpty() pair). Both leave termios setup to the
    // first write/read via the per-access AccessConfig.
    error::error_t open(const char* device_path, uint32_t baud = 115200);
    error::error_t attach(int fd);
    int nativeHandle() const
    {
        return _fd;
    }

    // Map a numeric baud to its termios B<rate> constant (returned in out_speed
    // as a speed_t cast to uint32_t). Returns false when this libc has no B*
    // constant for the rate — on macOS those are set via IOSSIOSPEED instead.
    // Static + public so the baud table can be unit-tested without a device.
    static bool baudToSpeed(uint32_t baud, uint32_t& out_speed);

    /*! @brief Reconfiguration-skip count (diagnostic only); see spec/design/uart.md §state mutex. */
    uint32_t reconfigSkips();

protected:
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    // Reconfiguration quiescence gate (spec/design/uart.md): `owner`/`entered`
    // identify the calling accessor and the channel it already holds so a
    // config change different from `_applied_cfg` can be gated through
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply on a fresh fd
    // (`!_begun`) — including the lazy open above — skips the gate.
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg);
    // Actual termios apply; assumes `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);
    // Drain the coalescing buffer (no-op when empty / coalescing disabled).
    // Self-locking (takes `_state_mutex`); call flushCoalescedLocked()
    // instead from a caller that already holds it (write()'s append path).
    result_t<void> flushCoalesced(uint32_t timeout_ms);
    result_t<void> flushCoalescedLocked(uint32_t timeout_ms);

    static constexpr size_t kCoalesceCapacity = 4096;

    const char* _device_path = nullptr;
    size_t _tx_coalesce      = 0;  // from BusConfig_posix::tx_coalesce_bytes (the base _config slices)
    int _fd                  = -1;
    bool _owns_fd            = false;
    bool _begun              = false;
    uart::AccessConfig _applied_cfg;
    size_t _co_used = 0;
    uint8_t _co_buf[kCoalesceCapacity];
    // Leaf mutex (see uart::IBus class comment) guarding
    // _fd/_owns_fd/_begun/_applied_cfg/_co_buf/_co_used against concurrent
    // TX/RX access (B12: the coalescing buffer is written by write() and
    // drained by read()/readableBytes(), i.e. from either channel).
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // skipped reconfigures (opposite channel busy); read via reconfigSkips()
};

// Facade backend selection: uart::Bus::init(BusConfig_posix) -> Bus_posix.
template <>
struct BackendFor<BusConfig_posix> {
    using type = Bus_posix;
};

}  // namespace m5::hal::v2::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
