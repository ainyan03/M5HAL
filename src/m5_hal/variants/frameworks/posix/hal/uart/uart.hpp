// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#include <string>
#include <termios.h>

// Host POSIX UART variant: a real serial port driven through termios. It
// is offered (and flat-injected as the host UART) only on a POSIX host
// build; gated by M5HAL_FRAMEWORK_HAS_POSIX (frameworks/_checker.hpp,
// on for non-ESP/non-Arduino hosts) plus the M5HAL_CONFIG_POSIX_UART
// opt-out, which suppresses this UART kind only (the variant's runtime
// kind is unaffected).

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

namespace m5::hal::v2::uart {

/*! @brief Caller-owned POSIX descriptor selected for a native UART binding. */
struct NativeFd {
    int value = -1;

    constexpr explicit NativeFd(int fd) : value{fd}
    {
    }
};

/*! @brief Existing POSIX device path from which a managed UART is opened. */
struct NativePath {
    std::string value;

    explicit NativePath(const char* path) : value{path != nullptr ? path : ""}
    {
    }
    explicit NativePath(std::string path) : value{std::move(path)}
    {
    }
};

/*! @brief POSIX-provider creation options (not part of portable BusConfig). */
struct NativeOptions {
    static constexpr size_t kMaxTxCoalesceBytes = 4096;
    size_t tx_coalesce_bytes                    = 0;
};

class Bus_posix : public uart::Bus_streaming {
public:
    ~Bus_posix() override;

    result_t<void> close(void)
    {
        return uart::IBus::close();
    }

    result_t<void> init(const IBusConfig& config);
    result_t<void> init(const IBusConfig& config, native::Borrowed<NativeFd> policy);
    result_t<void> init(const IBusConfig& config, native::Managed<NativePath> policy);
    result_t<void> init(const IBusConfig& config, native::Managed<NativePath, NativeOptions> policy);

    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;

    // Override write for coalescing optimisation; read uses Bus_streaming::read.
    result_t<size_t> writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                  size_t len) override;
    result_t<size_t> readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
                                 size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context) override;

public:
    result_t<void> adoptOwnedNative(
        int fd, const IBusConfig& config, size_t tx_coalesce_bytes,
        bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token);
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
    bus::CloseOutcome closeBackend(void) override;
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    bus::CloseOutcome teardownBackend(void);
    result_t<void> resetForInitialization(void);
    result_t<void> initOwnedDirect(int fd, const IBusConfig& config, const NativeOptions& options);
    // Reconfiguration quiescence gate (spec/design/uart.md): `owner`/`entered`
    // identify the calling accessor and the channel it already holds so a
    // config change different from `_applied_cfg` can be gated through
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply on a fresh fd
    // (`!_begun`) — including the lazy open above — skips the gate.
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg,
                               uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    // Actual termios apply; assumes `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);
    // Drain the coalescing buffer (no-op when empty / coalescing disabled).
    // Self-locking (takes `_state_mutex`); call flushCoalescedLocked()
    // instead from a caller that already holds it (write()'s append path).
    result_t<void> flushCoalesced(uint32_t timeout_ms);
    result_t<void> flushCoalescedLocked(uint32_t timeout_ms);

    static constexpr size_t kCoalesceCapacity = NativeOptions::kMaxTxCoalesceBytes;

    size_t _tx_coalesce          = 0;
    int _fd                      = -1;
    bool _owns_fd                = false;
    bool _begun                  = false;
    bool _original_termios_valid = false;
    struct termios _original_termios {};
    uart::AccessConfig _applied_cfg;
    size_t _co_used = 0;
    uint8_t _co_buf[kCoalesceCapacity];
    // Leaf mutex (see uart::IBus class comment) guarding
    // _fd/_owns_fd/_begun/_applied_cfg/_co_buf/_co_used against concurrent
    // TX/RX access (B12: the coalescing buffer is written by write() and
    // drained by read()/readableBytes(), i.e. from either channel).
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // rejected reconfigures (opposite channel busy); read via reconfigSkips()
    // `IBus::_local_resources.lifetime` co-owns the DomainState containing
    // this interner until after Bus_posix destruction.
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* _native_interner = nullptr;
    bus::NativeToken _native_token{};
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_posix(const bus::LocalResourceContext&, const IBusConfig&)
{
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

template <class Policy>
struct NativeProvider_posix {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

template <>
struct NativeProvider_posix<native::Borrowed<NativeFd>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, native::Borrowed<NativeFd>);
};

template <>
struct NativeProvider_posix<native::Managed<NativePath>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, native::Managed<NativePath>);
};

template <>
struct NativeProvider_posix<native::Managed<NativePath, NativeOptions>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&,
                                                   native::Managed<NativePath, NativeOptions>);
};

}  // namespace m5::hal::v2::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
