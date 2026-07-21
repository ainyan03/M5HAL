// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::uart {

template <class Policy>
struct NativeProvider_arduino;

class Bus_arduino : public uart::Bus_streaming {
public:
    ~Bus_arduino() override
    {
        (void)teardown();
    }

    result_t<void> close(void)
    {
        return uart::IBus::close();
    }

    result_t<void> init(const IBusConfig& config, native::Borrowed<::HardwareSerial> policy);
    result_t<void> init(const IBusConfig& config, native::Borrowed<::Stream> policy);

    types::backend_kind_t backendKind(void) const override
    {
        return _hw_serial ? types::backend_kind_t::Hardware : types::backend_kind_t::Software;
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;

    result_t<size_t> writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                  size_t len) override;
    result_t<size_t> readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
                                 size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context) override;

public:
    ::Stream* nativeStream() const
    {
        return _serial;
    }

    /*! @brief Reconfiguration-skip count (diagnostic only); see spec/design/uart.md §state mutex. */
    uint32_t reconfigSkips();

protected:
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    template <class Policy>
    friend struct NativeProvider_arduino;

    static result_t<std::shared_ptr<IBus>> acquireBorrowed(bus::IHalBackend& backend, const IBusConfig& config,
                                                           ::Stream& stream, bool hardware_serial, uint16_t provider);
    result_t<void> initBorrowedDirect(const IBusConfig& config, ::Stream& stream, bool hardware_serial);
    result_t<void> adoptBorrowedNative(
        ::Stream& stream, bool hardware_serial, const IBusConfig& config,
        bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token);
    result_t<void> teardown(void);

protected:
    bus::CloseOutcome closeBackend(void) override;

private:
    // Reconfiguration quiescence gate (spec/design/uart.md): `owner`/`entered`
    // identify the calling accessor and the channel it already holds so a
    // config change different from `_applied_cfg` can be gated through
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply (`!_begun`)
    // skips the gate. The gate applies uniformly regardless of `_hw_serial`
    // (a plain Stream still gates its `setTimeout` + bookkeeping update).
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg,
                               uint32_t timeout_ms);
    // Actual apply (HardwareSerial::begin / Stream::setTimeout); assumes
    // `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);

    ::Stream* _serial = nullptr;
    bool _hw_serial   = false;
    bool _begun       = false;
    bool _attached    = false;
    uart::AccessConfig _applied_cfg;
    // Leaf mutex (see uart::IBus class comment) guarding
    // _serial/_hw_serial/_begun/_attached/_applied_cfg against concurrent
    // TX/RX access.
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // rejected reconfigures (opposite channel busy); read via reconfigSkips()
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* _native_interner = nullptr;
    bus::NativeToken _native_token{};
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_arduino(const bus::LocalResourceContext&, const IBusConfig&)
{
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

template <class Policy>
struct NativeProvider_arduino {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

template <>
struct NativeProvider_arduino<native::Borrowed<::HardwareSerial>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&,
                                                   native::Borrowed<::HardwareSerial>);
};

template <>
struct NativeProvider_arduino<native::Borrowed<::Stream>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, native::Borrowed<::Stream>);
};

}  // namespace m5::hal::v2::uart

#endif

#endif
