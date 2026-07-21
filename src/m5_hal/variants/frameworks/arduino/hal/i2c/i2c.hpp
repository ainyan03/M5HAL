// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/i2c/i2c.hpp"
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::i2c {

template <class Policy>
struct NativeProvider_arduino;

namespace arduino_i2c_capability_detail {

// TwoWire has no portable runtime capacity query. Use each supported core's
// published default, and stay conservative on an unknown core. A caller-
// expanded setBufferSize() is intentionally not inferred.
constexpr size_t wireRxBufferLength()
{
#if defined(I2C_BUFFER_LENGTH)
    return I2C_BUFFER_LENGTH;
#elif (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)) && defined(WIRE_BUFFER_SIZE)
    return WIRE_BUFFER_SIZE;
#elif (defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_SPRESENSE)) && defined(BUFFER_LENGTH)
    return BUFFER_LENGTH;
#elif defined(ARDUINO_ARCH_SAMD) && defined(__SAMD51__)
    // The Wire ring holds 256 bytes, but requestFrom() returns uint8_t.
    return 255u;
#elif defined(ARDUINO_ARCH_NRF52) && defined(ARDUINO_NRF52_ADAFRUIT) && defined(SERIAL_BUFFER_SIZE)
    return SERIAL_BUFFER_SIZE;
#else
    return 32u;
#endif
}

static_assert(wireRxBufferLength() > 0, "Arduino Wire RX buffer length must be nonzero");

}  // namespace arduino_i2c_capability_detail

// I2C bus that delegates to the Arduino default TwoWire instance.
class Bus_arduino : public i2c::IBus {
public:
    ~Bus_arduino() override
    {
        (void)teardown();
    }

    result_t<void> close(void)
    {
        return i2c::IBus::close();
    }

    result_t<void> init(const IBusConfig& config);
    result_t<void> init(const IBusConfig& config, native::Borrowed<::TwoWire> policy);

    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        return bus::detail::BusCapabilitiesBuilder{bus::IBus::capabilities()}
            .enable(bus::BusFeature::MasterTransfer)
            .enable(bus::BusFeature::Transmit)
            .enable(bus::BusFeature::Receive)
            .setLimit(bus::BusLimit::MaxAtomicRxBytes,
                      static_cast<uint32_t>(arduino_i2c_capability_detail::wireRxBufferLength()))
            .build();
    }

    ::TwoWire* nativeHandle() const
    {
        return _wire;
    }

private:
    template <class Policy>
    friend struct NativeProvider_arduino;

    result_t<void> adoptBorrowedNative(
        ::TwoWire& wire, const IBusConfig& config,
        bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token);
    result_t<void> teardown(void);

protected:
    result_t<void> transferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context,
                                   const i2c::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                   size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context) override;
    bus::CloseOutcome closeBackend(void) override;

private:
    ::TwoWire* _wire          = nullptr;
    bool _owns_wire           = false;
    uint32_t _last_freq       = 0;            // 0 sentinel: no setClock call has been made yet
    uint32_t _last_timeout_ms = 0xFFFFFFFFu;  // sentinel: no setTimeOut call yet
    bus::TransferTotals _transfer_totals{};
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* _native_interner = nullptr;
    bus::NativeToken _native_token{};
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_arduino(const bus::LocalResourceContext& resources,
                                                                   const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_arduino, IBusConfig>(resources, config);
}

template <class Policy>
struct NativeProvider_arduino {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

template <>
struct NativeProvider_arduino<native::Borrowed<::TwoWire>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, native::Borrowed<::TwoWire>);
};

}  // namespace m5::hal::v2::i2c

#endif

#endif
