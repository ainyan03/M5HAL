// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <SPI.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::spi {

template <class Policy>
struct NativeProvider_arduino;

// SPI bus that delegates byte transfers to an Arduino SPIClass instance.
// CS and D/C are still driven by M5HAL so the shared
// MasterAccessor command/data transaction semantics stay identical across
// variants.
class Bus_arduino : public spi::IBus {
public:
    ~Bus_arduino() override
    {
        (void)teardown();
    }

    result_t<void> close(void)
    {
        return spi::IBus::close();
    }

    result_t<void> init(const IBusConfig& config);
    result_t<void> init(const IBusConfig& config, native::Borrowed<::SPIClass> policy);

    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        return bus::detail::BusCapabilitiesBuilder{bus::IBus::capabilities()}
            .enable(bus::BusFeature::MasterTransfer)
            .enable(bus::BusFeature::Transmit, supportsTransmit())
            .enable(bus::BusFeature::Receive, supportsReceive())
            .enable(bus::BusFeature::FullDuplex, supportsTransmit() && supportsReceive())
            .build();
    }

    ::SPIClass* nativeHandle() const
    {
        return _spi;
    }

private:
    bool usesDefaultPins(void) const
    {
        return _config.pin_clk < 0 && _config.pin_mosi < 0 && _config.pin_miso < 0;
    }
    bool supportsTransmit(void) const
    {
        return usesDefaultPins() || _config.pin_mosi >= 0;
    }
    bool supportsReceive(void) const
    {
        return usesDefaultPins() || _config.pin_miso >= 0;
    }

    template <class Policy>
    friend struct NativeProvider_arduino;

    result_t<void> adoptBorrowedNative(
        ::SPIClass& spi, const IBusConfig& config,
        bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token);
    result_t<void> teardown(void);

protected:
    bus::CloseOutcome closeBackend(void) override;
    result_t<void> beginOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> transferBackend(bus::OperationContext<spi::MasterAccessConfig>& context,
                                   const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                   size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;

private:
    ::SPIClass* _spi = nullptr;
    bool _owns_spi   = false;
    // Last accessor-level D/C pin switched to output (-1 = none yet);
    // avoids a pinMode call per transfer on the override path.
    types::gpio_number_t _last_acc_dc = -1;
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
struct NativeProvider_arduino<native::Borrowed<::SPIClass>> {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, native::Borrowed<::SPIClass>);
};

}  // namespace m5::hal::v2::spi

#endif

#endif
