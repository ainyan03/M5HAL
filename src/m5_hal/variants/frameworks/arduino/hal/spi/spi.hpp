// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <SPI.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::spi {

struct BusConfig_arduino : public spi::IBusConfig {
    // Inherit the tag-pin constructors (Clk / Mosi / Miso); the SPIClass
    // handle is set by field assignment afterwards.
    using spi::IBusConfig::IBusConfig;

    ::SPIClass* spi = nullptr;

    constexpr BusConfig_arduino(void) : spi::IBusConfig{}
    {
    }
};

// SPI bus that delegates byte transfers to an explicitly provided Arduino
// SPIClass instance. CS and D/C are still driven by M5HAL so the shared
// MasterAccessor command/data transaction semantics stay identical across
// variants.
class Bus_arduino : public spi::IBus {
public:
    ~Bus_arduino() override
    {
        release();
    }

    // Typed init: takes this variant's BusConfig_arduino. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    result_t<void> init(const BusConfig_arduino& config);
    result_t<void> release(void) override;

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;

    error::error_t attach(::SPIClass& spi);
    ::SPIClass* nativeHandle() const
    {
        return _spi;
    }

private:
    ::SPIClass* _spi = nullptr;
    bool _owns_spi   = false;
    // Last accessor-level D/C pin switched to output (-1 = none yet);
    // avoids a pinMode call per transfer on the override path.
    types::gpio_number_t _last_acc_dc = -1;
    bus::TransferTotals _transfer_totals{};
};

// Facade backend selection: spi::Bus::init(BusConfig_arduino) -> Bus_arduino.
template <>
struct BackendFor<BusConfig_arduino> {
    using type = Bus_arduino;
};

}  // namespace m5::hal::v2::spi

#endif

#endif
