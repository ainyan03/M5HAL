// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <SPI.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v1::spi {

struct BusConfig_arduino : public ::m5::hal::v1::spi::IBusConfig {
    ::SPIClass* spi = nullptr;

    constexpr BusConfig_arduino(void) : ::m5::hal::v1::spi::IBusConfig{}
    {
    }
};

// SPI bus that delegates byte transfers to an explicitly provided Arduino
// SPIClass instance. CS and D/C are still driven by M5HAL so the shared
// MasterAccessor command/data transaction semantics stay identical across
// variants.
class Bus_arduino : public ::m5::hal::v1::spi::IBus {
public:
    ~Bus_arduino() override
    {
        release();
    }

    // Typed init: takes this variant's BusConfig_arduino. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_arduino& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<void> beginTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                   const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<void> endTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                 const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::spi::MasterAccessConfig& cfg,
                                             const ::m5::hal::v1::spi::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

    ::m5::hal::v1::error::error_t attach(::SPIClass& spi);
    ::SPIClass* nativeHandle() const
    {
        return _spi;
    }

private:
    ::SPIClass* _spi = nullptr;
    bool _owns_spi   = false;
    // Last accessor-level D/C pin switched to output (-1 = none yet);
    // avoids a pinMode call per transfer on the override path.
    ::m5::hal::v1::types::gpio_number_t _last_acc_dc = -1;
};

}  // namespace m5::hal::v1::spi

#endif

#endif
