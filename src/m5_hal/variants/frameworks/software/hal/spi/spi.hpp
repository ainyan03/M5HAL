// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v1/gpio/port.hpp"
#include "../../../../../hal/v1/m5_hal.hpp"
#include "../../../../../hal/v1/service/service.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

// SPI bit-bang implementation. The bus stores CLK/MOSI/MISO/DC pins resolved
// from IBusConfig via M5_Hal.Gpio. Per-device CS is resolved from
// MasterAccessConfig for each transaction.
namespace m5::hal::v1::spi {

// This variant needs no fields beyond the abstract kind config; the
// empty derivation still gives `init` a variant-owned type, so a
// sibling variant's config cannot be passed by accident.
struct BusConfig_software : public ::m5::hal::v1::spi::IBusConfig {
    using IBusConfig::IBusConfig;
};

class Bus_software : public ::m5::hal::v1::spi::IBus {
public:
    // Typed init: takes this variant's BusConfig_software, so a sibling
    // variant's config is a compile error instead of a bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_software& config);
    ::m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v1::result_t<void> beginTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                   const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<void> endTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                 const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::spi::MasterAccessConfig& cfg,
                                             const ::m5::hal::v1::spi::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

private:
    ::m5::hal::v1::gpio::Pin _pin_clk{};
    ::m5::hal::v1::gpio::Pin _pin_dc{};
    // Accessor-level D/C override pin, resolved lazily on first use and
    // re-resolved only when the configured number changes.
    ::m5::hal::v1::gpio::Pin _pin_dc_acc{};
    ::m5::hal::v1::types::gpio_number_t _acc_dc_num = -1;
    ::m5::hal::v1::gpio::Pin _pin_mosi{};
    ::m5::hal::v1::gpio::Pin _pin_miso{};
    ::m5::hal::v1::gpio::Pin _transaction_cs{};
};

}  // namespace m5::hal::v1::spi

#endif
