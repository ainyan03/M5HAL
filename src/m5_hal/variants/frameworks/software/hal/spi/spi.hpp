// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v1/gpio/port.hpp"
#include "../../../../../hal/v1/m5_hal.hpp"
#include "../../../../../hal/v1/service/service.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

// SPI bit-bang implementation. The bus stores CLK/MOSI/MISO/DC pins resolved
// from SPIBusConfig via M5_Hal.Gpio. Per-device CS is resolved from
// SPIMasterAccessConfig for each transaction.
namespace m5::variants::frameworks::software::hal::v1::spi {

using namespace ::m5::hal::v1;

// This variant needs no fields beyond the abstract kind config; the alias
// keeps the `m5hal::spi::BusConfig` spelling available in every build
// (every variant publishes `Bus` + `BusConfig`, S17 E2).
using BusConfig = ::m5::hal::v1::spi::SPIBusConfig;

class Bus : public ::m5::hal::v1::spi::SPIBus {
public:
    // Typed init (S17 E1). BusConfig is an alias of the abstract
    // SPIBusConfig here; the signature still names the alias so every
    // variant reads the same.
    ::m5::hal::v1::result_t<void> init(const BusConfig& config);
    ::m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v1::result_t<void> beginTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                   const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<void> endTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                 const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::Accessor* owner,
                                             const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
                                             const ::m5::hal::v1::spi::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

private:
    ::m5::hal::v1::gpio::Pin _pin_clk{};
    ::m5::hal::v1::gpio::Pin _pin_dc{};
    ::m5::hal::v1::gpio::Pin _pin_mosi{};
    ::m5::hal::v1::gpio::Pin _pin_miso{};
    ::m5::hal::v1::gpio::Pin _transaction_cs{};
};

}  // namespace m5::variants::frameworks::software::hal::v1::spi

#endif
