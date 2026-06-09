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

class Bus : public ::m5::hal::v1::spi::SPIBus {
public:
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override
    {
        return {};
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> beginTransaction(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> endTransaction(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> transfer(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
        const ::m5::hal::v1::spi::TransferDesc& desc, ::m5::hal::v1::data::Source* tx,
        ::m5::hal::v1::data::Sink* rx) override;

private:
    ::m5::hal::v1::gpio::Pin _pin_clk{};
    ::m5::hal::v1::gpio::Pin _pin_dc{};
    ::m5::hal::v1::gpio::Pin _pin_mosi{};
    ::m5::hal::v1::gpio::Pin _pin_miso{};
    ::m5::hal::v1::gpio::Pin _transaction_cs{};
};

}  // namespace m5::variants::frameworks::software::hal::v1::spi

#endif
