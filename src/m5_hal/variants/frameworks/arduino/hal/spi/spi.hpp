#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <SPI.h>
#endif

#if defined(ARDUINO)

namespace m5::variants::frameworks::arduino::hal::v1::spi {

using namespace ::m5::hal::v1;

struct BusConfig : public ::m5::hal::v1::spi::SPIBusConfig {
    ::SPIClass* spi = nullptr;

    constexpr BusConfig(void) : ::m5::hal::v1::spi::SPIBusConfig{}
    {
    }
};

// SPI bus that delegates byte transfers to an explicitly provided Arduino
// SPIClass instance. CS and D/C are still driven by M5HAL so the shared
// SPIMasterAccessor command/data transaction semantics stay identical across
// variants.
class Bus : public ::m5::hal::v1::spi::SPIBus {
public:
    ~Bus() override
    {
        release();
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override;

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> beginTransaction(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> endTransaction(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> transfer(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
        const ::m5::hal::v1::spi::TransferDesc& desc, ::m5::hal::v1::data::Source* tx,
        ::m5::hal::v1::data::Sink* rx) override;

    ::m5::hal::v1::error::error_t attach(::SPIClass& spi);
    ::SPIClass* nativeHandle() const
    {
        return _spi;
    }

private:
    ::SPIClass* _spi = nullptr;
    bool _owns_spi   = false;
};

}  // namespace m5::variants::frameworks::arduino::hal::v1::spi

#endif

#endif
