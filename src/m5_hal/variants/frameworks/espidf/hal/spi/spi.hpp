// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <driver/spi_master.h>

namespace m5::variants::frameworks::espidf::hal::v1::spi {

using namespace ::m5::hal::v1;

struct BusConfig : public ::m5::hal::v1::spi::SPIBusConfig {
    ::spi_host_device_t host = SPI2_HOST;

    constexpr BusConfig(void) : ::m5::hal::v1::spi::SPIBusConfig{}
    {
    }
};

// ESP-IDF SPI master bus. CS and D/C are managed by M5HAL so the shared
// SPIMasterAccessor transaction semantics match the Arduino and software
// variants. Driver-generation differences stay behind detail/espidf_version.hpp
// and backend includes.
class Bus : public ::m5::hal::v1::spi::SPIBus {
public:
    ~Bus() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig (S17 E1). Passing the
    // abstract SPIBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<void> beginTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                   const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<void> endTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                 const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::Accessor* owner,
                                             const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
                                             const ::m5::hal::v1::spi::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

    ::m5::hal::v1::error::error_t attach(::spi_host_device_t host);
    ::spi_host_device_t nativeHost() const
    {
        return _host;
    }
    ::spi_device_handle_t nativeDevice() const
    {
        return _device;
    }

private:
    // true = the device was (re)created (SCK idle level may need settling).
    ::m5::hal::v1::result_t<bool> ensureDevice(const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg, bool half_duplex);
    ::m5::hal::v1::result_t<void> removeDevice(void);

    ::spi_host_device_t _host     = SPI2_HOST;
    ::spi_device_handle_t _device = nullptr;
    bool _owns_bus                = false;
    bool _transaction_active      = false;
    bool _device_half_duplex      = false;
    uint32_t _device_freq         = 0;
    uint8_t _device_mode          = 0;
    uint8_t _device_order         = 0;
};

}  // namespace m5::variants::frameworks::espidf::hal::v1::spi

#endif

#endif
