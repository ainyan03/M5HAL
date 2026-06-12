// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/spi/spi.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <driver/spi_master.h>

namespace m5::hal::v1::spi {

struct BusConfig_espidf : public ::m5::hal::v1::spi::IBusConfig {
    ::spi_host_device_t host = SPI2_HOST;

    constexpr BusConfig_espidf(void) : ::m5::hal::v1::spi::IBusConfig{}
    {
    }
};

// ESP-IDF SPI master bus. CS and D/C are managed by M5HAL so the shared
// MasterAccessor transaction semantics match the Arduino and software
// variants. Driver-generation differences stay behind detail/espidf_version.hpp
// and backend includes.
class Bus_espidf : public ::m5::hal::v1::spi::IBus {
public:
    ~Bus_espidf() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig_espidf. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_espidf& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<void> beginTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                   const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<void> endTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                 const ::m5::hal::v1::spi::MasterAccessConfig& cfg) override;
    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::spi::MasterAccessConfig& cfg,
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
    ::m5::hal::v1::result_t<bool> ensureDevice(const ::m5::hal::v1::spi::MasterAccessConfig& cfg, bool half_duplex);
    ::m5::hal::v1::result_t<void> removeDevice(void);

    ::spi_host_device_t _host     = SPI2_HOST;
    ::spi_device_handle_t _device = nullptr;
    // Last accessor-level D/C pin switched to output (-1 = none yet);
    // avoids a gpio reconfig per transfer on the override path.
    ::m5::hal::v1::types::gpio_number_t _last_acc_dc = -1;
    bool _owns_bus                                   = false;
    bool _transaction_active                         = false;
    bool _device_half_duplex                         = false;
    uint32_t _device_freq                            = 0;
    uint8_t _device_mode                             = 0;
    uint8_t _device_order                            = 0;
};

}  // namespace m5::hal::v1::spi

#endif

#endif
