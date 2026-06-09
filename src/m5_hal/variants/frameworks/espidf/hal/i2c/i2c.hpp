#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/i2c/i2c.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER

#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
#include <driver/i2c_master.h>
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
#include <driver/i2c.h>
#endif

namespace m5::variants::frameworks::espidf::hal::v1::i2c {

using namespace ::m5::hal::v1;  // resolve unqualified types::/bus:: refs

struct BusConfig : public ::m5::hal::v1::i2c::I2CBusConfig {
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    int i2c_port = -1;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t i2c_port = I2C_NUM_0;
#endif

    constexpr BusConfig(void) : ::m5::hal::v1::i2c::I2CBusConfig{}
    {
    }
    constexpr BusConfig(::m5::hal::v1::types::gpio_number_t scl, ::m5::hal::v1::types::gpio_number_t sda)
        : ::m5::hal::v1::i2c::I2CBusConfig{scl, sda}
    {
    }
};

// ESP-IDF I2C bus. The public Bus type is stable inside the espidf
// variant; ESP-IDF driver generation differences live in gen4/gen5 backend
// implementations selected by detail/espidf_version.hpp.
class Bus : public ::m5::hal::v1::i2c::I2CBus {
public:
    ~Bus() override
    {
        (void)release();
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override;

    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> transfer(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg,
        const ::m5::hal::v1::i2c::TransferDesc& desc, ::m5::hal::v1::data::Source* tx,
        ::m5::hal::v1::data::Sink* rx) override;

#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    ::m5::hal::v1::error::error_t attach(::i2c_master_bus_handle_t bus_handle);
    ::i2c_master_bus_handle_t nativeHandle() const
    {
        return _bus_handle;
    }
#endif
#if !M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t nativePort() const
    {
        return _port;
    }
#endif

private:
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> ensureDevice(
        const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg);
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> removeDevice(void);

    ::i2c_master_bus_handle_t _bus_handle = nullptr;
    ::i2c_master_dev_handle_t _dev_handle = nullptr;
    bool _owns_bus                        = false;
    uint16_t _dev_addr                    = 0;
    uint32_t _dev_freq                    = 0;
    uint32_t _dev_scl_wait_us             = 0;
    bool _dev_address_is_10bit            = false;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t _port = I2C_NUM_0;
    bool _installed    = false;
#endif
};

}  // namespace m5::variants::frameworks::espidf::hal::v1::i2c

#endif

#endif
