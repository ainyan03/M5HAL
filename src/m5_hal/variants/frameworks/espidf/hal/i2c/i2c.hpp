// SPDX-License-Identifier: MIT
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

namespace m5::hal::v1::i2c {

struct BusConfig_espidf : public ::m5::hal::v1::i2c::IBusConfig {
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    int i2c_port = -1;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t i2c_port = I2C_NUM_0;
#endif

    constexpr BusConfig_espidf(void) : ::m5::hal::v1::i2c::IBusConfig{}
    {
    }
};

// ESP-IDF I2C bus. The public Bus_espidf type is stable inside the espidf
// variant; ESP-IDF driver generation differences live in gen4/gen5 backend
// implementations selected by detail/espidf_version.hpp.
class Bus_espidf : public ::m5::hal::v1::i2c::IBus {
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

    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::i2c::MasterAccessConfig& cfg,
                                             const ::m5::hal::v1::i2c::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

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
    ::m5::hal::v1::result_t<void> ensureDevice(const ::m5::hal::v1::i2c::MasterAccessConfig& cfg);
    ::m5::hal::v1::result_t<void> removeDevice(void);

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
    // Last frequency pushed through i2c_param_config; skips the
    // re-config on every transfer when unchanged (0 = none applied).
    uint32_t _applied_freq = 0;
#endif
};

}  // namespace m5::hal::v1::i2c

#endif

#endif
