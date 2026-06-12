// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/i2c/i2c.hpp"
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v1::i2c {

struct BusConfig_arduino : public ::m5::hal::v1::i2c::IBusConfig {
    ::TwoWire* wire = nullptr;

    constexpr BusConfig_arduino(void) : ::m5::hal::v1::i2c::IBusConfig{}
    {
    }
};

// I2C bus that delegates to an explicitly provided Arduino TwoWire instance.
// `init(BusConfig_arduino)` calls begin/end on that instance; `attach(TwoWire&)` leaves
// lifecycle ownership to the caller.
class Bus_arduino : public ::m5::hal::v1::i2c::IBus {
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

    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::i2c::MasterAccessConfig& cfg,
                                             const ::m5::hal::v1::i2c::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

    ::m5::hal::v1::error::error_t attach(::TwoWire& wire);
    ::TwoWire* nativeHandle() const
    {
        return _wire;
    }

private:
    ::TwoWire* _wire          = nullptr;
    bool _owns_wire           = false;
    uint32_t _last_freq       = 0;            // 0 sentinel: no setClock call has been made yet
    uint32_t _last_timeout_ms = 0xFFFFFFFFu;  // sentinel: no setTimeOut call yet
};

}  // namespace m5::hal::v1::i2c

#endif

#endif
