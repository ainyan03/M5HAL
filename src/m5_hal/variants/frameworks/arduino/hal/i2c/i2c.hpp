// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/i2c/i2c.hpp"
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::i2c {

struct BusConfig_arduino : public i2c::IBusConfig {
    // Inherit the tag-pin constructors (Scl / Sda, either order);
    // `wire` keeps its member initializer and is set by assignment.
    using i2c::IBusConfig::IBusConfig;

    ::TwoWire* wire = nullptr;

    constexpr BusConfig_arduino(void) : i2c::IBusConfig{}
    {
    }
};

// I2C bus that delegates to an explicitly provided Arduino TwoWire instance.
// `init(BusConfig_arduino)` calls begin/end on that instance; `attach(TwoWire&)` leaves
// lifecycle ownership to the caller.
class Bus_arduino : public i2c::IBus {
public:
    ~Bus_arduino() override
    {
        release();
    }

    // Typed init: takes this variant's BusConfig_arduino. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    result_t<void> init(const BusConfig_arduino& config);
    result_t<void> release(void) override;

    result_t<void> transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg) override;

    error::error_t attach(::TwoWire& wire);
    ::TwoWire* nativeHandle() const
    {
        return _wire;
    }

private:
    ::TwoWire* _wire          = nullptr;
    bool _owns_wire           = false;
    uint32_t _last_freq       = 0;            // 0 sentinel: no setClock call has been made yet
    uint32_t _last_timeout_ms = 0xFFFFFFFFu;  // sentinel: no setTimeOut call yet
    bus::TransferTotals _transfer_totals{};
};

// Facade backend selection: i2c::Bus::init(BusConfig_arduino) -> Bus_arduino.
template <>
struct BackendFor<BusConfig_arduino> {
    using type = Bus_arduino;
};

}  // namespace m5::hal::v2::i2c

#endif

#endif
