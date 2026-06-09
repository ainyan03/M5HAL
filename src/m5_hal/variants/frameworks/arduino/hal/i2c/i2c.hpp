#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/i2c/i2c.hpp"
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::variants::frameworks::arduino::hal::v1::i2c {

using namespace ::m5::hal::v1;  // resolve unqualified interface::/types::/bus:: refs

struct BusConfig : public ::m5::hal::v1::i2c::I2CBusConfig {
    ::TwoWire* wire = nullptr;

    constexpr BusConfig(void) : ::m5::hal::v1::i2c::I2CBusConfig{}
    {
    }
    constexpr BusConfig(::TwoWire* wire_, ::m5::hal::v1::types::gpio_number_t scl,
                        ::m5::hal::v1::types::gpio_number_t sda)
        : ::m5::hal::v1::i2c::I2CBusConfig{scl, sda}, wire{wire_}
    {
    }
};

// I2C bus that delegates to an explicitly provided Arduino TwoWire instance.
// `init(BusConfig)` calls begin/end on that instance; `attach(TwoWire&)` leaves
// lifecycle ownership to the caller.
class Bus : public ::m5::hal::v1::i2c::I2CBus {
public:
    ~Bus() override
    {
        release();
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override;

    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> transfer(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg,
        const ::m5::hal::v1::i2c::TransferDesc& desc, ::m5::hal::v1::data::Source* tx,
        ::m5::hal::v1::data::Sink* rx) override;

    ::m5::hal::v1::error::error_t attach(::TwoWire& wire);
    ::TwoWire* nativeHandle() const
    {
        return _wire;
    }

private:
    ::TwoWire* _wire    = nullptr;
    bool _owns_wire     = false;
    uint32_t _last_freq = 0;  // 0 sentinel: no setClock call has been made yet
};

}  // namespace m5::variants::frameworks::arduino::hal::v1::i2c

#endif

#endif
