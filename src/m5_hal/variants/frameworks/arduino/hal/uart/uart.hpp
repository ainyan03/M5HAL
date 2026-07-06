// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::uart {

struct BusConfig_arduino : public uart::IBusConfig {
    using uart::IBusConfig::IBusConfig;

    constexpr BusConfig_arduino(void) : uart::IBusConfig{}
    {
    }

    void setSerial(::HardwareSerial& s)
    {
        serial     = &s;
        _hw_serial = true;
    }
    void setSerial(::Stream& s)
    {
        serial     = &s;
        _hw_serial = false;
    }

    ::Stream* serial = nullptr;
    bool _hw_serial  = false;
};

class Bus_arduino : public uart::Bus_streaming {
public:
    ~Bus_arduino() override
    {
        (void)release();
    }

    result_t<void> init(const BusConfig_arduino& config);
    result_t<void> release(void) override;

    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                           size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override;

    error::error_t attach(::HardwareSerial& serial);
    error::error_t attach(::Stream& stream);
    ::Stream* nativeStream() const
    {
        return _serial;
    }

protected:
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    result_t<void> applyConfig(const uart::AccessConfig& cfg);

    ::Stream* _serial = nullptr;
    bool _hw_serial   = false;
    bool _begun       = false;
    bool _attached    = false;
    uart::AccessConfig _applied_cfg;
};

// Facade backend selection: uart::Bus::init(BusConfig_arduino) -> Bus_arduino.
template <>
struct BackendFor<BusConfig_arduino> {
    using type = Bus_arduino;
};

}  // namespace m5::hal::v2::uart

#endif

#endif
