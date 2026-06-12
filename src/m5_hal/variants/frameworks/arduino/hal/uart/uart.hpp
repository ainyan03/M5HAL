// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/uart/uart.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v1::uart {

struct BusConfig_arduino : public ::m5::hal::v1::uart::IBusConfig {
    ::HardwareSerial* serial = nullptr;

    constexpr BusConfig_arduino(void) : ::m5::hal::v1::uart::IBusConfig{}
    {
    }
};

class Bus_arduino : public ::m5::hal::v1::uart::IBus {
public:
    ~Bus_arduino() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig_arduino. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_arduino& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<size_t> write(::m5::hal::v1::bus::IAccessor* owner,
                                          const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Source* tx,
                                          size_t len) override;
    ::m5::hal::v1::result_t<size_t> read(::m5::hal::v1::bus::IAccessor* owner,
                                         const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Sink* rx,
                                         size_t len) override;
    ::m5::hal::v1::result_t<size_t> readableBytes(::m5::hal::v1::bus::IAccessor* owner,
                                                  const ::m5::hal::v1::uart::AccessConfig& cfg) override;

    ::m5::hal::v1::error::error_t attach(::HardwareSerial& serial);
    ::HardwareSerial* nativeHandle() const
    {
        return _serial;
    }

private:
    ::m5::hal::v1::result_t<void> applyConfig(const ::m5::hal::v1::uart::AccessConfig& cfg);

    ::HardwareSerial* _serial = nullptr;
    bool _begun               = false;
    bool _attached            = false;
    ::m5::hal::v1::uart::AccessConfig _applied_cfg;
};

}  // namespace m5::hal::v1::uart

#endif

#endif
