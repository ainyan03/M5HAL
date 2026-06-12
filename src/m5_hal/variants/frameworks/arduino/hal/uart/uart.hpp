// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/uart/uart.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO)

namespace m5::variants::frameworks::arduino::hal::v1::uart {

using namespace ::m5::hal::v1;

struct BusConfig : public ::m5::hal::v1::uart::UARTBusConfig {
    ::HardwareSerial* serial = nullptr;

    constexpr BusConfig(void) : ::m5::hal::v1::uart::UARTBusConfig{}
    {
    }
};

class Bus : public ::m5::hal::v1::uart::UARTBus {
public:
    ~Bus() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig (S17 E1). Passing the
    // abstract UARTBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig& config);
    ::m5::hal::v1::result_t<void> release(void) override;

    ::m5::hal::v1::result_t<size_t> write(::m5::hal::v1::bus::Accessor* owner,
                                          const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                          ::m5::hal::v1::data::Source* tx, size_t len) override;
    ::m5::hal::v1::result_t<size_t> read(::m5::hal::v1::bus::Accessor* owner,
                                         const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                         ::m5::hal::v1::data::Sink* rx, size_t len) override;
    ::m5::hal::v1::result_t<size_t> readableBytes(::m5::hal::v1::bus::Accessor* owner,
                                                  const ::m5::hal::v1::uart::UARTAccessConfig& cfg) override;

    ::m5::hal::v1::error::error_t attach(::HardwareSerial& serial);
    ::HardwareSerial* nativeHandle() const
    {
        return _serial;
    }

private:
    ::m5::hal::v1::result_t<void> applyConfig(const ::m5::hal::v1::uart::UARTAccessConfig& cfg);

    ::HardwareSerial* _serial = nullptr;
    bool _begun               = false;
    bool _attached            = false;
    ::m5::hal::v1::uart::UARTAccessConfig _applied_cfg;
};

}  // namespace m5::variants::frameworks::arduino::hal::v1::uart

#endif

#endif
