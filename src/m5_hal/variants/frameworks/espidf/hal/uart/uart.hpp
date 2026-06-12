// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <driver/uart.h>
#include <esp_err.h>
#endif

#if defined(ESP_PLATFORM)

namespace m5::hal::v1::uart {

struct BusConfig_espidf : public ::m5::hal::v1::uart::IBusConfig {
    int8_t port_num = -1;  ///< UART_NUM_0 when negative.

    constexpr BusConfig_espidf(void) : ::m5::hal::v1::uart::IBusConfig{}
    {
    }
};

class Bus_espidf : public ::m5::hal::v1::uart::IBus {
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

    ::m5::hal::v1::result_t<size_t> write(::m5::hal::v1::bus::IAccessor* owner,
                                          const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Source* tx,
                                          size_t len) override;
    ::m5::hal::v1::result_t<size_t> read(::m5::hal::v1::bus::IAccessor* owner,
                                         const ::m5::hal::v1::uart::AccessConfig& cfg, ::m5::hal::v1::data::Sink* rx,
                                         size_t len) override;
    ::m5::hal::v1::result_t<size_t> readableBytes(::m5::hal::v1::bus::IAccessor* owner,
                                                  const ::m5::hal::v1::uart::AccessConfig& cfg) override;

    ::uart_port_t nativePort() const
    {
        return _port;
    }

private:
    ::m5::hal::v1::result_t<void> applyConfig(const ::m5::hal::v1::uart::AccessConfig& cfg);

    ::uart_port_t _port = UART_NUM_0;
    bool _installed     = false;
    bool _configured    = false;
    ::m5::hal::v1::uart::AccessConfig _applied_cfg;
};

}  // namespace m5::hal::v1::uart

#endif

#endif
