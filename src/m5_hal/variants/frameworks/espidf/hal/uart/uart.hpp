#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP

#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <driver/uart.h>
#include <esp_err.h>
#endif

#if defined(ESP_PLATFORM)

namespace m5::variants::frameworks::espidf::hal::v1::uart {

using namespace ::m5::hal::v1;

struct BusConfig : public ::m5::hal::v1::uart::UARTBusConfig {
    int8_t port_num = -1;  ///< UART_NUM_0 when negative.

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

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override;

    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> write(::m5::hal::v1::bus::Accessor* owner,
                                                                   const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                   ::m5::hal::v1::data::Source* tx,
                                                                   size_t len) override;
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> read(::m5::hal::v1::bus::Accessor* owner,
                                                                  const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                  ::m5::hal::v1::data::Sink* rx, size_t len) override;
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> readableBytes(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::uart::UARTAccessConfig& cfg) override;

    ::uart_port_t nativePort() const
    {
        return _port;
    }

private:
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> applyConfig(
        const ::m5::hal::v1::uart::UARTAccessConfig& cfg);

    ::uart_port_t _port = UART_NUM_0;
    bool _installed     = false;
    bool _configured    = false;
    ::m5::hal::v1::uart::UARTAccessConfig _applied_cfg;
};

}  // namespace m5::variants::frameworks::espidf::hal::v1::uart

#endif

#endif
