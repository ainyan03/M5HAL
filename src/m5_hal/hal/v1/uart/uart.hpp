#ifndef M5_HAL_UART_UART_HPP_
#define M5_HAL_UART_UART_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::uart {

enum class Parity : uint8_t {
    none = 0,
    even = 1,
    odd  = 2,
};
using parity_t = Parity;

struct UARTBusConfig : public bus::BusConfig {
    types::gpio_number_t pin_tx  = -1;
    types::gpio_number_t pin_rx  = -1;
    types::gpio_number_t pin_rts = -1;
    types::gpio_number_t pin_cts = -1;
    size_t rx_buffer_size        = 256;
    size_t tx_buffer_size        = 0;

    constexpr UARTBusConfig(void) : bus::BusConfig{types::bus_kind_t::UART}
    {
    }
};

struct UARTAccessConfig : public bus::AccessConfig {
    uint32_t baud_rate             = 115200;
    uint32_t timeout_ms            = 1000;  ///< Bus lock timeout.
    uint32_t first_byte_timeout_ms = 100;
    uint32_t inter_byte_timeout_ms = 20;
    uint32_t write_timeout_ms      = 1000;
    uint8_t data_bits              = 8;
    uint8_t stop_bits              = 1;
    parity_t parity                = parity_t::none;
    bool invert                    = false;

    constexpr UARTAccessConfig(void) : bus::AccessConfig{types::bus_kind_t::UART}
    {
    }
};

struct UARTBus;

struct UARTAccessor : public bus::Accessor {
    UARTAccessor(UARTBus& bus, const UARTAccessConfig& access_config);

    const UARTAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    UARTBus& getUARTBus(void) const;

    m5::stl::expected<void, error::error_t> setConfig(const UARTAccessConfig& cfg);

    m5::stl::expected<size_t, error::error_t> write(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, error::error_t> write(data::Source& tx, size_t len);
    m5::stl::expected<size_t, error::error_t> write(const uint8_t* tx, size_t len);

    m5::stl::expected<size_t, error::error_t> read(data::DataSpan rx_bytes);
    m5::stl::expected<size_t, error::error_t> read(data::Sink& rx, size_t len);
    m5::stl::expected<size_t, error::error_t> read(uint8_t* dst, size_t len);

    m5::stl::expected<size_t, error::error_t> readableBytes(void);

protected:
    UARTAccessConfig _access_config;
};

struct UARTBus : public bus::Bus {
    const bus::BusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual m5::stl::expected<size_t, error::error_t> write(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                            data::Source* tx, size_t len);
    virtual m5::stl::expected<size_t, error::error_t> read(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                           data::Sink* rx, size_t len);
    virtual m5::stl::expected<size_t, error::error_t> readableBytes(bus::Accessor* owner, const UARTAccessConfig& cfg);

protected:
    UARTBusConfig _config;
};

}  // namespace m5::hal::v1::uart

#endif
