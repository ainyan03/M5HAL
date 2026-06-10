#ifndef M5_HAL_UART_UART_HPP_
#define M5_HAL_UART_UART_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
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

enum class Channel : uint8_t {
    none = 0,
    tx   = 1u << 0,
    rx   = 1u << 1,
    txrx = (1u << 0) | (1u << 1),
};
using channel_t = Channel;

constexpr Channel operator|(Channel lhs, Channel rhs)
{
    return static_cast<Channel>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

constexpr Channel operator&(Channel lhs, Channel rhs)
{
    return static_cast<Channel>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

constexpr bool hasChannel(Channel value, Channel bit)
{
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) == static_cast<uint8_t>(bit);
}

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

struct UARTTxAccessor : public bus::Accessor, public data::StreamWriter {
    UARTTxAccessor(UARTBus& bus, const UARTAccessConfig& access_config);

    const UARTAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    UARTBus& getUARTBus(void) const;

    m5::stl::expected<void, error::error_t> setConfig(const UARTAccessConfig& cfg);
    m5::stl::expected<void, error::error_t> beginTxAccess(uint32_t timeout_ms = 0);
    m5::stl::expected<void, error::error_t> endTxAccess(void);
    bool inTxAccess(void) const
    {
        return _tx_access_depth > 0;
    }

    m5::stl::expected<size_t, error::error_t> write(data::ConstDataSpan tx_bytes) override;
    m5::stl::expected<size_t, error::error_t> write(data::Source& tx, size_t len);
    m5::stl::expected<size_t, error::error_t> write(const uint8_t* tx, size_t len);

protected:
    UARTAccessConfig _access_config;

private:
    uint32_t _tx_access_depth = 0;
    using bus::Accessor::beginAccess;
    using bus::Accessor::endAccess;
    using bus::Accessor::inAccess;
};

struct UARTRxAccessor : public bus::Accessor, public data::StreamReader {
    UARTRxAccessor(UARTBus& bus, const UARTAccessConfig& access_config);

    const UARTAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    UARTBus& getUARTBus(void) const;

    m5::stl::expected<void, error::error_t> setConfig(const UARTAccessConfig& cfg);
    m5::stl::expected<void, error::error_t> beginRxAccess(uint32_t timeout_ms = 0);
    m5::stl::expected<void, error::error_t> endRxAccess(void);
    bool inRxAccess(void) const
    {
        return _rx_access_depth > 0;
    }

    m5::stl::expected<size_t, error::error_t> read(data::DataSpan rx_bytes) override;
    m5::stl::expected<size_t, error::error_t> read(data::Sink& rx, size_t len);
    m5::stl::expected<size_t, error::error_t> read(uint8_t* dst, size_t len);

    m5::stl::expected<size_t, error::error_t> readableBytes(void) override;

protected:
    UARTAccessConfig _access_config;

private:
    uint32_t _rx_access_depth = 0;
    using bus::Accessor::beginAccess;
    using bus::Accessor::endAccess;
    using bus::Accessor::inAccess;
};

struct UARTAccessor {
    UARTAccessor(UARTBus& bus, const UARTAccessConfig& access_config);

    const UARTAccessConfig& getConfig(void) const
    {
        return _tx.getConfig();
    }
    UARTBus& getUARTBus(void) const;

    UARTTxAccessor& tx(void)
    {
        return _tx;
    }
    const UARTTxAccessor& tx(void) const
    {
        return _tx;
    }
    UARTRxAccessor& rx(void)
    {
        return _rx;
    }
    const UARTRxAccessor& rx(void) const
    {
        return _rx;
    }

    m5::stl::expected<void, error::error_t> setConfig(const UARTAccessConfig& cfg);
    m5::stl::expected<void, error::error_t> beginAccess(uint32_t timeout_ms = 0);
    m5::stl::expected<void, error::error_t> endAccess(void);
    bool inAccess(void) const
    {
        return _tx.inTxAccess() || _rx.inRxAccess();
    }

    m5::stl::expected<size_t, error::error_t> write(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, error::error_t> write(data::Source& tx, size_t len);
    m5::stl::expected<size_t, error::error_t> write(const uint8_t* tx, size_t len);

    m5::stl::expected<size_t, error::error_t> read(data::DataSpan rx_bytes);
    m5::stl::expected<size_t, error::error_t> read(data::Sink& rx, size_t len);
    m5::stl::expected<size_t, error::error_t> read(uint8_t* dst, size_t len);

    m5::stl::expected<size_t, error::error_t> readableBytes(void);

protected:
    UARTTxAccessor _tx;
    UARTRxAccessor _rx;
};

struct UARTBus : public bus::Bus {
    const UARTBusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual m5::stl::expected<size_t, error::error_t> write(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                            data::Source* tx, size_t len);
    virtual m5::stl::expected<size_t, error::error_t> read(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                           data::Sink* rx, size_t len);
    virtual m5::stl::expected<size_t, error::error_t> readableBytes(bus::Accessor* owner, const UARTAccessConfig& cfg);

    m5::stl::expected<void, error::error_t> lock(bus::Accessor* owner, uint32_t timeout_ms = 0) override;
    m5::stl::expected<void, error::error_t> unlock(bus::Accessor* owner) override;
    virtual m5::stl::expected<void, error::error_t> lockChannel(bus::Accessor* owner, Channel ch,
                                                                uint32_t timeout_ms = 0);
    virtual m5::stl::expected<void, error::error_t> unlockChannel(bus::Accessor* owner, Channel ch);

protected:
    UARTBusConfig _config;
    bus::Accessor* _tx_lock_owner = nullptr;
    bus::Accessor* _rx_lock_owner = nullptr;
};

}  // namespace m5::hal::v1::uart

#endif
