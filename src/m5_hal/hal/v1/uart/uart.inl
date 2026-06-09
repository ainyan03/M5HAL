#ifndef M5_HAL_UART_UART_INL_
#define M5_HAL_UART_UART_INL_

#include "uart.hpp"

namespace m5::hal::v1::uart {

UARTAccessor::UARTAccessor(UARTBus& bus, const UARTAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

UARTBus& UARTAccessor::getUARTBus(void) const
{
    return static_cast<UARTBus&>(getBus());
}

m5::stl::expected<void, error::error_t> UARTAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(data::Source& tx, size_t len)
{
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().write(this, _access_config, &tx, len);
    auto ea     = endAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(data::DataSpan rx_bytes)
{
    data::MemorySink sink{rx_bytes};
    return read(sink, rx_bytes.size);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(data::Sink& rx, size_t len)
{
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().read(this, _access_config, &rx, len);
    auto ea     = endAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::readableBytes(void)
{
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().readableBytes(this, _access_config);
    auto ea     = endAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> UARTBus::write(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                         data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::stl::expected<size_t, error::error_t> UARTBus::read(bus::Accessor* owner, const UARTAccessConfig& cfg,
                                                        data::Sink* rx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)rx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::stl::expected<size_t, error::error_t> UARTBus::readableBytes(bus::Accessor* owner, const UARTAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

}  // namespace m5::hal::v1::uart

#endif
