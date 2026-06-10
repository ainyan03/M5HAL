#ifndef M5_HAL_UART_UART_INL_
#define M5_HAL_UART_UART_INL_

#include "uart.hpp"

namespace m5::hal::v1::uart {

UARTTxAccessor::UARTTxAccessor(UARTBus& bus, const UARTAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

UARTBus& UARTTxAccessor::getUARTBus(void) const
{
    return static_cast<UARTBus&>(getBus());
}

m5::stl::expected<void, error::error_t> UARTTxAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inTxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<void, error::error_t> UARTTxAccessor::beginTxAccess(uint32_t timeout_ms)
{
    if (_tx_access_depth == 0) {
        auto r = getUARTBus().lockChannel(this, Channel::tx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_tx_access_depth;
    return {};
}

m5::stl::expected<void, error::error_t> UARTTxAccessor::endTxAccess(void)
{
    if (_tx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_tx_access_depth;
    if (_tx_access_depth == 0) {
        return getUARTBus().unlockChannel(this, Channel::tx);
    }
    return {};
}

m5::stl::expected<size_t, error::error_t> UARTTxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

m5::stl::expected<size_t, error::error_t> UARTTxAccessor::write(data::Source& tx, size_t len)
{
    auto ba = beginTxAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().write(this, _access_config, &tx, len);
    auto ea     = endTxAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> UARTTxAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

UARTRxAccessor::UARTRxAccessor(UARTBus& bus, const UARTAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

UARTBus& UARTRxAccessor::getUARTBus(void) const
{
    return static_cast<UARTBus&>(getBus());
}

m5::stl::expected<void, error::error_t> UARTRxAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inRxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<void, error::error_t> UARTRxAccessor::beginRxAccess(uint32_t timeout_ms)
{
    if (_rx_access_depth == 0) {
        auto r = getUARTBus().lockChannel(this, Channel::rx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_rx_access_depth;
    return {};
}

m5::stl::expected<void, error::error_t> UARTRxAccessor::endRxAccess(void)
{
    if (_rx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_rx_access_depth;
    if (_rx_access_depth == 0) {
        return getUARTBus().unlockChannel(this, Channel::rx);
    }
    return {};
}

m5::stl::expected<size_t, error::error_t> UARTRxAccessor::read(data::DataSpan rx_bytes)
{
    data::MemorySink sink{rx_bytes};
    return read(sink, rx_bytes.size);
}

m5::stl::expected<size_t, error::error_t> UARTRxAccessor::read(data::Sink& rx, size_t len)
{
    auto ba = beginRxAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().read(this, _access_config, &rx, len);
    auto ea     = endRxAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> UARTRxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::stl::expected<size_t, error::error_t> UARTRxAccessor::readableBytes(void)
{
    auto ba = beginRxAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getUARTBus().readableBytes(this, _access_config);
    auto ea     = endRxAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

UARTAccessor::UARTAccessor(UARTBus& bus, const UARTAccessConfig& access_config)
    : _tx{bus, access_config}, _rx{bus, access_config}
{
}

UARTBus& UARTAccessor::getUARTBus(void) const
{
    return _tx.getUARTBus();
}

m5::stl::expected<void, error::error_t> UARTAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto tx_result = _tx.setConfig(cfg);
    if (!tx_result.has_value()) {
        return tx_result;
    }
    return _rx.setConfig(cfg);
}

m5::stl::expected<void, error::error_t> UARTAccessor::beginAccess(uint32_t timeout_ms)
{
    auto tx_result = _tx.beginTxAccess(timeout_ms);
    if (!tx_result.has_value()) {
        return tx_result;
    }
    auto rx_result = _rx.beginRxAccess(timeout_ms);
    if (!rx_result.has_value()) {
        (void)_tx.endTxAccess();  // rollback; the primary (rx) error takes priority
        return rx_result;
    }
    return {};
}

m5::stl::expected<void, error::error_t> UARTAccessor::endAccess(void)
{
    auto rx_result = _rx.endRxAccess();
    auto tx_result = _tx.endTxAccess();
    if (!rx_result.has_value()) {
        return rx_result;
    }
    return tx_result;
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(data::ConstDataSpan tx_bytes)
{
    return _tx.write(tx_bytes);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(data::Source& tx, size_t len)
{
    return _tx.write(tx, len);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::write(const uint8_t* tx, size_t len)
{
    return _tx.write(tx, len);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(data::DataSpan rx_bytes)
{
    return _rx.read(rx_bytes);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(data::Sink& rx, size_t len)
{
    return _rx.read(rx, len);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::read(uint8_t* dst, size_t len)
{
    return _rx.read(dst, len);
}

m5::stl::expected<size_t, error::error_t> UARTAccessor::readableBytes(void)
{
    return _rx.readableBytes();
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

m5::stl::expected<void, error::error_t> UARTBus::lock(bus::Accessor* owner, uint32_t timeout_ms)
{
    return lockChannel(owner, Channel::txrx, timeout_ms);
}

m5::stl::expected<void, error::error_t> UARTBus::unlock(bus::Accessor* owner)
{
    return unlockChannel(owner, Channel::txrx);
}

m5::stl::expected<void, error::error_t> UARTBus::lockChannel(bus::Accessor* owner, Channel ch, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (owner == nullptr || (!hasChannel(ch, Channel::tx) && !hasChannel(ch, Channel::rx))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (hasChannel(ch, Channel::tx)) {
        if (_tx_lock_owner != nullptr) {
            return m5::stl::make_unexpected(error::error_t::BUSY);
        }
        _tx_lock_owner = owner;
    }
    if (hasChannel(ch, Channel::rx)) {
        if (_rx_lock_owner != nullptr) {
            if (hasChannel(ch, Channel::tx) && _tx_lock_owner == owner) {
                _tx_lock_owner = nullptr;
            }
            return m5::stl::make_unexpected(error::error_t::BUSY);
        }
        _rx_lock_owner = owner;
    }
    return {};
}

m5::stl::expected<void, error::error_t> UARTBus::unlockChannel(bus::Accessor* owner, Channel ch)
{
    if (owner == nullptr || (!hasChannel(ch, Channel::tx) && !hasChannel(ch, Channel::rx))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::tx) && _tx_lock_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::rx) && _rx_lock_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (hasChannel(ch, Channel::tx)) {
        _tx_lock_owner = nullptr;
    }
    if (hasChannel(ch, Channel::rx)) {
        _rx_lock_owner = nullptr;
    }
    return {};
}

}  // namespace m5::hal::v1::uart

#endif
