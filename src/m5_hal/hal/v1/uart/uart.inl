// SPDX-License-Identifier: MIT
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

result_t<void> UARTTxAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inTxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<void> UARTTxAccessor::beginTxAccess(uint32_t timeout_ms)
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

result_t<void> UARTTxAccessor::endTxAccess(void)
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

result_t<size_t> UARTTxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

result_t<size_t> UARTTxAccessor::write(data::Source& tx, size_t len)
{
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginTxAccess(_access_config.timeout_ms); },
                        [&] { return getUARTBus().write(this, _access_config, &tx, len); },
                        [&] { return endTxAccess(); });
}

result_t<size_t> UARTTxAccessor::write(const uint8_t* tx, size_t len)
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

result_t<void> UARTRxAccessor::setConfig(const UARTAccessConfig& cfg)
{
    if (inRxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<void> UARTRxAccessor::beginRxAccess(uint32_t timeout_ms)
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

result_t<void> UARTRxAccessor::endRxAccess(void)
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

result_t<size_t> UARTRxAccessor::read(data::DataSpan rx_bytes)
{
    data::MemorySink sink{rx_bytes};
    return read(sink, rx_bytes.size);
}

result_t<size_t> UARTRxAccessor::read(data::Sink& rx, size_t len)
{
    return bus::guarded([&] { return beginRxAccess(_access_config.timeout_ms); },
                        [&] { return getUARTBus().read(this, _access_config, &rx, len); },
                        [&] { return endRxAccess(); });
}

result_t<size_t> UARTRxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<size_t> UARTRxAccessor::readableBytes(void)
{
    return bus::guarded([&] { return beginRxAccess(_access_config.timeout_ms); },
                        [&] { return getUARTBus().readableBytes(this, _access_config); },
                        [&] { return endRxAccess(); });
}

UARTAccessor::UARTAccessor(UARTBus& bus, const UARTAccessConfig& access_config)
    : _tx{bus, access_config}, _rx{bus, access_config}
{
}

UARTBus& UARTAccessor::getUARTBus(void) const
{
    return _tx.getUARTBus();
}

result_t<void> UARTAccessor::setConfig(const UARTAccessConfig& cfg)
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

result_t<void> UARTAccessor::beginAccess(uint32_t timeout_ms)
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

result_t<void> UARTAccessor::endAccess(void)
{
    auto rx_result = _rx.endRxAccess();
    auto tx_result = _tx.endTxAccess();
    if (!rx_result.has_value()) {
        return rx_result;
    }
    return tx_result;
}

result_t<size_t> UARTAccessor::write(data::ConstDataSpan tx_bytes)
{
    return _tx.write(tx_bytes);
}

result_t<size_t> UARTAccessor::write(data::Source& tx, size_t len)
{
    return _tx.write(tx, len);
}

result_t<size_t> UARTAccessor::write(const uint8_t* tx, size_t len)
{
    return _tx.write(tx, len);
}

result_t<size_t> UARTAccessor::read(data::DataSpan rx_bytes)
{
    return _rx.read(rx_bytes);
}

result_t<size_t> UARTAccessor::read(data::Sink& rx, size_t len)
{
    return _rx.read(rx, len);
}

result_t<size_t> UARTAccessor::read(uint8_t* dst, size_t len)
{
    return _rx.read(dst, len);
}

result_t<size_t> UARTAccessor::readableBytes(void)
{
    return _rx.readableBytes();
}

result_t<size_t> UARTBus::write(bus::Accessor* owner, const UARTAccessConfig& cfg, data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> UARTBus::read(bus::Accessor* owner, const UARTAccessConfig& cfg, data::Sink* rx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)rx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> UARTBus::readableBytes(bus::Accessor* owner, const UARTAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<void> UARTBus::lock(bus::Accessor* owner, uint32_t timeout_ms)
{
    return lockChannel(owner, Channel::txrx, timeout_ms);
}

result_t<void> UARTBus::unlock(bus::Accessor* owner)
{
    return unlockChannel(owner, Channel::txrx);
}

result_t<void> UARTBus::lockChannel(bus::Accessor* owner, Channel ch, uint32_t timeout_ms)
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

result_t<void> UARTBus::unlockChannel(bus::Accessor* owner, Channel ch)
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
