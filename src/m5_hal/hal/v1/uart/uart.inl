// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_UART_INL_
#define M5_HAL_UART_UART_INL_

#include "uart.hpp"

namespace m5::hal::v1::uart {

TxAccessor::TxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

IBus& TxAccessor::getBus(void) const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> TxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inTxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<void> TxAccessor::beginTxAccess(uint32_t timeout_ms)
{
    M5HAL_ASSERT(isBound(), "accessor is not bound to a bus (bind() it first)");
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_tx_access_depth == 0) {
        auto r = getBus().lockChannel(this, Channel::tx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_tx_access_depth;
    return {};
}

result_t<void> TxAccessor::endTxAccess(void)
{
    if (_tx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_tx_access_depth;
    if (_tx_access_depth == 0) {
        return getBus().unlockChannel(this, Channel::tx);
    }
    return {};
}

result_t<size_t> TxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

result_t<size_t> TxAccessor::write(data::Source& tx, size_t len)
{
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginTxAccess(); }, [&] { return getBus().write(this, _access_config, &tx, len); },
                        [&] { return endTxAccess(); });
}

result_t<size_t> TxAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

RxAccessor::RxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

IBus& RxAccessor::getBus(void) const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> RxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inRxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<void> RxAccessor::beginRxAccess(uint32_t timeout_ms)
{
    M5HAL_ASSERT(isBound(), "accessor is not bound to a bus (bind() it first)");
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_rx_access_depth == 0) {
        auto r = getBus().lockChannel(this, Channel::rx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_rx_access_depth;
    return {};
}

result_t<void> RxAccessor::endRxAccess(void)
{
    if (_rx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_rx_access_depth;
    if (_rx_access_depth == 0) {
        return getBus().unlockChannel(this, Channel::rx);
    }
    return {};
}

result_t<size_t> RxAccessor::read(data::DataSpan rx_bytes)
{
    data::MemorySink sink{rx_bytes};
    return read(sink, rx_bytes.size);
}

result_t<size_t> RxAccessor::read(data::Sink& rx, size_t len)
{
    return bus::guarded([&] { return beginRxAccess(); }, [&] { return getBus().read(this, _access_config, &rx, len); },
                        [&] { return endRxAccess(); });
}

result_t<size_t> RxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<size_t> RxAccessor::readUntil(uint8_t delim, uint8_t* dst, size_t max_len)
{
    // One RX lock window for the whole line; the per-byte reads inside
    // data::readUntil nest through the depth counter without retaking
    // the channel mutex.
    return bus::guarded([&] { return beginRxAccess(); },
                        [&] { return data::readUntil(*this, delim, data::DataSpan{dst, max_len}); },
                        [&] { return endRxAccess(); });
}

result_t<size_t> RxAccessor::readableBytes(void)
{
    return bus::guarded([&] { return beginRxAccess(); }, [&] { return getBus().readableBytes(this, _access_config); },
                        [&] { return endRxAccess(); });
}

Accessor::Accessor(IBus& bus, const AccessConfig& access_config) : _tx{bus, access_config}, _rx{bus, access_config}
{
}

IBus& Accessor::getBus(void) const
{
    return _tx.getBus();
}

result_t<void> Accessor::setConfig(const AccessConfig& cfg)
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

result_t<void> Accessor::beginAccess(uint32_t timeout_ms)
{
    const uint32_t start = runtime::millis();
    auto tx_result       = _tx.beginTxAccess(timeout_ms);
    if (!tx_result.has_value()) {
        return tx_result;
    }
    // Spend what is left of the budget on the RX lock so the call as a
    // whole honours timeout_ms (an infinite budget stays infinite).
    uint32_t remaining = timeout_ms;
    if (timeout_ms != 0 && timeout_ms != types::TIMEOUT_FOREVER) {
        const uint32_t elapsed = runtime::millis() - start;
        remaining              = (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0;
    }
    auto rx_result = _rx.beginRxAccess(remaining);
    if (!rx_result.has_value()) {
        (void)_tx.endTxAccess();  // rollback; the primary (rx) error takes priority
        return rx_result;
    }
    return {};
}

result_t<void> Accessor::endAccess(void)
{
    auto rx_result = _rx.endRxAccess();
    auto tx_result = _tx.endTxAccess();
    if (!rx_result.has_value()) {
        return rx_result;
    }
    return tx_result;
}

result_t<size_t> Accessor::write(data::ConstDataSpan tx_bytes)
{
    return _tx.write(tx_bytes);
}

result_t<size_t> Accessor::write(data::Source& tx, size_t len)
{
    return _tx.write(tx, len);
}

result_t<size_t> Accessor::write(const uint8_t* tx, size_t len)
{
    return _tx.write(tx, len);
}

result_t<size_t> Accessor::read(data::DataSpan rx_bytes)
{
    return _rx.read(rx_bytes);
}

result_t<size_t> Accessor::read(data::Sink& rx, size_t len)
{
    return _rx.read(rx, len);
}

result_t<size_t> Accessor::read(uint8_t* dst, size_t len)
{
    return _rx.read(dst, len);
}

result_t<size_t> Accessor::readUntil(uint8_t delim, uint8_t* dst, size_t max_len)
{
    return _rx.readUntil(delim, dst, max_len);
}

result_t<size_t> Accessor::readableBytes(void)
{
    return _rx.readableBytes();
}

result_t<size_t> IBus::write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> IBus::read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* rx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)rx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> IBus::readableBytes(bus::IAccessor* owner, const AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<void> IBus::lock(bus::IAccessor* owner, uint32_t timeout_ms)
{
    return lockChannel(owner, Channel::txrx, timeout_ms);
}

result_t<void> IBus::unlock(bus::IAccessor* owner)
{
    return unlockChannel(owner, Channel::txrx);
}

result_t<void> IBus::lockChannel(bus::IAccessor* owner, Channel ch, uint32_t timeout_ms)
{
    if (owner == nullptr || (!hasChannel(ch, Channel::tx) && !hasChannel(ch, Channel::rx))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    const uint32_t start = runtime::millis();
    if (hasChannel(ch, Channel::tx)) {
        if (!_tx_mutex.lock(timeout_ms)) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        _tx_lock_owner = owner;
    }
    if (hasChannel(ch, Channel::rx)) {
        // The composite (txrx) lock spends what is left of the budget on
        // the second channel, so the call as a whole honours timeout_ms
        // (an infinite budget stays infinite).
        uint32_t remaining = timeout_ms;
        if (timeout_ms != 0 && timeout_ms != types::TIMEOUT_FOREVER) {
            const uint32_t elapsed = runtime::millis() - start;
            remaining              = (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0;
        }
        if (!_rx_mutex.lock(remaining)) {
            if (hasChannel(ch, Channel::tx)) {
                _tx_lock_owner = nullptr;
                _tx_mutex.unlock();
            }
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        _rx_lock_owner = owner;
    }
    return {};
}

result_t<void> IBus::unlockChannel(bus::IAccessor* owner, Channel ch)
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
        _tx_lock_owner = nullptr;  // cleared while the mutex is still held
        _tx_mutex.unlock();
    }
    if (hasChannel(ch, Channel::rx)) {
        _rx_lock_owner = nullptr;
        _rx_mutex.unlock();
    }
    return {};
}

}  // namespace m5::hal::v1::uart

#endif
