// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2S_I2S_INL_
#define M5_HAL_I2S_I2S_INL_

#include "i2s.hpp"

namespace m5::hal::v2::i2s {

TxAccessor::TxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

TxAccessor::TxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _access_config{access_config}
{
}

IBus& TxAccessor::getBus(void) const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> TxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _access_config = cfg;
    return {};
}

result_t<void> TxAccessor::beginAccess(uint32_t timeout_ms)
{
    M5HAL_ASSERT(isBound(), "accessor is not bound to a bus (bind() it first)");
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_tx_access_depth == 0) {
        auto r = getBus().lockChannel(this, Channel::Tx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_tx_access_depth;
    return {};
}

result_t<void> TxAccessor::endAccess(void)
{
    if (_tx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_tx_access_depth;
    if (_tx_access_depth == 0) {
        return getBus().unlockChannel(this, Channel::Tx);
    }
    return {};
}

result_t<void> TxAccessor::beginTransaction(uint32_t timeout_ms)
{
    if (_tx_txn_depth != 0) {
        ++_tx_txn_depth;
        return {};
    }

    auto ba = beginAccess(timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }

    _tx_txn_totals.clear();
    _tx_txn_depth = 1;
    return {};
}

result_t<bus::TransferTotals> TxAccessor::endTransaction(void)
{
    if (_tx_txn_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    --_tx_txn_depth;
    if (_tx_txn_depth != 0) {
        return _tx_txn_totals;
    }

    const auto totals = _tx_txn_totals;
    auto ea           = endAccess();
    _tx_txn_totals.clear();
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return totals;
}

result_t<size_t> TxAccessor::write(data::ConstDataSpan src_bytes)
{
    data::MemorySource src{src_bytes};
    return write(src, src_bytes.size);
}

result_t<size_t> TxAccessor::write(data::Source& src, size_t len)
{
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginTransaction(); },
                        [&] {
                            auto r = getBus().write(this, _access_config, &src, len);
                            if (r.has_value()) {
                                _tx_txn_totals.tx += r.value();
                            }
                            return r;
                        },
                        [&] { return endTransaction(); });
}

result_t<size_t> TxAccessor::write(const uint8_t* src, size_t len)
{
    return write(data::ConstDataSpan{src, len});
}

result_t<size_t> TxAccessor::writableBytes(void)
{
    return bus::guarded([&] { return beginAccess(); }, [&] { return getBus().writableBytes(this, _access_config); },
                        [&] { return endAccess(); });
}

RxAccessor::RxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

RxAccessor::RxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _access_config{access_config}
{
}

IBus& RxAccessor::getBus(void) const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> RxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _access_config = cfg;
    return {};
}

result_t<void> RxAccessor::beginAccess(uint32_t timeout_ms)
{
    M5HAL_ASSERT(isBound(), "accessor is not bound to a bus (bind() it first)");
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_rx_access_depth == 0) {
        auto r = getBus().lockChannel(this, Channel::Rx, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_rx_access_depth;
    return {};
}

result_t<void> RxAccessor::endAccess(void)
{
    if (_rx_access_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    --_rx_access_depth;
    if (_rx_access_depth == 0) {
        return getBus().unlockChannel(this, Channel::Rx);
    }
    return {};
}

result_t<void> RxAccessor::beginTransaction(uint32_t timeout_ms)
{
    if (_rx_txn_depth != 0) {
        ++_rx_txn_depth;
        return {};
    }

    auto ba = beginAccess(timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }

    _rx_txn_totals.clear();
    _rx_txn_depth = 1;
    return {};
}

result_t<bus::TransferTotals> RxAccessor::endTransaction(void)
{
    if (_rx_txn_depth == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    --_rx_txn_depth;
    if (_rx_txn_depth != 0) {
        return _rx_txn_totals;
    }

    const auto totals = _rx_txn_totals;
    auto ea           = endAccess();
    _rx_txn_totals.clear();
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return totals;
}

result_t<size_t> RxAccessor::read(data::DataSpan dst_bytes)
{
    data::MemorySink sink{dst_bytes};
    return read(sink, dst_bytes.size);
}

result_t<size_t> RxAccessor::read(data::Sink& dst, size_t len)
{
    return bus::guarded([&] { return beginTransaction(); },
                        [&] {
                            auto r = getBus().read(this, _access_config, &dst, len);
                            if (r.has_value()) {
                                _rx_txn_totals.rx += r.value();
                            }
                            return r;
                        },
                        [&] { return endTransaction(); });
}

result_t<size_t> RxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<size_t> RxAccessor::readableBytes(void)
{
    return bus::guarded([&] { return beginAccess(); }, [&] { return getBus().readableBytes(this, _access_config); },
                        [&] { return endAccess(); });
}

Accessor::Accessor(IBus& bus, const AccessConfig& access_config) : _tx{bus, access_config}, _rx{bus, access_config}
{
}

Accessor::Accessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : _tx{bus, access_config}, _rx{std::move(bus), access_config}
{
}

IBus& Accessor::getBus(void) const
{
    return _tx.getBus();
}

result_t<void> Accessor::setConfig(const AccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
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
    auto tx_result       = _tx.beginAccess(timeout_ms);
    if (!tx_result.has_value()) {
        return tx_result;
    }
    // Spend what is left of the budget on the RX lock so the call as a whole
    // honours timeout_ms (an infinite budget stays infinite).
    uint32_t remaining = timeout_ms;
    if (timeout_ms != 0 && timeout_ms != types::TIMEOUT_FOREVER) {
        const uint32_t elapsed = runtime::millis() - start;
        remaining              = (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0;
    }
    auto rx_result = _rx.beginAccess(remaining);
    if (!rx_result.has_value()) {
        (void)_tx.endAccess();  // rollback; the primary (rx) error takes priority
        return rx_result;
    }
    return {};
}

result_t<void> Accessor::endAccess(void)
{
    auto rx_result = _rx.endAccess();
    auto tx_result = _tx.endAccess();
    if (!rx_result.has_value()) {
        return rx_result;
    }
    return tx_result;
}

result_t<size_t> Accessor::write(data::ConstDataSpan src_bytes)
{
    return _tx.write(src_bytes);
}

result_t<size_t> Accessor::write(data::Source& src, size_t len)
{
    return _tx.write(src, len);
}

result_t<size_t> Accessor::write(const uint8_t* src, size_t len)
{
    return _tx.write(src, len);
}

result_t<size_t> Accessor::writableBytes(void)
{
    return _tx.writableBytes();
}

result_t<size_t> Accessor::read(data::DataSpan dst_bytes)
{
    return _rx.read(dst_bytes);
}

result_t<size_t> Accessor::read(data::Sink& dst, size_t len)
{
    return _rx.read(dst, len);
}

result_t<size_t> Accessor::read(uint8_t* dst, size_t len)
{
    return _rx.read(dst, len);
}

result_t<bus::TransferTotals> Accessor::transfer(data::Source& src, size_t tx_len, data::Sink& dst, size_t rx_len)
{
    if (tx_len == 0 && rx_len == 0) {
        return bus::TransferTotals{};
    }

    auto begin = [&]() -> result_t<void> {
        if (tx_len != 0) {
            auto tx_result = _tx.beginAccess();
            if (!tx_result.has_value()) {
                return tx_result;
            }
        }
        if (rx_len != 0) {
            auto rx_result = _rx.beginAccess();
            if (!rx_result.has_value()) {
                if (tx_len != 0) {
                    (void)_tx.endAccess();
                }
                return rx_result;
            }
        }
        return {};
    };

    auto body = [&]() -> result_t<bus::TransferTotals> {
        // Direction-specific config fields follow their channel, matching
        // the split write/read semantics: write_timeout_ms comes from the
        // TX accessor, read_timeout_ms from the RX accessor. Shared stream
        // parameters (sample rate, width) come from the active-TX side.
        const bool use_tx     = tx_len != 0;
        bus::IAccessor* owner = use_tx ? static_cast<bus::IAccessor*>(&_tx) : static_cast<bus::IAccessor*>(&_rx);
        AccessConfig cfg      = use_tx ? _tx.getConfig() : _rx.getConfig();
        if (use_tx && rx_len != 0) {
            cfg.read_timeout_ms = _rx.getConfig().read_timeout_ms;
        }
        return getBus().transfer(owner, cfg, use_tx ? &src : nullptr, tx_len, rx_len != 0 ? &dst : nullptr, rx_len);
    };

    auto end = [&]() -> result_t<void> {
        result_t<void> rx_result{};
        if (rx_len != 0) {
            rx_result = _rx.endAccess();
        }
        result_t<void> tx_result{};
        if (tx_len != 0) {
            tx_result = _tx.endAccess();
        }
        if (!rx_result.has_value()) {
            return rx_result;
        }
        return tx_result;
    };

    return bus::guarded(begin, body, end);
}

result_t<bus::TransferTotals> Accessor::transfer(data::ConstDataSpan src_bytes, data::DataSpan dst_bytes)
{
    data::MemorySource src{src_bytes};
    data::MemorySink dst{dst_bytes};
    return transfer(src, src_bytes.size, dst, dst_bytes.size);
}

result_t<size_t> Accessor::readableBytes(void)
{
    return _rx.readableBytes();
}

result_t<size_t> IBus::write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)src;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> IBus::writableBytes(bus::IAccessor* owner, const AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> IBus::read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* dst, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)dst;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<bus::TransferTotals> IBus::transfer(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src,
                                             size_t tx_len, data::Sink* dst, size_t rx_len)
{
    bus::TransferTotals totals{};
    if (tx_len != 0) {
        auto written = write(owner, cfg, src, tx_len);
        if (!written.has_value()) {
            return m5::stl::make_unexpected(written.error());
        }
        totals.tx = written.value();
    }
    if (rx_len != 0) {
        auto read_result = read(owner, cfg, dst, rx_len);
        if (!read_result.has_value()) {
            return m5::stl::make_unexpected(read_result.error());
        }
        totals.rx = read_result.value();
    }
    return totals;
}

result_t<size_t> IBus::readableBytes(bus::IAccessor* owner, const AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<void> IBus::lock(bus::IAccessor* owner, uint32_t timeout_ms)
{
    return lockChannel(owner, Channel::TxRx, timeout_ms);
}

result_t<void> IBus::unlock(bus::IAccessor* owner)
{
    return unlockChannel(owner, Channel::TxRx);
}

result_t<void> IBus::lockChannel(bus::IAccessor* owner, Channel ch, uint32_t timeout_ms)
{
    if (owner == nullptr || (!hasChannel(ch, Channel::Tx) && !hasChannel(ch, Channel::Rx))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    const uint32_t start = runtime::millis();
    if (hasChannel(ch, Channel::Tx)) {
        if (!_tx_mutex.lock(timeout_ms)) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        _tx_lock_owner = owner;
    }
    if (hasChannel(ch, Channel::Rx)) {
        // The composite (txrx) lock spends what is left of the budget on the
        // second channel, so the call as a whole honours timeout_ms (an
        // infinite budget stays infinite).
        uint32_t remaining = timeout_ms;
        if (timeout_ms != 0 && timeout_ms != types::TIMEOUT_FOREVER) {
            const uint32_t elapsed = runtime::millis() - start;
            remaining              = (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0;
        }
        if (!_rx_mutex.lock(remaining)) {
            if (hasChannel(ch, Channel::Tx)) {
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
    if (owner == nullptr || (!hasChannel(ch, Channel::Tx) && !hasChannel(ch, Channel::Rx))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::Tx) && _tx_lock_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::Rx) && _rx_lock_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (hasChannel(ch, Channel::Tx)) {
        _tx_lock_owner = nullptr;  // cleared while the mutex is still held
        _tx_mutex.unlock();
    }
    if (hasChannel(ch, Channel::Rx)) {
        _rx_lock_owner = nullptr;
        _rx_mutex.unlock();
    }
    return {};
}

}  // namespace m5::hal::v2::i2s

#endif
