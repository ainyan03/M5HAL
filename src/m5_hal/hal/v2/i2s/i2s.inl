// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2S_I2S_INL_
#define M5_HAL_I2S_I2S_INL_

#include "i2s.hpp"

namespace m5::hal::v2::i2s {

TxAccessor::TxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _context{makeOperationContext(access_config)}
{
}

TxAccessor::TxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _context{makeOperationContext(access_config)}
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
    _context.config = cfg;
    return {};
}

result_t<void> TxAccessor::beginAccess(uint32_t timeout_ms)
{
    return _beginOperationAccess(
        _context, timeout_ms, bus::OperationMode::Tx,
        [&](uint32_t remaining_ms) { return getBus().lockChannel(*this, Channel::Tx, remaining_ms); },
        [&] { return getBus().unlockChannel(*this, Channel::Tx); },
        [&](auto& context) { return getBus().beginOperation(context); });
}

result_t<void> TxAccessor::endAccess(uint32_t timeout_ms)
{
    return _endOperationAccess(
        _context, timeout_ms, [&](auto& context) { return getBus().endOperation(context); },
        [&] { return getBus().unlockChannel(*this, Channel::Tx); });
}

result_t<bus::TransferStatus> TxAccessor::getLastTransferStatus(void) const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

void TxAccessor::recordTransferResult(const result_t<size_t>& result, size_t requested)
{
    if (++_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;
    if (!result.has_value()) {
        _last_transfer_status.error      = result.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return;
    }
    _last_transfer_status.totals.tx = result.value();
    _last_transfer_status.completion =
        result.value() == requested ? bus::CompletionLevel::Complete : bus::CompletionLevel::Partial;
}

result_t<size_t> TxAccessor::write(data::ConstDataSpan src_bytes)
{
    if (src_bytes.size != 0 && src_bytes.data == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    data::MemorySource src{src_bytes};
    return write(src, src_bytes.size);
}

result_t<size_t> TxAccessor::write(data::Source& src, size_t len)
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto written = writeInCurrentAccess(src, len);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!written.has_value()) {
        return m5::stl::make_unexpected(written.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return written.value();
}

result_t<size_t> TxAccessor::writeInCurrentAccess(data::Source& src, size_t len)
{
    if (!inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto written = getBus().write(_context, &src, len);
    recordTransferResult(written, len);
    return written;
}

result_t<size_t> TxAccessor::write(const uint8_t* src, size_t len)
{
    return write(data::ConstDataSpan{src, len});
}

result_t<size_t> TxAccessor::writableBytes(void)
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto writable = getBus().writableBytes(_context);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!writable.has_value()) {
        return m5::stl::make_unexpected(writable.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return writable.value();
}

RxAccessor::RxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _context{makeOperationContext(access_config)}
{
}

RxAccessor::RxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _context{makeOperationContext(access_config)}
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
    _context.config = cfg;
    return {};
}

result_t<void> RxAccessor::beginAccess(uint32_t timeout_ms)
{
    return _beginOperationAccess(
        _context, timeout_ms, bus::OperationMode::Rx,
        [&](uint32_t remaining_ms) { return getBus().lockChannel(*this, Channel::Rx, remaining_ms); },
        [&] { return getBus().unlockChannel(*this, Channel::Rx); },
        [&](auto& context) { return getBus().beginOperation(context); });
}

result_t<void> RxAccessor::endAccess(uint32_t timeout_ms)
{
    return _endOperationAccess(
        _context, timeout_ms, [&](auto& context) { return getBus().endOperation(context); },
        [&] { return getBus().unlockChannel(*this, Channel::Rx); });
}

result_t<bus::TransferStatus> RxAccessor::getLastTransferStatus(void) const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

void RxAccessor::recordTransferResult(const result_t<size_t>& result, size_t requested)
{
    if (++_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;
    if (!result.has_value()) {
        _last_transfer_status.error      = result.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return;
    }
    _last_transfer_status.totals.rx = result.value();
    _last_transfer_status.completion =
        result.value() == requested ? bus::CompletionLevel::Complete : bus::CompletionLevel::Partial;
}

result_t<size_t> RxAccessor::read(data::DataSpan dst_bytes)
{
    if (dst_bytes.size != 0 && dst_bytes.data == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    data::MemorySink sink{dst_bytes};
    return read(sink, dst_bytes.size);
}

result_t<size_t> RxAccessor::read(data::Sink& dst, size_t len)
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto read_result = readInCurrentAccess(dst, len);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!read_result.has_value()) {
        return m5::stl::make_unexpected(read_result.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return read_result.value();
}

result_t<size_t> RxAccessor::readInCurrentAccess(data::Sink& dst, size_t len)
{
    if (!inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto read_result = getBus().read(_context, &dst, len);
    recordTransferResult(read_result, len);
    return read_result;
}

result_t<size_t> RxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<size_t> RxAccessor::readableBytes(void)
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto readable = getBus().readableBytes(_context);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!readable.has_value()) {
        return m5::stl::make_unexpected(readable.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return readable.value();
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
    if (_combined_active || _tx.inAccess() || _rx.inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
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
    _combined_active = true;
    return {};
}

result_t<void> Accessor::endAccess(uint32_t timeout_ms)
{
    if (!_combined_active) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const uint32_t start = runtime::millis();
    auto rx_result       = _rx.endAccess(timeout_ms);
    uint32_t remaining   = timeout_ms;
    if (timeout_ms != types::TIMEOUT_FOREVER) {
        const uint32_t elapsed = runtime::millis() - start;
        remaining              = elapsed < timeout_ms ? timeout_ms - elapsed : 0;
    }
    auto tx_result   = _tx.endAccess(remaining);
    _combined_active = false;
    if (!rx_result.has_value()) {
        return rx_result;
    }
    return tx_result;
}

result_t<size_t> Accessor::write(data::ConstDataSpan src_bytes)
{
    data::MemorySource src{src_bytes};
    return write(src, src_bytes.size);
}

result_t<size_t> Accessor::write(data::Source& src, size_t len)
{
    auto before = _tx.getLastTransferStatus();
    auto result = _tx.write(src, len);
    adoptTransferStatus(before, _tx.getLastTransferStatus());
    return result;
}

result_t<size_t> Accessor::write(const uint8_t* src, size_t len)
{
    return write(data::ConstDataSpan{src, len});
}

result_t<size_t> Accessor::writableBytes(void)
{
    return _tx.writableBytes();
}

result_t<size_t> Accessor::read(data::DataSpan dst_bytes)
{
    data::MemorySink dst{dst_bytes};
    return read(dst, dst_bytes.size);
}

result_t<size_t> Accessor::read(data::Sink& dst, size_t len)
{
    auto before = _rx.getLastTransferStatus();
    auto result = _rx.read(dst, len);
    adoptTransferStatus(before, _rx.getLastTransferStatus());
    return result;
}

result_t<size_t> Accessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<bus::TransferTotals> Accessor::transfer(data::Source& src, size_t tx_len, data::Sink& dst, size_t rx_len)
{
    if (tx_len == 0 && rx_len == 0) {
        return bus::TransferTotals{};
    }

    const bool tx_borrowed = tx_len == 0 || _tx.inAccess();
    const bool rx_borrowed = rx_len == 0 || _rx.inAccess();
    bool tx_opened         = false;
    bool rx_opened         = false;

    auto begin = [&]() -> result_t<void> {
        if (tx_len != 0 && !tx_borrowed) {
            auto tx_result = _tx.beginAccess();
            if (!tx_result.has_value()) {
                return tx_result;
            }
            tx_opened = true;
        }
        if (rx_len != 0 && !rx_borrowed) {
            auto rx_result = _rx.beginAccess();
            if (!rx_result.has_value()) {
                if (tx_opened) {
                    (void)_tx.endAccess();
                    tx_opened = false;
                }
                return rx_result;
            }
            rx_opened = true;
        }
        return {};
    };

    auto body = [&]() -> result_t<bus::TransferTotals> {
        result_t<bus::TransferTotals> transferred;
        if (tx_len == 0) {
            auto read_result = getBus().read(_rx._context, &dst, rx_len);
            if (read_result.has_value()) {
                transferred = bus::TransferTotals{0, read_result.value()};
            } else {
                transferred = m5::stl::make_unexpected(read_result.error());
            }
        } else if (rx_len == 0) {
            auto write_result = getBus().write(_tx._context, &src, tx_len);
            if (write_result.has_value()) {
                transferred = bus::TransferTotals{write_result.value(), 0};
            } else {
                transferred = m5::stl::make_unexpected(write_result.error());
            }
        } else {
            transferred = getBus().transfer(_tx._context, _rx._context, &src, tx_len, &dst, rx_len);
        }
        recordTransferResult(transferred, tx_len, rx_len);
        return transferred;
    };

    auto end = [&]() -> result_t<void> {
        result_t<void> rx_result{};
        if (rx_opened) {
            rx_result = _rx.endAccess();
        }
        result_t<void> tx_result{};
        if (tx_opened) {
            tx_result = _tx.endAccess();
        }
        if (!rx_result.has_value()) {
            return rx_result;
        }
        return tx_result;
    };

    return bus::guarded(begin, body, end);
}

result_t<bus::TransferStatus> Accessor::getLastTransferStatus(void) const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

void Accessor::adoptTransferStatus(const result_t<bus::TransferStatus>& before,
                                   const result_t<bus::TransferStatus>& after)
{
    if (!after.has_value() || (before.has_value() && before->transfer_id == after->transfer_id)) {
        return;
    }
    if (++_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = after.value();
    _last_transfer_status.transfer_id = _next_transfer_id;
}

void Accessor::recordTransferResult(const result_t<bus::TransferTotals>& result, size_t requested_tx,
                                    size_t requested_rx)
{
    if (++_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;
    if (!result.has_value()) {
        _last_transfer_status.error      = result.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return;
    }
    _last_transfer_status.totals     = result.value();
    _last_transfer_status.completion = result->tx == requested_tx && result->rx == requested_rx
                                           ? bus::CompletionLevel::Complete
                                           : bus::CompletionLevel::Partial;
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

bus::OperationSlot* IBus::operationSlot(bus::OperationContext<AccessConfig>& context)
{
    if (context.runtime.mode == bus::OperationMode::Tx) {
        return &_tx_operation_slot;
    }
    if (context.runtime.mode == bus::OperationMode::Rx) {
        return &_rx_operation_slot;
    }
    return nullptr;
}

const bus::OperationSlot* IBus::operationSlot(const bus::OperationContext<AccessConfig>& context) const
{
    if (context.runtime.mode == bus::OperationMode::Tx) {
        return &_tx_operation_slot;
    }
    if (context.runtime.mode == bus::OperationMode::Rx) {
        return &_rx_operation_slot;
    }
    return nullptr;
}

bus::IAccessor* IBus::operationOwner(bus::OperationContext<AccessConfig>& context)
{
    return &bus::OperationSlot::contextOwner(context);
}

result_t<void> IBus::beginOperation(bus::OperationContext<AccessConfig>& context)
{
    auto* slot            = operationSlot(context);
    bus::IAccessor* owner = context.runtime.mode == bus::OperationMode::Tx   ? _tx_lock_owner
                            : context.runtime.mode == bus::OperationMode::Rx ? _rx_lock_owner
                                                                             : nullptr;
    if (slot == nullptr || owner == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto registered = slot->registerContext(context, this, owner);
    if (!registered.has_value()) {
        return registered;
    }
    auto begun = beginOperationBackend(context);
    if (!begun.has_value()) {
        slot->invalidate(context);
    }
    return begun;
}

result_t<void> IBus::endOperation(bus::OperationContext<AccessConfig>& context)
{
    const auto registered_mode = bus::OperationSlot::registeredMode(context);
    bus::OperationSlot* slot   = registered_mode == bus::OperationMode::Tx   ? &_tx_operation_slot
                                 : registered_mode == bus::OperationMode::Rx ? &_rx_operation_slot
                                                                             : nullptr;
    bus::IAccessor* live_owner = slot == &_tx_operation_slot   ? _tx_lock_owner
                                 : slot == &_rx_operation_slot ? _rx_lock_owner
                                                               : nullptr;
    if (slot == nullptr || !slot->valid(context, this, live_owner)) {
        if (slot != nullptr) {
            if (slot->restoreRegisteredRuntime(context, this, live_owner)) {
                (void)endOperationBackend(context);
            }
            slot->invalidate(context);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto ended = endOperationBackend(context);
    slot->invalidate(context);
    return ended;
}

result_t<size_t> IBus::write(bus::OperationContext<AccessConfig>& context, data::Source* src, size_t len)
{
    if (!_tx_operation_slot.valid(context, this, _tx_lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return writeBackend(context, src, len);
}

result_t<size_t> IBus::writableBytes(bus::OperationContext<AccessConfig>& context)
{
    if (!_tx_operation_slot.valid(context, this, _tx_lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return writableBytesBackend(context);
}

result_t<size_t> IBus::read(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    if (!_rx_operation_slot.valid(context, this, _rx_lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return readBackend(context, dst, len);
}

result_t<bus::TransferTotals> IBus::transfer(bus::OperationContext<AccessConfig>& tx_context,
                                             bus::OperationContext<AccessConfig>& rx_context, data::Source* src,
                                             size_t tx_len, data::Sink* dst, size_t rx_len)
{
    if (!_tx_operation_slot.valid(tx_context, this, _tx_lock_owner) ||
        !_rx_operation_slot.valid(rx_context, this, _rx_lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return transferBackend(tx_context, rx_context, src, tx_len, dst, rx_len);
}

result_t<size_t> IBus::readableBytes(bus::OperationContext<AccessConfig>& context)
{
    if (!_rx_operation_slot.valid(context, this, _rx_lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return readableBytesBackend(context);
}

result_t<void> IBus::beginOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return {};
}

result_t<void> IBus::endOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return {};
}

result_t<size_t> IBus::writeBackend(bus::OperationContext<AccessConfig>& context, data::Source* src, size_t len)
{
    (void)context;
    (void)src;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<size_t> IBus::writableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<size_t> IBus::readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    (void)context;
    (void)dst;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<bus::TransferTotals> IBus::transferBackend(bus::OperationContext<AccessConfig>& tx_context,
                                                    bus::OperationContext<AccessConfig>& rx_context, data::Source* src,
                                                    size_t tx_len, data::Sink* dst, size_t rx_len)
{
    bus::TransferTotals totals{};
    auto written = writeBackend(tx_context, src, tx_len);
    if (!written.has_value()) {
        return m5::stl::make_unexpected(written.error());
    }
    totals.tx        = written.value();
    auto read_result = readBackend(rx_context, dst, rx_len);
    if (!read_result.has_value()) {
        return m5::stl::make_unexpected(read_result.error());
    }
    totals.rx = read_result.value();
    return totals;
}

result_t<size_t> IBus::readableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<void> IBus::acquireAccessLock(bus::IAccessor& owner, uint32_t timeout_ms)
{
    return lockChannel(owner, Channel::TxRx, timeout_ms);
}

result_t<void> IBus::releaseAccessLock(bus::IAccessor& owner)
{
    return unlockChannel(owner, Channel::TxRx);
}

result_t<void> IBus::lockChannel(bus::IAccessor& owner, Channel ch, uint32_t timeout_ms)
{
    if (!hasChannel(ch, Channel::Tx) && !hasChannel(ch, Channel::Rx)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (!accessLifecycleOpen()) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }

    const uint32_t start = runtime::millis();
    if (hasChannel(ch, Channel::Tx)) {
        auto locked = _tx_mutex.lock(timeout_ms);
        if (!locked.has_value()) {
            if (!accessLifecycleOpen()) {
                return m5::stl::make_unexpected(error::error_t::CLOSED);
            }
            return m5::stl::make_unexpected(locked.error());
        }
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
        auto locked = _rx_mutex.lock(remaining);
        if (!locked.has_value()) {
            if (hasChannel(ch, Channel::Tx)) {
                auto rolled_back = _tx_mutex.unlock();
                if (!rolled_back.has_value()) {
                    markAccessBroken();
                }
            }
            if (!accessLifecycleOpen()) {
                return m5::stl::make_unexpected(error::error_t::CLOSED);
            }
            return m5::stl::make_unexpected(locked.error());
        }
    }
    if (!accessLifecycleOpen()) {
        error::error_t cleanup_error = error::error_t::OK;
        if (hasChannel(ch, Channel::Rx)) {
            auto unlocked = _rx_mutex.unlock();
            if (!unlocked.has_value()) {
                cleanup_error = unlocked.error();
            }
        }
        if (hasChannel(ch, Channel::Tx)) {
            auto unlocked = _tx_mutex.unlock();
            if (!unlocked.has_value() && cleanup_error == error::error_t::OK) {
                cleanup_error = unlocked.error();
            }
        }
        if (cleanup_error != error::error_t::OK) {
            markAccessBroken();
        }
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    if (hasChannel(ch, Channel::Tx)) {
        _tx_lock_owner = &owner;
        _tx_lock_task.store(runtime::currentTaskId(), std::memory_order_relaxed);
    }
    if (hasChannel(ch, Channel::Rx)) {
        _rx_lock_owner = &owner;
        _rx_lock_task.store(runtime::currentTaskId(), std::memory_order_relaxed);
    }
    return {};
}

result_t<void> IBus::tryAcquireCloseBarrier(void)
{
    const void* current = runtime::currentTaskId();
    if (_tx_lock_task.load(std::memory_order_relaxed) == current ||
        _rx_lock_task.load(std::memory_order_relaxed) == current) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    auto tx_locked = _tx_mutex.lock(0);
    if (!tx_locked.has_value()) {
        return m5::stl::make_unexpected(tx_locked.error());
    }
    auto rx_locked = _rx_mutex.lock(0);
    if (!rx_locked.has_value()) {
        auto rolled_back = _tx_mutex.unlock();
        if (!rolled_back.has_value()) {
            quarantineLifecycleAfterPartialTeardown();
            return m5::stl::make_unexpected(rolled_back.error());
        }
        return m5::stl::make_unexpected(rx_locked.error());
    }
    return {};
}

result_t<void> IBus::releaseCloseBarrier(void)
{
    auto rx_unlocked = _rx_mutex.unlock();
    auto tx_unlocked = _tx_mutex.unlock();
    if (!rx_unlocked.has_value()) {
        return m5::stl::make_unexpected(rx_unlocked.error());
    }
    return tx_unlocked;
}

result_t<void> IBus::unlockChannel(bus::IAccessor& owner, Channel ch)
{
    if (!hasChannel(ch, Channel::Tx) && !hasChannel(ch, Channel::Rx)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::Tx) && _tx_lock_owner != &owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (hasChannel(ch, Channel::Rx) && _rx_lock_owner != &owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    error::error_t first_error = error::error_t::OK;
    if (hasChannel(ch, Channel::Tx)) {
        _tx_lock_owner = nullptr;  // cleared while the mutex is still held
        _tx_lock_task.store(nullptr, std::memory_order_relaxed);
        auto unlocked = _tx_mutex.unlock();
        if (!unlocked.has_value()) {
            first_error = unlocked.error();
        }
    }
    if (hasChannel(ch, Channel::Rx)) {
        _rx_lock_owner = nullptr;
        _rx_lock_task.store(nullptr, std::memory_order_relaxed);
        auto unlocked = _rx_mutex.unlock();
        if (!unlocked.has_value() && first_error == error::error_t::OK) {
            first_error = unlocked.error();
        }
    }
    if (first_error != error::error_t::OK) {
        markAccessBroken();
        return m5::stl::make_unexpected(first_error);
    }
    return {};
}

}  // namespace m5::hal::v2::i2s

#endif
