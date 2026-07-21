// SPDX-License-Identifier: MIT

#include "i2c.hpp"
#include "../error.hpp"

namespace m5::hal::v2::i2c {

MasterAccessor::MasterAccessor(IBus& bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{bus}, _context{makeOperationContext(access_config)}
{
}

MasterAccessor::MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _context{makeOperationContext(access_config)}
{
}

IBus& MasterAccessor::getBus(void) const
{
    // `_bus` was upcast from IBus& in the ctor, so the static_cast back is safe.
    return static_cast<IBus&>(*_bus);
}

bool MasterAccessor::transferBusy(void)
{
    return getBus().transferBusy(_context);
}

m5::hal::v2::result_t<void> MasterAccessor::setConfig(const MasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    _context.config = cfg;
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::beginAccess(uint32_t timeout_ms)
{
    return _beginOperationAccess(_context, timeout_ms, bus::OperationMode::TxRx,
                                 [&](auto& context) { return getBus().beginOperation(context); });
}

m5::hal::v2::result_t<void> MasterAccessor::endAccess(uint32_t timeout_ms)
{
    return _endOperationAccess(_context, timeout_ms, [&](auto& context) {
        error::error_t wait_error = error::error_t::OK;
        if (getBus().transferBusy(context)) {
            auto waited = getBus().waitTransfer(context);
            if (!waited.has_value()) {
                wait_error = waited.error();
            }
        }
        auto ended = getBus().endOperation(context);
        if (error::isError(wait_error)) {
            return result_t<void>{m5::stl::make_unexpected(wait_error)};
        }
        return ended;
    });
}

m5::hal::v2::result_t<bus::TransferStatus> MasterAccessor::getLastTransferStatus(void) const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transfer(const TransferDesc& desc,
                                                                    data::ConstDataSpan src_bytes,
                                                                    data::DataSpan dst_bytes)
{
    if (!_inOperationAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _span_src = data::MemorySource{src_bytes};
    _span_dst = data::MemorySink{dst_bytes};
    return startTransfer(desc, src_bytes.size ? &_span_src : nullptr, src_bytes.size,
                         dst_bytes.size ? &_span_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transfer(const TransferDesc& desc, data::Source* src,
                                                                    size_t tx_len, data::Sink* dst, size_t rx_len)
{
    if (!_inOperationAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return startTransfer(desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::startTransfer(const TransferDesc& desc, data::Source* src,
                                                                         size_t tx_len, data::Sink* dst, size_t rx_len)
{
    ++_next_transfer_id;
    if (_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;

    if (getBus().transferBusy(_context)) {
        auto previous = getBus().waitTransfer(_context);
        if (!previous.has_value()) {
            _last_transfer_status.error      = previous.error();
            _last_transfer_status.completion = bus::CompletionLevel::Aborted;
            return m5::stl::make_unexpected(previous.error());
        }
    }

    auto started = getBus().transfer(_context, desc, (src != nullptr && tx_len > 0) ? src : nullptr, tx_len,
                                     (dst != nullptr && rx_len > 0) ? dst : nullptr, rx_len);
    if (!started.has_value()) {
        _last_transfer_status.error = started.error();
        return m5::stl::make_unexpected(started.error());
    }
    _last_transfer_status.completion = bus::CompletionLevel::Accepted;

    auto waited = getBus().waitTransfer(_context);
    if (!waited.has_value()) {
        _last_transfer_status.error      = waited.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return m5::stl::make_unexpected(waited.error());
    }
    _last_transfer_status.totals     = waited.value();
    _last_transfer_status.completion = bus::CompletionLevel::Complete;
    return waited.value();
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transferSync(const TransferDesc& desc,
                                                                        data::ConstDataSpan src_bytes,
                                                                        data::DataSpan dst_bytes)
{
    data::MemorySource tx_src{src_bytes};
    data::MemorySink rx_dst{dst_bytes};
    return transferSync(desc, (src_bytes.size > 0) ? &tx_src : nullptr, src_bytes.size,
                        (dst_bytes.size > 0) ? &rx_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transferSync(const TransferDesc& desc, data::Source* src,
                                                                        size_t tx_len, data::Sink* dst, size_t rx_len)
{
    const bool borrowed = _inOperationAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto transferred = transfer(desc, src, tx_len, dst, rx_len);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!transferred.has_value()) {
        return m5::stl::make_unexpected(transferred.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return transferred.value();
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(data::Source& src, size_t len)
{
    auto r = transferSync(TransferDesc{}, &src, len, nullptr, 0);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(data::Sink& dst, size_t len)
{
    auto r = transferSync(TransferDesc{}, nullptr, 0, &dst, len);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(data::ConstDataSpan src_bytes)
{
    auto r = transferSync(TransferDesc{}, src_bytes, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(data::DataSpan dst_bytes)
{
    auto r = transferSync(TransferDesc{}, data::ConstDataSpan{}, dst_bytes);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(const uint8_t* src, size_t len)
{
    return write(data::ConstDataSpan{src, len});
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::hal::v2::result_t<void> MasterAccessor::probe(void)
{
    auto r = transferSync(TransferDesc{}, data::ConstDataSpan{}, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return {};
}

m5::hal::v2::result_t<void> IBus::probe(uint16_t i2c_addr, uint32_t freq, uint32_t timeout_ms)
{
    MasterAccessConfig cfg;
    cfg.i2c_addr = i2c_addr;
    cfg.freq     = freq;
    // Keep scans fast on stuck buses: the probe budget also bounds the
    // wire-level wait, not just the lock acquisition below.
    cfg.wire_timeout_ms = timeout_ms;
    MasterAccessor sentinel{*this, cfg};
    return bus::guarded([&] { return sentinel.beginAccess(timeout_ms); }, [&] { return sentinel.probe(); },
                        [&] { return sentinel.endAccess(); });
}

m5::hal::v2::result_t<void> IBus::beginOperation(bus::OperationContext<MasterAccessConfig>& context)
{
    auto registered = _operation_slot.registerContext(context, this, _lock_owner);
    if (!registered.has_value()) {
        return registered;
    }
    auto begun = beginOperationBackend(context);
    if (!begun.has_value()) {
        _operation_slot.invalidate(context);
    }
    return begun;
}

m5::hal::v2::result_t<void> IBus::endOperation(bus::OperationContext<MasterAccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        if (_operation_slot.registered(context, this)) {
            if (_operation_slot.restoreRegisteredRuntime(context, this, _lock_owner)) {
                (void)endOperationBackend(context);
            }
            _operation_slot.invalidate(context);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto ended = endOperationBackend(context);
    _operation_slot.invalidate(context);
    return ended;
}

m5::hal::v2::result_t<void> IBus::transfer(bus::OperationContext<MasterAccessConfig>& context, const TransferDesc& desc,
                                           data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return transferBackend(context, desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransfer(bus::OperationContext<MasterAccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return waitTransferBackend(context);
}

bool IBus::transferBusy(bus::OperationContext<MasterAccessConfig>& context)
{
    return _operation_slot.valid(context, this, _lock_owner) && transferBusyBackend(context);
}

m5::hal::v2::result_t<void> IBus::beginOperationBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return {};
}

m5::hal::v2::result_t<void> IBus::endOperationBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return {};
}

m5::hal::v2::result_t<void> IBus::transferBackend(bus::OperationContext<MasterAccessConfig>& context,
                                                  const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                  data::Sink* dst, size_t rx_len)
{
    (void)context;
    (void)desc;
    (void)src;
    (void)tx_len;
    (void)dst;
    (void)rx_len;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransferBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return bus::TransferTotals{};
}

bool IBus::transferBusyBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return false;
}

}  // namespace m5::hal::v2::i2c
