// SPDX-License-Identifier: MIT

#include "i2c.hpp"
#include "../error.hpp"

namespace m5::hal::v2::i2c {

namespace {

bus::TransferTotals diffTotals(const bus::TransferTotals& after, const bus::TransferTotals& before)
{
    return bus::TransferTotals{after.tx - before.tx, after.rx - before.rx};
}

}  // namespace

MasterAccessor::MasterAccessor(IBus& bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

MasterAccessor::MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _access_config{access_config}
{
}

IBus& MasterAccessor::getBus(void) const
{
    // `_bus` was upcast from IBus& in the ctor, so the static_cast back is safe.
    return static_cast<IBus&>(*_bus);
}

bool MasterAccessor::transferBusy(void)
{
    return getBus().transferBusy(this);
}

m5::hal::v2::result_t<void> MasterAccessor::waitTransfer(void)
{
    auto waited = getBus().waitTransfer(this, _access_config);
    if (!waited.has_value()) {
        const auto err     = waited.error();
        _transaction_error = err;
        return m5::stl::make_unexpected(err);
    }
    _transaction_totals.add(waited.value());
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::setConfig(const MasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    _access_config = cfg;
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::transfer(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                                     data::DataSpan dst_bytes)
{
    if (!inTransaction()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (error::isError(_transaction_error)) {
        return m5::stl::make_unexpected(_transaction_error);
    }
    auto waited = waitTransfer();
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }
    _span_src = data::MemorySource{src_bytes};
    _span_dst = data::MemorySink{dst_bytes};
    return startTransfer(desc, src_bytes.size ? &_span_src : nullptr, src_bytes.size,
                         dst_bytes.size ? &_span_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<void> MasterAccessor::transfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                     data::Sink* dst, size_t rx_len)
{
    if (!inTransaction()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (error::isError(_transaction_error)) {
        return m5::stl::make_unexpected(_transaction_error);
    }

    auto waited = waitTransfer();
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }

    return startTransfer(desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<void> MasterAccessor::startTransfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                          data::Sink* dst, size_t rx_len)
{
    auto r = getBus().transfer(this, _access_config, desc, (src != nullptr && tx_len > 0) ? src : nullptr, tx_len,
                               (dst != nullptr && rx_len > 0) ? dst : nullptr, rx_len);
    if (!r.has_value()) {
        // Latch like waitTransfer(): transaction segments form one logical
        // operation, so ANY failed segment — including a pre-flight
        // rejection that never touched the wire — invalidates the rest of
        // the transaction. Recovery is a fresh transaction. Contract:
        // spec/design/i2c.md §transaction 中のエラー.
        _transaction_error = r.error();
        return m5::stl::make_unexpected(r.error());
    }
    if (!transferBusy()) {
        auto done = waitTransfer();
        if (!done.has_value()) {
            return m5::stl::make_unexpected(done.error());
        }
    }
    return {};
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
    auto b = beginTransaction();
    if (!b.has_value()) {
        return m5::stl::make_unexpected(b.error());
    }
    const auto before = _transaction_totals;
    auto t            = transfer(desc, src, tx_len, dst, rx_len);
    auto e            = endTransaction();
    if (!t.has_value()) {
        return m5::stl::make_unexpected(t.error());
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error());
    }
    return diffTotals(e.value(), before);
}

m5::hal::v2::result_t<void> MasterAccessor::beginTransaction(uint32_t timeout_ms)
{
    if (_transaction_depth != 0) {
        ++_transaction_depth;
        return {};
    }

    auto ba = beginAccess(timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }

    _transaction_totals.clear();
    _transaction_error = error::error_t::OK;
    _transaction_depth = 1;
    return {};
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::endTransaction(void)
{
    if (_transaction_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }

    auto wait                    = waitTransfer();
    error::error_t primary_error = error::error_t::OK;
    if (!wait.has_value()) {
        primary_error = wait.error();
    } else if (error::isError(_transaction_error)) {
        primary_error = _transaction_error;
    }

    --_transaction_depth;
    if (_transaction_depth != 0) {
        if (error::isError(primary_error)) {
            return m5::stl::make_unexpected(primary_error);
        }
        return _transaction_totals;
    }

    const auto totals = _transaction_totals;
    auto ea           = endAccess();
    _transaction_totals.clear();
    _transaction_error = error::error_t::OK;
    if (error::isError(primary_error)) {
        return m5::stl::make_unexpected(primary_error);
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return totals;
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

m5::hal::v2::result_t<void> IBus::transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg,
                                           const TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                           size_t rx_len)
{
    (void)owner;
    (void)cfg;
    (void)desc;
    (void)src;
    (void)tx_len;
    (void)dst;
    (void)rx_len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransfer(bus::IAccessor* owner, const MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return bus::TransferTotals{};
}

bool IBus::transferBusy(bus::IAccessor* owner)
{
    (void)owner;
    return false;
}

}  // namespace m5::hal::v2::i2c
