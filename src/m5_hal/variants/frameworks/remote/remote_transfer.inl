// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_

#include "remote_transfer.hpp"

#include "../../../hal/v2/data/limited.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"
#include "detail_helpers.hpp"

#include <algorithm>

namespace m5::hal::v2::remote {

namespace detail {

inline result_t<void> gatherAtomicSource(data::Source* src, size_t len, data::DataSpan dst)
{
    if (len == 0) {
        return {};
    }
    if (src == nullptr || dst.data == nullptr || dst.size < len) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    size_t at = 0;
    while (at < len) {
        auto peeked = src->peek(len - at);
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        const size_t n = std::min(peeked->size, len - at);
        if (n == 0) {
            if (src->closed() || src->eof()) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_UNDERFLOW);
            }
            return m5::stl::make_unexpected(error::error_t::WOULD_BLOCK);
        }
        ::memcpy(dst.data + at, peeked->data, n);
        auto advanced = src->advance(n);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        at += n;
    }
    return {};
}

inline result_t<void> scatterAtomicResult(data::ConstDataSpan src, data::Sink* dst)
{
    if (src.size == 0) {
        return {};
    }
    if (dst == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    size_t at = 0;
    while (at < src.size) {
        auto reserved = dst->reserve(src.size - at);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }
        const size_t n = std::min(reserved->size, src.size - at);
        if (n == 0) {
            return m5::stl::make_unexpected(dst->closed() ? error::error_t::CLOSED : error::error_t::BUFFER_OVERFLOW);
        }
        ::memcpy(reserved->data, src.data + at, n);
        auto committed  = dst->commit(n);
        size_t accepted = committed.has_value() ? n : std::min(n, dst->partialCommitAccepted());
        at += accepted;
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
    }
    return {};
}

inline result_t<void> sendPendingConfig(RemoteSession& session, types::bus_kind_t kind, uint8_t bus_id,
                                        data::ConstDataSpan cfg_bytes, RemoteConfigCache* cache)
{
    const data::ConstDataSpan pending = cache == nullptr ? cfg_bytes : cache->pending(&session, cfg_bytes);
    if (pending.size == 0) {
        return {};
    }
    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto encoded = enc.configure(kind, bus_id, pending);
    if (encoded.has_value()) {
        encoded = enc.end();
    }
    if (!encoded.has_value()) {
        return m5::stl::make_unexpected(encoded.error());
    }
    auto requested = session.request({script_buf, script.written()});
    if (!requested.has_value()) {
        if (cache != nullptr) {
            cache->invalidate();
        }
        return m5::stl::make_unexpected(requested.error());
    }
    auto decoded = decodeResponseStatus(session.lastResponse());
    if (!decoded.has_value()) {
        if (cache != nullptr) {
            cache->invalidate();
        }
        return m5::stl::make_unexpected(decoded.error());
    }
    if (cache != nullptr) {
        cache->rememberSent(&session, pending);
    }
    return {};
}

template <typename Desc>
result_t<bus::TransferTotals> remoteAtomicTransferWireImpl(const std::shared_ptr<RemoteSessionHandle>& handle,
                                                           types::bus_kind_t kind, uint8_t bus_id,
                                                           data::ConstDataSpan cfg_bytes, const Desc& desc,
                                                           data::Source* src, size_t tx_len, size_t tx_limit,
                                                           data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                                           RemoteConfigCache* config_cache)
{
    if (!handle) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    if (tx_len > tx_limit || rx_len > kMaxTransferRx) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    if ((tx_len != 0 && src == nullptr) || (rx_len != 0 && dst == nullptr)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    RemoteSessionHandle::Lease lease{*handle};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }

    uint8_t tx_buf[kMaxAtomicI2CTxBase];
    auto gathered = gatherAtomicSource(src, tx_len, {tx_buf, sizeof(tx_buf)});
    if (!gathered.has_value()) {
        return m5::stl::make_unexpected(gathered.error());
    }

    RemoteSession& session                = lease.session();
    const RemoteSession::Config saved_cfg = session.getConfig();
    RemoteSession::Config transfer_cfg    = saved_cfg;
    transfer_cfg.response_timeout_ms      = timeout_ms;
    session.setConfig(transfer_cfg);

    auto configured = sendPendingConfig(session, kind, bus_id, cfg_bytes, config_cache);
    if (!configured.has_value()) {
        session.setConfig(saved_cfg);
        return m5::stl::make_unexpected(configured.error());
    }

    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto encoded = enc.transfer(bus_id, desc, {tx_buf, tx_len}, rx_len, kDefaultStoreId);
    if (encoded.has_value()) {
        encoded = enc.end();
    }
    if (!encoded.has_value()) {
        session.setConfig(saved_cfg);
        return m5::stl::make_unexpected(encoded.error());
    }

    auto requested = session.request({script_buf, script.written()});
    bytecode::BytecodeRunner response_runner{memory::defaultAllocator()};
    auto decoded = decodeResponseStatus(session.lastResponse(), &response_runner);
    session.setConfig(saved_cfg);
    if (!requested.has_value()) {
        if (config_cache != nullptr) {
            config_cache->invalidate();
        }
        return m5::stl::make_unexpected(requested.error());
    }
    if (!decoded.has_value()) {
        if (config_cache != nullptr) {
            config_cache->invalidate();
        }
        return m5::stl::make_unexpected(decoded.error());
    }

    const auto received = response_runner.storedData(kDefaultStoreId);
    if (received.size > rx_len) {
        if (config_cache != nullptr) {
            config_cache->invalidate();
        }
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    auto scattered = scatterAtomicResult(received, dst);
    if (!scattered.has_value()) {
        return m5::stl::make_unexpected(scattered.error());
    }
    return bus::TransferTotals{tx_len, received.size};
}

}  // namespace detail

result_t<bus::TransferTotals> remoteAtomicTransferWire(const std::shared_ptr<RemoteSessionHandle>& handle,
                                                       uint8_t bus_id, data::ConstDataSpan cfg_bytes,
                                                       const i2c::TransferDesc& desc, data::Source* src, size_t tx_len,
                                                       data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                                       RemoteConfigCache* config_cache)
{
    if (desc.prefix_len > i2c::TransferDesc::PREFIX_CAPACITY || desc.prefix_len > kMaxAtomicI2CTxBase) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return detail::remoteAtomicTransferWireImpl(handle, types::bus_kind_t::I2C, bus_id, cfg_bytes, desc, src, tx_len,
                                                kMaxAtomicI2CTxBase - desc.prefix_len, dst, rx_len, timeout_ms,
                                                config_cache);
}

result_t<bus::TransferTotals> remoteAtomicTransferWire(const std::shared_ptr<RemoteSessionHandle>& handle,
                                                       uint8_t bus_id, data::ConstDataSpan cfg_bytes,
                                                       const spi::TransferDesc& desc, data::Source* src, size_t tx_len,
                                                       data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                                       RemoteConfigCache* config_cache)
{
    return detail::remoteAtomicTransferWireImpl(handle, types::bus_kind_t::SPI, bus_id, cfg_bytes, desc, src, tx_len,
                                                kMaxAtomicSPITx, dst, rx_len, timeout_ms, config_cache);
}

result_t<bus::TransferTotals> remoteTransferWire(const std::shared_ptr<RemoteSessionHandle>& handle,
                                                 types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan cfg_bytes,
                                                 data::ConstDataSpan meta, data::Source* src, size_t tx_len,
                                                 data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                                 RemoteConfigCache* config_cache)
{
    if (!handle) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    if (tx_len > 0xFFFFFFFFu || rx_len > 0xFFFFFFFFu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    data::LimitedSource limited_src{src, tx_len};
    data::Source* attach_src = nullptr;
    if (tx_len > 0) {
        attach_src = &limited_src;
    }
    data::Sink* attach_dst = nullptr;
    if (rx_len > 0) {
        attach_dst = dst;
    }

    RemoteSessionHandle::Lease lease{*handle};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    RemoteSession* session  = &lease.session();
    const uint8_t stream_id = session->attachStream(attach_src, attach_dst);
    if (stream_id == 0xFF) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};

    const data::ConstDataSpan pending_cfg =
        config_cache == nullptr ? cfg_bytes : config_cache->pending(session, cfg_bytes);
    result_t<void> r;
    if (pending_cfg.size != 0) {
        r = enc.configure(kind, bus_id, pending_cfg);
    }
    if (r.has_value()) {
        r = enc.streamTransfer(kind, bus_id, stream_id, static_cast<uint32_t>(tx_len), static_cast<uint32_t>(rx_len),
                               meta);
    }
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        session->detachStream(stream_id);
        return m5::stl::make_unexpected(r.error());
    }

    const RemoteSession::Config saved_cfg = session->getConfig();
    RemoteSession::Config transfer_cfg    = saved_cfg;
    transfer_cfg.response_timeout_ms      = timeout_ms;
    session->setConfig(transfer_cfg);

    uint8_t seq = 0;
    auto req    = session->request({script_buf, script.written()}, &seq);
    // Any failure after the Request frame left the local encoder queue
    // (session->lastRequestEnqueued()) is ambiguous, not just a response
    // timeout: a post-enqueue pump error (e.g. a wire read failure while
    // awaiting the Response) looks identical to the device from a timeout —
    // the script may already be executing or the Data stream still open —
    // so it gets the same treatment. Only a failure *before* enqueue (the
    // script never left this host) is provably safe to free immediately.
    // Success also means a terminal frame was observed, so detach is safe
    // there too. See RemoteSession::quarantineStream() /
    // spec/design/remote.md §timeout / resync.
    if (!req.has_value() && session->lastRequestEnqueued()) {
        session->quarantineStream(stream_id, seq);
    } else {
        session->detachStream(stream_id);
    }
    bytecode::BytecodeRunner response_runner{memory::defaultAllocator()};
    auto chk = detail::decodeResponseStatus(session->lastResponse(), &response_runner);
    session->setConfig(saved_cfg);

    if (!req.has_value()) {
        if (config_cache != nullptr) {
            config_cache->invalidate();
        }
        return m5::stl::make_unexpected(req.error());
    }
    if (!chk.has_value()) {
        if (config_cache != nullptr) {
            config_cache->invalidate();
        }
        return m5::stl::make_unexpected(chk.error());
    }
    if (pending_cfg.size != 0 && config_cache != nullptr) {
        config_cache->rememberSent(session, pending_cfg);
    }
    bus::TransferTotals totals{tx_len, rx_len};
    const auto stored = response_runner.storedData(kDefaultStoreId);
    // Protocol v1 servers predating transfer totals return no StoreData.
    // Falling back to the requested lengths keeps a new host compatible
    // with those exact-transfer peers. New servers return two u32-LE values.
    if (stored.size == 8) {
        totals.tx = static_cast<size_t>(stored.data[0]) | (static_cast<size_t>(stored.data[1]) << 8) |
                    (static_cast<size_t>(stored.data[2]) << 16) | (static_cast<size_t>(stored.data[3]) << 24);
        totals.rx = static_cast<size_t>(stored.data[4]) | (static_cast<size_t>(stored.data[5]) << 8) |
                    (static_cast<size_t>(stored.data[6]) << 16) | (static_cast<size_t>(stored.data[7]) << 24);
        if (totals.tx > tx_len || totals.rx > rx_len) {
            if (config_cache != nullptr) {
                config_cache->invalidate();
            }
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
    }
    return totals;
}

result_t<bus::TransferTotals> remoteTransferWire(RemoteSession* session, types::bus_kind_t kind, uint8_t bus_id,
                                                 data::ConstDataSpan cfg_bytes, data::ConstDataSpan meta,
                                                 data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len,
                                                 uint32_t timeout_ms, RemoteConfigCache* config_cache)
{
    if (session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto handle = makeBorrowedSessionHandle(*session);
    if (!handle) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    return remoteTransferWire(handle, kind, bus_id, cfg_bytes, meta, src, tx_len, dst, rx_len, timeout_ms,
                              config_cache);
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_
