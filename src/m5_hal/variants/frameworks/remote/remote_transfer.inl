// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_

#include "remote_transfer.hpp"

#include "../../../hal/v2/data/limited.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"

namespace m5::hal::v2::remote {

result_t<size_t> remoteTransferWire(RemoteSession* session, types::bus_kind_t kind, uint8_t bus_id,
                                    data::ConstDataSpan cfg_bytes, data::ConstDataSpan meta, data::Source* src,
                                    size_t tx_len, data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                    RemoteConfigCache* config_cache)
{
    if (session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
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

    auto req = session->request({script_buf, script.written()});
    session->detachStream(stream_id);
    auto chk = session->checkResponse();
    session->setConfig(saved_cfg);

    if (!req.has_value()) {
        return m5::stl::make_unexpected(req.error());
    }
    if (!chk.has_value()) {
        return m5::stl::make_unexpected(chk.error());
    }
    if (pending_cfg.size != 0 && config_cache != nullptr) {
        config_cache->rememberSent(session, pending_cfg);
    }
    return tx_len + rx_len;
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_INL_
