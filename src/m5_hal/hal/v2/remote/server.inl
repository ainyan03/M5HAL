// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_INL_
#define M5_HAL_REMOTE_SERVER_INL_

#include "./server.hpp"

#include "../diag.hpp"

#include <string.h>

namespace m5::hal::v2::remote {

using remote_error_t = m5::hal::v2::error::error_t;

namespace detail {

inline uint32_t getU32LE(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// Read a u32 config field at `off` inside `cfg` when present; tolerant
// configs may legitimately be shorter (bytecode.md, tolerant decode).
inline bool u32FieldExceeds(data::ConstDataSpan cfg, size_t off, uint32_t limit)
{
    return (cfg.size >= off + 4) && (getU32LE(cfg.data + off) > limit);
}

}  // namespace detail

// ---- Server -----------------------------------------------------------------

void Server::checkScratch()
{
    M5HAL_ASSERT(_scratch.data != nullptr && _scratch.size >= kMaxScriptSize,
                 "Server: response_scratch must hold at least kMaxScriptSize (%u) bytes",
                 static_cast<unsigned>(kMaxScriptSize));
    if (_scratch.data == nullptr || _scratch.size < kMaxScriptSize) {
        _scratch = data::DataSpan{};
    }
}

void Server::initRunnerHooks()
{
    _runner.setStreamTransferHandler(&streamTransferThunk, this);
}

void Server::beginFrameRequest(uint8_t seq, data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec)
{
    _current_seq            = seq;
    _current_enc            = &enc;
    _current_dec            = &dec;
    _in_frame_request       = true;
    _defer_current_response = false;
}

void Server::endFrameRequest()
{
    _current_enc      = nullptr;
    _current_dec      = nullptr;
    _in_frame_request = false;
}

result_t<void> Server::recordCapability(types::bus_kind_t kind, uint8_t bus_id)
{
    // Idempotent: re-registering the same (kind, bus_id) pair is a no-op success
    // so that Server::register* can be called multiple times without exhausting
    // kMaxEntries (e.g. on reconnect).
    for (size_t i = 0; i < _cap_count; ++i) {
        if (_caps[i].kind == kind && _caps[i].bus_id == bus_id) {
            return {};
        }
    }
    if (_cap_count >= Capabilities::kMaxEntries) {
        return m5::stl::make_unexpected(remote_error_t::OUT_OF_RESOURCE);
    }
    _caps[_cap_count++] = Capabilities::BusEntry{kind, bus_id};
    return {};
}

void Server::removeCapability(types::bus_kind_t kind, uint8_t bus_id)
{
    for (size_t i = 0; i < _cap_count; ++i) {
        if (_caps[i].kind == kind && _caps[i].bus_id == bus_id) {
            _caps[i] = _caps[--_cap_count];
            break;
        }
    }
}

void Server::setBusCreateHandler(bus_create_app_fn_t fn, void* ctx)
{
    _bus_create_app_fn  = fn;
    _bus_create_app_ctx = ctx;
    _runner.setBusCreateHandler(&busCreateThunk, this);
    _runner.setGpioAllowlistHandler(&gpioAllowlistThunk, this);
}

result_t<void> Server::busCreateThunk(void* ctx, bool create, types::bus_kind_t kind, uint8_t bus_id,
                                      data::ConstDataSpan pin_config)
{
    return static_cast<Server*>(ctx)->handleBusCreate(create, kind, bus_id, pin_config);
}

result_t<void> Server::handleBusCreate(bool create, types::bus_kind_t kind, uint8_t bus_id,
                                       data::ConstDataSpan pin_config)
{
    if (_bus_create_app_fn == nullptr) {
        return m5::stl::make_unexpected(remote_error_t::UNSUPPORTED);
    }
    auto r = _bus_create_app_fn(_bus_create_app_ctx, create, kind, bus_id, pin_config);
    if (!r.has_value()) {
        return r;
    }
    if (create) {
        return recordCapability(kind, bus_id);
    }
    removeCapability(kind, bus_id);
    return {};
}

result_t<void> Server::gpioAllowlistThunk(void* ctx, const uint8_t* pins, size_t count)
{
    return static_cast<Server*>(ctx)->handleGpioAllowlist(pins, count);
}

// TODO: build AllowlistGPIO + GPIOGroup from pin list and call
// setGPIOGroup(). Currently a stub.
result_t<void> Server::handleGpioAllowlist(const uint8_t* pins, size_t count)
{
    (void)pins;
    (void)count;
    return m5::stl::make_unexpected(remote_error_t::UNSUPPORTED);
}

result_t<void> Server::streamTransferThunk(void* ctx, const bytecode::BytecodeRunner::StreamTransferDesc& desc)
{
    return static_cast<Server*>(ctx)->handleStreamTransfer(desc);
}

result_t<void> Server::handleStreamTransfer(const bytecode::BytecodeRunner::StreamTransferDesc& desc)
{
    if (!_runner.hasBinding(desc.kind, desc.bus_id)) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_STATE);
    }
    if (!_in_frame_request || _current_enc == nullptr || _current_dec == nullptr) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_STATE);
    }
    if (desc.meta.size > sizeof(_pending_stream.meta)) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }

    if (desc.tx_len == 0 && desc.rx_len == 0) {
        size_t actual_tx = 0;
        size_t actual_rx = 0;
        return _runner.streamTransferChunk(desc.kind, desc.bus_id, desc.meta, data::ConstDataSpan{}, data::DataSpan{},
                                           actual_tx, actual_rx);
    }

    if (desc.tx_len > 0 || desc.rx_len > 0) {
        if (_pending_stream.active) {
            M5HAL_DIAG("stream busy bus_id=%u stream_id=%u (already active)", static_cast<unsigned>(desc.bus_id),
                       static_cast<unsigned>(desc.stream_id));
            return m5::stl::make_unexpected(remote_error_t::BUSY);
        }
        data::Source* rx_src = nullptr;
        if (desc.tx_len > 0) {
            rx_src = _current_dec->createBlockStream(desc.stream_id);
            if (rx_src == nullptr) {
                return m5::stl::make_unexpected(remote_error_t::OUT_OF_RESOURCE);
            }
        }

        _pending_stream           = PendingStreamTransfer{};
        _pending_stream.active    = true;
        _pending_stream.seq       = _current_seq;
        _pending_stream.kind      = desc.kind;
        _pending_stream.bus_id    = desc.bus_id;
        _pending_stream.stream_id = desc.stream_id;
        _pending_stream.tx_len    = desc.tx_len;
        _pending_stream.rx_len    = desc.rx_len;
        _pending_stream.rx_src    = rx_src;
        _pending_stream.dec       = desc.tx_len > 0 ? _current_dec : nullptr;
        _pending_stream.meta_size = desc.meta.size;
        _pending_stream.status    = remote_error_t::OK;
        if (desc.meta.size != 0) {
            ::memcpy(_pending_stream.meta, desc.meta.data, desc.meta.size);
        }
        M5HAL_DIAG("stream begin bus_id=%u kind=%u stream_id=%u tx=%u rx=%u", static_cast<unsigned>(desc.bus_id),
                   static_cast<unsigned>(desc.kind), static_cast<unsigned>(desc.stream_id),
                   static_cast<unsigned>(desc.tx_len), static_cast<unsigned>(desc.rx_len));
        _defer_current_response = true;
        return {};
    }
    return {};
}

result_t<void> Server::writeDeferredResponse(data::MuxFrameEncoder& enc, uint8_t seq, remote_error_t status)
{
    if ((seq & 0x80) != 0) {
        return {};
    }
    uint8_t resp_buf[frame::kMaxPayload];
    data::MemorySink resp_sink{resp_buf, sizeof(resp_buf)};
    bytecode::BytecodeEncoder resp{resp_sink};
    auto r = error::isError(status) ? resp.reportError(status, 0) : resp.reportComplete(status);
    if (r.has_value()) {
        r = resp.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!enc.writeFrame(frame::Kind::Response, seq, {resp_buf, resp_sink.written()})) {
        return m5::stl::make_unexpected(remote_error_t::BUFFER_OVERFLOW);
    }
    return {};
}

result_t<void> Server::completePendingStream(data::MuxFrameEncoder& enc, remote_error_t status)
{
    if (!_pending_stream.active) {
        return {};
    }
    const uint8_t seq       = _pending_stream.seq;
    const uint8_t stream_id = _pending_stream.stream_id;
    auto* dec               = _pending_stream.dec;
    if (dec != nullptr) {
        dec->destroyStream(stream_id);
    }
    _pending_stream = PendingStreamTransfer{};
    return writeDeferredResponse(enc, seq, status);
}

void Server::abortPendingStream()
{
    if (!_pending_stream.active) {
        return;
    }
    if (_pending_stream.dec != nullptr) {
        _pending_stream.dec->destroyStream(_pending_stream.stream_id);
    }
    _pending_stream = PendingStreamTransfer{};
}

result_t<void> Server::poll(data::MuxFrameEncoder& enc, uint32_t now_ms)
{
    if (!_pending_stream.active) {
        return {};
    }
    if (!_pending_stream.has_progress_time) {
        _pending_stream.last_progress_ms  = now_ms;
        _pending_stream.has_progress_time = true;
    }
    while (_pending_stream.tx_consumed < _pending_stream.tx_len ||
           _pending_stream.rx_produced < _pending_stream.rx_len) {
        data::ConstDataSpan tx_chunk{};
        const bool need_tx = _pending_stream.tx_consumed < _pending_stream.tx_len;
        if (need_tx) {
            if (_pending_stream.rx_src == nullptr) {
                return completePendingStream(enc, remote_error_t::INVALID_STATE);
            }
            const size_t remaining_tx = static_cast<size_t>(_pending_stream.tx_len) - _pending_stream.tx_consumed;
            const size_t max_tx       = remaining_tx < frame::kMaxDataPayload ? remaining_tx : frame::kMaxDataPayload;
            auto p                    = _pending_stream.rx_src->peek(max_tx);
            if (!p.has_value()) {
                return completePendingStream(enc, p.error());
            }
            if (p.value().size == 0) {
                if (static_cast<uint32_t>(now_ms - _pending_stream.last_progress_ms) > _pending_stream_timeout_ms) {
                    M5HAL_DIAG("stream timeout bus_id=%u stream_id=%u elapsed_ms=%u limit_ms=%u",
                               static_cast<unsigned>(_pending_stream.bus_id),
                               static_cast<unsigned>(_pending_stream.stream_id),
                               static_cast<unsigned>(now_ms - _pending_stream.last_progress_ms),
                               static_cast<unsigned>(_pending_stream_timeout_ms));
                    return completePendingStream(enc, remote_error_t::TIMEOUT_ERROR);
                }
                return {};
            }
            tx_chunk = p.value();
        }

        uint8_t rx_buf[frame::kMaxDataPayload];
        const size_t remaining_rx = _pending_stream.rx_len > _pending_stream.rx_produced
                                        ? static_cast<size_t>(_pending_stream.rx_len) - _pending_stream.rx_produced
                                        : 0;
        const size_t want_rx      = remaining_rx < sizeof(rx_buf) ? remaining_rx : sizeof(rx_buf);
        if (want_rx != 0 && enc.output().blockCount() >= data::BlockSource::kMaxBlocks) {
            return {};
        }
        size_t actual_tx = 0;
        size_t actual_rx = 0;
        auto r = _runner.streamTransferChunk(_pending_stream.kind, _pending_stream.bus_id, pendingMeta(), tx_chunk,
                                             data::DataSpan{rx_buf, want_rx}, actual_tx, actual_rx);
        if (!r.has_value()) {
            return completePendingStream(enc, r.error());
        }
        if (actual_tx > tx_chunk.size || actual_rx > want_rx) {
            return completePendingStream(enc, remote_error_t::IO_ERROR);
        }
        if (actual_rx != 0) {
            if (!enc.writeFrame(frame::Kind::Data, _pending_stream.stream_id, {rx_buf, actual_rx})) {
                return completePendingStream(enc, remote_error_t::BUFFER_OVERFLOW);
            }
            _pending_stream.rx_produced += actual_rx;
            _pending_stream.last_progress_ms = now_ms;
        }
        if (actual_tx == 0) {
            if (actual_rx == 0) {
                if (static_cast<uint32_t>(now_ms - _pending_stream.last_progress_ms) > _pending_stream_timeout_ms) {
                    M5HAL_DIAG("stream timeout bus_id=%u stream_id=%u elapsed_ms=%u limit_ms=%u",
                               static_cast<unsigned>(_pending_stream.bus_id),
                               static_cast<unsigned>(_pending_stream.stream_id),
                               static_cast<unsigned>(now_ms - _pending_stream.last_progress_ms),
                               static_cast<unsigned>(_pending_stream_timeout_ms));
                    return completePendingStream(enc, remote_error_t::TIMEOUT_ERROR);
                }
                return {};
            }
            if (need_tx) {
                return {};
            }
            continue;
        }
        if (need_tx) {
            auto adv = _pending_stream.rx_src->advance(actual_tx);
            if (!adv.has_value()) {
                return completePendingStream(enc, adv.error());
            }
            _pending_stream.tx_consumed += actual_tx;
            _pending_stream.last_progress_ms = now_ms;
            if (_pending_stream.tx_consumed >= _pending_stream.tx_len &&
                _pending_stream.rx_produced < _pending_stream.rx_len) {
                return {};
            }
        }
    }

    if (_pending_stream.tx_consumed >= _pending_stream.tx_len &&
        _pending_stream.rx_produced >= _pending_stream.rx_len) {
        if (enc.output().blockCount() >= data::BlockSource::kMaxBlocks) {
            return {};
        }
        return completePendingStream(enc, _pending_stream.status);
    }
    return {};
}

result_t<void> Server::registerI2C(uint8_t bus_id, i2c::MasterAccessor& acc)
{
    if (acc.getConfig().wire_timeout_ms > _config.max_bus_timeout_ms) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }
    auto r = _runner.registerI2C(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::I2C, bus_id);
}

result_t<void> Server::registerSPI(uint8_t bus_id, spi::MasterAccessor& acc)
{
    auto r = _runner.registerSPI(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::SPI, bus_id);
}

result_t<void> Server::registerUART(uint8_t bus_id, uart::Accessor& acc)
{
    const auto& cfg = acc.getConfig();
    if (cfg.first_byte_timeout_ms > _config.max_bus_timeout_ms ||
        cfg.inter_byte_timeout_ms > _config.max_bus_timeout_ms || cfg.write_timeout_ms > _config.max_bus_timeout_ms) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }
    auto r = _runner.registerUART(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::UART, bus_id);
}

result_t<void> Server::registerI2S(uint8_t bus_id, i2s::Accessor& acc)
{
    const auto& cfg = acc.getConfig();
    if (cfg.write_timeout_ms > _config.max_bus_timeout_ms || cfg.read_timeout_ms > _config.max_bus_timeout_ms) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }
    auto r = _runner.registerI2S(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::I2S, bus_id);
}

result_t<void> Server::registerI2S(uint8_t bus_id, i2s::TxAccessor& acc)
{
    const auto& cfg = acc.getConfig();
    if (cfg.write_timeout_ms > _config.max_bus_timeout_ms) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }
    auto r = _runner.registerI2S(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::I2S, bus_id);
}

result_t<void> Server::registerI2S(uint8_t bus_id, i2s::RxAccessor& acc)
{
    const auto& cfg = acc.getConfig();
    if (cfg.read_timeout_ms > _config.max_bus_timeout_ms) {
        return m5::stl::make_unexpected(remote_error_t::INVALID_ARGUMENT);
    }
    auto r = _runner.registerI2S(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::I2S, bus_id);
}

remote_error_t Server::prescan(data::ConstDataSpan script, size_t& offset) const
{
    // One pass over the length-prefixed instructions. Only the execution-
    // policy checks live here (delay budget, bus timeouts); malformed
    // scripts fall through so the runner reports them with its own,
    // more precise error and offset.
    size_t at            = 0;
    uint64_t delay_total = 0;
    bool stream_must_end = false;
    while (at < script.size) {
        const auto lv = bytecode::decodeLenVar(data::ConstDataSpan{script.data + at, script.size - at});
        if (lv.consumed == 0 || !lv.valid || lv.value == 0) {
            break;  // terminator or malformed -> the runner decides
        }
        const size_t instr_at = at;
        at += lv.consumed;
        if (lv.value > script.size - at) {
            break;  // truncated -> the runner reports it
        }
        const uint8_t opcode = script.data[at];
        const data::ConstDataSpan payload{script.data + at + 1, lv.value - 1};
        at += lv.value;

        if (stream_must_end) {
            M5HAL_DIAG("script rejected offset=%zu reason=stream-not-terminal", instr_at);
            offset = instr_at;
            return remote_error_t::INVALID_ARGUMENT;
        }

        if (opcode == static_cast<uint8_t>(bytecode::OpCode::DelayMs)) {
            if (payload.size >= 4) {
                delay_total += detail::getU32LE(payload.data);
                if (delay_total > _config.max_delay_ms) {
                    M5HAL_DIAG("script rejected offset=%zu reason=delay-budget delay_total=%llu limit_ms=%u", instr_at,
                               static_cast<unsigned long long>(delay_total),
                               static_cast<unsigned>(_config.max_delay_ms));
                    offset = instr_at;
                    return remote_error_t::INVALID_ARGUMENT;
                }
            }
        } else if (opcode == static_cast<uint8_t>(bytecode::OpCode::BusConfigure)) {
            if (payload.size >= 2) {
                const data::ConstDataSpan cfg{payload.data + 2, payload.size - 2};
                bool exceeded = false;
                // Field offsets: bytecode.hpp BusConfigure config payload
                // layout (shared with encodeConfig).
                switch (static_cast<types::bus_kind_t>(payload.data[0])) {
                    case types::bus_kind_t::I2C:
                        exceeded = detail::u32FieldExceeds(cfg, bytecode::kI2CConfigWireTimeoutOffset,
                                                           _config.max_bus_timeout_ms);
                        break;
                    case types::bus_kind_t::UART:
                        exceeded = detail::u32FieldExceeds(cfg, bytecode::kUARTConfigFirstByteTimeoutOffset,
                                                           _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, bytecode::kUARTConfigInterByteTimeoutOffset,
                                                           _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, bytecode::kUARTConfigWriteTimeoutOffset,
                                                           _config.max_bus_timeout_ms);
                        break;
                    case types::bus_kind_t::I2S:
                        exceeded = detail::u32FieldExceeds(cfg, bytecode::kI2SConfigWriteTimeoutOffset,
                                                           _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, bytecode::kI2SConfigReadTimeoutOffset,
                                                           _config.max_bus_timeout_ms);
                        break;
                    // SPI carries no wire-time field anymore (its former
                    // timeout_ms was lock-only); nothing to bound.
                    default:
                        break;
                }
                if (exceeded) {
                    M5HAL_DIAG("script rejected offset=%zu reason=bus-timeout kind=%u", instr_at,
                               static_cast<unsigned>(payload.data[0]));
                    offset = instr_at;
                    return remote_error_t::INVALID_ARGUMENT;
                }
            }
        } else if (opcode == static_cast<uint8_t>(bytecode::OpCode::BusTransfer)) {
            // payload: [kind:1][bus_id:1][store_id:1][rx_len:LenVar]...
            // (mirrors BytecodeRunner::opBusTransfer). Cap the up-front rx
            // allocation a wire message can request. When the data is stored
            // for the response, also cap it to the response frame capacity
            // before any non-idempotent bus read can run.
            if (payload.size > 3) {
                const bool stores_rx = payload.data[2] != bytecode::kDiscardStoreId;
                const auto rx_len    = bytecode::decodeLenVar(data::ConstDataSpan{payload.data + 3, payload.size - 3});
                const auto limit =
                    (stores_rx && _config.max_transfer_rx > kMaxTransferRx) ? kMaxTransferRx : _config.max_transfer_rx;
                if (rx_len.valid && rx_len.consumed != 0 && rx_len.value > limit) {
                    M5HAL_DIAG("script rejected offset=%zu reason=transfer-rx-limit rx_len=%u limit=%u", instr_at,
                               static_cast<unsigned>(rx_len.value), static_cast<unsigned>(limit));
                    offset = instr_at;
                    return remote_error_t::INVALID_ARGUMENT;
                }
            }
        } else if (opcode == static_cast<uint8_t>(bytecode::OpCode::BusStreamTransfer)) {
            // payload: [kind:1][bus_id:1][stream_id:1][meta_size:1][tx_len:4LE][rx_len:4LE][meta]
            // Any stream transfer that defers the response (tx or rx
            // pending) must be the script's last instruction: the deferred
            // Response carries only Report*, so response slots stored by a
            // later instruction would be silently dropped.
            if ((payload.size >= 8 && detail::getU32LE(payload.data + 4) > 0) ||
                (payload.size >= 12 && detail::getU32LE(payload.data + 8) > 0)) {
                stream_must_end = true;
            }
        }
    }
    return remote_error_t::OK;
}

Server::ExecOutcome Server::execute(data::ConstDataSpan script)
{
    ExecOutcome out;
    out.status = prescan(script, out.offset);
    if (error::isError(out.status)) {
        return out;
    }
    out.ran    = true;
    auto r     = _runner.run(script);
    out.status = r.has_value() ? remote_error_t::OK : r.error();
    return out;
}

result_t<remote_error_t> Server::processScript(data::ConstDataSpan script, data::Sink& dst)
{
    const ExecOutcome outcome = execute(script);
    if (_defer_current_response && error::isError(outcome.status)) {
        if (_pending_stream.active && _current_enc != nullptr) {
            auto c = completePendingStream(*_current_enc, outcome.status);
            if (!c.has_value()) {
                return m5::stl::make_unexpected(c.error());
            }
        }
        _defer_current_response = false;
    }
    if (_defer_current_response) {
        return outcome.status;
    }
    if (outcome.ran) {
        auto w = _runner.writeResponse(dst, outcome.status);
        if (!w.has_value()) {
            return m5::stl::make_unexpected(w.error());
        }
    } else {
        // Rejected before execution: a minimal response script carries
        // the policy error and the offending instruction's offset.
        bytecode::BytecodeEncoder enc{dst};
        auto e = enc.reportError(outcome.status, outcome.offset);
        if (e.has_value()) {
            e = enc.end();
        }
        if (!e.has_value()) {
            return m5::stl::make_unexpected(e.error());
        }
    }
    return outcome.status;
}

}  // namespace m5::hal::v2::remote

#endif
