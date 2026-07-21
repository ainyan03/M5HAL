// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_INL_

#include "session.hpp"
#include "detail_helpers.hpp"

#include "../../../hal/v2/remote/wire_drain.hpp"

#include <M5Utility.hpp>

#include <cstring>

namespace m5::hal::v2::remote {

RemoteSession::RemoteSession(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec, data::Source& wire_rx,
                             data::Sink& wire_tx)
    : _enc{&enc},
      _dec{&dec},
      _wire_rx{&wire_rx},
      _wire_tx{&wire_tx},
      _session_handle{std::make_shared<RemoteSessionHandle>()}
{
    if (_session_handle && _session_handle->valid()) {
        _session_handle->bind(*this);
    } else {
        _session_handle.reset();
    }
    _dec->setFrameHandler(frameHandlerThunk, this);
    _dec->setStaleDataObserver(staleDataThunk, this);
}

RemoteSession::~RemoteSession()
{
    if (_session_handle) {
        (void)_session_handle->close();
    }
}

result_t<void> RemoteSession::request(data::ConstDataSpan script)
{
    return request(script, nullptr);
}

result_t<void> RemoteSession::request(data::ConstDataSpan script, uint8_t* out_seq)
{
    if (script.size > frame::kMaxPayload) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    uint8_t seq = nextSeq();
    if (out_seq != nullptr) {
        *out_seq = seq;
    }
    _last_request_enqueued = false;
    if (auto queued = _enc->writeFrame(frame::Kind::Request, seq, script); !queued.has_value()) {
        return m5::stl::make_unexpected(queued.error());
    }
    _last_request_enqueued = true;
    auto flushed           = flushTx();
    if (!flushed.has_value()) {
        return m5::stl::make_unexpected(flushed.error());
    }
    return awaitResponse(seq, frame::Kind::Response);
}

result_t<void> RemoteSession::requestNoResponse(data::ConstDataSpan script)
{
    if (script.size > frame::kMaxPayload) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    uint8_t seq = nextSeq();
    if (auto queued = _enc->writeFrame(frame::Kind::Request, seq | 0x80, script); !queued.has_value()) {
        return m5::stl::make_unexpected(queued.error());
    }
    // No response is expected for this frame, so nothing else in this
    // session would naturally read the wire afterward. A transport that
    // coalesces small writes only flushes on its own read path, so a
    // bare flushTx() can leave these bytes sitting in that buffer
    // indefinitely. Pump once to force the same flush-before-read the
    // read path already performs.
    return pumpWire();
}

result_t<void> RemoteSession::hello()
{
    uint8_t seq = nextSeq();
    if (auto queued = _enc->writeFrame(frame::Kind::HelloReq, seq, {}); !queued.has_value()) {
        return m5::stl::make_unexpected(queued.error());
    }
    ++_config_generation;
    auto flushed = flushTx();
    if (!flushed.has_value()) {
        return m5::stl::make_unexpected(flushed.error());
    }
    auto resp = awaitResponse(seq, frame::Kind::HelloResp);
    if (resp.has_value()) {
        // A fresh session handshake supersedes any in-flight timeout
        // resync state from the previous connection.
        clearAllQuarantine();
    }
    return resp;
}

result_t<void> RemoteSession::ping()
{
    uint8_t seq = nextSeq();
    if (auto queued = _enc->writeFrame(frame::Kind::Ping, seq, {}); !queued.has_value()) {
        return m5::stl::make_unexpected(queued.error());
    }
    auto flushed = flushTx();
    if (!flushed.has_value()) {
        return m5::stl::make_unexpected(flushed.error());
    }
    return awaitResponse(seq, frame::Kind::Pong);
}

result_t<void> RemoteSession::reset()
{
    uint8_t seq = nextSeq();
    if (auto queued = _enc->writeFrame(frame::Kind::Control, seq, {}); !queued.has_value()) {
        return m5::stl::make_unexpected(queued.error());
    }
    // Reset is fire-and-forget: the host API does not wait for the
    // device's reset Response (spec/design/remote.md §Control), so a
    // successful pumpWire() here only proves the local reset frame left
    // the encoder queue — it says nothing about whether the device has
    // received it, let alone finished any transfer that was still active.
    // Quarantine entries are therefore left untouched; they can only be
    // released by their own terminal-frame match, insurance timer, or a
    // subsequent successful hello() (see quarantineStream()).
    return pumpWire();
}

result_t<size_t> RemoteSession::poll(size_t max_msgs)
{
    (void)max_msgs;
    auto pumped = pumpWire();
    if (!pumped.has_value()) {
        return m5::stl::make_unexpected(pumped.error());
    }
    if (_peer_poll != nullptr) {
        _peer_poll(_peer_ctx);
    }
    pumped = pumpWire();
    if (!pumped.has_value()) {
        return m5::stl::make_unexpected(pumped.error());
    }
    return _rx_count;
}

uint8_t RemoteSession::attachStream(data::Source* tx, data::Sink* rx)
{
    uint8_t id = allocateStreamId();
    if (id == 0xFF) {
        return 0xFF;
    }
    if (tx != nullptr) {
        _enc->attach(id, *tx);
    }
    if (rx != nullptr) {
        _dec->setSink(id, *rx);
    }
    return id;
}

void RemoteSession::detachStream(uint8_t stream_id)
{
    if (stream_id >= kMaxStreams) {
        return;
    }
    _enc->detach(stream_id);
    _dec->clearSink(stream_id);
    _stream_used &= ~(1u << stream_id);
    _stream_quarantined &= ~(1u << stream_id);
}

void RemoteSession::quarantineStream(uint8_t stream_id, uint8_t seq)
{
    if (stream_id >= kMaxStreams) {
        return;
    }
    _enc->detach(stream_id);
    _dec->clearSink(stream_id);
    // _stream_used stays set — the id remains unavailable to
    // allocateStreamId() until this entry is released.
    _stream_quarantined |= (1u << stream_id);
    _quarantine[stream_id].seq      = seq;
    _quarantine[stream_id].start_ms = static_cast<uint32_t>(m5::utility::millis());
}

result_t<void> RemoteSession::checkResponse()
{
    auto resp = lastResponse();
    // A response must carry a terminal Report (ReportComplete/ReportError). An
    // empty payload or a script that never reports is a protocol violation, not
    // success — matches detail::decodeResponseStatus's contract.
    return detail::decodeResponseStatus(resp);
}

uint8_t RemoteSession::allocateStreamId()
{
    sweepExpiredQuarantine();
    for (uint8_t i = 0; i < kMaxStreams; ++i) {
        if ((_stream_used & (1u << i)) == 0) {
            _stream_used |= (1u << i);
            return i;
        }
    }
    return 0xFF;
}

void RemoteSession::releaseQuarantineForSeq(uint8_t seq)
{
    for (uint8_t i = 0; i < kMaxStreams; ++i) {
        if ((_stream_quarantined & (1u << i)) != 0 && _quarantine[i].seq == seq) {
            _stream_quarantined &= ~(1u << i);
            _stream_used &= ~(1u << i);
        }
    }
}

void RemoteSession::sweepExpiredQuarantine()
{
    if (_stream_quarantined == 0) {
        return;
    }
    const uint32_t now = static_cast<uint32_t>(m5::utility::millis());
    for (uint8_t i = 0; i < kMaxStreams; ++i) {
        if ((_stream_quarantined & (1u << i)) == 0) {
            continue;
        }
        // Same wraparound-safe unsigned-subtraction idiom as
        // awaitResponse()'s timeout check.
        if (static_cast<uint32_t>(now - _quarantine[i].start_ms) >= _config.stream_quarantine_ms) {
            _stream_quarantined &= ~(1u << i);
            _stream_used &= ~(1u << i);
        }
    }
}

void RemoteSession::clearAllQuarantine()
{
    _stream_used &= ~_stream_quarantined;
    _stream_quarantined = 0;
}

result_t<void> RemoteSession::pumpWire()
{
    auto drained = drainTx();
    if (!drained.has_value()) {
        return m5::stl::make_unexpected(drained.error());
    }
    auto decoded = _dec->pump(*_wire_rx);
    if (!decoded.has_value()) {
        return m5::stl::make_unexpected(decoded.error());
    }
    _rx_count = decoded.value();
    sendCreditIfChanged();
    return flushTx();
}

result_t<void> RemoteSession::flushTx()
{
    auto encoded = _enc->pump();
    if (!encoded.has_value()) {
        return m5::stl::make_unexpected(encoded.error());
    }
    auto drained = drainTx();
    if (!drained.has_value()) {
        return m5::stl::make_unexpected(drained.error());
    }
    return {};
}

result_t<remote::detail::DrainProgress> RemoteSession::drainTx()
{
    return remote::detail::drainToSink(_enc->output(), *_wire_tx);
}

void RemoteSession::sendCreditIfChanged()
{
    _credit.pump(*_enc, *_dec);
}

result_t<void> RemoteSession::awaitResponse(uint8_t seq, frame::Kind want_kind)
{
    _got_response        = false;
    _got_error           = false;
    _awaiting_seq        = seq;
    _awaiting_kind       = want_kind;
    const uint32_t start = static_cast<uint32_t>(m5::utility::millis());
    for (;;) {
        if (static_cast<uint32_t>(m5::utility::millis()) - start > _config.response_timeout_ms) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        auto pumped = pumpWire();
        if (!pumped.has_value()) {
            return m5::stl::make_unexpected(pumped.error());
        }
        if (_peer_poll != nullptr) {
            _peer_poll(_peer_ctx);
        }
        pumped = pumpWire();
        if (!pumped.has_value()) {
            return m5::stl::make_unexpected(pumped.error());
        }
        if (_got_response) {
            return {};
        }
        if (_got_error) {
            return m5::stl::make_unexpected(_last_error);
        }
    }
}

void RemoteSession::frameHandlerThunk(void* ctx, const frame::View& view)
{
    static_cast<RemoteSession*>(ctx)->onFrame(view);
}

void RemoteSession::staleDataThunk(void* ctx, uint8_t stream_id)
{
    static_cast<RemoteSession*>(ctx)->onStaleData(stream_id);
}

void RemoteSession::onStaleData(uint8_t stream_id)
{
    // Only meaningful for a stream_id currently in quarantine — a Data
    // frame the decoder had nowhere to route for any other id (e.g. a
    // tx-only transfer's rx side, which legitimately has no Sink) is not
    // evidence of anything and must not perturb the quarantine table.
    if (stream_id >= kMaxStreams || (_stream_quarantined & (1u << stream_id)) == 0) {
        return;
    }
    // Mirrors the server's own inactivity semantics (see
    // Config::stream_quarantine_ms): the peer is still producing for this
    // id, so the insurance timer must not expire out from under it.
    _quarantine[stream_id].start_ms = static_cast<uint32_t>(m5::utility::millis());
}

void RemoteSession::onFrame(const frame::View& view)
{
    // A quarantined stream_id is released by the first Response/Control
    // this session observes that carries the same seq as the request that
    // triggered the quarantine (the wire is FIFO, so no more Data for that
    // id can still be in flight once its terminal frame has arrived). This
    // must fire regardless of whether `view` matches the *current*
    // awaitResponse() target — both a mismatched frame seen while awaiting
    // a later request, and a frame seen only via poll() with no pending
    // await, are valid release points.
    if (_stream_quarantined != 0 && (view.kind == frame::Kind::Response || view.kind == frame::Kind::Control)) {
        releaseQuarantineForSeq(view.b3);
    }
    switch (view.kind) {
        case frame::Kind::Response:
        case frame::Kind::HelloResp:
        case frame::Kind::Pong:
            if (view.b3 == _awaiting_seq && view.kind == _awaiting_kind) {
                // A payload larger than _resp_buf violates the decoder's
                // kMaxPayload contract. Recording its length while copying
                // nothing would hand callers uninitialized bytes; drop the
                // frame instead and let the caller time out.
                if (view.payload.size <= sizeof(_resp_buf)) {
                    _resp_len = view.payload.size;
                    if (view.payload.size > 0) {
                        ::memcpy(_resp_buf, view.payload.data, view.payload.size);
                    }
                    _got_response = true;
                }
            }
            break;
        case frame::Kind::Control:
            if (view.payload.size >= 1) {
                _last_error = mapRemoteError(static_cast<int8_t>(view.payload.data[0]));
                if (view.b3 == _awaiting_seq) {
                    _got_error = true;
                }
            }
            break;
        case frame::Kind::Event:
            if (_event_fn != nullptr) {
                _event_fn(_event_ctx, view.b3, view.payload);
            }
            break;
        case frame::Kind::Credit:
            _enc->updateRemoteCredit(view.b3);
            break;
        default:
            break;
    }
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_INL_
