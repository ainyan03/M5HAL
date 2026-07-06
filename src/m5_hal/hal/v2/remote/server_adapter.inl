// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_ADAPTER_INL_
#define M5_HAL_REMOTE_SERVER_ADAPTER_INL_

#include "./server_adapter.hpp"

#include "../diag.hpp"

#include <cstring>

namespace m5::hal::v2::remote {

result_t<size_t> RemoteServerAdapter::service()
{
    if (_poll_handler != nullptr) {
        auto poll = _poll_handler(_poll_handler_ctx, *_enc);
        if (!poll.has_value()) {
            return m5::stl::make_unexpected(poll.error());
        }
    }
    if (!_external_poll) {
        drainTx();
    }
    if (!_external_poll) {
        pumpWire();
    }
    size_t count   = _pending_count;
    _pending_count = 0;
    if (_poll_handler != nullptr) {
        auto poll = _poll_handler(_poll_handler_ctx, *_enc);
        if (!poll.has_value()) {
            return m5::stl::make_unexpected(poll.error());
        }
    }
    if (!_external_poll) {
        flushTx();
    }
    return count;
}

void RemoteServerAdapter::pumpWire()
{
    drainTx();
    _dec->pump(*_wire_rx);
    sendCreditIfChanged();
    flushTx();
}

void RemoteServerAdapter::flushTx()
{
    _enc->pump();
    drainTx();
}

void RemoteServerAdapter::drainTx()
{
    auto& out = _enc->output();
    while (!out.eof()) {
        auto p = out.peek(4096);
        if (!p.has_value() || p.value().size == 0) {
            break;
        }
        auto rsv = _wire_tx->reserve(p.value().size);
        if (!rsv.has_value() || rsv.value().size == 0) {
            break;
        }
        size_t n = rsv.value().size < p.value().size ? rsv.value().size : p.value().size;
        ::memcpy(rsv.value().data, p.value().data, n);
        auto c = _wire_tx->commit(n);
        if (!c.has_value()) {
            break;
        }
        (void)out.advance(n);
    }
}

void RemoteServerAdapter::frameHandlerThunk(void* ctx, const frame::View& view)
{
    static_cast<RemoteServerAdapter*>(ctx)->onFrame(view);
}

void RemoteServerAdapter::onFrame(const frame::View& view)
{
    if (view.kind == frame::Kind::Credit) {
        _enc->updateRemoteCredit(view.b3);
        M5HAL_DIAG("credit applied remote=%u", static_cast<unsigned>(view.b3));
        return;
    }
    if (_handler != nullptr) {
        (void)_handler(_handler_ctx, view.kind, view.b3, view.payload, *_enc, *_dec);
    } else {
        M5HAL_DIAG("frame dropped kind=%d (no handler installed)", static_cast<int>(view.kind));
    }
    ++_pending_count;
}

void RemoteServerAdapter::sendCreditIfChanged()
{
    _credit.pump(*_enc, *_dec);
}

service::ServicePoll RemoteWireService::serviceImpl(const service::ServiceContext& ctx)
{
    (void)ctx;
    drainTx();
    _dec->pump(*_wire_rx);
    _credit.pump(*_enc, *_dec);
    _enc->pump();
    drainTx();
    return service::ServiceResult::Progress;
}

void RemoteWireService::drainTx()
{
    auto& out = _enc->output();
    while (!out.eof()) {
        auto p = out.peek(4096);
        if (!p.has_value() || p.value().size == 0) {
            break;
        }
        auto rsv = _wire_tx->reserve(p.value().size);
        if (!rsv.has_value() || rsv.value().size == 0) {
            break;
        }
        size_t n = rsv.value().size < p.value().size ? rsv.value().size : p.value().size;
        ::memcpy(rsv.value().data, p.value().data, n);
        (void)_wire_tx->commit(n);
        (void)out.advance(n);
    }
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_ADAPTER_INL_
