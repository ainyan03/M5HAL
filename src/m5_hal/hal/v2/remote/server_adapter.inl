// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_ADAPTER_INL_
#define M5_HAL_REMOTE_SERVER_ADAPTER_INL_

#include "./server_adapter.hpp"

#include "../diag.hpp"
#include "./wire_drain.hpp"

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
        auto drained = drainTx();
        if (!drained.has_value()) {
            return m5::stl::make_unexpected(drained.error());
        }
    }
    if (!_external_poll) {
        auto pumped = pumpWire();
        if (!pumped.has_value()) {
            return m5::stl::make_unexpected(pumped.error());
        }
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
        auto flushed = flushTx();
        if (!flushed.has_value()) {
            return m5::stl::make_unexpected(flushed.error());
        }
    }
    return count;
}

result_t<void> RemoteServerAdapter::pumpWire()
{
    auto drained = drainTx();
    if (!drained.has_value()) {
        return m5::stl::make_unexpected(drained.error());
    }
    auto decoded = _dec->pump(*_wire_rx);
    if (!decoded.has_value()) {
        return m5::stl::make_unexpected(decoded.error());
    }
    sendCreditIfChanged();
    return flushTx();
}

result_t<void> RemoteServerAdapter::flushTx()
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

result_t<detail::DrainProgress> RemoteServerAdapter::drainTx()
{
    return detail::drainToSink(_enc->output(), *_wire_tx);
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
    auto drained = drainTx();
    if (!drained.has_value()) {
        return service::ServiceResult::Error;
    }
    auto decoded = _dec->pump(*_wire_rx);
    if (!decoded.has_value()) {
        return service::ServiceResult::Error;
    }
    _credit.pump(*_enc, *_dec);
    auto encoded = _enc->pump();
    if (!encoded.has_value()) {
        return service::ServiceResult::Error;
    }
    drained = drainTx();
    if (!drained.has_value()) {
        return service::ServiceResult::Error;
    }
    return service::ServiceResult::Progress;
}

result_t<detail::DrainProgress> RemoteWireService::drainTx()
{
    return detail::drainToSink(_enc->output(), *_wire_tx);
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_ADAPTER_INL_
