// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_ADAPTER_HPP_
#define M5_HAL_REMOTE_SERVER_ADAPTER_HPP_

#include "../service/service.hpp"
#include "./credit_notifier.hpp"
#include "./remote.hpp"
#include "./wire_drain.hpp"

namespace m5::hal::v2::remote {

// RemoteSession lives in variants/frameworks/remote/session.hpp.

// ---- RemoteServerAdapter — device-side service over MuxFrameEncoder/Decoder --
//
// Receives request frames via MuxFrameDecoder's frame handler, dispatches to
// a user handler, and sends response frames via MuxFrameEncoder::writeFrame().
class RemoteServerAdapter {
public:
    using handler_t      = result_t<void> (*)(void* ctx, frame::Kind kind, uint8_t seq, data::ConstDataSpan payload,
                                         data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec);
    using poll_handler_t = result_t<void> (*)(void* ctx, data::MuxFrameEncoder& enc);

    RemoteServerAdapter(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec, data::Source& wire_rx,
                        data::Sink& wire_tx)
        : _enc{&enc}, _dec{&dec}, _wire_rx{&wire_rx}, _wire_tx{&wire_tx}
    {
        _dec->setFrameHandler(frameHandlerThunk, this);
    }

    // Same self-registration constraint as RemoteSession: not copyable,
    // not movable (the decoder holds `this` as its handler context).
    RemoteServerAdapter(const RemoteServerAdapter&)            = delete;
    RemoteServerAdapter& operator=(const RemoteServerAdapter&) = delete;

    void setHandler(handler_t fn, void* ctx)
    {
        _handler     = fn;
        _handler_ctx = ctx;
    }

    void setPollHandler(poll_handler_t fn, void* ctx)
    {
        _poll_handler     = fn;
        _poll_handler_ctx = ctx;
    }

    void setExternalPoll(bool external)
    {
        _external_poll = external;
    }

    result_t<size_t> service();
    result_t<void> pumpWire();
    result_t<void> flushTx();
    result_t<detail::DrainProgress> drainTx();

private:
    static void frameHandlerThunk(void* ctx, const frame::View& view);
    void onFrame(const frame::View& view);
    void sendCreditIfChanged();

    data::MuxFrameEncoder* _enc;
    data::MuxFrameDecoder* _dec;
    data::Source* _wire_rx;
    data::Sink* _wire_tx;
    handler_t _handler           = nullptr;
    void* _handler_ctx           = nullptr;
    poll_handler_t _poll_handler = nullptr;
    void* _poll_handler_ctx      = nullptr;
    bool _external_poll          = false;
    detail::CreditNotifier _credit;
    size_t _pending_count               = 0;
    error::error_t _frame_handler_error = error::error_t::OK;
};

// ---- RemoteWireService — IService wrapper for pump/flush --------------------
//
// Drives wire I/O cooperatively alongside bus transfers.
class RemoteWireService : public service::IService {
public:
    RemoteWireService(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec, data::Source& wire_rx,
                      data::Sink& wire_tx)
        : _enc{&enc}, _dec{&dec}, _wire_rx{&wire_rx}, _wire_tx{&wire_tx}
    {
    }

private:
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    result_t<detail::DrainProgress> drainTx();

    data::MuxFrameEncoder* _enc;
    data::MuxFrameDecoder* _dec;
    data::Source* _wire_rx;
    data::Sink* _wire_tx;
    detail::CreditNotifier _credit;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_ADAPTER_HPP_
