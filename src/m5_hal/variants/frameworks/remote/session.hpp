// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/data.hpp"
#include "../../../hal/v2/data/mux.hpp"
#include "../../../hal/v2/frame/frame.hpp"
#include "../../../hal/v2/remote/credit_notifier.hpp"
#include "../../../hal/v2/remote/remote.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::remote {

// ---- RemoteSession — host-side session over MuxFrameEncoder/Decoder -------
//
// Sends request/hello/ping frames via MuxFrameEncoder::writeFrame().
// Receives response/hello_resp/pong/error via MuxFrameDecoder's frame handler.
// Also drives data streams (attach/detach) through the encoder/decoder.
class RemoteSession {
public:
    struct Config {
        uint32_t response_timeout_ms = 2000;
    };

    using event_handler_t = void (*)(void* ctx, uint8_t seq, data::ConstDataSpan body);
    using peer_poll_fn_t  = void (*)(void*);

    RemoteSession(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec, data::Source& wire_rx, data::Sink& wire_tx);

    // The constructor registers `this` as the decoder's frame-handler
    // context; a moved/copied instance would leave that registration
    // dangling (responses silently written into a dead object). Rebind by
    // destroy + placement-new instead.
    RemoteSession(const RemoteSession&)            = delete;
    RemoteSession& operator=(const RemoteSession&) = delete;

    void setConfig(const Config& cfg)
    {
        _config = cfg;
    }

    const Config& getConfig() const
    {
        return _config;
    }

    void setEventHandler(event_handler_t fn, void* ctx)
    {
        _event_fn  = fn;
        _event_ctx = ctx;
    }

    void setPeerPoll(peer_poll_fn_t fn, void* ctx)
    {
        _peer_poll = fn;
        _peer_ctx  = ctx;
    }

    result_t<void> request(data::ConstDataSpan script);
    result_t<void> requestNoResponse(data::ConstDataSpan script);
    result_t<void> hello();
    result_t<void> ping();
    result_t<void> reset();
    result_t<size_t> poll(size_t max_msgs = 8);

    data::ConstDataSpan lastResponse() const
    {
        return {_resp_buf, _resp_len};
    }

    error::error_t lastRemoteError() const
    {
        return _last_error;
    }
    void clearRemoteError()
    {
        _last_error = error::error_t::OK;
    }

    static constexpr uint8_t kMaxStreams = data::MuxFrameEncoder::kMaxStreams;

    uint8_t attachStream(data::Source* tx, data::Sink* rx);
    void detachStream(uint8_t stream_id);
    result_t<void> checkResponse();

    uint32_t configGeneration() const
    {
        return _config_generation;
    }

    data::MuxFrameEncoder& encoder()
    {
        return *_enc;
    }
    data::MuxFrameDecoder& decoder()
    {
        return *_dec;
    }

private:
    // Sequence numbers live in the low 7 bits of B3; bit 7 is the
    // response-suppression flag (requestNoResponse). The counter must
    // never spill into that bit, or the peer silently withholds the
    // Response for a normal request.
    uint8_t nextSeq()
    {
        uint8_t seq = _seq;
        _seq        = static_cast<uint8_t>((_seq + 1) & 0x7F);
        return seq;
    }

    uint8_t allocateStreamId();
    void pumpWire();
    void flushTx();
    void drainTx();
    void sendCreditIfChanged();
    result_t<void> awaitResponse(uint8_t seq, frame::Kind want_kind);
    static void frameHandlerThunk(void* ctx, const frame::View& view);
    void onFrame(const frame::View& view);

    data::MuxFrameEncoder* _enc;
    data::MuxFrameDecoder* _dec;
    data::Source* _wire_rx;
    data::Sink* _wire_tx;
    Config _config;
    event_handler_t _event_fn  = nullptr;
    void* _event_ctx           = nullptr;
    peer_poll_fn_t _peer_poll  = nullptr;
    void* _peer_ctx            = nullptr;
    uint8_t _seq               = 0;
    uint16_t _stream_used      = 0;
    error::error_t _last_error = error::error_t::OK;
    uint8_t _resp_buf[frame::kMaxPayload];
    size_t _resp_len   = 0;
    size_t _rx_count   = 0;
    bool _got_response = false;
    bool _got_error    = false;
    detail::CreditNotifier _credit;
    uint8_t _awaiting_seq       = 0;
    frame::Kind _awaiting_kind  = frame::Kind::Response;
    uint32_t _config_generation = 0;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_HPP_
