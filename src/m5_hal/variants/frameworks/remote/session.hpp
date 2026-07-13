// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_SESSION_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/data.hpp"
#include "../../../hal/v2/data/mux.hpp"
#include "../../../hal/v2/frame/frame.hpp"
#include "../../../hal/v2/remote/credit_notifier.hpp"
#include "../../../hal/v2/remote/remote.hpp"
#include "../../../hal/v2/remote/session_handle.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

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
        // Insurance timer for a stream_id put into quarantine by
        // quarantineStream() (see below). This is an *inactivity* timer,
        // not a total lifetime bound: it is reset on every stray Data frame
        // observed for the quarantined id (see the stale-Data observer
        // wired up in the constructor), mirroring the server-side pending
        // stream timeout it backstops (server.inl's
        // `_pending_stream.last_progress_ms`, refreshed on every chunk —
        // not a fixed deadline either). Default = the server's default
        // pending-stream timeout (5000ms, server.hpp
        // Server::Config::_pending_stream_timeout_ms) plus margin: in the
        // common case a matching Response/Control releases the entry first,
        // this backstop only fires when that terminal frame itself is lost.
        uint32_t stream_quarantine_ms = 6000;
    };

    using event_handler_t = void (*)(void* ctx, uint8_t seq, data::ConstDataSpan body);
    using peer_poll_fn_t  = void (*)(void*);

    RemoteSession(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec, data::Source& wire_rx, data::Sink& wire_tx);
    ~RemoteSession();

    // The constructor registers `this` as the decoder's frame-handler
    // context; a moved/copied instance would leave that registration
    // dangling (responses silently written into a dead object). Rebind by
    // destroy + placement-new instead.
    RemoteSession(const RemoteSession&)            = delete;
    RemoteSession& operator=(const RemoteSession&) = delete;

    // One canonical gate per session. Low-level compatibility proxies must
    // obtain this handle instead of manufacturing independent mutexes around
    // the same mutable protocol state.
    std::shared_ptr<RemoteSessionHandle> sharedHandle() const
    {
        return _session_handle;
    }

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

    // Both callbacks run synchronously while the canonical session lease is
    // held. They must not re-enter or destroy this RemoteSession (including
    // destroying its owning Hal). Dropping ordinary proxy references is
    // permitted; proxy retirement never recursively waits on this lease.
    void setPeerPoll(peer_poll_fn_t fn, void* ctx)
    {
        _peer_poll = fn;
        _peer_ctx  = ctx;
    }

    result_t<void> request(data::ConstDataSpan script);
    // Same as above, but also reports the seq this request used via
    // *out_seq (regardless of outcome) — callers that may need to
    // quarantineStream() on failure must key the entry on the exact seq
    // this call consumed, not on a value inferred from session-wide state
    // (a shared session can interleave other request()/requestNoResponse()
    // calls between enqueue and failure — see quarantineStream() below).
    result_t<void> request(data::ConstDataSpan script, uint8_t* out_seq);
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

    // Timeout/ambiguous-failure variant of detachStream(): the stream_id is
    // not returned to the free pool immediately (an unanswered or
    // inconclusive request does not imply the device has stopped writing
    // Data for it — see spec/design/remote.md §timeout / resync). `seq`
    // must be the exact seq the failed request() call used (its out_seq
    // out-param — see request() above), not inferred from session-wide
    // state. encoder/decoder are unregistered like detachStream so any
    // straggling Data is discarded, but _stream_used keeps the id reserved
    // until releaseQuarantineForSeq()/sweepExpiredQuarantine()/
    // clearAllQuarantine() frees it.
    void quarantineStream(uint8_t stream_id, uint8_t seq);

    // True once the most recent request() call's Request frame left the
    // local encoder queue (writeFrame() succeeded), regardless of what
    // happened after. A caller that fails after this point cannot assume
    // the device never saw the request — see quarantineStream().
    bool lastRequestEnqueued() const
    {
        return _last_request_enqueued;
    }

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
    //
    // A quarantine entry's release condition is keyed solely on seq
    // (releaseQuarantineForSeq()), and Data frames carry no seq/generation
    // at all. With only a 7 bit space (128 values) and no skip, cycling
    // seq back onto a still-quarantined value would let an unrelated
    // Response for the *new* request holding that seq release the old
    // quarantine while the original transfer may still be live. Skipping
    // quarantined seqs closes that hole. At most kMaxStreams (16) entries
    // can be quarantined at once, each pinning exactly one seq out of 128,
    // so this loop is always bounded (worst case 17 iterations) and always
    // terminates.
    uint8_t nextSeq()
    {
        for (;;) {
            uint8_t seq = _seq;
            _seq        = static_cast<uint8_t>((_seq + 1) & 0x7F);
            if (!isQuarantinedSeq(seq)) {
                return seq;
            }
        }
    }

    bool isQuarantinedSeq(uint8_t seq) const
    {
        for (uint8_t i = 0; i < kMaxStreams; ++i) {
            if ((_stream_quarantined & (1u << i)) != 0 && _quarantine[i].seq == seq) {
                return true;
            }
        }
        return false;
    }

    uint8_t allocateStreamId();
    result_t<void> pumpWire();
    result_t<void> flushTx();
    void drainTx();
    void sendCreditIfChanged();
    result_t<void> awaitResponse(uint8_t seq, frame::Kind want_kind);
    static void frameHandlerThunk(void* ctx, const frame::View& view);
    void onFrame(const frame::View& view);
    static void staleDataThunk(void* ctx, uint8_t stream_id);
    void onStaleData(uint8_t stream_id);

    // Quarantine bookkeeping (stream_id timeout resync — see
    // quarantineStream() above and spec/design/remote.md §timeout / resync).
    void releaseQuarantineForSeq(uint8_t seq);
    void sweepExpiredQuarantine();
    void clearAllQuarantine();

    data::MuxFrameEncoder* _enc;
    data::MuxFrameDecoder* _dec;
    data::Source* _wire_rx;
    data::Sink* _wire_tx;
    std::shared_ptr<RemoteSessionHandle> _session_handle;
    Config _config;
    event_handler_t _event_fn   = nullptr;
    void* _event_ctx            = nullptr;
    peer_poll_fn_t _peer_poll   = nullptr;
    void* _peer_ctx             = nullptr;
    uint8_t _seq                = 0;
    uint16_t _stream_used       = 0;
    bool _last_request_enqueued = false;
    // Bit i set = stream i is timed-out-but-not-yet-released (see
    // quarantineStream()). _stream_used stays set for these too, so a
    // quarantined id is unavailable to allocateStreamId() until the matching
    // _quarantine[] entry is released.
    uint16_t _stream_quarantined = 0;
    struct QuarantineEntry {
        uint8_t seq       = 0;  // request() seq the entry waits to observe in a Response/Control
        uint32_t start_ms = 0;  // quarantineStream() timestamp; compared against Config::stream_quarantine_ms
    };
    QuarantineEntry _quarantine[kMaxStreams]{};
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
