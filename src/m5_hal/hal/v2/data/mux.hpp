// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_MUX_HPP_
#define M5_HAL_DATA_MUX_HPP_

#include "../data.hpp"
#include "../frame/frame.hpp"
#include "../memory/allocator.hpp"
#include "./block.hpp"
#include "./ring.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace m5::hal::v2::data {

// ---------------------------------------------------------------------------
// MuxFrameEncoder — Source-pull model encoder using the new frame format.
//
// attach(Source&) registers a data Source under a stream_id. pump()
// pulls data from attached Sources, builds frames via
// frame::buildDataFrame into TempBuffer blocks, and queues them into
// an internal BlockSource. output() returns that BlockSource.
//
//
// ---------------------------------------------------------------------------
class MuxFrameEncoder {
public:
    static constexpr size_t kMaxStreams = 16;

    MuxFrameEncoder() = default;

    explicit MuxFrameEncoder(memory::Allocator& alloc);

    void setAllocator(memory::Allocator& alloc);

    bool attach(uint8_t stream_id, Source& src);

    void detach(uint8_t stream_id);

    Source* stream(uint8_t stream_id) const;

    void updateRemoteCredit(uint8_t blocks);

    uint8_t remoteCredit() const;

    bool creditGated() const;

    memory::Allocator* allocator() const;

    size_t pump();

    bool writeFrame(frame::Kind kind, uint8_t b3, ConstDataSpan payload = {});

    bool writeDelimiter();

    BlockSource& output();

    void releaseAll();

private:
    struct Stream {
        Source* source = nullptr;
    };

    BlockSource _output;
    memory::Allocator* _alloc = nullptr;
    Stream _streams[kMaxStreams];
    uint8_t _remote_credit = 0;
    bool _credit_gated     = false;
};

// ---------------------------------------------------------------------------
// MuxFrameDecoder — frame-pull model decoder using the new frame format.
//
// pump(wire_in) reads frames from a wire Source, strips headers, and
// deposits data payloads into per-stream RingFIFO buffers. Upper
// layers read those buffers through source(stream_id).
//
// Non-data frames (credit, control, etc.) are passed to an optional
// frame handler callback for upper-layer processing.
//
// This is the only mux transport used by the frame-based remote layer.
// ---------------------------------------------------------------------------
class MuxFrameDecoder {
public:
    static constexpr size_t kMaxStreams = 16;

    // Upper bound of decode iterations per pump() call. Without a bound a
    // peer that keeps the wire busy (e.g. a GPIO event flood arriving faster
    // than the transport read timeout) livelocks the caller: response flags
    // set by the frame handler are only observable between pump() calls.
    static constexpr size_t kPumpMaxFramesPerCall = 32;

    using frame_handler_t = void (*)(void* ctx, const frame::View& view);

    MuxFrameDecoder() = default;

    explicit MuxFrameDecoder(memory::Allocator& alloc);

    void setAllocator(memory::Allocator& alloc);

    void setFrameHandler(frame_handler_t fn, void* ctx);

    Source* createStream(uint8_t stream_id, uint8_t* buf, size_t buf_size);

    Source* createStream(uint8_t stream_id, size_t buf_size);

    Source* createBlockStream(uint8_t stream_id);

    void destroyStream(uint8_t stream_id);

    bool setSink(uint8_t stream_id, Sink& sink);

    void clearSink(uint8_t stream_id);

    Source* source(uint8_t stream_id);

    memory::Allocator* allocator() const;

    /*!
      @brief Remaining queue slots of the most congested block-mode stream.

      Returns SIZE_MAX when no block-mode stream is active. The credit
      notifier caps the advertised credit with this value so a peer is
      never granted more frames than a block stream can actually queue
      (`BlockSource::kMaxBlocks`), which may be smaller than the temp
      pool block count.
     */
    size_t blockStreamFreeSlots() const;

    size_t blockStreamReleasedTotal() const;

    size_t pump(Source& wire_in);

    void releaseAll();

    ~MuxFrameDecoder();

    MuxFrameDecoder(const MuxFrameDecoder&)            = delete;
    MuxFrameDecoder& operator=(const MuxFrameDecoder&) = delete;
    MuxFrameDecoder(MuxFrameDecoder&&)                 = delete;
    MuxFrameDecoder& operator=(MuxFrameDecoder&&)      = delete;

private:
    struct Stream {
        RingFIFO ring;
        BlockSource blocks;
        Sink* direct_sink  = nullptr;
        uint8_t* owned_buf = nullptr;
        bool active        = false;
        bool block_mode    = false;
    };

    bool deliverData(uint8_t stream_id, ConstDataSpan payload);

    bool deliverBlockData(Stream& s, ConstDataSpan payload);

    bool beginPendingFrame(Source& wire_in, ConstDataSpan bytes);
    bool fillPendingFrame(Source& wire_in);
    void updatePendingFrameNeed();
    void consumePendingFrame(size_t consumed);
    void clearPendingFrame();

    memory::Allocator* _alloc = nullptr;
    frame_handler_t _handler  = nullptr;
    void* _handler_ctx        = nullptr;
    Stream _streams[kMaxStreams];
    uint8_t _pending_frame[frame::kMaxFrameSize]{};
    size_t _pending_len  = 0;
    size_t _pending_need = 0;
};

}  // namespace m5::hal::v2::data

#endif
