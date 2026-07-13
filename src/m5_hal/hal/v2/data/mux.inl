// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_DATA_MUX_INL_
#define M5_HAL_HAL_V2_DATA_MUX_INL_

#include "mux.hpp"

namespace m5::hal::v2::data {

MuxFrameEncoder::MuxFrameEncoder(memory::Allocator& alloc) : _output{alloc}, _alloc{&alloc}
{
}

void MuxFrameEncoder::setAllocator(memory::Allocator& alloc)
{
    _alloc = &alloc;
    _output.setAllocator(alloc);
}

bool MuxFrameEncoder::attach(uint8_t stream_id, Source& src)
{
    if (stream_id >= kMaxStreams) {
        return false;
    }
    _streams[stream_id].source = &src;
    return true;
}

void MuxFrameEncoder::detach(uint8_t stream_id)
{
    if (stream_id < kMaxStreams) {
        _streams[stream_id].source = nullptr;
    }
}

Source* MuxFrameEncoder::stream(uint8_t stream_id) const
{
    if (stream_id < kMaxStreams) {
        return _streams[stream_id].source;
    }
    return nullptr;
}

void MuxFrameEncoder::updateRemoteCredit(uint8_t blocks)
{
    _remote_credit = blocks;
    _credit_gated  = true;
}

uint8_t MuxFrameEncoder::remoteCredit() const
{
    return _remote_credit;
}

bool MuxFrameEncoder::creditGated() const
{
    return _credit_gated;
}

memory::Allocator* MuxFrameEncoder::allocator() const
{
    return _alloc;
}

m5::hal::v2::result_t<size_t> MuxFrameEncoder::pump()
{
    if (_alloc == nullptr) {
        return size_t{0};
    }
    // Drain attached sources until credit, allocation, or data runs out.
    // The outer loop keeps per-stream fairness round-robin (one frame per
    // stream per pass); producing a single frame per call would cap
    // throughput at the caller's pump cadence.
    size_t frames   = 0;
    bool progressed = true;
    while (progressed) {
        progressed = false;
        for (uint8_t i = 0; i < kMaxStreams; ++i) {
            auto* src = _streams[i].source;
            if (src == nullptr || src->eof()) {
                continue;
            }
            // Skip open-but-idle sources before spending credit checks or a
            // Temp block on them: pre-peek is safe (peek is idempotent and
            // borrows only until buildDataFrame's own peek replaces it), and
            // it stops a per-pump allocate/free churn on every idle stream.
            auto peeked = src->peek(1);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            if (peeked->size == 0) {
                continue;
            }
            if (_credit_gated && _remote_credit == 0) {
                return frames;
            }
            // buildDataFrame() consumes the source, so make sure the output
            // queue can take the block *before* building — a post-build
            // addBlock failure would silently drop consumed payload.
            if (_output.blockCount() >= BlockSource::kMaxBlocks) {
                return frames;
            }
            auto* block = static_cast<uint8_t*>(_alloc->allocate(frame::kMaxFrameSize, memory::usage_t::Temp));
            if (block == nullptr) {
                return frames;
            }
            auto frame_size = frame::buildDataFrame(block, i, *src);
            if (!frame_size.has_value()) {
                _alloc->deallocate(block);
                return m5::stl::make_unexpected(frame_size.error());
            }
            if (frame_size.value() == 0) {
                _alloc->deallocate(block);
                continue;
            }
            if (!_output.addBlock(block, frame_size.value())) {
                _alloc->deallocate(block);
                return frames;
            }
            ++frames;
            progressed = true;
            if (_credit_gated && _remote_credit > 0) {
                --_remote_credit;
            }
        }
    }
    return frames;
}

bool MuxFrameEncoder::writeFrame(frame::Kind kind, uint8_t b3, ConstDataSpan payload)
{
    if (_alloc == nullptr) {
        return false;
    }
    auto* block = static_cast<uint8_t*>(_alloc->allocate(frame::kMaxFrameSize, memory::usage_t::Temp));
    if (block == nullptr) {
        return false;
    }
    auto encoded = frame::encodeChecked({block, frame::kMaxFrameSize}, kind, b3, payload);
    if (!encoded.has_value()) {
        _alloc->deallocate(block);
        return false;
    }
    if (!_output.addBlock(block, encoded.value())) {
        _alloc->deallocate(block);
        return false;
    }
    return true;
}

bool MuxFrameEncoder::writeDelimiter()
{
    if (_alloc == nullptr) {
        return false;
    }
    auto* block = static_cast<uint8_t*>(_alloc->allocate(frame::kMaxFrameSize, memory::usage_t::Temp));
    if (block == nullptr) {
        return false;
    }
    auto encoded = frame::encodeDelimiter({block, frame::kMaxFrameSize});
    if (!encoded.has_value()) {
        _alloc->deallocate(block);
        return false;
    }
    if (!_output.addBlock(block, encoded.value())) {
        _alloc->deallocate(block);
        return false;
    }
    return true;
}

BlockSource& MuxFrameEncoder::output()
{
    return _output;
}

void MuxFrameEncoder::releaseAll()
{
    _output.releaseAll();
    for (auto& s : _streams) {
        s.source = nullptr;
    }
}

MuxFrameDecoder::MuxFrameDecoder(memory::Allocator& alloc) : _alloc{&alloc}
{
}

void MuxFrameDecoder::setAllocator(memory::Allocator& alloc)
{
    _alloc = &alloc;
    for (auto& s : _streams) {
        s.blocks.setAllocator(alloc);
    }
}

void MuxFrameDecoder::setFrameHandler(frame_handler_t fn, void* ctx)
{
    _handler     = fn;
    _handler_ctx = ctx;
}

void MuxFrameDecoder::setStaleDataObserver(stale_data_fn fn, void* ctx)
{
    _stale_data_fn  = fn;
    _stale_data_ctx = ctx;
}

Source* MuxFrameDecoder::createStream(uint8_t stream_id, uint8_t* buf, size_t buf_size)
{
    if (stream_id >= kMaxStreams) {
        return nullptr;
    }
    auto& s = _streams[stream_id];
    if (s.active) {
        return s.block_mode ? static_cast<Source*>(&s.blocks) : static_cast<Source*>(&s.ring.source());
    }
    s.ring.setBuf(buf, buf_size);
    s.direct_sink = nullptr;
    s.block_mode  = false;
    s.active      = true;
    return &s.ring.source();
}

Source* MuxFrameDecoder::createStream(uint8_t stream_id, size_t buf_size)
{
    if (_alloc == nullptr || stream_id >= kMaxStreams) {
        return nullptr;
    }
    auto* buf = static_cast<uint8_t*>(_alloc->allocate(buf_size, memory::usage_t::Persistent));
    if (buf == nullptr) {
        return nullptr;
    }
    auto* src = createStream(stream_id, buf, buf_size);
    if (src != nullptr) {
        _streams[stream_id].owned_buf = buf;
    } else {
        _alloc->deallocate(buf);
    }
    return src;
}

Source* MuxFrameDecoder::createBlockStream(uint8_t stream_id)
{
    if (_alloc == nullptr || stream_id >= kMaxStreams) {
        return nullptr;
    }
    auto& s = _streams[stream_id];
    if (s.active) {
        return s.block_mode ? static_cast<Source*>(&s.blocks) : static_cast<Source*>(&s.ring.source());
    }
    s.blocks.setAllocator(*_alloc);
    s.direct_sink = nullptr;
    s.block_mode  = true;
    s.active      = true;
    return &s.blocks;
}

void MuxFrameDecoder::destroyStream(uint8_t stream_id)
{
    if (stream_id >= kMaxStreams) {
        return;
    }
    auto& s = _streams[stream_id];
    if (!s.active) {
        return;
    }
    s.blocks.releaseAll();
    // Unbind (not just reset) the ring BEFORE freeing its backing buffer:
    // reset() keeps the buffer bound, so a Source/Sink view held across
    // destroyStream() would still look open and could lend out freed
    // memory. Unbound views report eof()/closed() and lend nothing.
    s.ring.setBuf(nullptr, 0);
    if (s.owned_buf != nullptr && _alloc != nullptr) {
        _alloc->deallocate(s.owned_buf);
    }
    s.direct_sink    = nullptr;
    s.owned_buf      = nullptr;
    s.block_mode     = false;
    s.active         = false;
    s.partial_offset = 0;
}

bool MuxFrameDecoder::setSink(uint8_t stream_id, Sink& sink)
{
    if (stream_id >= kMaxStreams) {
        return false;
    }
    auto& s = _streams[stream_id];
    if (s.active && s.block_mode) {
        s.blocks.releaseAll();
        s.block_mode = false;
    }
    s.direct_sink = &sink;
    s.active      = true;
    // A new destination invalidates any in-progress resume point for the
    // previous one.
    s.partial_offset = 0;
    return true;
}

void MuxFrameDecoder::clearSink(uint8_t stream_id)
{
    if (stream_id < kMaxStreams) {
        _streams[stream_id].direct_sink    = nullptr;
        _streams[stream_id].partial_offset = 0;
    }
}

Source* MuxFrameDecoder::source(uint8_t stream_id)
{
    if (stream_id >= kMaxStreams || !_streams[stream_id].active) {
        return nullptr;
    }
    auto& s = _streams[stream_id];
    return s.block_mode ? static_cast<Source*>(&s.blocks) : static_cast<Source*>(&s.ring.source());
}

memory::Allocator* MuxFrameDecoder::allocator() const
{
    return _alloc;
}

size_t MuxFrameDecoder::blockStreamFreeSlots() const
{
    size_t min_slots = SIZE_MAX;
    for (const auto& s : _streams) {
        if (s.active && s.block_mode) {
            const size_t remaining = BlockSource::kMaxBlocks - s.blocks.blockCount();
            if (remaining < min_slots) {
                min_slots = remaining;
            }
        }
    }
    return min_slots;
}

size_t MuxFrameDecoder::blockStreamReleasedTotal() const
{
    // Monotonic across stream destroy/recreate: the BlockSource members are
    // embedded in the fixed stream slots and never reset their counter.
    size_t total = 0;
    for (const auto& s : _streams) {
        total += s.blocks.releasedTotal();
    }
    return total;
}

m5::hal::v2::result_t<size_t> MuxFrameDecoder::pump(Source& wire_in)
{
    size_t count = 0;
    for (size_t iter = 0; iter < kPumpMaxFramesPerCall; ++iter) {
        frame::View view;
        frame::DecodeResult result;
        bool pending = _pending_len != 0;

        if (pending) {
            auto filled = fillPendingFrame(wire_in);
            if (!filled.has_value()) {
                return m5::stl::make_unexpected(filled.error());
            }
            if (!filled.value()) {
                break;
            }
            result = frame::decode({_pending_frame, _pending_len}, view);
        } else {
            auto peeked = wire_in.peek(frame::kMaxFrameSize);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            if (peeked.value().size == 0) {
                break;
            }
            result = frame::decode(peeked.value(), view);
            if (result.status == frame::DecodeStatus::NeedMore) {
                auto begun = beginPendingFrame(wire_in, peeked.value());
                if (!begun.has_value()) {
                    return m5::stl::make_unexpected(begun.error());
                }
                if (!begun.value()) {
                    break;
                }
                continue;
            }
        }

        switch (result.status) {
            case frame::DecodeStatus::NeedMore:
                return count;
            case frame::DecodeStatus::Padding:
            case frame::DecodeStatus::Delimiter:
                if (pending) {
                    consumePendingFrame(result.consumed);
                } else {
                    auto advanced = wire_in.advance(result.consumed);
                    if (!advanced.has_value()) {
                        return m5::stl::make_unexpected(advanced.error());
                    }
                }
                continue;
            case frame::DecodeStatus::InvalidPrefix:
            case frame::DecodeStatus::InvalidSize:
            case frame::DecodeStatus::InvalidCheck:
                if (pending) {
                    consumePendingFrame(result.consumed);
                } else {
                    auto advanced = wire_in.advance(result.consumed);
                    if (!advanced.has_value()) {
                        return m5::stl::make_unexpected(advanced.error());
                    }
                }
                continue;
            case frame::DecodeStatus::Ok:
                break;
        }

        if (view.kind == frame::Kind::Data) {
            if (!deliverData(view.b3, view.payload)) {
                return count;
            }
        } else if (_handler != nullptr) {
            _handler(_handler_ctx, view);
        }

        if (pending) {
            consumePendingFrame(result.consumed);
        } else {
            auto advanced = wire_in.advance(result.consumed);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
        }
        ++count;
    }
    return count;
}

m5::hal::v2::result_t<bool> MuxFrameDecoder::beginPendingFrame(Source& wire_in, ConstDataSpan bytes)
{
    if (bytes.data == nullptr || bytes.size == 0 || bytes.size > sizeof(_pending_frame)) {
        return false;
    }
    ::memcpy(_pending_frame, bytes.data, bytes.size);
    auto advanced = wire_in.advance(bytes.size);
    if (!advanced.has_value()) {
        clearPendingFrame();
        return m5::stl::make_unexpected(advanced.error());
    }
    _pending_len = bytes.size;
    updatePendingFrameNeed();
    return true;
}

m5::hal::v2::result_t<bool> MuxFrameDecoder::fillPendingFrame(Source& wire_in)
{
    updatePendingFrameNeed();
    while (_pending_len < _pending_need) {
        auto peeked = wire_in.peek(_pending_need - _pending_len);
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        if (peeked.value().size == 0) {
            return false;
        }
        size_t n = peeked.value().size;
        if (n > _pending_need - _pending_len) {
            n = _pending_need - _pending_len;
        }
        ::memcpy(_pending_frame + _pending_len, peeked.value().data, n);
        auto advanced = wire_in.advance(n);
        if (!advanced.has_value()) {
            clearPendingFrame();
            return m5::stl::make_unexpected(advanced.error());
        }
        _pending_len += n;
        updatePendingFrameNeed();
    }
    return true;
}

void MuxFrameDecoder::updatePendingFrameNeed()
{
    if (_pending_len == 0) {
        _pending_need = 1;
        return;
    }
    const uint8_t len_byte = _pending_frame[0];
    if (len_byte == 0) {
        _pending_need =
            (_pending_len >= 2 && _pending_frame[1] == static_cast<uint8_t>(frame::Kind::Delimiter)) ? 2 : 1;
        return;
    }
    if (_pending_len < frame::kPrefixSize) {
        _pending_need = frame::kPrefixSize;
        return;
    }
    const size_t need = frame::kPrefixSize + static_cast<size_t>(len_byte);
    _pending_need     = need <= sizeof(_pending_frame) ? need : sizeof(_pending_frame);
}

void MuxFrameDecoder::consumePendingFrame(size_t consumed)
{
    if (consumed >= _pending_len) {
        clearPendingFrame();
        return;
    }
    if (consumed == 0) {
        return;
    }
    ::memmove(_pending_frame, _pending_frame + consumed, _pending_len - consumed);
    _pending_len -= consumed;
    updatePendingFrameNeed();
}

void MuxFrameDecoder::clearPendingFrame()
{
    _pending_len  = 0;
    _pending_need = 0;
}

void MuxFrameDecoder::releaseAll()
{
    clearPendingFrame();
    for (uint8_t i = 0; i < kMaxStreams; ++i) {
        destroyStream(i);
    }
}

MuxFrameDecoder::~MuxFrameDecoder()
{
    releaseAll();
}

bool MuxFrameDecoder::deliverData(uint8_t stream_id, ConstDataSpan payload)
{
    if (stream_id >= kMaxStreams) {
        return true;
    }
    if (!_streams[stream_id].active) {
        // A Data frame with nowhere to go: `active` is only set by
        // setSink()/createStream()/createBlockStream(), so this is the
        // normal case for a stream_id that was attached without a Sink
        // (e.g. a tx-only transfer) or never attached at all. It is also
        // what a quarantined tx-only/no-sink stream_id looks like on this
        // path, since quarantineStream() detaches without ever having set a
        // Sink for it. Report it so RemoteSession can treat it as evidence
        // the peer is still producing for that id and keep its insurance
        // timer alive instead of expiring on a fixed deadline.
        if (payload.size > 0 && _stale_data_fn != nullptr) {
            _stale_data_fn(_stale_data_ctx, stream_id);
        }
        return true;
    }
    if (payload.size == 0) {
        return true;
    }
    auto& s = _streams[stream_id];
    if (s.block_mode) {
        return deliverBlockData(s, payload);
    }
    Sink* dst = s.direct_sink != nullptr ? s.direct_sink : (s.ring.capacity() > 0 ? &s.ring.sink() : nullptr);
    if (dst == nullptr) {
        // Reachable only for a stream whose direct Sink was later removed by
        // clearSink() (setSink() always installs one and `active` stays set):
        // this is what a quarantined rx-bearing stream_id looks like, so it
        // must feed the same staleness evidence as the !active branch above
        // or the insurance timer would expire under an alive producer.
        if (_stale_data_fn != nullptr) {
            _stale_data_fn(_stale_data_ctx, stream_id);
        }
        return true;
    }
    // Resume from wherever the previous attempt at *this same* frame left
    // off. pump() only advances past a Data frame once this function
    // returns true, so a call that returns false here is retried on the
    // next pump() with the identical payload bytes (same stream_id, same
    // content) — restarting from offset 0 would recommit the already-
    // delivered prefix and duplicate it on the sink. An all-or-nothing
    // pre-check (require free() >= payload.size before committing
    // anything) was considered instead but rejected: a payload larger
    // than the ring's total capacity would then never be deliverable,
    // permanently stalling that stream.
    size_t offset = s.partial_offset < payload.size ? s.partial_offset : 0;
    while (offset < payload.size) {
        auto rsv = dst->reserve(payload.size - offset);
        if (!rsv.has_value() || rsv.value().size == 0) {
            s.partial_offset = offset;
            return false;
        }
        size_t n = rsv.value().size;
        if (n > payload.size - offset) {
            n = payload.size - offset;
        }
        ::memcpy(rsv.value().data, payload.data + offset, n);
        auto c = dst->commit(n);
        if (!c.has_value()) {
            // A failing commit() may still have accepted a prefix (a
            // StreamSink direct sink whose transport short-writes). Count
            // that prefix as delivered, or the retry would recommit it.
            size_t accepted = dst->partialCommitAccepted();
            if (accepted > n) {
                accepted = n;
            }
            s.partial_offset = offset + accepted;
            return false;
        }
        offset += n;
    }
    s.partial_offset = 0;
    return true;
}

bool MuxFrameDecoder::deliverBlockData(MuxFrameDecoder::Stream& s, ConstDataSpan payload)
{
    if (_alloc == nullptr || payload.size > memory::Allocator::tempBlockSize()) {
        return false;
    }
    const size_t before = _alloc->usedBlocks();
    auto* block         = static_cast<uint8_t*>(_alloc->allocate(payload.size, memory::usage_t::Temp));
    if (block == nullptr) {
        return false;
    }
    if (_alloc->usedBlocks() == before) {
        _alloc->deallocate(block);
        return false;
    }
    ::memcpy(block, payload.data, payload.size);
    if (!s.blocks.addBlock(block, payload.size)) {
        _alloc->deallocate(block);
        return false;
    }
    return true;
}

}  // namespace m5::hal::v2::data

#endif  // M5_HAL_HAL_V2_DATA_MUX_INL_
