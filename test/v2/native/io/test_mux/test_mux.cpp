// SPDX-License-Identifier: MIT
// Native gtest for MuxFrameEncoder / MuxFrameDecoder.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

namespace data   = m5::hal::v2::data;
namespace frame  = m5::hal::v2::frame;
namespace memory = m5::hal::v2::memory;
using error_t    = m5::hal::v2::error::error_t;

size_t pumpValue(data::MuxFrameEncoder& enc)
{
    auto r = enc.pump();
    if (!r.has_value()) {
        ADD_FAILURE() << "MuxFrameEncoder::pump failed: " << m5::hal::v2::error::toString(r.error());
        return 0;
    }
    return r.value();
}

size_t pumpValue(data::MuxFrameDecoder& dec, data::Source& src)
{
    auto r = dec.pump(src);
    if (!r.has_value()) {
        ADD_FAILURE() << "MuxFrameDecoder::pump failed: " << m5::hal::v2::error::toString(r.error());
        return 0;
    }
    return r.value();
}

class ErrorOnPeekSource : public data::Source {
public:
    m5::hal::v2::result_t<data::ConstDataSpan> peek(size_t max_len) override
    {
        (void)max_len;
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }

    m5::hal::v2::result_t<void> advance(size_t N) override
    {
        (void)N;
        return {};
    }

    bool eof() const override
    {
        return false;
    }
};

class ErrorOnAdvanceSource : public data::Source {
public:
    m5::hal::v2::result_t<data::ConstDataSpan> peek(size_t max_len) override
    {
        (void)max_len;
        return data::ConstDataSpan{_bytes, sizeof(_bytes)};
    }

    m5::hal::v2::result_t<void> advance(size_t N) override
    {
        (void)N;
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }

    bool eof() const override
    {
        return false;
    }

private:
    uint8_t _bytes[1] = {0x00};  // valid padding for decoder; payload for encoder
};

TEST(MuxFrameEncoder, AttachAndDetach)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    uint8_t dummy_data[] = {1, 2, 3};
    data::MemorySource src{dummy_data, sizeof(dummy_data)};

    ASSERT_TRUE(enc.attach(0, src));
    uint8_t id = 0;
    EXPECT_LT(id, data::MuxFrameEncoder::kMaxStreams);
    EXPECT_EQ(enc.stream(id), &src);

    enc.detach(id);
    EXPECT_EQ(enc.stream(id), nullptr);
}

TEST(MuxFrameEncoder, PumpProducesFramesInOutput)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    const uint8_t raw[] = {0xAA, 0xBB, 0xCC, 0xDD};
    data::MemorySource src{raw, sizeof(raw)};
    ASSERT_TRUE(enc.attach(0, src));
    uint8_t stream_id = 0;

    size_t frames = pumpValue(enc);
    EXPECT_EQ(frames, 1u);
    EXPECT_TRUE(src.eof());

    auto& out   = enc.output();
    auto peeked = out.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_GT(peeked.value().size, 0u);

    frame::View view;
    auto result = frame::decode(peeked.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, stream_id);
    ASSERT_EQ(view.payload.size, sizeof(raw));
    EXPECT_EQ(::memcmp(view.payload.data, raw, sizeof(raw)), 0);

    enc.releaseAll();
}

TEST(MuxFrameEncoder, LargeSourceProducesMultipleFrames)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    std::array<uint8_t, 600> big{};
    for (size_t i = 0; i < big.size(); ++i) {
        big[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource src{big.data(), big.size()};
    ASSERT_TRUE(enc.attach(0, src));
    uint8_t stream_id = 0;

    size_t total_frames = 0;
    while (!src.eof()) {
        total_frames += pumpValue(enc);
    }
    EXPECT_EQ(total_frames, 3u);  // 252 + 252 + 96

    auto& out       = enc.output();
    size_t verified = 0;
    while (!out.eof()) {
        auto peeked = out.peek(256);
        ASSERT_TRUE(peeked.has_value());
        if (peeked.value().size == 0) {
            break;
        }

        frame::View view;
        auto result = frame::decode(peeked.value(), view);
        ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
        EXPECT_EQ(view.b3, stream_id);

        for (size_t i = 0; i < view.payload.size; ++i) {
            EXPECT_EQ(view.payload.data[i], static_cast<uint8_t>((verified + i) & 0xFF));
        }
        verified += view.payload.size;
        ASSERT_TRUE(out.advance(result.consumed).has_value());
    }
    EXPECT_EQ(verified, big.size());

    enc.releaseAll();
}

TEST(MuxFrameEncoder, MultipleStreams)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    const uint8_t d0[] = {0x10, 0x11};
    const uint8_t d1[] = {0x20, 0x21, 0x22};
    data::MemorySource s0{d0, sizeof(d0)};
    data::MemorySource s1{d1, sizeof(d1)};
    ASSERT_TRUE(enc.attach(0, s0));
    uint8_t id0 = 0;
    ASSERT_TRUE(enc.attach(1, s1));
    uint8_t id1 = 1;
    EXPECT_NE(id0, id1);

    size_t frames = pumpValue(enc);
    EXPECT_EQ(frames, 2u);

    auto& out = enc.output();
    frame::View view;

    auto peeked = out.peek(256);
    auto result = frame::decode(peeked.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, id0);
    ASSERT_EQ(view.payload.size, sizeof(d0));
    ASSERT_TRUE(out.advance(result.consumed).has_value());

    peeked = out.peek(256);
    result = frame::decode(peeked.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, id1);
    ASSERT_EQ(view.payload.size, sizeof(d1));

    enc.releaseAll();
}

TEST(MuxFrameEncoder, EmptySourceNoFrames)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    data::MemorySource empty{nullptr, 0};
    enc.attach(0, empty);

    size_t frames = pumpValue(enc);
    EXPECT_EQ(frames, 0u);
    EXPECT_TRUE(enc.output().eof());
}

// An open source with nothing to send right now (streaming idle state):
// peek() lends nothing but eof() stays false.
class IdleOpenSource : public data::Source {
public:
    m5::hal::v2::result_t<data::ConstDataSpan> peek(size_t) override
    {
        return data::ConstDataSpan{};
    }
    m5::hal::v2::result_t<void> advance(size_t) override
    {
        return {};
    }
    bool eof() const override
    {
        return false;
    }
};

size_t g_fallback_allocs = 0;
void* countingMalloc(size_t size, memory::usage_t)
{
    ++g_fallback_allocs;
    return std::malloc(size);
}
void countingFree(void* p)
{
    std::free(p);
}

TEST(MuxFrameEncoder, IdleOpenStreamPumpDoesNotAllocate)
{
    // Regression anchor: pump() used to allocate (and free) one Temp block
    // per idle stream on every call before discovering there was nothing
    // to send — steady-state churn on every open-but-quiet stream.
    static memory::Allocator alloc;  // fresh pool, no fallback yet
    data::MuxFrameEncoder enc{alloc};

    IdleOpenSource idle;
    ASSERT_TRUE(enc.attach(0, idle));

    // Fill the fixed Temp pool so any allocation inside pump() must go
    // through the counting fallback (an Allocator without hooks falls back
    // to std::malloc, so "allocate until nullptr" would never terminate;
    // the first counted fallback call is the "pool is full" signal).
    g_fallback_allocs = 0;
    alloc.setFallback(countingMalloc, countingFree);
    std::vector<void*> held;
    while (g_fallback_allocs == 0) {
        void* p = alloc.allocate(64);
        ASSERT_NE(p, nullptr);
        if (g_fallback_allocs > 0) {
            alloc.deallocate(p);  // came from the fallback: pool is now full
            break;
        }
        held.push_back(p);
        ASSERT_LT(held.size(), 4096u);  // safety bound
    }

    g_fallback_allocs = 0;
    EXPECT_EQ(pumpValue(enc), 0u);
    EXPECT_EQ(g_fallback_allocs, 0u);

    for (void* p : held) {
        alloc.deallocate(p);
    }
}

TEST(MuxFrameDecoder, DestroyStreamUnbindsHeldSourceView)
{
    // Regression anchor: destroyStream() freed the owned buffer but only
    // reset() the ring, leaving it bound to the freed storage — a Source
    // view held across the destroy still looked open and could lend out
    // freed memory.
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    auto* src = dec.createStream(2, 64);
    ASSERT_NE(src, nullptr);
    EXPECT_FALSE(src->eof());

    dec.destroyStream(2);

    EXPECT_TRUE(src->eof());
    EXPECT_TRUE(src->closed());
    auto peeked = src->peek(8);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 0u);
}

TEST(MuxFrameEncoder, WriteControlFrame)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    const uint8_t payload[] = {0x01, 0x02};
    ASSERT_TRUE(enc.writeFrame(frame::Kind::Request, 0x07, {payload, sizeof(payload)}));

    auto& out   = enc.output();
    auto peeked = out.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_GT(peeked.value().size, 0u);

    frame::View view;
    auto result = frame::decode(peeked.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Request);
    EXPECT_EQ(view.b3, 0x07);
    ASSERT_EQ(view.payload.size, sizeof(payload));
    EXPECT_EQ(::memcmp(view.payload.data, payload, sizeof(payload)), 0);

    enc.releaseAll();
}

TEST(MuxFrameEncoder, SourceAdvanceErrorPropagates)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    ErrorOnAdvanceSource src;
    ASSERT_TRUE(enc.attach(0, src));

    auto pumped = enc.pump();
    ASSERT_FALSE(pumped.has_value());
    EXPECT_EQ(pumped.error(), error_t::IO_ERROR);

    enc.releaseAll();
}

TEST(MuxFrameEncoder, MixedDataAndControlFrames)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    const uint8_t d[] = {0xAA};
    data::MemorySource src{d, sizeof(d)};
    ASSERT_TRUE(enc.attach(0, src));
    uint8_t stream_id = 0;

    ASSERT_TRUE(enc.writeFrame(frame::Kind::HelloReq, 0, {}));
    pumpValue(enc);

    auto& out = enc.output();
    frame::View view;

    auto p      = out.peek(256);
    auto result = frame::decode(p.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::HelloReq);
    ASSERT_TRUE(out.advance(result.consumed).has_value());

    p      = out.peek(256);
    result = frame::decode(p.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, stream_id);

    enc.releaseAll();
}

TEST(MuxFrameEncoder, CreditGateLimitsDataFrames)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    std::array<uint8_t, frame::kMaxDataPayload * 3> raw{};
    for (size_t i = 0; i < raw.size(); ++i) {
        raw[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource src{raw.data(), raw.size()};
    ASSERT_TRUE(enc.attach(0, src));

    EXPECT_FALSE(enc.creditGated());
    enc.updateRemoteCredit(2);
    EXPECT_TRUE(enc.creditGated());
    EXPECT_EQ(enc.remoteCredit(), 2u);

    // pump() batches: one call drains until the credit gate closes.
    EXPECT_EQ(pumpValue(enc), 2u);
    EXPECT_EQ(enc.remoteCredit(), 0u);
    EXPECT_EQ(pumpValue(enc), 0u);
    EXPECT_FALSE(src.eof());
    EXPECT_EQ(enc.output().blockCount(), 2u);

    ASSERT_TRUE(enc.writeFrame(frame::Kind::Ping, 0x44, {}));
    EXPECT_EQ(enc.output().blockCount(), 3u);
    EXPECT_EQ(pumpValue(enc), 0u);

    enc.updateRemoteCredit(1);
    EXPECT_EQ(pumpValue(enc), 1u);
    EXPECT_EQ(enc.remoteCredit(), 0u);
    EXPECT_TRUE(src.eof());

    enc.releaseAll();
}

TEST(MuxFrameEncoder, WriteDelimiter)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};

    ASSERT_TRUE(enc.writeDelimiter());

    auto& out   = enc.output();
    auto peeked = out.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_GE(peeked.value().size, 2u);
    EXPECT_EQ(peeked.value().data[0], 0x00);
    EXPECT_EQ(peeked.value().data[1], 0x55);

    enc.releaseAll();
}

TEST(MuxFrameEncoder, OptionalPrefixCannotConsumeRequiredFrameAllocation)
{
    memory::Allocator alloc;
    alloc.setFallback(+[](size_t, memory::usage_t) -> void* { return nullptr; }, +[](void*) {});
    data::MuxFrameEncoder enc{alloc};
    std::array<void*, memory::Allocator::tempBlockCount() - 1> held{};
    for (auto& block : held) {
        block = alloc.allocate(frame::kMaxFrameSize, memory::usage_t::Temp);
        ASSERT_NE(block, nullptr);
    }

    const uint8_t event_payload[]    = {0x11};
    const uint8_t response_payload[] = {0x22};
    bool prefix_written              = true;
    ASSERT_TRUE(enc.writeFrameWithOptionalPrefix(frame::Kind::Event, 1, {event_payload, sizeof(event_payload)},
                                                 frame::Kind::Response, 2, {response_payload, sizeof(response_payload)},
                                                 &prefix_written));
    EXPECT_FALSE(prefix_written);
    ASSERT_EQ(enc.output().blockCount(), 1u);

    auto bytes = enc.output().peek(frame::kMaxFrameSize);
    ASSERT_TRUE(bytes.has_value());
    frame::View view;
    auto decoded = frame::decode(bytes.value(), view);
    ASSERT_EQ(decoded.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Response);
    EXPECT_EQ(view.b3, 2u);
    ASSERT_EQ(view.payload.size, sizeof(response_payload));
    EXPECT_EQ(view.payload.data[0], response_payload[0]);

    enc.releaseAll();
    for (auto* block : held) {
        alloc.deallocate(block);
    }
}

// ============================================================================
// MuxFrameDecoder (frame-pull model, new frame format)
// ============================================================================

TEST(MuxFrameDecoder, CreateStreamAndReceiveData)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    auto* src = dec.createStream(3, 512);
    ASSERT_NE(src, nullptr);
    EXPECT_EQ(dec.source(3), src);

    const uint8_t payload[] = {0xAA, 0xBB, 0xCC};
    uint8_t wire[frame::kMaxFrameSize];
    auto encoded = frame::encodeData({wire, sizeof(wire)}, 3, {payload, sizeof(payload)});
    ASSERT_TRUE(encoded.has_value());

    data::MemorySource wire_src{wire, encoded.value()};
    size_t count = pumpValue(dec, wire_src);
    EXPECT_EQ(count, 1u);

    auto peeked = src->peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, sizeof(payload));
    EXPECT_EQ(::memcmp(peeked.value().data, payload, sizeof(payload)), 0);

    dec.releaseAll();
}

TEST(MuxFrameDecoder, SourceErrorsPropagate)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    ErrorOnPeekSource peek_error;
    auto pumped = dec.pump(peek_error);
    ASSERT_FALSE(pumped.has_value());
    EXPECT_EQ(pumped.error(), error_t::IO_ERROR);

    ErrorOnAdvanceSource advance_error;
    pumped = dec.pump(advance_error);
    ASSERT_FALSE(pumped.has_value());
    EXPECT_EQ(pumped.error(), error_t::IO_ERROR);
}

// Regression for a partial-delivery bug: when the destination ring is too
// small to take a Data frame's whole payload in one go, deliverData() used
// to leave the frame unconsumed and restart from payload offset 0 on the
// next pump() -- duplicating the prefix already committed to the ring. The
// fix threads a per-stream resume offset through the decoder so a retried
// pump() continues where the previous attempt left off; the frame is only
// consumed off the wire once its payload is fully delivered. Spec:
// spec/design/remote.md (MuxFrameDecoder) / spec/design/data_io.md, Sink
// contract section.
TEST(MuxFrameDecoder, DeliverDataResumesFromPartialOffsetAcrossPumps)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    const uint8_t stream_id = 5;
    uint8_t ring_buf[8];  // smaller than the payload below on purpose
    auto* stream_src = dec.createStream(stream_id, ring_buf, sizeof(ring_buf));
    ASSERT_NE(stream_src, nullptr);

    uint8_t payload[20];
    for (size_t i = 0; i < sizeof(payload); ++i) {
        payload[i] = static_cast<uint8_t>(0x50 + i);
    }
    uint8_t wire[frame::kMaxFrameSize];
    auto encoded = frame::encodeData({wire, sizeof(wire)}, stream_id, {payload, sizeof(payload)});
    ASSERT_TRUE(encoded.has_value());

    data::MemorySource wire_src{wire, encoded.value()};

    std::vector<uint8_t> received;
    size_t pumps = 0;
    while (received.size() < sizeof(payload)) {
        pumpValue(dec, wire_src);
        ++pumps;
        ASSERT_LT(pumps, 20u) << "no progress -- deliverData looped without draining the ring";

        // Drain whatever made it into the ring so the next pump() (if any)
        // has room again, exactly as a real consumer would between polls.
        for (;;) {
            auto peeked = stream_src->peek(64);
            if (!peeked.has_value() || peeked.value().size == 0) {
                break;
            }
            received.insert(received.end(), peeked.value().data, peeked.value().data + peeked.value().size);
            (void)stream_src->advance(peeked.value().size);
        }
    }

    // 8 + 8 + 4 bytes across three ring-full/drain rounds.
    EXPECT_EQ(pumps, 3u);
    ASSERT_EQ(received.size(), sizeof(payload));
    EXPECT_EQ(::memcmp(received.data(), payload, sizeof(payload)), 0);

    // The frame is only consumed once fully delivered: nothing is left to
    // decode afterward, and no leftover/duplicated bytes are still queued.
    EXPECT_EQ(pumpValue(dec, wire_src), 0u);
    auto trailing = stream_src->peek(64);
    ASSERT_TRUE(trailing.has_value());
    EXPECT_EQ(trailing.value().size, 0u);

    dec.releaseAll();
}

// A direct sink whose commit() accepts only a bounded prefix and reports it
// via partialCommitAccepted() -- the StreamSink-over-a-congested-transport
// shape. deliverData() must count that accepted prefix as delivered so the
// retried pump() resumes after it instead of recommitting it (the direct-sink
// twin of the ring backpressure regression above).
class PartialCommitSink : public data::Sink {
public:
    explicit PartialCommitSink(size_t max_accept) : _max_accept{max_accept}
    {
    }
    m5::hal::v2::result_t<data::DataSpan> reserve(size_t max_len) override
    {
        return data::DataSpan{_scratch, max_len < sizeof(_scratch) ? max_len : sizeof(_scratch)};
    }
    m5::hal::v2::result_t<void> commit(size_t N) override
    {
        _last_accepted = N < _max_accept ? N : _max_accept;
        _bytes.insert(_bytes.end(), _scratch, _scratch + _last_accepted);
        if (_last_accepted != N) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
        return {};
    }
    bool closed() const override
    {
        return false;
    }
    size_t partialCommitAccepted() const override
    {
        return _last_accepted;
    }
    const std::vector<uint8_t>& bytes() const
    {
        return _bytes;
    }

private:
    uint8_t _scratch[64];
    size_t _max_accept;
    size_t _last_accepted = 0;
    std::vector<uint8_t> _bytes;
};

TEST(MuxFrameDecoder, DeliverDataCountsPartiallyAcceptedCommitPrefix)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    const uint8_t stream_id = 3;
    PartialCommitSink sink{5};  // accepts at most 5 bytes per commit, then errors
    ASSERT_TRUE(dec.setSink(stream_id, sink));

    uint8_t payload[16];
    for (size_t i = 0; i < sizeof(payload); ++i) {
        payload[i] = static_cast<uint8_t>(0xA0 + i);
    }
    uint8_t wire[frame::kMaxFrameSize];
    auto encoded = frame::encodeData({wire, sizeof(wire)}, stream_id, {payload, sizeof(payload)});
    ASSERT_TRUE(encoded.has_value());
    data::MemorySource wire_src{wire, encoded.value()};

    size_t pumps = 0;
    while (sink.bytes().size() < sizeof(payload)) {
        pumpValue(dec, wire_src);
        ++pumps;
        ASSERT_LT(pumps, 20u) << "no progress -- accepted prefix not counted as delivered";
    }

    // 5 + 5 + 5 + 1: three partially accepted commits, then a full one.
    EXPECT_EQ(pumps, 4u);
    ASSERT_EQ(sink.bytes().size(), sizeof(payload));
    EXPECT_EQ(::memcmp(sink.bytes().data(), payload, sizeof(payload)), 0);

    // Fully delivered -> the frame is consumed, nothing is redelivered.
    EXPECT_EQ(pumpValue(dec, wire_src), 0u);
    EXPECT_EQ(sink.bytes().size(), sizeof(payload));

    dec.releaseAll();
}

TEST(MuxFrameDecoder, MultipleStreams)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    auto* s0 = dec.createStream(0, 512);
    auto* s1 = dec.createStream(1, 512);
    ASSERT_NE(s0, nullptr);
    ASSERT_NE(s1, nullptr);

    uint8_t wire[frame::kMaxFrameSize * 2];
    size_t used        = 0;
    const uint8_t d0[] = {0x10, 0x11};
    const uint8_t d1[] = {0x20, 0x21, 0x22};
    used += frame::encodeData({wire, sizeof(wire)}, 0, {d0, sizeof(d0)}).value();
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 1, {d1, sizeof(d1)}).value();

    data::MemorySource wire_src{wire, used};
    size_t count = pumpValue(dec, wire_src);
    EXPECT_EQ(count, 2u);

    auto p0 = s0->peek(256);
    ASSERT_EQ(p0.value().size, sizeof(d0));
    EXPECT_EQ(::memcmp(p0.value().data, d0, sizeof(d0)), 0);

    auto p1 = s1->peek(256);
    ASSERT_EQ(p1.value().size, sizeof(d1));
    EXPECT_EQ(::memcmp(p1.value().data, d1, sizeof(d1)), 0);

    dec.releaseAll();
}

TEST(MuxFrameDecoder, UnknownStreamIdIsDropped)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    dec.createStream(0, 256);

    const uint8_t payload[] = {0xFF};
    uint8_t wire[frame::kMaxFrameSize];
    auto encoded = frame::encodeData({wire, sizeof(wire)}, 5, {payload, sizeof(payload)});
    ASSERT_TRUE(encoded.has_value());

    data::MemorySource wire_src{wire, encoded.value()};
    size_t count = pumpValue(dec, wire_src);
    EXPECT_EQ(count, 1u);

    auto* s0    = dec.source(0);
    auto peeked = s0->peek(256);
    EXPECT_EQ(peeked.value().size, 0u);

    dec.releaseAll();
}

TEST(MuxFrameDecoder, SkipsPaddingAndDelimiter)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    auto* src = dec.createStream(1, 256);
    ASSERT_NE(src, nullptr);

    uint8_t wire[64];
    size_t used             = 0;
    wire[used++]            = 0x00;  // padding
    wire[used++]            = 0x00;  // delimiter prefix
    wire[used++]            = 0x55;  // delimiter
    const uint8_t payload[] = {0x42};
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 1, {payload, sizeof(payload)}).value();

    data::MemorySource wire_src{wire, used};
    size_t count = pumpValue(dec, wire_src);
    EXPECT_EQ(count, 1u);

    auto peeked = src->peek(256);
    ASSERT_EQ(peeked.value().size, sizeof(payload));
    EXPECT_EQ(peeked.value().data[0], 0x42);

    dec.releaseAll();
}

TEST(MuxFrameDecoder, FrameHandlerCalledForNonData)
{
    auto& alloc = memory::defaultAllocator();
    data::MuxFrameDecoder dec{alloc};

    struct Ctx {
        int calls        = 0;
        frame::Kind kind = frame::Kind::Padding;
        uint8_t b3       = 0;
    } ctx;

    dec.setFrameHandler(
        [](void* c, const frame::View& v) {
            auto* p = static_cast<Ctx*>(c);
            p->calls++;
            p->kind = v.kind;
            p->b3   = v.b3;
        },
        &ctx);

    uint8_t wire[frame::kMaxFrameSize];
    auto encoded = frame::encodeChecked({wire, sizeof(wire)}, frame::Kind::Ping, 0x07, {});
    ASSERT_TRUE(encoded.has_value());

    data::MemorySource wire_src{wire, encoded.value()};
    size_t count = pumpValue(dec, wire_src);
    EXPECT_EQ(count, 1u);
    EXPECT_EQ(ctx.calls, 1);
    EXPECT_EQ(ctx.kind, frame::Kind::Ping);
    EXPECT_EQ(ctx.b3, 0x07);
}

TEST(MuxFrameDecoder, BlockStreamUsesTempBlocksUntilConsumed)
{
    memory::Allocator alloc;
    data::MuxFrameDecoder dec{alloc};

    auto* src = dec.createBlockStream(2);
    ASSERT_NE(src, nullptr);
    EXPECT_EQ(dec.source(2), src);

    uint8_t wire[frame::kMaxFrameSize * 3];
    size_t used        = 0;
    const uint8_t d0[] = {0x10, 0x11};
    const uint8_t d1[] = {0x20, 0x21, 0x22};
    const uint8_t d2[] = {0x30, 0x31, 0x32, 0x33};
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 2, {d0, sizeof(d0)}).value();
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 2, {d1, sizeof(d1)}).value();
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 2, {d2, sizeof(d2)}).value();

    const size_t before = alloc.usedBlocks();
    data::MemorySource wire_src{wire, used};
    EXPECT_EQ(pumpValue(dec, wire_src), 3u);
    EXPECT_EQ(alloc.usedBlocks(), before + 3);

    auto p = src->peek(256);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(p.value().size, sizeof(d0));
    EXPECT_EQ(::memcmp(p.value().data, d0, sizeof(d0)), 0);
    ASSERT_TRUE(src->advance(p.value().size).has_value());
    EXPECT_EQ(alloc.usedBlocks(), before + 2);

    p = src->peek(256);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(p.value().size, sizeof(d1));
    EXPECT_EQ(::memcmp(p.value().data, d1, sizeof(d1)), 0);
    ASSERT_TRUE(src->advance(p.value().size).has_value());

    p = src->peek(256);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(p.value().size, sizeof(d2));
    EXPECT_EQ(::memcmp(p.value().data, d2, sizeof(d2)), 0);
    ASSERT_TRUE(src->advance(p.value().size).has_value());
    EXPECT_EQ(alloc.usedBlocks(), before);

    dec.releaseAll();
}

// ============================================================================
// MuxFrameEncoder + MuxFrameDecoder roundtrip
// ============================================================================

TEST(MuxFrameRoundtrip, EncodeDecodeEndToEnd)
{
    auto& alloc = memory::defaultAllocator();

    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};

    const uint8_t raw[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    data::MemorySource input{raw, sizeof(raw)};
    ASSERT_TRUE(enc.attach(0, input));
    uint8_t stream_id = 0;

    auto* rx_src = dec.createStream(stream_id, 512);
    ASSERT_NE(rx_src, nullptr);

    pumpValue(enc);
    size_t decoded = pumpValue(dec, enc.output());
    EXPECT_EQ(decoded, 1u);

    auto peeked = rx_src->peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, sizeof(raw));
    EXPECT_EQ(::memcmp(peeked.value().data, raw, sizeof(raw)), 0);

    enc.releaseAll();
    dec.releaseAll();
}

TEST(MuxFrameRoundtrip, LargeDataMultiFrame)
{
    auto& alloc = memory::defaultAllocator();

    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};

    std::array<uint8_t, 700> big{};
    for (size_t i = 0; i < big.size(); ++i) {
        big[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource input{big.data(), big.size()};
    ASSERT_TRUE(enc.attach(0, input));
    uint8_t stream_id = 0;

    auto* rx_src = dec.createStream(stream_id, 1024);
    ASSERT_NE(rx_src, nullptr);

    while (!input.eof()) {
        pumpValue(enc);
        pumpValue(dec, enc.output());
    }

    size_t verified = 0;
    while (verified < big.size()) {
        auto peeked = rx_src->peek(1024);
        ASSERT_TRUE(peeked.has_value());
        if (peeked.value().size == 0) {
            break;
        }
        for (size_t i = 0; i < peeked.value().size; ++i) {
            EXPECT_EQ(peeked.value().data[i], static_cast<uint8_t>((verified + i) & 0xFF))
                << "at byte " << (verified + i);
        }
        verified += peeked.value().size;
        ASSERT_TRUE(rx_src->advance(peeked.value().size).has_value());
    }
    EXPECT_EQ(verified, big.size());

    enc.releaseAll();
    dec.releaseAll();
}

TEST(MuxFrameRoundtrip, ControlFrameEndToEnd)
{
    auto& alloc = memory::defaultAllocator();

    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};

    struct Ctx {
        int calls        = 0;
        frame::Kind kind = frame::Kind::Padding;
        uint8_t b3       = 0;
        size_t payload   = 0;
    } ctx;

    dec.setFrameHandler(
        [](void* c, const frame::View& v) {
            auto* p = static_cast<Ctx*>(c);
            p->calls++;
            p->kind    = v.kind;
            p->b3      = v.b3;
            p->payload = v.payload.size;
        },
        &ctx);

    const uint8_t body[] = {0xDE, 0xAD};
    ASSERT_TRUE(enc.writeFrame(frame::Kind::Request, 0x03, {body, sizeof(body)}));

    pumpValue(dec, enc.output());
    EXPECT_EQ(ctx.calls, 1);
    EXPECT_EQ(ctx.kind, frame::Kind::Request);
    EXPECT_EQ(ctx.b3, 0x03);
    EXPECT_EQ(ctx.payload, sizeof(body));

    enc.releaseAll();
    dec.releaseAll();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
