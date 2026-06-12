// SPDX-License-Identifier: MIT
// Native gtest for the frame codec (hal/v1/frame/frame.hpp).
//
// Mechanically verifies the wire codec against spec/design/frame.md:
// CRC16-CCITT vectors, encode/decode roundtrips for every frame shape,
// padding / delimiter semantics, resync after unknown kinds and
// corruption, and the Source/Sink-driven FrameReader / FrameWriter
// (deferred advance, split arrival, error passthrough).

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using ::m5::hal::v1::result_t;

namespace {

namespace frame = m5::hal::v1::frame;
using m5::hal::v1::data::ConstDataSpan;
using m5::hal::v1::data::DataSpan;
using m5::hal::v1::data::MemorySink;
using m5::hal::v1::data::MemorySource;
using m5::hal::v1::data::StreamReader;
using m5::hal::v1::data::StreamSink;
using m5::hal::v1::data::StreamSource;
using m5::hal::v1::data::StreamWriter;
using error_t = m5::hal::v1::error::error_t;

// Same scripted fakes as test_stream_adapter: `feed` makes bytes
// arrive, read consumes what is pending (0 = timed out).
class FakeStreamReader : public StreamReader {
public:
    void feed(const uint8_t* bytes, size_t len)
    {
        _pending.insert(_pending.end(), bytes, bytes + len);
    }

    result_t<size_t> read(DataSpan dst) override
    {
        ++read_calls;
        const size_t n = std::min(dst.size, _pending.size());
        std::memcpy(dst.data, _pending.data(), n);
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<ptrdiff_t>(n));
        return n;
    }
    result_t<size_t> readableBytes(void) override
    {
        return _pending.size();
    }

    size_t read_calls = 0;  // each call models one (possibly timeout-long) blocking read

private:
    std::vector<uint8_t> _pending;
};

class FakeStreamWriter : public StreamWriter {
public:
    result_t<size_t> write(ConstDataSpan src) override
    {
        const size_t n = std::min(src.size, accept_limit);
        written.insert(written.end(), src.data, src.data + n);
        return n;
    }

    size_t accept_limit = static_cast<size_t>(-1);
    std::vector<uint8_t> written;
};

// ============================================================================
// CRC16-CCITT
// ============================================================================

TEST(FrameCrc16, MatchesStandardVector)
{
    // CRC-16/CCITT-FALSE: "123456789" -> 0x29B1.
    uint16_t crc = 0xFFFF;
    for (char c : {'1', '2', '3', '4', '5', '6', '7', '8', '9'}) {
        crc = frame::crc16CcittUpdate(crc, static_cast<uint8_t>(c));
    }
    EXPECT_EQ(crc, 0x29B1);
}

TEST(FrameCrc16, Check16ZeroesTheCheckBytes)
{
    // check16 must give the same result whatever the check field holds.
    uint8_t a[] = {0x05, 0x01, 0x00, 0x00, 0x07, 0x10, 0x20};
    uint8_t b[] = {0x05, 0x01, 0xAB, 0xCD, 0x07, 0x10, 0x20};
    EXPECT_EQ(frame::check16({a, sizeof(a)}), frame::check16({b, sizeof(b)}));
}

// ============================================================================
// encode
// ============================================================================

TEST(FrameEncode, DelimiterShape)
{
    std::array<uint8_t, 4> buf{};
    auto written = frame::encodeDelimiter({buf.data(), buf.size()});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::kPrefixSize);
    EXPECT_EQ(buf[0], 0x00);
    EXPECT_EQ(buf[1], static_cast<uint8_t>(frame::Kind::delimiter));

    uint8_t tiny[1];
    auto overflow = frame::encodeDelimiter({tiny, sizeof(tiny)});
    ASSERT_FALSE(overflow.has_value());
    EXPECT_EQ(overflow.error(), error_t::BUFFER_OVERFLOW);
}

TEST(FrameEncode, DataFrameShapeAndSelfCheck)
{
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    std::array<uint8_t, 16> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 7, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::checkedFrameWireSize(1 + sizeof(payload)));

    EXPECT_EQ(buf[0], frame::kCheckSize + 1 + sizeof(payload));  // SIZE
    EXPECT_EQ(buf[1], static_cast<uint8_t>(frame::Kind::data));  // KIND
    const uint16_t stored = static_cast<uint16_t>((buf[2] << 8) | buf[3]);
    EXPECT_EQ(stored, frame::check16({buf.data(), written.value()}));  // self-verifying
    EXPECT_EQ(buf[4], 7);                                              // stream id
    EXPECT_EQ(std::memcmp(buf.data() + 5, payload, sizeof(payload)), 0);
}

TEST(FrameEncode, ArgumentErrors)
{
    std::array<uint8_t, 300> buf{};
    std::array<uint8_t, 300> body{};

    // Oversized payload / kind body.
    auto big_data = frame::encodeData({buf.data(), buf.size()}, 1, {body.data(), frame::kMaxDataPayload + 1});
    ASSERT_FALSE(big_data.has_value());
    EXPECT_EQ(big_data.error(), error_t::INVALID_ARGUMENT);

    auto big_body =
        frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::control, {body.data(), frame::kMaxBodySize});
    ASSERT_FALSE(big_body.has_value());
    EXPECT_EQ(big_body.error(), error_t::INVALID_ARGUMENT);

    // Unchecked kinds cannot go through encodeChecked.
    auto delim = frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::delimiter, {});
    ASSERT_FALSE(delim.has_value());
    EXPECT_EQ(delim.error(), error_t::INVALID_ARGUMENT);

    // Destination too small.
    auto small = frame::encodeData({buf.data(), 4}, 1, {body.data(), 4});
    ASSERT_FALSE(small.has_value());
    EXPECT_EQ(small.error(), error_t::BUFFER_OVERFLOW);
}

// ============================================================================
// decode
// ============================================================================

TEST(FrameDecode, RoundtripDataAndChecked)
{
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::array<uint8_t, 32> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 9, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    auto result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(result.consumed, written.value());
    EXPECT_EQ(view.kind, frame::Kind::data);
    EXPECT_TRUE(view.has_check);
    ASSERT_EQ(view.kind_body.size, 1 + sizeof(payload));
    EXPECT_EQ(view.kind_body.data[0], 9);
    EXPECT_EQ(std::memcmp(view.kind_body.data + 1, payload, sizeof(payload)), 0);

    // Bare checked kind (no stream id rule): control.
    const uint8_t ctrl[] = {0x01, 0x02};
    written              = frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::control, {ctrl, sizeof(ctrl)});
    ASSERT_TRUE(written.has_value());
    result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::control);
    ASSERT_EQ(view.kind_body.size, sizeof(ctrl));
    EXPECT_EQ(std::memcmp(view.kind_body.data, ctrl, sizeof(ctrl)), 0);
}

TEST(FrameDecode, RoundtripEmptyAndMaxPayload)
{
    std::array<uint8_t, frame::kMaxWireSize> buf{};
    frame::View view;

    auto written = frame::encodeData({buf.data(), buf.size()}, 0, {});
    ASSERT_TRUE(written.has_value());
    auto result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.size, 1u);  // stream id only

    std::array<uint8_t, frame::kMaxDataPayload> payload{};
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i);
    }
    written = frame::encodeData({buf.data(), buf.size()}, 0xFF, {payload.data(), payload.size()});
    ASSERT_TRUE(written.has_value());
    result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    ASSERT_EQ(view.kind_body.size, 1 + payload.size());
    EXPECT_EQ(std::memcmp(view.kind_body.data + 1, payload.data(), payload.size()), 0);
}

TEST(FrameDecode, PaddingConsumesOneByteAtATime)
{
    const uint8_t wire[] = {0x00, 0x00, 0x55};
    frame::View view;

    auto result = frame::decode({wire, sizeof(wire)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::padding);
    EXPECT_EQ(result.consumed, 1u);

    result = frame::decode({wire + 1, sizeof(wire) - 1}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(result.consumed, 2u);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);
    EXPECT_FALSE(view.has_check);
}

TEST(FrameDecode, UnknownKindSkipsItsSelfDescribedSize)
{
    // kind 0x02 unknown, SIZE=3 -> discard 5 bytes, then a delimiter.
    const uint8_t wire[] = {0x03, 0x02, 0xAA, 0xBB, 0xCC, 0x00, 0x55};
    frame::View view;

    auto result = frame::decode({wire, sizeof(wire)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::invalid_prefix);
    EXPECT_EQ(result.consumed, 5u);

    result = frame::decode({wire + result.consumed, sizeof(wire) - result.consumed}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);
}

TEST(FrameDecode, CorruptionYieldsInvalidCheck)
{
    const uint8_t payload[] = {0x44, 0x55};
    std::array<uint8_t, 16> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 3, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    buf[5] ^= 0x01;  // flip one payload bit

    frame::View view;
    auto result = frame::decode({buf.data(), written.value()}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::invalid_check);
    EXPECT_EQ(result.consumed, written.value());
}

TEST(FrameDecode, NeedMoreOnPartialInput)
{
    const uint8_t payload[] = {1, 2, 3, 4, 5};
    std::array<uint8_t, 16> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 1, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    for (size_t len = 0; len < written.value(); ++len) {
        auto result = frame::decode({buf.data(), len}, view);
        EXPECT_EQ(result.status, frame::DecodeStatus::need_more) << "at len " << len;
        EXPECT_EQ(result.consumed, 0u);
    }
    EXPECT_EQ(frame::decode({buf.data(), written.value()}, view).status, frame::DecodeStatus::ok);
}

TEST(FrameDecode, SequentialFramesInOneBuffer)
{
    std::array<uint8_t, 64> buf{};
    size_t used = 0;
    auto delim  = frame::encodeDelimiter({buf.data(), buf.size()});
    used += delim.value();
    const uint8_t p1[] = {0x11};
    used += frame::encodeData({buf.data() + used, buf.size() - used}, 1, {p1, sizeof(p1)}).value();
    const uint8_t p2[] = {0x22, 0x33};
    used += frame::encodeData({buf.data() + used, buf.size() - used}, 2, {p2, sizeof(p2)}).value();

    frame::View view;
    size_t cursor = 0;
    auto result   = frame::decode({buf.data() + cursor, used - cursor}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);
    cursor += result.consumed;

    result = frame::decode({buf.data() + cursor, used - cursor}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 1);
    cursor += result.consumed;

    result = frame::decode({buf.data() + cursor, used - cursor}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 2);
    cursor += result.consumed;
    EXPECT_EQ(cursor, used);
}

// ============================================================================
// FrameReader
// ============================================================================

TEST(FrameReader, ExtractsSequentialFramesFromMemorySource)
{
    std::array<uint8_t, 64> buf{};
    size_t used        = frame::encodeDelimiter({buf.data(), buf.size()}).value();
    const uint8_t p1[] = {0xA1, 0xA2};
    used += frame::encodeData({buf.data() + used, buf.size() - used}, 5, {p1, sizeof(p1)}).value();

    MemorySource source{ConstDataSpan{buf.data(), used}};
    frame::FrameReader reader{source};

    frame::View view;
    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);

    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::data);
    ASSERT_EQ(view.kind_body.size, 1 + sizeof(p1));
    EXPECT_EQ(view.kind_body.data[0], 5);

    // Drained source -> END_OF_STREAM through the error path.
    result = reader.next(view);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::END_OF_STREAM);
}

TEST(FrameReader, TruncatedTailOnClosedSourceEndsInsteadOfLivelocking)
{
    std::array<uint8_t, 32> buf{};
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    auto written            = frame::encodeData({buf.data(), buf.size()}, 7, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    // Cut the last byte off: the prefix announces more than the source
    // can ever deliver. A MemorySource is closed (content fixed at
    // construction), so the reader reports END_OF_STREAM instead of a
    // need_more the caller would retry forever.
    MemorySource source{ConstDataSpan{buf.data(), written.value() - 1}};
    frame::FrameReader reader{source};
    frame::View view;
    auto result = reader.next(view);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::END_OF_STREAM);
    // The partial tail stays unconsumed; eof() == false marks the
    // mid-frame ending for callers that care.
    EXPECT_FALSE(source.eof());

    // A lone byte (shorter than the 2-byte prefix) ends the same way.
    MemorySource lone{ConstDataSpan{buf.data(), 1}};
    frame::FrameReader lone_reader{lone};
    auto lone_result = lone_reader.next(view);
    ASSERT_FALSE(lone_result.has_value());
    EXPECT_EQ(lone_result.error(), error_t::END_OF_STREAM);
}

TEST(FrameReader, HandlesSplitArrivalAcrossPeeks)
{
    std::array<uint8_t, 32> buf{};
    const uint8_t payload[] = {0x77, 0x88, 0x99};
    auto written            = frame::encodeData({buf.data(), buf.size()}, 4, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    FakeStreamReader stream;
    std::array<uint8_t, frame::kMaxWireSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};
    frame::View view;

    // First half arrives: the frame is incomplete.
    stream.feed(buf.data(), 3);
    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().status, frame::DecodeStatus::need_more);

    // Rest arrives: the same call now yields the frame.
    stream.feed(buf.data() + 3, written.value() - 3);
    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 4);
    EXPECT_EQ(std::memcmp(view.kind_body.data + 1, payload, sizeof(payload)), 0);
}

TEST(FrameReader, BufferedFramesDecodeWithoutExtraBlockingReads)
{
    // Latency regression guard: with frames fully buffered, next() must
    // not issue further blocking reads (each read models a potential
    // full-timeout wait on a real stream). The exact-size acquisition
    // (prefix peek -> frame-size peek) keeps already-arrived frames
    // decoding immediately.
    std::array<uint8_t, 64> buf{};
    const uint8_t p1[] = {0x01, 0x02};
    const uint8_t p2[] = {0x03};
    size_t used        = frame::encodeData({buf.data(), buf.size()}, 1, {p1, sizeof(p1)}).value();
    used += frame::encodeData({buf.data() + used, buf.size() - used}, 2, {p2, sizeof(p2)}).value();

    FakeStreamReader stream;
    stream.feed(buf.data(), used);
    std::array<uint8_t, frame::kMaxWireSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};

    frame::View view;
    auto first = reader.next(view);
    ASSERT_TRUE(first.has_value());
    ASSERT_EQ(first.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 1);

    auto second = reader.next(view);
    ASSERT_TRUE(second.has_value());
    ASSERT_EQ(second.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 2);

    // One read filled the buffer; both frames came out of it.
    EXPECT_EQ(stream.read_calls, 1u);
}

TEST(FrameReader, SkipsPaddingAndSurfacesInvalidThenRecovers)
{
    std::array<uint8_t, 64> wire{};
    size_t used = 0;
    // padding, padding, corrupt frame, good frame
    wire[used++]      = 0x00;
    wire[used++]      = 0x00;
    const uint8_t p[] = {0x42};
    auto corrupt      = frame::encodeData({wire.data() + used, wire.size() - used}, 1, {p, sizeof(p)});
    wire[used + 4] ^= 0xFF;  // corrupt the stream id byte
    used += corrupt.value();
    const uint8_t good[] = {0x24};
    size_t good_at       = used;
    used += frame::encodeData({wire.data() + used, wire.size() - used}, 2, {good, sizeof(good)}).value();
    (void)good_at;

    MemorySource source{ConstDataSpan{wire.data(), used}};
    frame::FrameReader reader{source};
    frame::View view;

    // Padding is skipped internally; the corruption surfaces as a status.
    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().status, frame::DecodeStatus::invalid_check);

    // The reader already advanced past the bad frame; next call recovers.
    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 2);
}

TEST(FrameReader, TimeoutPassesThroughAndViewSurvivesUntilNextCall)
{
    std::array<uint8_t, 32> buf{};
    const uint8_t payload[] = {0x55, 0x66};
    auto written            = frame::encodeData({buf.data(), buf.size()}, 8, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    FakeStreamReader stream;
    std::array<uint8_t, frame::kMaxWireSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};
    frame::View view;

    // Nothing arrived: StreamSource reports TIMEOUT_ERROR, passed through.
    auto result = reader.next(view);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::TIMEOUT_ERROR);

    stream.feed(buf.data(), written.value());
    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    // The View stays valid until the next reader call (deferred advance).
    EXPECT_EQ(view.kind_body.data[0], 8);
    EXPECT_EQ(view.kind_body.data[1], 0x55);
}

// ============================================================================
// FrameWriter
// ============================================================================

TEST(FrameWriter, RoundtripThroughMemorySink)
{
    std::array<uint8_t, 64> buf{};
    MemorySink sink{DataSpan{buf.data(), buf.size()}};
    frame::FrameWriter writer{sink};

    auto delim = writer.writeDelimiter();
    ASSERT_TRUE(delim.has_value());
    const uint8_t payload[] = {0x10, 0x32};
    auto data               = writer.writeData(6, {payload, sizeof(payload)});
    ASSERT_TRUE(data.has_value());

    frame::View view;
    auto result = frame::decode({buf.data(), delim.value() + data.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);

    result = frame::decode({buf.data() + result.consumed, data.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::data);
    EXPECT_EQ(view.kind_body.data[0], 6);
}

TEST(FrameWriter, WritesThroughStreamSink)
{
    FakeStreamWriter stream;
    std::array<uint8_t, frame::kMaxWireSize> scratch{};
    StreamSink sink{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameWriter writer{sink};

    const uint8_t payload[] = {0xCA, 0xFE};
    auto written            = writer.writeData(3, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    auto result = frame::decode({stream.written.data(), stream.written.size()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind_body.data[0], 3);
    EXPECT_EQ(std::memcmp(view.kind_body.data + 1, payload, sizeof(payload)), 0);
}

TEST(FrameWriter, ShortWriteAndArgumentErrors)
{
    FakeStreamWriter stream;
    stream.accept_limit = 2;  // force a short write
    std::array<uint8_t, frame::kMaxWireSize> scratch{};
    StreamSink sink{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameWriter writer{sink};

    const uint8_t payload[] = {1, 2, 3};
    auto written            = writer.writeData(1, {payload, sizeof(payload)});
    ASSERT_FALSE(written.has_value());
    EXPECT_EQ(written.error(), error_t::TIMEOUT_ERROR);  // short write = retryable (S16 D10)

    std::array<uint8_t, 300> big{};
    auto invalid = writer.writeData(1, {big.data(), frame::kMaxDataPayload + 1});
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), error_t::INVALID_ARGUMENT);
}

TEST(FrameWriter, SinkTooSmallIsBufferOverflow)
{
    std::array<uint8_t, 4> buf{};  // smaller than the frame
    MemorySink sink{DataSpan{buf.data(), buf.size()}};
    frame::FrameWriter writer{sink};

    const uint8_t payload[] = {1, 2, 3, 4};
    auto written            = writer.writeData(1, {payload, sizeof(payload)});
    ASSERT_FALSE(written.has_value());
    EXPECT_EQ(written.error(), error_t::BUFFER_OVERFLOW);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
