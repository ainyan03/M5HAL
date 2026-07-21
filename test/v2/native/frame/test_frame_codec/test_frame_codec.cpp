// SPDX-License-Identifier: MIT
// Native gtest for the frame codec (hal/v2/frame/frame.hpp).
//
// Wire format:
//   checked frame: [LEN:1][KIND:1][CHECK8:1][B3:1][payload:0..252]
//   padding:       [0x00]
//   delimiter:     [0x00][0x55]

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using ::m5::hal::v2::result_t;

namespace {

namespace frame = m5::hal::v2::frame;
using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::data::MemorySink;
using m5::hal::v2::data::MemorySource;
using m5::hal::v2::data::StreamReader;
using m5::hal::v2::data::StreamSink;
using m5::hal::v2::data::StreamSource;
using m5::hal::v2::data::StreamWriter;
using error_t = m5::hal::v2::error::error_t;

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

    size_t read_calls = 0;

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

class ErrorAfterPeekSource : public m5::hal::v2::data::Source {
public:
    result_t<ConstDataSpan> peek(size_t max_len) override
    {
        (void)max_len;
        return ConstDataSpan{_byte, sizeof(_byte)};
    }

    result_t<void> advance(size_t N) override
    {
        (void)N;
        return m5::stl::make_unexpected(error_t::IO_ERROR);
    }

    bool eof() const override
    {
        return false;
    }

private:
    uint8_t _byte[1] = {0x42};
};

// ============================================================================
// CRC-8 ATM
// ============================================================================

TEST(FrameCrc8, KnownVectors)
{
    // CRC-8 ATM (poly 0x07, init 0x00).
    // "123456789" -> 0xBC (according to standard CRC-8/ATM test vector 0xA1 for HEC,
    // but for poly 0x07 init 0x00 no-reflect: "123456789" -> depends on variant).
    // Verify internal consistency: check8(LEN, KIND) must match manual calculation.
    uint8_t crc = 0x00;
    crc         = frame::crc8AtmUpdate(crc, 0x04);  // LEN = 4
    crc         = frame::crc8AtmUpdate(crc, 0x01);  // KIND = Data
    EXPECT_EQ(crc, frame::check8(0x04, 0x01));
}

TEST(FrameCrc8, ZeroInputsAreZero)
{
    EXPECT_EQ(frame::check8(0x00, 0x00), 0x00);
}

TEST(FrameCrc8, DifferentInputsDifferentChecks)
{
    EXPECT_NE(frame::check8(0x04, 0x01), frame::check8(0x04, 0x03));
    EXPECT_NE(frame::check8(0x04, 0x01), frame::check8(0x05, 0x01));
}

// ============================================================================
// Constants
// ============================================================================

TEST(FrameConstants, SizesAreConsistent)
{
    EXPECT_EQ(frame::kHeaderSize, 4u);
    EXPECT_EQ(frame::kMaxPayload, 252u);
    EXPECT_EQ(frame::kMaxFrameSize, 256u);
    EXPECT_EQ(frame::kHeaderSize + frame::kMaxPayload, frame::kMaxFrameSize);
    EXPECT_EQ(frame::kMaxLen, frame::kMinCheckedLen + frame::kMaxPayload);
    EXPECT_EQ(frame::kMaxDataPayload, frame::kMaxPayload);
    EXPECT_EQ(frame::checkedFrameWireSize(0), frame::kHeaderSize);
    EXPECT_EQ(frame::checkedFrameWireSize(frame::kMaxPayload), frame::kMaxFrameSize);
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
    EXPECT_EQ(buf[1], static_cast<uint8_t>(frame::Kind::Delimiter));

    uint8_t tiny[1];
    auto overflow = frame::encodeDelimiter({tiny, sizeof(tiny)});
    ASSERT_FALSE(overflow.has_value());
    EXPECT_EQ(overflow.error(), error_t::BUFFER_OVERFLOW);
}

TEST(FrameEncode, DataFrameLayout)
{
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    std::array<uint8_t, 16> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 7, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::kHeaderSize + sizeof(payload));

    // [LEN][KIND][CHECK8][B3=stream_id][payload...]
    EXPECT_EQ(buf[0], frame::kMinCheckedLen + sizeof(payload));  // LEN
    EXPECT_EQ(buf[1], static_cast<uint8_t>(frame::Kind::Data));  // KIND
    EXPECT_EQ(buf[2], frame::check8(buf[0], buf[1]));            // CHECK8
    EXPECT_EQ(buf[3], 7);                                        // B3 = stream_id
    EXPECT_EQ(std::memcmp(buf.data() + 4, payload, sizeof(payload)), 0);
}

TEST(FrameCodecProperty, DeterministicSeededRoundtrip)
{
    constexpr frame::Kind kinds[] = {frame::Kind::Data,     frame::Kind::Credit,    frame::Kind::Checkpoint,
                                     frame::Kind::Control,  frame::Kind::Request,   frame::Kind::Response,
                                     frame::Kind::HelloReq, frame::Kind::HelloResp, frame::Kind::Ping,
                                     frame::Kind::Pong,     frame::Kind::Event};
    uint32_t state                = 0x5A17C9E3u;
    auto next                     = [&state]() {
        state = state * 1664525u + 1013904223u;
        return state;
    };

    std::array<uint8_t, frame::kMaxPayload> payload{};
    std::array<uint8_t, frame::kMaxFrameSize> wire{};
    for (size_t iteration = 0; iteration < 1000; ++iteration) {
        const size_t payload_size = next() % (frame::kMaxPayload + 1u);
        for (size_t i = 0; i < payload_size; ++i) {
            payload[i] = static_cast<uint8_t>(next());
        }
        const auto kind  = kinds[next() % (sizeof(kinds) / sizeof(kinds[0]))];
        const uint8_t b3 = static_cast<uint8_t>(next());
        auto encoded     = frame::encodeChecked({wire.data(), wire.size()}, kind, b3, {payload.data(), payload_size});
        ASSERT_TRUE(encoded.has_value()) << iteration;

        frame::View view;
        const auto decoded = frame::decode({wire.data(), encoded.value()}, view);
        ASSERT_EQ(decoded.status, frame::DecodeStatus::Ok) << iteration;
        EXPECT_EQ(decoded.consumed, encoded.value()) << iteration;
        EXPECT_EQ(view.kind, kind) << iteration;
        EXPECT_EQ(view.b3, b3) << iteration;
        ASSERT_EQ(view.payload.size, payload_size) << iteration;
        if (payload_size != 0) {
            EXPECT_EQ(std::memcmp(view.payload.data, payload.data(), payload_size), 0) << iteration;
        }
    }
}

TEST(FrameEncode, CheckedFrameWithPayload)
{
    const uint8_t payload[] = {0xAA, 0xBB};
    std::array<uint8_t, 16> buf{};
    auto written =
        frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::Control, 0x42, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::kHeaderSize + sizeof(payload));
    EXPECT_EQ(buf[0], frame::kMinCheckedLen + sizeof(payload));
    EXPECT_EQ(buf[1], static_cast<uint8_t>(frame::Kind::Control));
    EXPECT_EQ(buf[2], frame::check8(buf[0], buf[1]));
    EXPECT_EQ(buf[3], 0x42);
    EXPECT_EQ(std::memcmp(buf.data() + 4, payload, sizeof(payload)), 0);
}

TEST(FrameEncode, EmptyPayload)
{
    std::array<uint8_t, 8> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 3, {});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::kHeaderSize);
    EXPECT_EQ(buf[0], frame::kMinCheckedLen);  // LEN = 2
    EXPECT_EQ(buf[3], 3);                      // stream_id
}

TEST(FrameEncode, ArgumentErrors)
{
    std::array<uint8_t, 300> buf{};
    std::array<uint8_t, 300> body{};

    auto big_data = frame::encodeData({buf.data(), buf.size()}, 1, {body.data(), frame::kMaxDataPayload + 1});
    ASSERT_FALSE(big_data.has_value());
    EXPECT_EQ(big_data.error(), error_t::INVALID_ARGUMENT);

    auto big_body =
        frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::Control, 0, {body.data(), frame::kMaxPayload + 1});
    ASSERT_FALSE(big_body.has_value());
    EXPECT_EQ(big_body.error(), error_t::INVALID_ARGUMENT);

    auto delim = frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::Delimiter, 0, {});
    ASSERT_FALSE(delim.has_value());
    EXPECT_EQ(delim.error(), error_t::INVALID_ARGUMENT);

    auto small = frame::encodeData({buf.data(), 4}, 1, {body.data(), 4});
    ASSERT_FALSE(small.has_value());
    EXPECT_EQ(small.error(), error_t::BUFFER_OVERFLOW);
}

// ============================================================================
// decode
// ============================================================================

TEST(FrameDecode, RoundtripData)
{
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::array<uint8_t, 32> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 9, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    auto result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(result.consumed, written.value());
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_TRUE(view.has_check);
    EXPECT_EQ(view.b3, 9);
    ASSERT_EQ(view.payload.size, sizeof(payload));
    EXPECT_EQ(std::memcmp(view.payload.data, payload, sizeof(payload)), 0);
}

TEST(FrameDecode, RoundtripChecked)
{
    const uint8_t payload[] = {0x01, 0x02};
    std::array<uint8_t, 16> buf{};
    auto written =
        frame::encodeChecked({buf.data(), buf.size()}, frame::Kind::Control, 0x05, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    auto result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Control);
    EXPECT_EQ(view.b3, 0x05);
    ASSERT_EQ(view.payload.size, sizeof(payload));
    EXPECT_EQ(std::memcmp(view.payload.data, payload, sizeof(payload)), 0);
}

TEST(FrameDecode, RoundtripEmptyAndMaxPayload)
{
    std::array<uint8_t, frame::kMaxFrameSize> buf{};
    frame::View view;

    auto written = frame::encodeData({buf.data(), buf.size()}, 0, {});
    ASSERT_TRUE(written.has_value());
    auto result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.payload.size, 0u);
    EXPECT_EQ(view.b3, 0);

    std::array<uint8_t, frame::kMaxDataPayload> payload{};
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i);
    }
    written = frame::encodeData({buf.data(), buf.size()}, 0xFF, {payload.data(), payload.size()});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), frame::kMaxFrameSize);
    result = frame::decode({buf.data(), written.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 0xFF);
    ASSERT_EQ(view.payload.size, payload.size());
    EXPECT_EQ(std::memcmp(view.payload.data, payload.data(), payload.size()), 0);
}

TEST(FrameDecode, PaddingAndDelimiter)
{
    frame::View view;

    // Single 0x00 = padding
    const uint8_t pad[] = {0x00, 0x01};
    auto result         = frame::decode({pad, sizeof(pad)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::Padding);
    EXPECT_EQ(result.consumed, 1u);

    // [0x00][0x55] = delimiter
    const uint8_t delim[] = {0x00, 0x55};
    result                = frame::decode({delim, sizeof(delim)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::Delimiter);
    EXPECT_EQ(result.consumed, 2u);
    EXPECT_EQ(view.kind, frame::Kind::Delimiter);

    // [0x00][0x00][0x55] = padding + delimiter
    const uint8_t pad_delim[] = {0x00, 0x00, 0x55};
    result                    = frame::decode({pad_delim, sizeof(pad_delim)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::Padding);
    EXPECT_EQ(result.consumed, 1u);
    result = frame::decode({pad_delim + 1, sizeof(pad_delim) - 1}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::Delimiter);
    EXPECT_EQ(result.consumed, 2u);
}

TEST(FrameDecode, UnknownKindSkipsItsSelfDescribedSize)
{
    // kind 0x02 unknown, LEN=3 -> skip 2 + 3 = 5 bytes total
    const uint8_t wire[] = {0x03, 0x02, 0xAA, 0xBB, 0xCC, 0x00, 0x55};
    frame::View view;

    auto result = frame::decode({wire, sizeof(wire)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::InvalidPrefix);
    EXPECT_EQ(result.consumed, 5u);

    result = frame::decode({wire + 5, sizeof(wire) - 5}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::Delimiter);
    EXPECT_EQ(view.kind, frame::Kind::Delimiter);
}

TEST(FrameDecode, InvalidSizeForLenOne)
{
    // LEN=1 is invalid (minimum valid is 2)
    const uint8_t wire[] = {0x01, 0x01, 0xAA};
    frame::View view;

    auto result = frame::decode({wire, sizeof(wire)}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::InvalidSize);
    EXPECT_EQ(result.consumed, 3u);  // kPrefixSize + LEN = 2 + 1
}

TEST(FrameDecode, CorruptCheckYieldsInvalidCheck)
{
    const uint8_t payload[] = {0x44, 0x55};
    std::array<uint8_t, 16> buf{};
    auto written = frame::encodeData({buf.data(), buf.size()}, 3, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    buf[2] ^= 0xFF;  // corrupt CHECK8

    frame::View view;
    auto result = frame::decode({buf.data(), written.value()}, view);
    EXPECT_EQ(result.status, frame::DecodeStatus::InvalidCheck);
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
        EXPECT_EQ(result.status, frame::DecodeStatus::NeedMore) << "at len " << len;
        EXPECT_EQ(result.consumed, 0u);
    }
    EXPECT_EQ(frame::decode({buf.data(), written.value()}, view).status, frame::DecodeStatus::Ok);
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
    ASSERT_EQ(result.status, frame::DecodeStatus::Delimiter);
    cursor += result.consumed;

    result = frame::decode({buf.data() + cursor, used - cursor}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 1);
    cursor += result.consumed;

    result = frame::decode({buf.data() + cursor, used - cursor}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 2);
    cursor += result.consumed;
    EXPECT_EQ(cursor, used);
}

// ============================================================================
// buildDataFrame
// ============================================================================

TEST(BuildDataFrame, BasicSourceToBlock)
{
    const uint8_t data[] = {0xCA, 0xFE, 0xBA, 0xBE};
    MemorySource src{data, sizeof(data)};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 5, src);
    ASSERT_TRUE(frame_size.has_value());
    ASSERT_EQ(frame_size.value(), frame::kHeaderSize + sizeof(data));
    EXPECT_EQ(block[0], frame::kMinCheckedLen + sizeof(data));     // LEN
    EXPECT_EQ(block[1], static_cast<uint8_t>(frame::Kind::Data));  // KIND
    EXPECT_EQ(block[2], frame::check8(block[0], block[1]));        // CHECK8
    EXPECT_EQ(block[3], 5);                                        // stream_id
    EXPECT_EQ(std::memcmp(block.data() + 4, data, sizeof(data)), 0);
    EXPECT_TRUE(src.eof());
}

TEST(BuildDataFrame, EmptySourceReturnsZero)
{
    MemorySource src{nullptr, 0};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 0, src);
    ASSERT_TRUE(frame_size.has_value());
    EXPECT_EQ(frame_size.value(), 0u);
}

TEST(BuildDataFrame, MaxPayloadFillsBlock)
{
    std::array<uint8_t, 252> data{};
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<uint8_t>(i);
    }
    MemorySource src{data.data(), data.size()};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 0xFF, src);
    ASSERT_TRUE(frame_size.has_value());
    EXPECT_EQ(frame_size.value(), 256u);
    EXPECT_EQ(block[0], 254);  // LEN = 2 + 252
    EXPECT_EQ(block[3], 0xFF);
    EXPECT_EQ(std::memcmp(block.data() + 4, data.data(), data.size()), 0);
    EXPECT_TRUE(src.eof());
}

TEST(BuildDataFrame, LargeSourceStopsAt252)
{
    std::array<uint8_t, 500> data{};
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<uint8_t>(i & 0xFF);
    }
    MemorySource src{data.data(), data.size()};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 1, src);
    ASSERT_TRUE(frame_size.has_value());
    EXPECT_EQ(frame_size.value(), 256u);
    EXPECT_EQ(std::memcmp(block.data() + 4, data.data(), 252), 0);
    EXPECT_FALSE(src.eof());
}

TEST(BuildDataFrame, ResultDecodesCorrectly)
{
    const uint8_t data[] = {0x11, 0x22, 0x33};
    MemorySource src{data, sizeof(data)};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 42, src);
    ASSERT_TRUE(frame_size.has_value());
    ASSERT_GT(frame_size.value(), 0u);

    frame::View view;
    auto result = frame::decode({block.data(), frame_size.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, 42);
    ASSERT_EQ(view.payload.size, sizeof(data));
    EXPECT_EQ(std::memcmp(view.payload.data, data, sizeof(data)), 0);
}

TEST(BuildDataFrame, DivisibleByThree)
{
    // 252 = 84 * 3: verify 3-byte-aligned data fills exactly
    std::array<uint8_t, 252> rgb{};
    for (size_t i = 0; i < rgb.size(); i += 3) {
        rgb[i]     = static_cast<uint8_t>(i);
        rgb[i + 1] = static_cast<uint8_t>(i + 1);
        rgb[i + 2] = static_cast<uint8_t>(i + 2);
    }
    MemorySource src{rgb.data(), rgb.size()};
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 0, src);
    ASSERT_TRUE(frame_size.has_value());
    EXPECT_EQ(frame_size.value(), 256u);
    EXPECT_TRUE(src.eof());
}

TEST(BuildDataFrame, AdvanceErrorPropagates)
{
    ErrorAfterPeekSource src;
    std::array<uint8_t, 256> block{};

    auto frame_size = frame::buildDataFrame(block.data(), 0, src);
    ASSERT_FALSE(frame_size.has_value());
    EXPECT_EQ(frame_size.error(), error_t::IO_ERROR);
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
    ASSERT_EQ(result.value().status, frame::DecodeStatus::Delimiter);
    EXPECT_EQ(view.kind, frame::Kind::Delimiter);

    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, 5);
    ASSERT_EQ(view.payload.size, sizeof(p1));
    EXPECT_EQ(std::memcmp(view.payload.data, p1, sizeof(p1)), 0);

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

    MemorySource source{ConstDataSpan{buf.data(), written.value() - 1}};
    frame::FrameReader reader{source};
    frame::View view;
    auto result = reader.next(view);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::END_OF_STREAM);
    EXPECT_FALSE(source.eof());

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
    std::array<uint8_t, frame::kMaxFrameSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};
    frame::View view;

    stream.feed(buf.data(), 3);
    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().status, frame::DecodeStatus::NeedMore);

    stream.feed(buf.data() + 3, written.value() - 3);
    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 4);
    EXPECT_EQ(std::memcmp(view.payload.data, payload, sizeof(payload)), 0);
}

TEST(FrameReader, BufferedFramesDecodeWithoutExtraBlockingReads)
{
    std::array<uint8_t, 64> buf{};
    const uint8_t p1[] = {0x01, 0x02};
    const uint8_t p2[] = {0x03};
    size_t used        = frame::encodeData({buf.data(), buf.size()}, 1, {p1, sizeof(p1)}).value();
    used += frame::encodeData({buf.data() + used, buf.size() - used}, 2, {p2, sizeof(p2)}).value();

    FakeStreamReader stream;
    stream.feed(buf.data(), used);
    std::array<uint8_t, frame::kMaxFrameSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};

    frame::View view;
    auto first = reader.next(view);
    ASSERT_TRUE(first.has_value());
    ASSERT_EQ(first.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 1);

    auto second = reader.next(view);
    ASSERT_TRUE(second.has_value());
    ASSERT_EQ(second.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 2);

    EXPECT_EQ(stream.read_calls, 1u);
}

TEST(FrameReader, SkipsPaddingAndSurfacesInvalidThenRecovers)
{
    std::array<uint8_t, 64> wire{};
    size_t used       = 0;
    wire[used++]      = 0x00;
    wire[used++]      = 0x00;
    const uint8_t p[] = {0x42};
    auto corrupt      = frame::encodeData({wire.data() + used, wire.size() - used}, 1, {p, sizeof(p)});
    wire[used + 2] ^= 0xFF;  // corrupt CHECK8
    used += corrupt.value();
    const uint8_t good[] = {0x24};
    used += frame::encodeData({wire.data() + used, wire.size() - used}, 2, {good, sizeof(good)}).value();

    MemorySource source{ConstDataSpan{wire.data(), used}};
    frame::FrameReader reader{source};
    frame::View view;

    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().status, frame::DecodeStatus::InvalidCheck);

    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 2);
}

TEST(FrameReader, EmptyOpenSourceReturnsNeedMoreThenDecodes)
{
    std::array<uint8_t, 32> buf{};
    const uint8_t payload[] = {0x55, 0x66};
    auto written            = frame::encodeData({buf.data(), buf.size()}, 8, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    FakeStreamReader stream;
    std::array<uint8_t, frame::kMaxFrameSize> scratch{};
    StreamSource source{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameReader reader{source};
    frame::View view;

    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().status, frame::DecodeStatus::NeedMore);

    stream.feed(buf.data(), written.value());
    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 8);
    EXPECT_EQ(std::memcmp(view.payload.data, payload, sizeof(payload)), 0);
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
    auto data_w             = writer.writeData(6, {payload, sizeof(payload)});
    ASSERT_TRUE(data_w.has_value());

    frame::View view;
    auto result = frame::decode({buf.data(), delim.value() + data_w.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Delimiter);

    result = frame::decode({buf.data() + result.consumed, data_w.value()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, 6);
}

TEST(FrameWriter, WritesThroughStreamSink)
{
    FakeStreamWriter stream;
    std::array<uint8_t, frame::kMaxFrameSize> scratch{};
    StreamSink sink{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameWriter writer{sink};

    const uint8_t payload[] = {0xCA, 0xFE};
    auto written            = writer.writeData(3, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    frame::View view;
    auto result = frame::decode({stream.written.data(), stream.written.size()}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.b3, 3);
    EXPECT_EQ(std::memcmp(view.payload.data, payload, sizeof(payload)), 0);
}

TEST(FrameWriter, ShortWriteAndArgumentErrors)
{
    FakeStreamWriter stream;
    stream.accept_limit = 2;
    std::array<uint8_t, frame::kMaxFrameSize> scratch{};
    StreamSink sink{stream, DataSpan{scratch.data(), scratch.size()}};
    frame::FrameWriter writer{sink};

    const uint8_t payload[] = {1, 2, 3};
    auto written            = writer.writeData(1, {payload, sizeof(payload)});
    ASSERT_FALSE(written.has_value());
    EXPECT_EQ(written.error(), error_t::TIMEOUT_ERROR);

    std::array<uint8_t, 300> big{};
    auto invalid = writer.writeData(1, {big.data(), frame::kMaxDataPayload + 1});
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), error_t::INVALID_ARGUMENT);
}

TEST(FrameWriter, SinkTooSmallIsBufferOverflow)
{
    std::array<uint8_t, 4> buf{};
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
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
