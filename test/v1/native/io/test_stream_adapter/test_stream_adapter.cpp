// SPDX-License-Identifier: MIT
// Native gtest for StreamSource / StreamSink (hal/v1/data/stream.hpp).
//
// Mechanically verifies the adapter contract on top of scripted fake
// streams: `peek` is idempotent and monotonically non-decreasing, a
// timeout surfaces as TIMEOUT_ERROR (never as an empty span = EOF),
// `advance` past the buffered bytes becomes a skip reservation that
// later arrivals consume automatically, `reserve` lends a stable
// scratch span, `commit` passes bytes through the writer and reports
// short writes as IO_ERROR. Spec: spec/design/data_io.md §Stream
// adapters.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using ::m5::hal::v1::result_t;

namespace {

using m5::hal::v1::data::ConstDataSpan;
using m5::hal::v1::data::DataSpan;
using m5::hal::v1::data::StreamReader;
using m5::hal::v1::data::StreamSink;
using m5::hal::v1::data::StreamSource;
using m5::hal::v1::data::StreamWriter;
using error_t = m5::hal::v1::error::error_t;

// Scripted pull stream: `feed` makes bytes "arrive"; `read` consumes
// what is available and returns 0 when nothing is pending (the way a
// real transport reports a timeout). An armed error is returned once.
class FakeStreamReader : public StreamReader {
public:
    void feed(const uint8_t* bytes, size_t len)
    {
        _pending.insert(_pending.end(), bytes, bytes + len);
    }
    void feedCounting(size_t len, uint8_t start = 0)
    {
        for (size_t i = 0; i < len; ++i) {
            _pending.push_back(static_cast<uint8_t>(start + i));
        }
    }
    void armError(error_t err)
    {
        _armed_error = err;
        _has_error   = true;
    }

    result_t<size_t> read(DataSpan dst) override
    {
        ++read_calls;
        if (_has_error) {
            _has_error = false;
            return m5::stl::make_unexpected(_armed_error);
        }
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
    error_t _armed_error = error_t::UNKNOWN_ERROR;
    bool _has_error      = false;
};

// Scripted push stream: records everything written; `accept_limit`
// caps a single write to simulate a short write (write timeout).
class FakeStreamWriter : public StreamWriter {
public:
    result_t<size_t> write(ConstDataSpan src) override
    {
        if (_has_error) {
            _has_error = false;
            return m5::stl::make_unexpected(_armed_error);
        }
        const size_t n = std::min(src.size, accept_limit);
        written.insert(written.end(), src.data, src.data + n);
        return n;
    }
    void armError(error_t err)
    {
        _armed_error = err;
        _has_error   = true;
    }

    size_t accept_limit = static_cast<size_t>(-1);
    std::vector<uint8_t> written;

private:
    error_t _armed_error = error_t::UNKNOWN_ERROR;
    bool _has_error      = false;
};

// ============================================================================
// StreamSource
// ============================================================================

TEST(StreamSource, DetachedIsEofAndPeeksEmpty)
{
    uint8_t scratch[8];
    StreamSource src{static_cast<StreamReader*>(nullptr), DataSpan{scratch, sizeof scratch}};
    EXPECT_TRUE(src.eof());

    auto peeked = src.peek(4);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 0u);

    EXPECT_TRUE(src.advance(4).has_value());
}

TEST(StreamSource, EmptyScratchIsInvalidArgument)
{
    FakeStreamReader reader;
    StreamSource src{reader, DataSpan{}};

    auto peeked = src.peek(4);
    ASSERT_FALSE(peeked.has_value());
    EXPECT_EQ(peeked.error(), error_t::INVALID_ARGUMENT);
}

TEST(StreamSource, TimeoutIsAnErrorNotEof)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    auto peeked = src.peek(4);
    ASSERT_FALSE(peeked.has_value());
    EXPECT_EQ(peeked.error(), error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(src.eof());

    // Recoverable: data arriving later makes the next peek succeed.
    reader.feedCounting(3, 0x10);
    auto retry = src.peek(4);
    ASSERT_TRUE(retry.has_value());
    EXPECT_EQ(retry->size, 3u);
    EXPECT_EQ(retry->data[0], 0x10);
}

TEST(StreamSource, PeekIsIdempotentAndMonotonic)
{
    FakeStreamReader reader;
    uint8_t scratch[16];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    reader.feedCounting(3, 0x20);
    auto first = src.peek(8);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->size, 3u);

    // Same request again without advance: identical bytes, no shrink.
    auto again = src.peek(8);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(again->size, 3u);
    EXPECT_EQ(again->data[0], 0x20);
    EXPECT_EQ(again->data[2], 0x22);

    // More data arrives: the prefix must stay stable, size may grow,
    // and the top-up must not block (buffer already holds bytes).
    reader.feedCounting(5, 0x23);
    auto grown = src.peek(8);
    ASSERT_TRUE(grown.has_value());
    EXPECT_EQ(grown->size, 8u);
    for (size_t i = 0; i < 8; ++i) {
        EXPECT_EQ(grown->data[i], 0x20 + i) << "at " << i;
    }
}

TEST(StreamSource, PeekIsCappedByScratchCapacity)
{
    FakeStreamReader reader;
    uint8_t scratch[4];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    reader.feedCounting(10, 0x30);
    auto peeked = src.peek(10);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 4u);
    EXPECT_EQ(peeked->data[3], 0x33);
}

TEST(StreamSource, AdvanceConsumesBufferedBytes)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    reader.feedCounting(6, 0x40);
    auto peeked = src.peek(3);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 3u);

    ASSERT_TRUE(src.advance(2).has_value());
    auto rest = src.peek(8);
    ASSERT_TRUE(rest.has_value());
    EXPECT_EQ(rest->size, 4u);
    EXPECT_EQ(rest->data[0], 0x42);
    EXPECT_EQ(rest->data[3], 0x45);
}

TEST(StreamSource, AdvancePastBufferBecomesSkipReservation)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    reader.feedCounting(4, 0x00);
    auto peeked = src.peek(8);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 4u);

    // 4 buffered + 6 not yet arrived. The excess becomes a reservation
    // (no caller-side wait loop), consumed once data shows up.
    ASSERT_TRUE(src.advance(10).has_value());
    EXPECT_EQ(src.pendingSkip(), 6u);

    auto starved = src.peek(8);
    ASSERT_FALSE(starved.has_value());
    EXPECT_EQ(starved.error(), error_t::TIMEOUT_ERROR);

    reader.feedCounting(8, 0x50);  // bytes 0x50..0x57; 0x50..0x55 fall into the skip
    auto after = src.peek(8);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(src.pendingSkip(), 0u);
    EXPECT_EQ(after->size, 2u);
    EXPECT_EQ(after->data[0], 0x56);
    EXPECT_EQ(after->data[1], 0x57);
}

TEST(StreamSource, SkipReservationDrainsOpportunisticallyOnAdvance)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    // Nothing buffered: advance(3) reserves 3, nothing readable yet.
    ASSERT_TRUE(src.advance(3).has_value());
    EXPECT_EQ(src.pendingSkip(), 3u);

    // New arrival is consumed by the next advance without blocking.
    reader.feedCounting(5, 0x60);
    ASSERT_TRUE(src.advance(0).has_value());
    EXPECT_EQ(src.pendingSkip(), 0u);

    auto peeked = src.peek(8);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 2u);
    EXPECT_EQ(peeked->data[0], 0x63);
}

TEST(StreamSource, ReaderErrorsPropagate)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    reader.armError(error_t::IO_ERROR);
    auto peeked = src.peek(4);
    ASSERT_FALSE(peeked.has_value());
    EXPECT_EQ(peeked.error(), error_t::IO_ERROR);
    EXPECT_FALSE(src.eof());
}

// ============================================================================
// StreamSink
// ============================================================================

TEST(StreamSink, DetachedIsClosedAndCommitFails)
{
    uint8_t scratch[8];
    StreamSink snk{static_cast<StreamWriter*>(nullptr), DataSpan{scratch, sizeof scratch}};
    EXPECT_TRUE(snk.closed());

    auto reserved = snk.reserve(4);
    ASSERT_TRUE(reserved.has_value());
    EXPECT_EQ(reserved->size, 0u);

    auto committed = snk.commit(4);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::CLOSED);
}

TEST(StreamSink, EmptyScratchIsInvalidArgument)
{
    FakeStreamWriter writer;
    StreamSink snk{writer, DataSpan{}};

    auto reserved = snk.reserve(4);
    ASSERT_FALSE(reserved.has_value());
    EXPECT_EQ(reserved.error(), error_t::INVALID_ARGUMENT);
}

TEST(StreamSink, ReserveIsStableAndCapped)
{
    FakeStreamWriter writer;
    uint8_t scratch[4];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};
    EXPECT_FALSE(snk.closed());

    auto first = snk.reserve(16);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->size, 4u);
    EXPECT_EQ(first->data, scratch);

    auto again = snk.reserve(16);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(again->data, first->data);
    EXPECT_EQ(again->size, first->size);
}

TEST(StreamSink, CommitPassesBytesThrough)
{
    FakeStreamWriter writer;
    uint8_t scratch[8];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};

    auto reserved = snk.reserve(8);
    ASSERT_TRUE(reserved.has_value());
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::memcpy(reserved->data, payload, sizeof payload);
    ASSERT_TRUE(snk.commit(sizeof payload).has_value());

    ASSERT_EQ(writer.written.size(), sizeof payload);
    EXPECT_EQ(std::memcmp(writer.written.data(), payload, sizeof payload), 0);

    // commit(0) is a no-op that must not touch the writer.
    ASSERT_TRUE(snk.commit(0).has_value());
    EXPECT_EQ(writer.written.size(), sizeof payload);
}

TEST(StreamSink, OversizedCommitIsBufferOverflow)
{
    FakeStreamWriter writer;
    uint8_t scratch[4];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};

    auto reserved = snk.reserve(4);
    ASSERT_TRUE(reserved.has_value());
    std::memset(reserved->data, 0xA5, reserved->size);
    auto committed = snk.commit(5);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::BUFFER_OVERFLOW);
    EXPECT_TRUE(writer.written.empty());
}

TEST(StreamSink, ShortWriteIsTimeout)
{
    FakeStreamWriter writer;
    writer.accept_limit = 2;
    uint8_t scratch[8];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};

    auto reserved = snk.reserve(4);
    ASSERT_TRUE(reserved.has_value());
    std::memset(reserved->data, 0xA5, 4);
    auto committed = snk.commit(4);
    ASSERT_FALSE(committed.has_value());
    // Short writes classify as TIMEOUT_ERROR (retryable), symmetric with
    // the read side.
    EXPECT_EQ(committed.error(), error_t::TIMEOUT_ERROR);
}

TEST(StreamSink, WriterErrorsPropagate)
{
    FakeStreamWriter writer;
    writer.armError(error_t::TIMEOUT_ERROR);
    uint8_t scratch[8];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};

    auto reserved = snk.reserve(2);
    ASSERT_TRUE(reserved.has_value());
    auto committed = snk.commit(2);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::TIMEOUT_ERROR);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
