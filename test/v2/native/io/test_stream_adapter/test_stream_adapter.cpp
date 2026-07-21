// SPDX-License-Identifier: MIT
// Native gtest for StreamSource / StreamSink (hal/v2/data/stream.hpp).
//
// Mechanically verifies the adapter contract on top of scripted fake
// streams: `peek` is idempotent and monotonically non-decreasing, an
// idle timeout with no bytes surfaces as an empty successful span,
// `advance` past the buffered bytes becomes a skip reservation that
// later arrivals consume automatically, `reserve` lends a stable
// scratch span, `commit` passes bytes through the writer and reports
// short writes as TIMEOUT_ERROR. Spec: spec/design/data_io.md §Stream
// adapters.

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

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::data::StreamReader;
using m5::hal::v2::data::StreamSink;
using m5::hal::v2::data::StreamSource;
using m5::hal::v2::data::StreamWriter;
using error_t = m5::hal::v2::error::error_t;

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
            _has_error     = false;
            const size_t n = std::min(src.size, accepted_before_error);
            written.insert(written.end(), src.data, src.data + n);
            _last_accepted = n;
            return m5::stl::make_unexpected(_armed_error);
        }
        _last_accepted = 0;
        const size_t n = std::min(src.size, accept_limit);
        written.insert(written.end(), src.data, src.data + n);
        return n;
    }
    size_t partialWriteAccepted() const override
    {
        return _last_accepted;
    }
    void armError(error_t err)
    {
        _armed_error = err;
        _has_error   = true;
    }

    size_t accept_limit          = static_cast<size_t>(-1);
    size_t accepted_before_error = 0;
    std::vector<uint8_t> written;

private:
    error_t _armed_error  = error_t::UNKNOWN_ERROR;
    bool _has_error       = false;
    size_t _last_accepted = 0;
};

class OverreportingReader : public StreamReader {
public:
    result_t<size_t> read(DataSpan dst) override
    {
        return overreport ? dst.size + 1u : 0u;
    }
    result_t<size_t> readableBytes(void) override
    {
        return 1u;
    }

    bool overreport = true;
};

class OverreportingWriter : public StreamWriter {
public:
    enum class Mode { Success, PartialError };

    explicit OverreportingWriter(Mode mode) : _mode{mode}
    {
    }

    result_t<size_t> write(ConstDataSpan src) override
    {
        _offered = src.size;
        if (!overreport) {
            return src.size;
        }
        if (_mode == Mode::PartialError) {
            return m5::stl::make_unexpected(error_t::CLOSED);
        }
        return src.size + 1u;
    }
    size_t partialWriteAccepted() const override
    {
        return _offered + 1u;
    }

    bool overreport = true;

private:
    Mode _mode;
    size_t _offered = 0;
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

TEST(StreamSource, TimeoutIsEmptyProgressNotEof)
{
    FakeStreamReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    auto peeked = src.peek(4);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 0u);
    EXPECT_FALSE(src.eof());
    EXPECT_FALSE(src.closed());

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
    ASSERT_TRUE(starved.has_value());
    EXPECT_EQ(starved->size, 0u);
    EXPECT_FALSE(src.eof());

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

#if !defined(NDEBUG)
TEST(StreamSourceDeathTest, ReaderOverreportAsserts)
{
    OverreportingReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    EXPECT_DEATH({ (void)src.peek(4); }, "more bytes than requested");
}
#else
TEST(StreamSource, ReaderOverreportReturnsIoErrorAndFaultsAdapter)
{
    OverreportingReader reader;
    uint8_t scratch[8];
    StreamSource src{reader, DataSpan{scratch, sizeof scratch}};

    auto first = src.peek(4);
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error_t::IO_ERROR);
    EXPECT_EQ(src.buffered(), 0u);

    reader.overreport = false;
    auto retry        = src.peek(4);
    ASSERT_FALSE(retry.has_value());
    EXPECT_EQ(retry.error(), error_t::IO_ERROR);
}
#endif

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
    // The accepted prefix (what actually reached the writer before the
    // short write) must be readable back so a caller relaying bytes from
    // an upstream Source can advance by exactly that much instead of
    // resending it or dropping the remainder.
    EXPECT_EQ(snk.partialCommitAccepted(), 2u);
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
    // A hard writer error (as opposed to a short write) accepts nothing.
    EXPECT_EQ(snk.partialCommitAccepted(), 0u);
}

TEST(StreamSink, WriterHardErrorPreservesAcceptedPrefix)
{
    FakeStreamWriter writer;
    writer.accepted_before_error = 3;
    writer.armError(error_t::CLOSED);
    uint8_t scratch[8];
    StreamSink snk{writer, DataSpan{scratch, sizeof scratch}};

    auto reserved = snk.reserve(5);
    ASSERT_TRUE(reserved.has_value());
    std::memset(reserved->data, 0x5A, 5);
    auto committed = snk.commit(5);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::CLOSED);
    EXPECT_EQ(snk.partialCommitAccepted(), 3u);
    EXPECT_EQ(writer.written.size(), 3u);
}

#if !defined(NDEBUG)
TEST(StreamSinkDeathTest, WriterOverreportAsserts)
{
    OverreportingWriter writer{OverreportingWriter::Mode::Success};
    uint8_t scratch[8];
    StreamSink sink{writer, DataSpan{scratch, sizeof scratch}};

    EXPECT_DEATH({ (void)sink.commit(4); }, "more bytes than offered");
}

TEST(StreamSinkDeathTest, PartialWriteOverreportAsserts)
{
    OverreportingWriter writer{OverreportingWriter::Mode::PartialError};
    uint8_t scratch[8];
    StreamSink sink{writer, DataSpan{scratch, sizeof scratch}};

    EXPECT_DEATH({ (void)sink.commit(4); }, "partial count exceeds");
}
#else
TEST(StreamSink, WriterOverreportReturnsIoErrorAndFaultsAdapter)
{
    OverreportingWriter writer{OverreportingWriter::Mode::Success};
    uint8_t scratch[8];
    StreamSink sink{writer, DataSpan{scratch, sizeof scratch}};

    auto first = sink.commit(4);
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error_t::IO_ERROR);
    EXPECT_EQ(sink.partialCommitAccepted(), 0u);

    writer.overreport = false;
    auto retry        = sink.commit(4);
    ASSERT_FALSE(retry.has_value());
    EXPECT_EQ(retry.error(), error_t::IO_ERROR);
}

TEST(StreamSink, PartialWriteOverreportReturnsIoErrorAndFaultsAdapter)
{
    OverreportingWriter writer{OverreportingWriter::Mode::PartialError};
    uint8_t scratch[8];
    StreamSink sink{writer, DataSpan{scratch, sizeof scratch}};

    auto first = sink.commit(4);
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error_t::IO_ERROR);
    EXPECT_EQ(sink.partialCommitAccepted(), 0u);

    writer.overreport = false;
    auto retry        = sink.reserve(4);
    ASSERT_FALSE(retry.has_value());
    EXPECT_EQ(retry.error(), error_t::IO_ERROR);
}
#endif

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
