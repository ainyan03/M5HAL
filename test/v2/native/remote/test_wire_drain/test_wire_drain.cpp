// SPDX-License-Identifier: MIT
// Native gtest for m5::hal::v2::remote::detail::drainToSink()
// (hal/v2/remote/wire_drain.hpp).
//
// This is the helper RemoteSession::drainTx(), RemoteServerAdapter::drainTx()
// and RemoteWireService::drainTx() all delegate to. Regression for a
// three-way inconsistency: two of the three implementations advanced the
// upstream Source by 0 bytes on a commit() failure (re-sending an
// already-flushed prefix on the next attempt when the sink's commit() itself
// performs a partial/short write before failing), while the third advanced
// by the full attempted amount regardless of commit()'s result (silently
// dropping unsent bytes). The unified contract: advance the Source only by
// the bytes the Sink actually accepted (Sink::partialCommitAccepted() on a
// failing commit()), so a resumed drain neither re-sends nor drops anything.
// Spec: spec/design/data_io.md, Sink contract section.
//
// This drives the shared drain helper directly against a scripted `Sink`
// double rather than standing up a full RemoteSession/RemoteServerAdapter
// pair: the helper's contract is defined purely in terms of the abstract
// Source/Sink interfaces, and StreamSink's own short-write behavior is
// already covered by test_stream_adapter.cpp.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/remote/wire_drain.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

namespace {

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::data::MemorySource;
using m5::hal::v2::data::Sink;
using error_t = m5::hal::v2::error::error_t;
using m5::hal::v2::remote::detail::DrainStop;

// A Sink whose commit() never accepts more than `max_accept` bytes in a
// single call, mimicking a transport write() that always performs a short
// write (e.g. a congested TCP socket or a rate-limited UART). Accepted
// bytes are recorded in `captured` in the order they were committed so a
// test can check the reassembled byte stream for duplication or gaps.
class ShortWriteSink : public Sink {
public:
    explicit ShortWriteSink(size_t max_accept) : _max_accept{max_accept}
    {
    }

    m5::hal::v2::result_t<DataSpan> reserve(size_t max_len) override
    {
        size_t n = std::min(max_len, sizeof(_scratch));
        return DataSpan{_scratch, n};
    }

    m5::hal::v2::result_t<void> commit(size_t N) override
    {
        const size_t accept = std::min(N, _max_accept);
        captured.insert(captured.end(), _scratch, _scratch + accept);
        _last_accepted = accept;
        if (accept != N) {
            return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
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

    std::vector<uint8_t> captured;

private:
    uint8_t _scratch[256];
    size_t _max_accept;
    size_t _last_accepted = 0;
};

// A Sink that never has room: reserve() always reports end-of-writes.
// Models a wire completely backed up (e.g. a full ring buffer downstream).
class NoRoomSink : public Sink {
public:
    m5::hal::v2::result_t<DataSpan> reserve(size_t) override
    {
        return DataSpan{};
    }
    m5::hal::v2::result_t<void> commit(size_t) override
    {
        ADD_FAILURE() << "commit() must not be called when reserve() offered no room";
        return {};
    }
    bool closed() const override
    {
        return false;
    }
};

class ScriptedSource : public m5::hal::v2::data::Source {
public:
    explicit ScriptedSource(std::vector<uint8_t> bytes = {}) : _bytes{std::move(bytes)}
    {
    }

    m5::hal::v2::result_t<ConstDataSpan> peek(size_t max_len) override
    {
        if (peek_error != error_t::OK) {
            return m5::stl::make_unexpected(peek_error);
        }
        if (force_empty || _cursor >= _bytes.size()) {
            return ConstDataSpan{};
        }
        return ConstDataSpan{_bytes.data() + _cursor, std::min(max_len, _bytes.size() - _cursor)};
    }

    m5::hal::v2::result_t<void> advance(size_t n) override
    {
        ++advance_calls;
        if (advance_error != error_t::OK) {
            return m5::stl::make_unexpected(advance_error);
        }
        _cursor += std::min(n, _bytes.size() - _cursor);
        return {};
    }

    bool eof() const override
    {
        return closed_state && _cursor >= _bytes.size();
    }

    bool closed() const override
    {
        return closed_state;
    }

    size_t cursor() const
    {
        return _cursor;
    }

    error_t peek_error    = error_t::OK;
    error_t advance_error = error_t::OK;
    bool force_empty      = false;
    bool closed_state     = true;
    size_t advance_calls  = 0;

private:
    std::vector<uint8_t> _bytes;
    size_t _cursor = 0;
};

class ScriptedSink : public Sink {
public:
    m5::hal::v2::result_t<DataSpan> reserve(size_t max_len) override
    {
        if (reserve_error != error_t::OK) {
            return m5::stl::make_unexpected(reserve_error);
        }
        if (no_room) {
            return DataSpan{};
        }
        return DataSpan{scratch, std::min(max_len, sizeof(scratch))};
    }

    m5::hal::v2::result_t<void> commit(size_t n) override
    {
        ++commit_calls;
        attempted = n;
        if (commit_error != error_t::OK) {
            return m5::stl::make_unexpected(commit_error);
        }
        return {};
    }

    bool closed() const override
    {
        return closed_state;
    }

    size_t partialCommitAccepted() const override
    {
        return accepted_on_error;
    }

    uint8_t scratch[32]{};
    error_t reserve_error    = error_t::OK;
    error_t commit_error     = error_t::OK;
    size_t accepted_on_error = 0;
    size_t attempted         = 0;
    size_t commit_calls      = 0;
    bool no_room             = false;
    bool closed_state        = false;
};

std::vector<uint8_t> countingBytes(size_t n, uint8_t start = 0)
{
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) {
        v[i] = static_cast<uint8_t>(start + i);
    }
    return v;
}

TEST(WireDrain, FullyAcceptingSinkDrainsEverythingInOneCall)
{
    auto payload = countingBytes(37, 0x10);
    MemorySource src{payload.data(), payload.size()};
    ShortWriteSink sink{/*max_accept=*/1024};  // never short

    auto drained = m5::hal::v2::remote::detail::drainToSink(src, sink);

    ASSERT_TRUE(drained);
    EXPECT_EQ(drained->stop, DrainStop::Drained);
    EXPECT_EQ(drained->accepted_bytes, payload.size());
    EXPECT_TRUE(src.eof());
    ASSERT_EQ(sink.captured.size(), payload.size());
    EXPECT_EQ(std::memcmp(sink.captured.data(), payload.data(), payload.size()), 0);
}

// The core regression check: a sink that only ever accepts a few bytes per
// commit() forces many drainToSink() calls ("pumps") to fully drain the
// source. Across all of them the reassembled stream must equal the
// original payload exactly -- no repeated prefix (the old break-without-
// advance bug) and no skipped bytes (the old advance-regardless-of-commit
// bug).
TEST(WireDrain, ShortWriteSinkAcrossMultiplePumpsHasNoDuplicateNoLoss)
{
    auto payload = countingBytes(37, 0x40);
    MemorySource src{payload.data(), payload.size()};
    ShortWriteSink sink{/*max_accept=*/5};

    size_t pumps = 0;
    while (!src.eof()) {
        auto drained = m5::hal::v2::remote::detail::drainToSink(src, sink);
        if (!src.eof()) {
            ASSERT_TRUE(drained);
            EXPECT_EQ(drained->stop, DrainStop::WouldBlock);
            EXPECT_EQ(drained->accepted_bytes, 5u);
        } else {
            ASSERT_TRUE(drained);
        }
        ++pumps;
        ASSERT_LT(pumps, 100u) << "no progress -- drainToSink looped without draining the source";
    }

    // ceil(37 / 5) == 8 short commits, one per pump (each pump's loop stops
    // as soon as a commit() falls short).
    EXPECT_EQ(pumps, 8u);
    ASSERT_EQ(sink.captured.size(), payload.size());
    EXPECT_EQ(std::memcmp(sink.captured.data(), payload.data(), payload.size()), 0);
}

// reserve() offering zero room must not advance the source or touch
// commit() at all -- there is nothing to advance by yet, and retrying
// later (once room frees up) must resume from the very first byte.
TEST(WireDrain, NoRoomSinkMakesNoProgressAndNeverCallsCommit)
{
    auto payload = countingBytes(10, 0x80);
    MemorySource src{payload.data(), payload.size()};
    NoRoomSink sink;

    auto drained = m5::hal::v2::remote::detail::drainToSink(src, sink);

    ASSERT_TRUE(drained);
    EXPECT_EQ(drained->stop, DrainStop::WouldBlock);
    EXPECT_EQ(drained->accepted_bytes, 0u);
    EXPECT_FALSE(src.eof());
    auto peeked = src.peek(payload.size());
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, payload.size());
    EXPECT_EQ(std::memcmp(peeked.value().data, payload.data(), payload.size()), 0);
}

TEST(WireDrain, PropagatesPeekAndReserveErrorsExactly)
{
    ScriptedSource peek_fails{countingBytes(4)};
    ScriptedSink sink;
    peek_fails.peek_error = error_t::IO_ERROR;
    auto result           = m5::hal::v2::remote::detail::drainToSink(peek_fails, sink);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::IO_ERROR);

    ScriptedSource reserve_source{countingBytes(4)};
    sink.reserve_error = error_t::CLOSED;
    result             = m5::hal::v2::remote::detail::drainToSink(reserve_source, sink);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::CLOSED);
    EXPECT_EQ(reserve_source.cursor(), 0u);
}

TEST(WireDrain, EmptySourceDistinguishesWouldBlockFromDrained)
{
    ScriptedSink sink;
    ScriptedSource open;
    open.closed_state = false;
    open.force_empty  = true;
    auto result       = m5::hal::v2::remote::detail::drainToSink(open, sink);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->stop, DrainStop::WouldBlock);

    ScriptedSource closed;
    result = m5::hal::v2::remote::detail::drainToSink(closed, sink);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->stop, DrainStop::Drained);
}

TEST(WireDrain, EmptySinkDistinguishesBackpressureFromClosure)
{
    ScriptedSource source{countingBytes(4)};
    ScriptedSink sink;
    sink.no_room = true;
    auto result  = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->stop, DrainStop::WouldBlock);

    sink.closed_state = true;
    result            = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::CLOSED);
}

TEST(WireDrain, CommitErrorAdvancesOnlyAcceptedPrefixAndPropagatesError)
{
    ScriptedSource source{countingBytes(10)};
    ScriptedSink sink;
    sink.commit_error      = error_t::IO_ERROR;
    sink.accepted_on_error = 3;
    auto result            = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::IO_ERROR);
    EXPECT_EQ(source.cursor(), 3u);
    EXPECT_EQ(source.advance_calls, 1u);
}

TEST(WireDrain, TimeoutCommitIsRetryableBackpressure)
{
    ScriptedSource source{countingBytes(10)};
    ScriptedSink sink;
    sink.commit_error      = error_t::TIMEOUT_ERROR;
    sink.accepted_on_error = 3;
    auto result            = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->stop, DrainStop::WouldBlock);
    EXPECT_EQ(result->accepted_bytes, 3u);
    EXPECT_EQ(source.cursor(), 3u);
}

TEST(WireDrain, AcceptedPrefixIsClampedToAttemptedSize)
{
    ScriptedSource source{countingBytes(4)};
    ScriptedSink sink;
    sink.commit_error      = error_t::TIMEOUT_ERROR;
    sink.accepted_on_error = 100;
    auto result            = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->stop, DrainStop::WouldBlock);
    EXPECT_EQ(result->accepted_bytes, 4u);
    EXPECT_EQ(source.cursor(), 4u);
}

TEST(WireDrain, AdvanceErrorTakesPrecedenceAfterCommit)
{
    ScriptedSource source{countingBytes(4)};
    source.advance_error = error_t::PROTOCOL_ERROR;
    ScriptedSink sink;
    sink.commit_error      = error_t::IO_ERROR;
    sink.accepted_on_error = 2;
    auto result            = m5::hal::v2::remote::detail::drainToSink(source, sink);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::PROTOCOL_ERROR);
    EXPECT_EQ(source.cursor(), 0u);
}

TEST(WireDrain, ZeroChunkLimitIsInvalidArgument)
{
    ScriptedSource source{countingBytes(1)};
    ScriptedSink sink;
    auto result = m5::hal::v2::remote::detail::drainToSink(source, sink, 0);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), error_t::INVALID_ARGUMENT);
    EXPECT_EQ(sink.commit_calls, 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
