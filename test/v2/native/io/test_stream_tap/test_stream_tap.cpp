// SPDX-License-Identifier: MIT
// Native gtest for TapReader / TapWriter (hal/v2/data/tap.hpp).
//
// Mechanically verifies the mirror contract on top of scripted fake
// streams: only bytes that actually flowed through the primary path are
// replicated (never a timeout or an error), replication happens at
// record granularity (one primary call = at most one mirror call), the
// mirror's own return value/errors never affect the primary result, and
// mirror == nullptr is a pure pass-through. Spec: spec/design/data_io.md
// §Tap 装飾.

#include <gtest/gtest.h>
#include <M5HAL_v2.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

using ::m5::hal::v2::result_t;

namespace {

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::data::StreamReader;
using m5::hal::v2::data::StreamWriter;
using m5::hal::v2::data::TapReader;
using m5::hal::v2::data::TapWriter;
using error_t = m5::hal::v2::error::error_t;

// Scripted pull stream: `feed` makes bytes "arrive"; `read` consumes what
// is available and returns 0 when nothing is pending (the way a real
// transport reports a timeout). An armed error is returned once. Mirrors
// the fake in test_stream_adapter.cpp.
class FakeStreamReader : public StreamReader {
public:
    void feed(const uint8_t* bytes, size_t len)
    {
        _pending.insert(_pending.end(), bytes, bytes + len);
    }
    void armError(error_t err)
    {
        _armed_error = err;
        _has_error   = true;
    }

    result_t<size_t> read(DataSpan dst) override
    {
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

private:
    std::vector<uint8_t> _pending;
    error_t _armed_error = error_t::UNKNOWN_ERROR;
    bool _has_error      = false;
};

// Scripted push stream: records everything accepted; `accept_limit` caps
// a single write to simulate a short write (write timeout).
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

// Records every mirror write call as one entry (call-granularity), so
// tests can assert "one primary call = one mirror call" rather than just
// total byte content. Can optionally fail or short-write to prove the
// primary path ignores the mirror's result.
class MirrorRecorder : public StreamWriter {
public:
    result_t<size_t> write(ConstDataSpan src) override
    {
        calls.emplace_back(src.data, src.data + src.size);
        if (_has_error) {
            _has_error = false;
            return m5::stl::make_unexpected(_armed_error);
        }
        const size_t n = std::min(src.size, accept_limit);
        return n;
    }
    void armError(error_t err)
    {
        _armed_error = err;
        _has_error   = true;
    }

    size_t accept_limit = static_cast<size_t>(-1);
    std::vector<std::vector<uint8_t>> calls;

private:
    error_t _armed_error = error_t::UNKNOWN_ERROR;
    bool _has_error      = false;
};

// ============================================================================
// TapReader
// ============================================================================

TEST(TapReader, MirrorsSuccessfulReadsAtRecordGranularity)
{
    FakeStreamReader inner;
    MirrorRecorder mirror;
    TapReader tap{inner, &mirror};
    uint8_t buf[8];

    const uint8_t first[] = {0x10, 0x11, 0x12};
    inner.feed(first, sizeof first);
    auto r1 = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1.value(), 3u);

    const uint8_t second[] = {0x20, 0x21};
    inner.feed(second, sizeof second);
    auto r2 = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r2.value(), 2u);

    // One mirror call per read call, each carrying exactly the bytes
    // that read stored (not a running concatenation).
    ASSERT_EQ(mirror.calls.size(), 2u);
    EXPECT_EQ(mirror.calls[0], std::vector<uint8_t>(first, first + sizeof first));
    EXPECT_EQ(mirror.calls[1], std::vector<uint8_t>(second, second + sizeof second));
}

TEST(TapReader, TimeoutIsNotMirrored)
{
    FakeStreamReader inner;
    MirrorRecorder mirror;
    TapReader tap{inner, &mirror};
    uint8_t buf[8];

    // Nothing fed: read returns 0 (timeout), never an error.
    auto r = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0u);
    EXPECT_TRUE(mirror.calls.empty());
}

TEST(TapReader, ErrorPropagatesAndIsNotMirrored)
{
    FakeStreamReader inner;
    inner.armError(error_t::IO_ERROR);
    MirrorRecorder mirror;
    TapReader tap{inner, &mirror};
    uint8_t buf[8];

    auto r = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::IO_ERROR);
    EXPECT_TRUE(mirror.calls.empty());
}

TEST(TapReader, MirrorErrorDoesNotAffectPrimaryResult)
{
    FakeStreamReader inner;
    const uint8_t payload[] = {0x30, 0x31};
    inner.feed(payload, sizeof payload);
    MirrorRecorder mirror;
    mirror.armError(error_t::TIMEOUT_ERROR);
    TapReader tap{inner, &mirror};
    uint8_t buf[8];

    auto r = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 2u);
    EXPECT_EQ(buf[0], 0x30);
    EXPECT_EQ(buf[1], 0x31);
    // The mirror was still invoked once (best-effort delivery attempted),
    // its failure just doesn't leak back into the primary path.
    ASSERT_EQ(mirror.calls.size(), 1u);
}

TEST(TapReader, NullMirrorIsPurePassThrough)
{
    FakeStreamReader inner;
    const uint8_t payload[] = {0x40, 0x41, 0x42};
    inner.feed(payload, sizeof payload);
    TapReader tap{inner, nullptr};
    uint8_t buf[8];

    auto r = tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);
    EXPECT_EQ(buf[0], 0x40);
}

TEST(TapReader, ReadableBytesPassesThrough)
{
    FakeStreamReader inner;
    const uint8_t payload[] = {1, 2, 3, 4, 5};
    inner.feed(payload, sizeof payload);
    MirrorRecorder mirror;
    TapReader tap{inner, &mirror};

    auto n = tap.readableBytes();
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(n.value(), 5u);
    // readableBytes is a pure pass-through: it never touches the mirror.
    EXPECT_TRUE(mirror.calls.empty());
}

TEST(TapReader, NestedTapsBothMirrorTheSameBytes)
{
    FakeStreamReader inner;
    const uint8_t payload[] = {0x50, 0x51, 0x52};
    inner.feed(payload, sizeof payload);

    MirrorRecorder outer_mirror;
    MirrorRecorder inner_mirror;
    TapReader inner_tap{inner, &inner_mirror};
    TapReader outer_tap{inner_tap, &outer_mirror};

    uint8_t buf[8];
    auto r = outer_tap.read(DataSpan{buf, sizeof buf});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);

    const std::vector<uint8_t> expected(payload, payload + sizeof payload);
    ASSERT_EQ(inner_mirror.calls.size(), 1u);
    EXPECT_EQ(inner_mirror.calls[0], expected);
    ASSERT_EQ(outer_mirror.calls.size(), 1u);
    EXPECT_EQ(outer_mirror.calls[0], expected);
}

// ============================================================================
// TapWriter
// ============================================================================

TEST(TapWriter, MirrorsAcceptedBytesAtRecordGranularity)
{
    FakeStreamWriter inner;
    MirrorRecorder mirror;
    TapWriter tap{inner, &mirror};

    const uint8_t payload[] = {0x60, 0x61, 0x62};
    auto r                  = tap.write(ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);

    ASSERT_EQ(mirror.calls.size(), 1u);
    EXPECT_EQ(mirror.calls[0], std::vector<uint8_t>(payload, payload + sizeof payload));
}

TEST(TapWriter, ShortWriteMirrorsOnlyTheAcceptedPrefix)
{
    FakeStreamWriter inner;
    inner.accept_limit = 2;
    MirrorRecorder mirror;
    TapWriter tap{inner, &mirror};

    const uint8_t payload[] = {0x70, 0x71, 0x72, 0x73};
    auto r                  = tap.write(ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 2u);

    ASSERT_EQ(mirror.calls.size(), 1u);
    EXPECT_EQ(mirror.calls[0], std::vector<uint8_t>(payload, payload + 2));
}

TEST(TapWriter, ErrorPropagatesAndIsNotMirrored)
{
    FakeStreamWriter inner;
    inner.armError(error_t::TIMEOUT_ERROR);
    MirrorRecorder mirror;
    TapWriter tap{inner, &mirror};

    const uint8_t payload[] = {0x80, 0x81};
    auto r                  = tap.write(ConstDataSpan{payload, sizeof payload});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::TIMEOUT_ERROR);
    EXPECT_TRUE(mirror.calls.empty());
}

TEST(TapWriter, MirrorShortWriteDoesNotAffectPrimaryResult)
{
    FakeStreamWriter inner;
    MirrorRecorder mirror;
    mirror.accept_limit = 1;  // mirror "accepts" fewer bytes than given
    TapWriter tap{inner, &mirror};

    const uint8_t payload[] = {0x90, 0x91, 0x92};
    auto r                  = tap.write(ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    // The primary result reflects what `inner` accepted, not the mirror.
    EXPECT_EQ(r.value(), 3u);
    ASSERT_EQ(inner.written.size(), 3u);
}

TEST(TapWriter, NullMirrorIsPurePassThrough)
{
    FakeStreamWriter inner;
    TapWriter tap{inner, nullptr};

    const uint8_t payload[] = {0xA0, 0xA1};
    auto r                  = tap.write(ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 2u);
    ASSERT_EQ(inner.written.size(), 2u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
