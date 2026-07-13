// SPDX-License-Identifier: MIT
// Native gtest for StdioSource::advance (data/stdio.hpp).
//
// Regression for a clamp-target bug: advance(N) clamped the skip count
// to the total buffered byte count (_used) instead of the unread
// remainder (_used - _cursor). Repeated advance calls could then push
// _cursor past _used, and the next peek()'s compact() computed
// `_used - _cursor` as an unsigned underflow and fed the (huge) result
// to memmove() -- an immediate crash. Spec: spec/design/data_io.md.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/data/stdio.hpp>

#include <cstdint>
#include <cstdio>

namespace {

using m5::hal::v2::data::StdioSource;

// A FILE* backed by tmpfile() (auto-removed on close) seeded with bytes
// and rewound so StdioSource reads from the start.
class TmpFile {
public:
    TmpFile() : _f(std::tmpfile())
    {
    }
    ~TmpFile()
    {
        if (_f != nullptr) {
            std::fclose(_f);
        }
    }
    FILE* file() const
    {
        return _f;
    }
    void seed(const uint8_t* data, size_t len)
    {
        std::fwrite(data, 1, len, _f);
        std::fflush(_f);
        std::rewind(_f);
    }

private:
    FILE* _f;
};

TEST(StdioSource, AdvancePastUnreadRemainderDoesNotOverrunCursor)
{
    TmpFile tmp;
    ASSERT_NE(tmp.file(), nullptr);
    const uint8_t bytes[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    tmp.seed(bytes, sizeof bytes);

    StdioSource src{tmp.file()};

    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, sizeof bytes);

    // Consume every buffered byte...
    ASSERT_TRUE(src.advance(sizeof bytes).has_value());
    // ...then advance again past what remains (0 bytes unread). Pre-fix
    // this clamped to `_used` (10) instead of the unread remainder (0),
    // pushing the cursor to 11 -- past `_used`.
    ASSERT_TRUE(src.advance(1).has_value());

    // The next peek() runs compact() first. Pre-fix, `_used - _cursor`
    // underflowed to SIZE_MAX and fed memmove() a huge length; reaching
    // this point without crashing is the regression check.
    auto peeked2 = src.peek(64);
    ASSERT_TRUE(peeked2.has_value());
    // No bytes were skipped past the file's actual content, so the
    // compacted buffer is simply empty (file already at EOF).
    EXPECT_EQ(peeked2.value().size, 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
