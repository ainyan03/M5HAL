// SPDX-License-Identifier: MIT
// Native gtest for BlockSource (data/block.hpp).

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <array>
#include <cstdint>
#include <cstring>

namespace {

namespace data  = m5::hal::v2::data;
namespace mem   = m5::hal::v2::memory;
namespace frame = m5::hal::v2::frame;

// ============================================================================
// Basic BlockSource
// ============================================================================

TEST(BlockSource, EmptyIsEof)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};
    EXPECT_TRUE(bs.eof());
    EXPECT_EQ(bs.blockCount(), 0u);
    EXPECT_EQ(bs.totalBuffered(), 0u);

    auto peeked = bs.peek(256);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked.value().size, 0u);
}

TEST(BlockSource, DefaultConstructorUsesDefaultAllocator)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    const size_t before   = alloc.usedBlocks();
    data::BlockSource blocks;
    auto* block = static_cast<uint8_t*>(alloc.allocate(16, mem::usage_t::Temp));
    ASSERT_NE(block, nullptr);
    ASSERT_TRUE(blocks.addBlock(block, 1));
    blocks.releaseAll();
    EXPECT_EQ(alloc.usedBlocks(), before);
}

TEST(BlockSource, SingleBlockPeekAdvance)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};

    auto* block = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    ASSERT_NE(block, nullptr);
    block[0] = 0xAA;
    block[1] = 0xBB;
    block[2] = 0xCC;

    ASSERT_TRUE(bs.addBlock(block, 3));
    EXPECT_EQ(bs.blockCount(), 1u);
    EXPECT_EQ(bs.totalBuffered(), 3u);
    EXPECT_FALSE(bs.eof());

    auto peeked = bs.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, 3u);
    EXPECT_EQ(peeked.value().data[0], 0xAA);
    EXPECT_EQ(peeked.value().data[1], 0xBB);
    EXPECT_EQ(peeked.value().data[2], 0xCC);

    ASSERT_TRUE(bs.advance(2).has_value());
    EXPECT_EQ(bs.totalBuffered(), 1u);

    peeked = bs.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, 1u);
    EXPECT_EQ(peeked.value().data[0], 0xCC);

    ASSERT_TRUE(bs.advance(1).has_value());
    EXPECT_TRUE(bs.eof());
    EXPECT_EQ(bs.blockCount(), 0u);
}

TEST(BlockSource, MultipleBlocksConsumedInOrder)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};

    auto* b0 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    auto* b1 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    ASSERT_NE(b0, nullptr);
    ASSERT_NE(b1, nullptr);

    b0[0] = 0x10;
    b0[1] = 0x11;
    b1[0] = 0x20;
    b1[1] = 0x21;
    b1[2] = 0x22;

    ASSERT_TRUE(bs.addBlock(b0, 2));
    ASSERT_TRUE(bs.addBlock(b1, 3));
    EXPECT_EQ(bs.blockCount(), 2u);
    EXPECT_EQ(bs.totalBuffered(), 5u);

    auto peeked = bs.peek(256);
    ASSERT_EQ(peeked.value().size, 2u);
    EXPECT_EQ(peeked.value().data[0], 0x10);

    ASSERT_TRUE(bs.advance(2).has_value());
    EXPECT_EQ(bs.blockCount(), 1u);

    peeked = bs.peek(256);
    ASSERT_EQ(peeked.value().size, 3u);
    EXPECT_EQ(peeked.value().data[0], 0x20);

    ASSERT_TRUE(bs.advance(3).has_value());
    EXPECT_TRUE(bs.eof());
    EXPECT_EQ(bs.blockCount(), 0u);
}

TEST(BlockSource, AdvanceAcrossBlockBoundary)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};

    auto* b0 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    auto* b1 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    b0[0]    = 0xA0;
    b0[1]    = 0xA1;
    b1[0]    = 0xB0;
    b1[1]    = 0xB1;

    ASSERT_TRUE(bs.addBlock(b0, 2));
    ASSERT_TRUE(bs.addBlock(b1, 2));

    ASSERT_TRUE(bs.advance(3).has_value());
    EXPECT_EQ(bs.blockCount(), 1u);
    EXPECT_EQ(bs.totalBuffered(), 1u);

    auto peeked = bs.peek(256);
    ASSERT_EQ(peeked.value().size, 1u);
    EXPECT_EQ(peeked.value().data[0], 0xB1);
}

TEST(BlockSource, PeekClampsToMaxLen)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};

    auto* b = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    std::memset(b, 0xFF, 100);
    ASSERT_TRUE(bs.addBlock(b, 100));

    auto peeked = bs.peek(10);
    ASSERT_EQ(peeked.value().size, 10u);
}

TEST(BlockSource, ReleaseAllFreesBlocks)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    const size_t before   = alloc.usedBlocks();

    data::BlockSource bs{alloc};
    auto* b0 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    auto* b1 = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    ASSERT_TRUE(bs.addBlock(b0, 10));
    ASSERT_TRUE(bs.addBlock(b1, 10));
    EXPECT_GT(alloc.usedBlocks(), before);

    bs.releaseAll();
    EXPECT_EQ(bs.blockCount(), 0u);
    EXPECT_TRUE(bs.eof());
    EXPECT_EQ(alloc.usedBlocks(), before);
}

TEST(BlockSource, DestructorFreesBlocks)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    const size_t before   = alloc.usedBlocks();

    {
        data::BlockSource bs{alloc};
        auto* b = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
        ASSERT_TRUE(bs.addBlock(b, 50));
    }

    EXPECT_EQ(alloc.usedBlocks(), before);
}

TEST(BlockSource, AddBlockRejectsOverflow)
{
    mem::Allocator& alloc = mem::defaultAllocator();
    data::BlockSource bs{alloc};

    for (size_t i = 0; i < data::BlockSource::kMaxBlocks; ++i) {
        auto* b = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
        ASSERT_TRUE(bs.addBlock(b, 1)) << "block " << i;
    }
    uint8_t dummy = 0;
    EXPECT_FALSE(bs.addBlock(&dummy, 1));

    bs.releaseAll();
}

// ============================================================================
// BlockSource + buildDataFrame roundtrip
// ============================================================================

TEST(BlockSource, BuildDataFrameRoundtrip)
{
    mem::Allocator& alloc = mem::defaultAllocator();

    const uint8_t raw_data[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    data::MemorySource src{raw_data, sizeof(raw_data)};

    auto* block = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
    ASSERT_NE(block, nullptr);

    auto frame_size = frame::buildDataFrame(block, 7, src);
    ASSERT_TRUE(frame_size.has_value());
    ASSERT_GT(frame_size.value(), 0u);
    EXPECT_TRUE(src.eof());

    data::BlockSource bs{alloc};
    ASSERT_TRUE(bs.addBlock(block, frame_size.value()));

    auto peeked = bs.peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, frame_size.value());

    frame::View view;
    auto result = frame::decode(peeked.value(), view);
    ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
    EXPECT_EQ(view.kind, frame::Kind::Data);
    EXPECT_EQ(view.b3, 7);
    ASSERT_EQ(view.payload.size, sizeof(raw_data));
    EXPECT_EQ(std::memcmp(view.payload.data, raw_data, sizeof(raw_data)), 0);
}

TEST(BlockSource, MultiFrameRoundtrip)
{
    mem::Allocator& alloc = mem::defaultAllocator();

    std::array<uint8_t, 600> big{};
    for (size_t i = 0; i < big.size(); ++i) {
        big[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource src{big.data(), big.size()};

    data::BlockSource bs{alloc};

    size_t total_payload = 0;
    while (!src.eof()) {
        auto* block = static_cast<uint8_t*>(alloc.allocate(256, mem::usage_t::Temp));
        ASSERT_NE(block, nullptr);
        auto frame_size = frame::buildDataFrame(block, 1, src);
        ASSERT_TRUE(frame_size.has_value());
        if (frame_size.value() == 0) {
            alloc.deallocate(block);
            break;
        }
        ASSERT_TRUE(bs.addBlock(block, frame_size.value()));
        total_payload += frame_size.value() - frame::kHeaderSize;
    }

    EXPECT_EQ(total_payload, big.size());
    EXPECT_EQ(bs.blockCount(), 3u);  // 252 + 252 + 96

    size_t verified = 0;
    while (!bs.eof()) {
        auto peeked = bs.peek(256);
        ASSERT_TRUE(peeked.has_value());
        if (peeked.value().size == 0) {
            break;
        }

        frame::View view;
        auto result = frame::decode(peeked.value(), view);
        ASSERT_EQ(result.status, frame::DecodeStatus::Ok);
        EXPECT_EQ(view.b3, 1);

        for (size_t i = 0; i < view.payload.size; ++i) {
            EXPECT_EQ(view.payload.data[i], static_cast<uint8_t>((verified + i) & 0xFF))
                << "at byte " << (verified + i);
        }
        verified += view.payload.size;

        ASSERT_TRUE(bs.advance(result.consumed).has_value());
    }

    EXPECT_EQ(verified, big.size());
    EXPECT_EQ(bs.blockCount(), 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
