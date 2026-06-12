#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <utility>

namespace {

using m5::hal::v1::getM5_Hal;
using m5::hal::v1::memory::Allocator;
using m5::hal::v1::memory::TempBuffer;
using m5::hal::v1::memory::usage_t;

struct FallbackCounters {
    static inline size_t malloc_count               = 0;
    static inline size_t realloc_count              = 0;
    static inline size_t free_count                 = 0;
    static inline usage_t last_usage                = usage_t::temp;
    static inline void* last_freed                  = nullptr;
    static inline size_t last_realloc_preserve_size = 0;
    static inline size_t last_realloc_new_size      = 0;

    static void reset()
    {
        malloc_count               = 0;
        realloc_count              = 0;
        free_count                 = 0;
        last_usage                 = usage_t::temp;
        last_freed                 = nullptr;
        last_realloc_preserve_size = 0;
        last_realloc_new_size      = 0;
    }

    static void* mallocFn(size_t size, usage_t usage)
    {
        ++malloc_count;
        last_usage = usage;
        return std::malloc(size);
    }

    static void* reallocFn(void* ptr, size_t preserve_size, size_t new_size, usage_t usage)
    {
        ++realloc_count;
        last_usage                 = usage;
        last_realloc_preserve_size = preserve_size;
        last_realloc_new_size      = new_size;
        return std::realloc(ptr, new_size);
    }

    static void freeFn(void* ptr)
    {
        ++free_count;
        last_freed = ptr;
        std::free(ptr);
    }

    static void freeNoop(void* ptr)
    {
        ++free_count;
        last_freed = ptr;
    }
};

}  // namespace

TEST(MemoryAllocator, ZeroSizeReturnsNull)
{
    Allocator alloc;
    EXPECT_EQ(alloc.allocate(0), nullptr);
}

TEST(MemoryAllocator, ZeroSizeDoesNotCallFallback)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    EXPECT_EQ(alloc.allocate(0), nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 0u);
}

TEST(MemoryAllocator, OneByteAllocationUsesOneBlock)
{
    Allocator alloc;
    void* p = alloc.allocate(1);

    ASSERT_NE(p, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 1u);
    EXPECT_EQ(alloc.largestFreeRun(), Allocator::tempBlockCount() - 1u);

    alloc.deallocate(p);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
    EXPECT_EQ(alloc.largestFreeRun(), Allocator::tempBlockCount());
}

TEST(MemoryAllocator, BoundaryAndMultiBlockAllocation)
{
    Allocator alloc;
    void* one = alloc.allocate(Allocator::tempBlockSize());
    void* two = alloc.allocate(Allocator::tempBlockSize() + 1u);

    ASSERT_NE(one, nullptr);
    ASSERT_NE(two, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 3u);

    alloc.deallocate(two);
    alloc.deallocate(one);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, SmallAllocationsAlternateHeadAndTail)
{
    Allocator alloc;
    auto* first  = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    auto* second = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    auto* third  = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));

    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    ASSERT_GT(second, first);
    EXPECT_EQ(static_cast<size_t>(second - first), Allocator::tempPoolSize() - Allocator::tempBlockSize());
    EXPECT_EQ(third, first + Allocator::tempBlockSize());

    alloc.deallocate(third);
    alloc.deallocate(second);
    alloc.deallocate(first);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, FullPoolAllocationUsesAllBlocks)
{
    Allocator alloc;
    void* p = alloc.allocate(Allocator::tempPoolSize());

    ASSERT_NE(p, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount());
    EXPECT_EQ(alloc.largestFreeRun(), 0u);

    alloc.deallocate(p);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, OversizeTempFallsBack)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    void* p = alloc.allocate(Allocator::tempPoolSize() + 1u, usage_t::temp);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(FallbackCounters::last_usage, usage_t::temp);
    EXPECT_EQ(alloc.usedBlocks(), 0u);

    alloc.deallocate(p);
    EXPECT_EQ(FallbackCounters::free_count, 1u);
}

TEST(MemoryAllocator, PoolExhaustionFallsBack)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    void* full = alloc.allocate(Allocator::tempPoolSize());
    ASSERT_NE(full, nullptr);

    void* extra = alloc.allocate(1, usage_t::temp);
    ASSERT_NE(extra, nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount());

    alloc.deallocate(extra);
    alloc.deallocate(full);
    EXPECT_EQ(FallbackCounters::free_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, RepeatedSmallAllocationsExhaustPoolThenFallback)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    std::array<void*, Allocator::tempBlockCount()> blocks{};
    for (auto& block : blocks) {
        block = alloc.allocate(1, usage_t::temp);
        ASSERT_NE(block, nullptr);
    }
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount());
    EXPECT_EQ(alloc.largestFreeRun(), 0u);
    EXPECT_EQ(FallbackCounters::malloc_count, 0u);

    void* extra = alloc.allocate(1, usage_t::temp);
    ASSERT_NE(extra, nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount());

    alloc.deallocate(extra);
    for (auto* block : blocks) {
        alloc.deallocate(block);
    }
    EXPECT_EQ(FallbackCounters::free_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, PersistentBypassesPool)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    void* p = alloc.allocate(16, usage_t::persistent);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(FallbackCounters::last_usage, usage_t::persistent);
    EXPECT_EQ(alloc.usedBlocks(), 0u);

    alloc.deallocate(p);
    EXPECT_EQ(FallbackCounters::free_count, 1u);
}

TEST(MemoryAllocator, PersistentReallocateUsesFallbackReallocator)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::reallocFn, &FallbackCounters::freeFn);

    auto* p = static_cast<unsigned char*>(alloc.allocate(16, usage_t::persistent));
    ASSERT_NE(p, nullptr);
    p[0] = 0x56;

    auto* grown = static_cast<unsigned char*>(alloc.reallocate(p, 16, 64, usage_t::persistent));
    ASSERT_NE(grown, nullptr);
    EXPECT_EQ(grown[0], 0x56);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(FallbackCounters::realloc_count, 1u);
    EXPECT_EQ(FallbackCounters::last_usage, usage_t::persistent);
    EXPECT_EQ(FallbackCounters::last_realloc_preserve_size, 16u);
    EXPECT_EQ(FallbackCounters::last_realloc_new_size, 64u);
    EXPECT_EQ(FallbackCounters::free_count, 0u);
    EXPECT_EQ(alloc.usedBlocks(), 0u);

    alloc.deallocate(grown);
    EXPECT_EQ(FallbackCounters::free_count, 1u);
}

TEST(MemoryAllocator, PoolPointerReallocAcrossUsageMovesOutOfPool)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::reallocFn, &FallbackCounters::freeFn);

    auto* p = static_cast<unsigned char*>(alloc.allocate(16, usage_t::temp));
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 1u);
    std::memset(p, 0xA5, 16);

    // Promoting a pool-owned buffer to persistent must NOT hand the pool
    // pointer to the fallback reallocator (it is not a heap pointer);
    // it must allocate at the destination, copy, and release the block.
    auto* promoted = static_cast<unsigned char*>(alloc.reallocate(p, 16, 64, usage_t::persistent));
    ASSERT_NE(promoted, nullptr);
    EXPECT_EQ(FallbackCounters::realloc_count, 0u);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(FallbackCounters::last_usage, usage_t::persistent);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
    for (size_t i = 0; i < 16; ++i) {
        EXPECT_EQ(promoted[i], 0xA5);
    }

    alloc.deallocate(promoted);
    EXPECT_EQ(FallbackCounters::free_count, 1u);
}

TEST(MemoryAllocator, DeallocateNullIsSafe)
{
    Allocator alloc;
    alloc.deallocate(nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, DeallocateNullDoesNotCallFallback)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    alloc.deallocate(nullptr);
    EXPECT_EQ(FallbackCounters::free_count, 0u);
}

// An invalid pointer inside the pool (interior pointer or double free)
// must never reach the fallback `free` (heap corruption). Debug builds
// assert; release builds ignore it. The death tests below pin down the
// debug behaviour; the release branch checks the silent-ignore path.
#if !defined(NDEBUG)
TEST(MemoryAllocatorDeathTest, InteriorPoolPointerAsserts)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeNoop);

    auto* p = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    ASSERT_NE(p, nullptr);

    EXPECT_DEATH({ alloc.deallocate(p + 1); }, "invalid in-pool pointer");
    EXPECT_EQ(FallbackCounters::free_count, 0u);

    alloc.deallocate(p);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocatorDeathTest, DoubleFreeOfPoolBlockAsserts)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeNoop);

    auto* p = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    ASSERT_NE(p, nullptr);
    alloc.deallocate(p);
    EXPECT_EQ(alloc.usedBlocks(), 0u);

    EXPECT_DEATH({ alloc.deallocate(p); }, "invalid in-pool pointer");
    EXPECT_EQ(FallbackCounters::free_count, 0u);
}
#else
TEST(MemoryAllocator, NonBoundaryPoolPointerIgnoredWithoutFallbackFree)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeNoop);

    auto* p = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    ASSERT_NE(p, nullptr);

    alloc.deallocate(p + 1);
    EXPECT_EQ(FallbackCounters::free_count, 0u);
    EXPECT_EQ(alloc.usedBlocks(), 1u);

    alloc.deallocate(p);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}
#endif  // !defined(NDEBUG)

TEST(MemoryAllocator, MixedAllocateFreeReusesReleasedBlocks)
{
    Allocator alloc;
    auto* first  = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    auto* second = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    auto* third  = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    auto* fourth = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));

    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(third, nullptr);
    ASSERT_NE(fourth, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 4u);

    alloc.deallocate(second);
    alloc.deallocate(third);
    EXPECT_EQ(alloc.usedBlocks(), 2u);

    void* reused_tail = alloc.allocate(Allocator::tempBlockSize());
    void* reused_head = alloc.allocate(Allocator::tempBlockSize());
    EXPECT_EQ(reused_tail, second);
    EXPECT_EQ(reused_head, third);
    EXPECT_EQ(alloc.usedBlocks(), 4u);

    alloc.deallocate(reused_head);
    alloc.deallocate(reused_tail);
    alloc.deallocate(fourth);
    alloc.deallocate(first);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, FragmentedPoolCanAllocateFromAnotherRun)
{
    Allocator alloc;
    void* a = alloc.allocate(Allocator::tempBlockSize());
    void* b = alloc.allocate(Allocator::tempBlockSize());
    void* c = alloc.allocate(Allocator::tempBlockSize());

    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_NE(c, nullptr);

    alloc.deallocate(b);
    EXPECT_EQ(alloc.usedBlocks(), 2u);

    void* large = alloc.allocate(Allocator::tempBlockSize() * 4u);
    ASSERT_NE(large, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 6u);

    alloc.deallocate(large);
    alloc.deallocate(c);
    alloc.deallocate(a);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, FragmentedPoolFallsBackWhenNoContiguousRunFits)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    std::array<unsigned char*, Allocator::tempBlockCount()> blocks{};
    for (auto& block : blocks) {
        block = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
        ASSERT_NE(block, nullptr);
    }
    std::sort(blocks.begin(), blocks.end());

    for (size_t i = 1; i < blocks.size(); i += 2) {
        alloc.deallocate(blocks[i]);
    }
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount() / 2u);
    EXPECT_EQ(alloc.largestFreeRun(), 1u);

    void* needs_two_blocks = alloc.allocate(Allocator::tempBlockSize() * 2u, usage_t::temp);
    ASSERT_NE(needs_two_blocks, nullptr);
    EXPECT_EQ(FallbackCounters::malloc_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), Allocator::tempBlockCount() / 2u);

    alloc.deallocate(needs_two_blocks);
    for (size_t i = 0; i < blocks.size(); i += 2) {
        alloc.deallocate(blocks[i]);
    }
    EXPECT_EQ(FallbackCounters::free_count, 1u);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, ReallocateGrowsPoolBlockInPlaceWhenTailIsFree)
{
    Allocator alloc;
    auto* p = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    ASSERT_NE(p, nullptr);
    p[0] = 0x12;

    void* grown = alloc.reallocate(p, Allocator::tempBlockSize(), Allocator::tempBlockSize() * 2u);
    EXPECT_EQ(grown, p);
    EXPECT_EQ(static_cast<unsigned char*>(grown)[0], 0x12);
    EXPECT_EQ(alloc.usedBlocks(), 2u);

    alloc.deallocate(grown);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, ReallocateMovesPoolBlockWhenTailIsOccupied)
{
    Allocator alloc;
    FallbackCounters::reset();
    alloc.setFallback(&FallbackCounters::mallocFn, &FallbackCounters::freeFn);

    auto* p       = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    void* tail    = alloc.allocate(Allocator::tempBlockSize());
    void* blocker = alloc.allocate(Allocator::tempBlockSize());
    ASSERT_NE(p, nullptr);
    ASSERT_NE(tail, nullptr);
    ASSERT_NE(blocker, nullptr);
    p[0] = 0x34;

    void* grown = alloc.reallocate(p, Allocator::tempBlockSize(), Allocator::tempBlockSize() * 2u);
    ASSERT_NE(grown, nullptr);
    EXPECT_NE(grown, p);
    EXPECT_EQ(static_cast<unsigned char*>(grown)[0], 0x34);
    EXPECT_EQ(alloc.usedBlocks(), 4u);
    EXPECT_EQ(FallbackCounters::malloc_count, 0u);

    alloc.deallocate(grown);
    alloc.deallocate(blocker);
    alloc.deallocate(tail);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
    EXPECT_EQ(FallbackCounters::free_count, 0u);
}

TEST(MemoryAllocator, ReallocateShrinksPoolBlockInPlace)
{
    Allocator alloc;
    void* p = alloc.allocate(Allocator::tempBlockSize() * 3u);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 3u);

    void* shrunk = alloc.reallocate(p, Allocator::tempBlockSize() * 3u, Allocator::tempBlockSize());
    EXPECT_EQ(shrunk, p);
    EXPECT_EQ(alloc.usedBlocks(), 1u);
    EXPECT_EQ(alloc.largestFreeRun(), Allocator::tempBlockCount() - 1u);

    alloc.deallocate(shrunk);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, ReallocateCanDropPreservedData)
{
    Allocator alloc;
    auto* p = static_cast<unsigned char*>(alloc.allocate(Allocator::tempBlockSize()));
    ASSERT_NE(p, nullptr);

    void* grown = alloc.reallocate(p, 0, Allocator::tempBlockSize() * 2u);
    ASSERT_NE(grown, nullptr);
    EXPECT_EQ(alloc.usedBlocks(), 2u);

    alloc.deallocate(grown);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, TempBufferReleasesOnDestruction)
{
    Allocator alloc;
    {
        TempBuffer buf{alloc, Allocator::tempBlockSize() + 1u};
        ASSERT_TRUE(buf);
        EXPECT_EQ(buf.size(), Allocator::tempBlockSize() + 1u);
        EXPECT_EQ(alloc.usedBlocks(), 2u);
    }
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, TempBufferReallocatePreservesData)
{
    Allocator alloc;
    TempBuffer buf{alloc, Allocator::tempBlockSize()};
    ASSERT_TRUE(buf);
    std::memset(buf.data(), 0x5A, buf.size());

    void* ptr = buf.data();
    ASSERT_TRUE(buf.reallocate(Allocator::tempBlockSize() * 2u));
    EXPECT_EQ(buf.data(), ptr);
    EXPECT_EQ(buf.size(), Allocator::tempBlockSize() * 2u);
    EXPECT_EQ(static_cast<unsigned char*>(buf.data())[0], 0x5A);
    EXPECT_EQ(alloc.usedBlocks(), 2u);
}

TEST(MemoryAllocator, TempBufferMoveConstructorTransfersOwnership)
{
    Allocator alloc;
    TempBuffer first{alloc, 64};
    void* ptr = first.data();
    ASSERT_NE(ptr, nullptr);

    TempBuffer second{std::move(first)};
    EXPECT_FALSE(first);
    EXPECT_TRUE(second);
    EXPECT_EQ(second.data(), ptr);
    EXPECT_EQ(alloc.usedBlocks(), 1u);
}

TEST(MemoryAllocator, TempBufferMoveAssignmentTransfersOwnership)
{
    Allocator alloc;
    TempBuffer first{alloc, 64};
    TempBuffer second{alloc, 32};

    void* ptr = first.data();
    second    = std::move(first);

    EXPECT_FALSE(first);
    EXPECT_TRUE(second);
    EXPECT_EQ(second.data(), ptr);
    EXPECT_EQ(second.size(), 64u);
    EXPECT_EQ(alloc.usedBlocks(), 1u);
}

TEST(MemoryAllocator, TempBufferReleaseGivesUpOwnership)
{
    Allocator alloc;
    TempBuffer buf{alloc, 32};
    void* ptr = buf.data();
    ASSERT_NE(ptr, nullptr);

    void* released = buf.release();
    EXPECT_EQ(released, ptr);
    EXPECT_FALSE(buf);
    EXPECT_EQ(alloc.usedBlocks(), 1u);

    alloc.deallocate(released);
    EXPECT_EQ(alloc.usedBlocks(), 0u);
}

TEST(MemoryAllocator, DefaultAllocatorIsM5HALCoreMemory)
{
    EXPECT_EQ(&m5::hal::v1::memory::defaultAllocator(), &getM5_Hal().Memory);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
