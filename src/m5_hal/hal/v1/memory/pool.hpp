#ifndef M5_HAL_HAL_V1_MEMORY_POOL_HPP_
#define M5_HAL_HAL_V1_MEMORY_POOL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace m5::hal::v1::memory::detail {

template <size_t BlockSize, size_t BlockCount>
class FixedBlockPool {
    static_assert(BlockSize >= 4, "BlockSize must be >= 4");
    static_assert((BlockSize & 3) == 0, "BlockSize must be a multiple of 4");
    static_assert(BlockCount >= 1, "BlockCount must be >= 1");
    static_assert(BlockCount <= 32, "This pool variant stores block usage in one 32-bit bitmap");

public:
    void* allocate(size_t size);
    void* reallocate(void* ptr, size_t preserve_size, size_t new_size);
    bool deallocate(void* ptr);
    bool owns(const void* ptr) const;
    size_t usedBlocks() const;
    size_t largestFreeRun() const;

    static constexpr size_t blockSize()
    {
        return BlockSize;
    }

    static constexpr size_t blockCount()
    {
        return BlockCount;
    }

    static constexpr size_t poolSize()
    {
        return kPoolSize;
    }

private:
    static constexpr size_t kPoolSize = BlockSize * BlockCount;

    void lockPool() const;
    void unlockPool() const;
    bool containsPointer(const void* ptr) const;
    void* search(size_t needed);
    void* allocateRun(size_t index, size_t count);
    bool isRunFree(uint32_t bitmap, size_t index, size_t count) const;
    bool findFreeRunInBitmap(uint32_t bitmap, size_t needed, size_t& found) const;
    bool findFreeRun(size_t needed, size_t& found) const;

    alignas(16) uint8_t storage_[kPoolSize]{};
    uint32_t bitmap_ = 0;
    uint8_t block_counts_[BlockCount]{};
    mutable std::atomic_flag _lock = ATOMIC_FLAG_INIT;
};

}  // namespace m5::hal::v1::memory::detail

#endif  // M5_HAL_HAL_V1_MEMORY_POOL_HPP_
