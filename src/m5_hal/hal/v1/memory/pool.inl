// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V1_MEMORY_POOL_INL_
#define M5_HAL_HAL_V1_MEMORY_POOL_INL_

#include <algorithm>
#include <cstring>

namespace m5::hal::v1::memory::detail {

constexpr uint32_t poolLowMask(size_t bits)
{
    return bits >= 32 ? uint32_t{0xFFFFFFFFu} : ((uint32_t{1} << bits) - 1u);
}

template <size_t BlockSize, size_t BlockCount>
void* FixedBlockPool<BlockSize, BlockCount>::allocate(size_t size)
{
    if (size == 0 || size > kPoolSize) {
        return nullptr;
    }
    const size_t needed = (size + BlockSize - 1) / BlockSize;
    lockPool();
    void* result = search(needed);
    unlockPool();
    return result;
}

template <size_t BlockSize, size_t BlockCount>
void* FixedBlockPool<BlockSize, BlockCount>::reallocate(void* ptr, size_t preserve_size, size_t new_size)
{
    if (new_size == 0 || new_size > kPoolSize || !containsPointer(ptr)) {
        return nullptr;
    }

    void* result        = nullptr;
    auto* bytes         = static_cast<uint8_t*>(ptr);
    const size_t offset = static_cast<size_t>(bytes - storage_);
    if ((offset % BlockSize) == 0) {
        const size_t needed = (new_size + BlockSize - 1) / BlockSize;
        lockPool();
        const size_t index  = offset / BlockSize;
        const uint8_t count = block_counts_[index];
        if (count != 0 && index + count <= BlockCount) {
            const uint32_t old_mask        = poolLowMask(count) << index;
            const uint32_t released_bitmap = bitmap_ & ~old_mask;
            size_t found                   = index;
            if (!isRunFree(released_bitmap, index, needed) && !findFreeRunInBitmap(released_bitmap, needed, found)) {
                unlockPool();
                return nullptr;
            }

            auto* dst = storage_ + found * BlockSize;
            const size_t copy_size =
                std::min(std::min(preserve_size, new_size), static_cast<size_t>(count) * BlockSize);
            bitmap_              = released_bitmap | (poolLowMask(needed) << found);
            block_counts_[index] = 0;
            block_counts_[found] = static_cast<uint8_t>(needed);
            if (dst != ptr && copy_size != 0) {
                std::memmove(dst, ptr, copy_size);
            }
            result = dst;
        }
        unlockPool();
    }
    return result;
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::deallocate(void* ptr)
{
    if (!containsPointer(ptr)) {
        return false;
    }

    bool result         = false;
    const auto* bytes   = static_cast<const uint8_t*>(ptr);
    const size_t offset = static_cast<size_t>(bytes - storage_);
    if ((offset % BlockSize) == 0) {
        lockPool();
        const size_t index  = offset / BlockSize;
        const uint8_t count = block_counts_[index];
        result              = !(count == 0 || index + count > BlockCount);
        if (result) {
            bitmap_ &= ~(poolLowMask(count) << index);
            block_counts_[index] = 0;
        }
        unlockPool();
    }
    return result;
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::owns(const void* ptr) const
{
    return containsPointer(ptr);
}

template <size_t BlockSize, size_t BlockCount>
size_t FixedBlockPool<BlockSize, BlockCount>::allocationSize(const void* ptr) const
{
    if (!containsPointer(ptr)) {
        return 0;
    }
    const auto* bytes   = static_cast<const uint8_t*>(ptr);
    const size_t offset = static_cast<size_t>(bytes - storage_);
    if ((offset % BlockSize) != 0) {
        return 0;
    }
    lockPool();
    const size_t index  = offset / BlockSize;
    const uint8_t count = block_counts_[index];
    const size_t result = (count != 0 && index + count <= BlockCount) ? static_cast<size_t>(count) * BlockSize : 0;
    unlockPool();
    return result;
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::containsPointer(const void* ptr) const
{
    const auto* p = static_cast<const uint8_t*>(ptr);
    return p >= storage_ && p < storage_ + kPoolSize;
}

template <size_t BlockSize, size_t BlockCount>
size_t FixedBlockPool<BlockSize, BlockCount>::usedBlocks() const
{
    lockPool();
#if defined(__GNUC__) || defined(__clang__)
    const size_t result = static_cast<size_t>(__builtin_popcount(bitmap_));
#else
    size_t used   = 0;
    uint32_t bits = bitmap_;
    while (bits != 0) {
        used += bits & 1u;
        bits >>= 1;
    }
    const size_t result = used;
#endif
    unlockPool();
    return result;
}

template <size_t BlockSize, size_t BlockCount>
size_t FixedBlockPool<BlockSize, BlockCount>::largestFreeRun() const
{
    lockPool();
    size_t best = 0;
    size_t run  = 0;
    for (size_t i = 0; i < BlockCount; ++i) {
        if ((bitmap_ & (uint32_t{1} << i)) == 0) {
            ++run;
            if (run > best) {
                best = run;
            }
        } else {
            run = 0;
        }
    }
    unlockPool();
    return best;
}

template <size_t BlockSize, size_t BlockCount>
void FixedBlockPool<BlockSize, BlockCount>::lockPool() const
{
    while (_lock.test_and_set(std::memory_order_acquire)) {
    }
}

template <size_t BlockSize, size_t BlockCount>
void FixedBlockPool<BlockSize, BlockCount>::unlockPool() const
{
    _lock.clear(std::memory_order_release);
}

template <size_t BlockSize, size_t BlockCount>
void* FixedBlockPool<BlockSize, BlockCount>::search(size_t needed)
{
    size_t index = 0;
    if (findFreeRun(needed, index)) {
        return allocateRun(index, needed);
    }
    return nullptr;
}

template <size_t BlockSize, size_t BlockCount>
void* FixedBlockPool<BlockSize, BlockCount>::allocateRun(size_t index, size_t count)
{
    bitmap_ |= poolLowMask(count) << index;
    block_counts_[index] = static_cast<uint8_t>(count);
    return storage_ + index * BlockSize;
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::isRunFree(uint32_t bitmap, size_t index, size_t count) const
{
    return count <= BlockCount && index <= BlockCount - count && (bitmap & (poolLowMask(count) << index)) == 0;
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::findFreeRun(size_t needed, size_t& found) const
{
    return findFreeRunInBitmap(bitmap_, needed, found);
}

template <size_t BlockSize, size_t BlockCount>
bool FixedBlockPool<BlockSize, BlockCount>::findFreeRunInBitmap(uint32_t bitmap, size_t needed, size_t& found) const
{
    const size_t last_start = BlockCount - needed;
    uint32_t front_mask     = poolLowMask(needed);
    uint32_t back_mask      = front_mask << last_start;
    size_t offset           = 0;
    do {
        if ((bitmap & front_mask) == 0) {
            found = offset;
            return true;
        }
        if ((bitmap & back_mask) == 0) {
            found = last_start - offset;
            return true;
        }
        ++offset;
        front_mask <<= 1;
        back_mask >>= 1;
    } while (back_mask >= front_mask);
    return false;
}

}  // namespace m5::hal::v1::memory::detail

#endif  // M5_HAL_HAL_V1_MEMORY_POOL_INL_
