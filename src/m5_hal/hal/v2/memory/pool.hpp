// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_MEMORY_POOL_HPP_
#define M5_HAL_HAL_V2_MEMORY_POOL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::memory::detail {

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
    /// Byte capacity of the block run starting at `ptr` (0 when `ptr` is
    /// not a live run start). Lets the owner clamp copies when moving a
    /// pool allocation out to the fallback heap.
    size_t allocationSize(const void* ptr) const;
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
    /*!
      Task-context only. Calling from an ISR on a single-core system will
      deadlock immediately because the ISR preempts the task that holds the
      flag and then spins forever waiting for itself. Equally, this is a
      no-yield busy-wait, so any priority-inversion scenario (a low-priority
      task holds the lock while a high-priority task spins) will stall the
      high-priority task for the full allocation duration.
     */
#if defined(PICO_RP2040)
    // RP2040's Cortex-M0+ (ARMv6-M) has no hardware atomic
    // read-modify-write instruction, and arduino-pico ships no libatomic,
    // so std::atomic_flag is unlinkable there (undefined reference to
    // __atomic_test_and_set). lockPool()/unlockPool() fall back to an
    // interrupt-disable critical section instead — same task-context-only,
    // no-ISR contract as above, but note this guards only the CURRENT core:
    // RP2040's second core is not covered (no hardware spinlock used), so a
    // pool must not be shared across cores on that target. RP2350
    // (Cortex-M33) and SAMD51 (Cortex-M4F, ARMv7E-M) have atomic RMW
    // instructions and stay on the std::atomic_flag path below.
    mutable bool _lock = false;
#else
    mutable std::atomic_flag _lock = ATOMIC_FLAG_INIT;
#endif
};

}  // namespace m5::hal::v2::memory::detail

#endif  // M5_HAL_HAL_V2_MEMORY_POOL_HPP_
