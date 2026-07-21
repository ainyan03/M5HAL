// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_MEMORY_ALLOCATOR_HPP_
#define M5_HAL_HAL_V2_MEMORY_ALLOCATOR_HPP_

#include "../../../../m5_hal_config.hpp"
#include "pool.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <atomic>

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    namespace memory {

    enum class usage_t : uint8_t {
        Temp,
        Persistent,
        PersistentSlow,
    };

    struct FallbackOps {
        using malloc_fn_t  = void* (*)(size_t, usage_t);
        using realloc_fn_t = void* (*)(void*, size_t, size_t, usage_t);
        using free_fn_t    = void (*)(void*);

        constexpr FallbackOps() = default;
        constexpr FallbackOps(malloc_fn_t malloc, free_fn_t free) : malloc_fn{malloc}, free_fn{free}
        {
        }
        constexpr FallbackOps(malloc_fn_t malloc, realloc_fn_t realloc, free_fn_t free)
            : malloc_fn{malloc}, realloc_fn{realloc}, free_fn{free}
        {
        }

        constexpr bool valid() const
        {
            const bool standard = malloc_fn == nullptr && realloc_fn == nullptr && free_fn == nullptr;
            const bool custom   = malloc_fn != nullptr && free_fn != nullptr;
            return standard || custom;
        }

        malloc_fn_t malloc_fn   = nullptr;
        realloc_fn_t realloc_fn = nullptr;
        free_fn_t free_fn       = nullptr;
    };

    class Allocator {
    public:
        using malloc_fn_t = FallbackOps::malloc_fn_t;
        /// Fallback reallocator hook.
        ///
        /// `preserve_size` is the number of bytes the caller wants to keep
        /// from `ptr`, not necessarily the backend allocation capacity.
        /// Implementations may use it as the copy bound when they cannot grow
        /// in place.
        using realloc_fn_t = FallbackOps::realloc_fn_t;
        using free_fn_t    = FallbackOps::free_fn_t;

        Allocator();
        explicit Allocator(FallbackOps fallback);

        /// Allocates a buffer.
        ///
        /// Temporary allocations use the internal fixed-block pool first and
        /// fall back to the configured fallback allocator when the pool cannot
        /// satisfy the request.
        void* allocate(size_t size, usage_t usage = usage_t::Temp);

        /// Reallocates a buffer while preserving existing data.
        ///
        /// This is a low-level API. `preserve_size` is supplied by the caller
        /// and may be smaller than the old allocation, including zero, to
        /// intentionally reduce copy cost. M5HAL preserves up to
        /// `min(preserve_size, new_size)` bytes. For pool-owned buffers the
        /// copy is additionally clamped to the actual pool allocation, so an
        /// oversized `preserve_size` cannot read past it; for fallback-owned
        /// buffers the old allocation size is unknown here, and a
        /// `preserve_size` larger than the readable old allocation may read
        /// undefined memory during fallback copy.
        ///
        /// The returned pointer may differ from `ptr`, including moves between
        /// the temp pool and fallback allocator. On failure, returns `nullptr`
        /// and leaves ownership and contents of `ptr` unchanged.
        void* reallocate(void* ptr, size_t preserve_size, size_t new_size, usage_t usage = usage_t::Temp);

        /// Deallocates a buffer returned by this allocator.
        void deallocate(void* ptr);

        size_t usedBlocks() const;
        size_t largestFreeRun() const;
        size_t tempReleaseCount() const;

        /// Returns whether `ptr` is the start of a live temp-pool allocation.
        ///
        /// This distinguishes pool storage from fallback allocations without
        /// inferring ownership from shared pool usage counters.
        bool isTempPoolAllocation(const void* ptr) const;

        static constexpr size_t tempBlockSize()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES;
        }

        static constexpr size_t tempBlockCount()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT;
        }

        static constexpr size_t tempPoolSize()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES * M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT;
        }

    private:
        void* mallocFallback(size_t size, usage_t usage);
        void* reallocFallback(void* ptr, size_t preserve_size, size_t new_size, usage_t usage);
        void freeFallback(void* ptr);

        detail::FixedBlockPool<M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES, M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT>
            _temp_pool;
        std::atomic<size_t> _temp_release_count{0};
        const FallbackOps _fallback;
    };

    class TempBuffer {
    public:
        TempBuffer() = default;
        TempBuffer(Allocator& alloc, size_t size, usage_t usage = usage_t::Temp);
        ~TempBuffer();

        TempBuffer(TempBuffer&& other) noexcept;
        TempBuffer& operator=(TempBuffer&& other) noexcept;
        TempBuffer(const TempBuffer&)            = delete;
        TempBuffer& operator=(const TempBuffer&) = delete;

        void* data() const
        {
            return _ptr;
        }

        size_t size() const
        {
            return _size;
        }

        explicit operator bool() const
        {
            return _ptr != nullptr;
        }

        /// Reallocates this buffer.
        ///
        /// The buffer keeps track of its current valid data size and preserves
        /// that range automatically. The data pointer may change; call `data()`
        /// again after a successful reallocation.
        bool reallocate(size_t size);
        void reset();
        void* release();

    private:
        Allocator* _alloc = nullptr;
        void* _ptr        = nullptr;
        size_t _size      = 0;
        usage_t _usage    = usage_t::Temp;
    };

    Allocator& defaultAllocator();

    inline void* alloc_temp(size_t size)
    {
        return defaultAllocator().allocate(size, usage_t::Temp);
    }

    inline void* alloc(size_t size, usage_t usage = usage_t::Persistent)
    {
        return defaultAllocator().allocate(size, usage);
    }

    inline void* alloc_psram(size_t size)
    {
        return defaultAllocator().allocate(size, usage_t::PersistentSlow);
    }

    inline void free(void* ptr)
    {
        defaultAllocator().deallocate(ptr);
    }

    }  // namespace memory
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V2_MEMORY_ALLOCATOR_HPP_
