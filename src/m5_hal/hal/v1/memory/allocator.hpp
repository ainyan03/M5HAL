// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V1_MEMORY_ALLOCATOR_HPP_
#define M5_HAL_HAL_V1_MEMORY_ALLOCATOR_HPP_

#include "../../../../m5_hal_config.hpp"
#include "pool.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
    namespace memory {

    enum class usage_t : uint8_t {
        temp,
        persistent,
        persistent_slow,
    };

    class Allocator {
    public:
        using malloc_fn_t = void* (*)(size_t, usage_t);
        /// Fallback reallocator hook.
        ///
        /// `preserve_size` is the number of bytes the caller wants to keep
        /// from `ptr`, not necessarily the backend allocation capacity.
        /// Implementations may use it as the copy bound when they cannot grow
        /// in place.
        using realloc_fn_t = void* (*)(void*, size_t, size_t, usage_t);
        using free_fn_t    = void (*)(void*);

        /// Allocates a buffer.
        ///
        /// Temporary allocations use the internal fixed-block pool first and
        /// fall back to the configured fallback allocator when the pool cannot
        /// satisfy the request.
        void* allocate(size_t size, usage_t usage = usage_t::temp);

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
        void* reallocate(void* ptr, size_t preserve_size, size_t new_size, usage_t usage = usage_t::temp);

        /// Deallocates a buffer returned by this allocator.
        void deallocate(void* ptr);

        /// Sets fallback allocation and free hooks.
        ///
        /// Fallback reallocation uses malloc-copy-free when no realloc hook is
        /// registered.
        void setFallback(malloc_fn_t malloc_fn, free_fn_t free_fn)
        {
            _malloc_fn  = malloc_fn;
            _realloc_fn = nullptr;
            _free_fn    = free_fn;
        }

        /// Sets fallback allocation, reallocation, and free hooks.
        ///
        /// Call this during initialization only; concurrent updates while
        /// allocation APIs are running are not synchronized.
        void setFallback(malloc_fn_t malloc_fn, realloc_fn_t realloc_fn, free_fn_t free_fn)
        {
            _malloc_fn  = malloc_fn;
            _realloc_fn = realloc_fn;
            _free_fn    = free_fn;
        }

        size_t usedBlocks() const;
        size_t largestFreeRun() const;

        static constexpr size_t tempBlockSize()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE;
        }

        static constexpr size_t tempBlockCount()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT;
        }

        static constexpr size_t tempPoolSize()
        {
            return M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE * M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT;
        }

    private:
        void* mallocFallback(size_t size, usage_t usage);
        void* reallocFallback(void* ptr, size_t preserve_size, size_t new_size, usage_t usage);
        void freeFallback(void* ptr);

        detail::FixedBlockPool<M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE, M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT> _temp_pool;
        malloc_fn_t _malloc_fn   = nullptr;
        realloc_fn_t _realloc_fn = nullptr;
        free_fn_t _free_fn       = nullptr;
    };

    class TempBuffer {
    public:
        TempBuffer() = default;
        TempBuffer(Allocator& alloc, size_t size, usage_t usage = usage_t::temp);
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
        usage_t _usage    = usage_t::temp;
    };

    Allocator& defaultAllocator();

    inline void* alloc_temp(size_t size)
    {
        return defaultAllocator().allocate(size, usage_t::temp);
    }

    inline void* alloc(size_t size, usage_t usage = usage_t::persistent)
    {
        return defaultAllocator().allocate(size, usage);
    }

    inline void* alloc_psram(size_t size)
    {
        return defaultAllocator().allocate(size, usage_t::persistent_slow);
    }

    inline void free(void* ptr)
    {
        defaultAllocator().deallocate(ptr);
    }

    }  // namespace memory
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V1_MEMORY_ALLOCATOR_HPP_
