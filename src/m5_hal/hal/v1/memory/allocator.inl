#ifndef M5_HAL_HAL_V1_MEMORY_ALLOCATOR_INL_
#define M5_HAL_HAL_V1_MEMORY_ALLOCATOR_INL_

#include "allocator.hpp"
#include "../m5_hal.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
    namespace memory {

    void* Allocator::allocate(size_t size, usage_t usage)
    {
        if (usage == usage_t::temp) {
            void* p = _temp_pool.allocate(size);
            if (p != nullptr) {
                return p;
            }
        }

        return mallocFallback(size, usage);
    }

    void* Allocator::reallocate(void* ptr, size_t preserve_size, size_t new_size, usage_t usage)
    {
        if (ptr == nullptr) {
            return allocate(new_size, usage);
        }
        if (new_size == 0) {
            deallocate(ptr);
            return nullptr;
        }

        if (usage == usage_t::temp && _temp_pool.owns(ptr)) {
            void* pool_ptr = _temp_pool.reallocate(ptr, preserve_size, new_size);
            if (pool_ptr != nullptr) {
                return pool_ptr;
            }
            void* next = allocate(new_size, usage);
            if (next == nullptr) {
                return nullptr;
            }
            std::memcpy(next, ptr, std::min(preserve_size, new_size));
            deallocate(ptr);
            return next;
        }

        return reallocFallback(ptr, preserve_size, new_size, usage);
    }

    void Allocator::deallocate(void* ptr)
    {
        if (_temp_pool.deallocate(ptr)) {
            return;
        }

        freeFallback(ptr);
    }

    size_t Allocator::usedBlocks() const
    {
        return _temp_pool.usedBlocks();
    }

    size_t Allocator::largestFreeRun() const
    {
        return _temp_pool.largestFreeRun();
    }

    void* Allocator::mallocFallback(size_t size, usage_t usage)
    {
        if (size == 0) {
            return nullptr;
        }

        malloc_fn_t malloc_fn = _malloc_fn;
        if (malloc_fn != nullptr) {
            return malloc_fn(size, usage);
        }
        (void)usage;
        return std::malloc(size);
    }

    void* Allocator::reallocFallback(void* ptr, size_t preserve_size, size_t new_size, usage_t usage)
    {
        realloc_fn_t realloc_fn = _realloc_fn;
        if (realloc_fn != nullptr) {
            return realloc_fn(ptr, preserve_size, new_size, usage);
        }

        void* next = mallocFallback(new_size, usage);
        if (next == nullptr) {
            return nullptr;
        }
        std::memcpy(next, ptr, std::min(preserve_size, new_size));
        freeFallback(ptr);
        return next;
    }

    void Allocator::freeFallback(void* ptr)
    {
        if (ptr == nullptr) {
            return;
        }

        free_fn_t free_fn = _free_fn;
        if (free_fn != nullptr) {
            free_fn(ptr);
            return;
        }
        std::free(ptr);
    }

    TempBuffer::TempBuffer(Allocator& alloc, size_t size, usage_t usage)
        : _alloc(&alloc), _ptr(alloc.allocate(size, usage)), _size(_ptr != nullptr ? size : 0), _usage(usage)
    {
    }

    TempBuffer::~TempBuffer()
    {
        reset();
    }

    TempBuffer::TempBuffer(TempBuffer&& other) noexcept
        : _alloc(other._alloc), _ptr(other._ptr), _size(other._size), _usage(other._usage)
    {
        other._alloc = nullptr;
        other._ptr   = nullptr;
        other._size  = 0;
        other._usage = usage_t::temp;
    }

    TempBuffer& TempBuffer::operator=(TempBuffer&& other) noexcept
    {
        if (this != &other) {
            reset();
            _alloc       = other._alloc;
            _ptr         = other._ptr;
            _size        = other._size;
            _usage       = other._usage;
            other._alloc = nullptr;
            other._ptr   = nullptr;
            other._size  = 0;
            other._usage = usage_t::temp;
        }
        return *this;
    }

    bool TempBuffer::reallocate(size_t size)
    {
        if (_alloc == nullptr) {
            return size == 0;
        }
        void* ptr = _alloc->reallocate(_ptr, _size, size, _usage);
        if (ptr == nullptr && size != 0) {
            return false;
        }
        _ptr  = ptr;
        _size = size;
        return true;
    }

    void TempBuffer::reset()
    {
        if (_ptr != nullptr && _alloc != nullptr) {
            _alloc->deallocate(_ptr);
        }
        _alloc = nullptr;
        _ptr   = nullptr;
        _size  = 0;
        _usage = usage_t::temp;
    }

    void* TempBuffer::release()
    {
        void* p = _ptr;
        _alloc  = nullptr;
        _ptr    = nullptr;
        _size   = 0;
        _usage  = usage_t::temp;
        return p;
    }

    Allocator& defaultAllocator()
    {
        return getM5_Hal().Memory;
    }

    }  // namespace memory
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V1_MEMORY_ALLOCATOR_INL_
