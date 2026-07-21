// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdlib>

constexpr unsigned MALLOC_CAP_DMA = 1;

inline void* heap_caps_malloc(size_t size, unsigned)
{
    return std::malloc(size);
}

inline void heap_caps_free(void* ptr)
{
    std::free(ptr);
}
