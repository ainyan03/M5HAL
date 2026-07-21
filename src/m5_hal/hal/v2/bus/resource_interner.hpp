// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_RESOURCE_INTERNER_HPP_
#define M5_HAL_BUS_RESOURCE_INTERNER_HPP_

#include "./resource_key.hpp"
#include "../runtime/runtime.hpp"

#include <cstdlib>
#include <cstring>
#include <limits>
#include <type_traits>

namespace m5::hal::v2::bus {

/*! @brief Type-erased optional path identity table owned by a ResourceDomain. */
class IPathInterner {
public:
    virtual ~IPathInterner()                                                         = default;
    virtual result_t<PathToken> intern(const char* bytes, size_t length)             = 0;
    virtual result_t<void> release(PathToken token)                                  = 0;
    virtual result_t<size_t> copy(PathToken token, char* out, size_t capacity) const = 0;
};

/*! @brief Fixed-capacity exact path table; hash is an accelerator, never identity. */
template <size_t Capacity, size_t MaxPathLength, uint32_t InitialGeneration = 0>
class FixedPathInterner final : public IPathInterner {
public:
    static_assert(Capacity != 0 && Capacity <= std::numeric_limits<uint16_t>::max(), "invalid path capacity");
    static_assert(MaxPathLength != 0, "path storage must not be empty");
    static_assert(InitialGeneration < std::numeric_limits<uint32_t>::max(), "initial generation must be reusable");

    using hash_fn_t = uint32_t (*)(const char*, size_t);

    explicit FixedPathInterner(hash_fn_t hash = &defaultHash) : _hash{hash != nullptr ? hash : &defaultHash}
    {
    }

    result_t<PathToken> intern(const char* bytes, size_t length) override
    {
        if (bytes == nullptr || length == 0 || length > MaxPathLength) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        const uint32_t hash = _hash(bytes, length);
        Guard guard{_mutex};
        size_t free_slot = Capacity;
        for (size_t i = 0; i < Capacity; ++i) {
            Entry& entry = _entries[i];
            if (entry.occupied) {
                if (entry.hash == hash && entry.length == length && std::memcmp(entry.bytes, bytes, length) == 0) {
                    if (entry.references == std::numeric_limits<uint32_t>::max()) {
                        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
                    }
                    ++entry.references;
                    return PathToken{static_cast<uint16_t>(i), 0, entry.generation};
                }
            } else if (free_slot == Capacity && entry.generation != std::numeric_limits<uint32_t>::max()) {
                free_slot = i;
            }
        }
        if (free_slot == Capacity) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        Entry& entry = _entries[free_slot];
        ++entry.generation;
        entry.hash       = hash;
        entry.length     = length;
        entry.references = 1;
        entry.occupied   = true;
        std::memcpy(entry.bytes, bytes, length);
        return PathToken{static_cast<uint16_t>(free_slot), 0, entry.generation};
    }

    result_t<void> release(PathToken token) override
    {
        if (!token.valid() || token.slot >= Capacity) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        Entry& entry = _entries[token.slot];
        if (!entry.occupied || entry.generation != token.generation || entry.references == 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        if (--entry.references == 0) {
            entry.occupied = false;
            entry.length   = 0;
            entry.hash     = 0;
        }
        return {};
    }

    result_t<size_t> copy(PathToken token, char* out, size_t capacity) const override
    {
        if (!token.valid() || token.slot >= Capacity || out == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        const Entry& entry = _entries[token.slot];
        if (!entry.occupied || entry.generation != token.generation) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        if (capacity < entry.length) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        std::memcpy(out, entry.bytes, entry.length);
        return entry.length;
    }

private:
    static uint32_t defaultHash(const char* bytes, size_t length)
    {
        uint32_t value = 2166136261u;
        for (size_t i = 0; i < length; ++i) {
            value ^= static_cast<uint8_t>(bytes[i]);
            value *= 16777619u;
        }
        return value;
    }

    struct Entry {
        char bytes[MaxPathLength];
        size_t length       = 0;
        uint32_t hash       = 0;
        uint32_t generation = InitialGeneration;
        uint32_t references = 0;
        bool occupied       = false;
    };

    using Guard = runtime::MutexGuard;

    Entry _entries[Capacity];
    hash_fn_t _hash;
    mutable runtime::Mutex _mutex;
};

/*! @brief Fixed-capacity exact native-identity table with ABA-safe tokens. */
template <class Identity, size_t Capacity, uint32_t InitialGeneration = 0>
class FixedNativeInterner {
public:
    static_assert(std::is_trivially_copyable<Identity>::value, "native identity must be value-like");
    static_assert(Capacity != 0 && Capacity <= std::numeric_limits<uint16_t>::max(), "invalid native capacity");
    static_assert(InitialGeneration < std::numeric_limits<uint32_t>::max(), "initial generation must be reusable");

    result_t<NativeToken> intern(const Identity& identity)
    {
        Guard guard{_mutex};
        size_t free_slot = Capacity;
        for (size_t i = 0; i < Capacity; ++i) {
            Entry& entry = _entries[i];
            if (entry.occupied) {
                if (entry.identity == identity) {
                    if (entry.references == std::numeric_limits<uint32_t>::max()) {
                        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
                    }
                    ++entry.references;
                    return NativeToken{static_cast<uint16_t>(i), 0, entry.generation};
                }
            } else if (free_slot == Capacity && entry.generation != std::numeric_limits<uint32_t>::max()) {
                free_slot = i;
            }
        }
        if (free_slot == Capacity) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        Entry& entry   = _entries[free_slot];
        entry.identity = identity;
        ++entry.generation;
        entry.references = 1;
        entry.occupied   = true;
        return NativeToken{static_cast<uint16_t>(free_slot), 0, entry.generation};
    }

    result_t<void> release(NativeToken token)
    {
        if (!token.valid() || token.slot >= Capacity) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        Entry& entry = _entries[token.slot];
        if (!entry.occupied || entry.generation != token.generation || entry.references == 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        if (--entry.references == 0) {
            entry.occupied = false;
        }
        return {};
    }

    result_t<Identity> resolve(NativeToken token) const
    {
        if (!token.valid() || token.slot >= Capacity) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        const Entry& entry = _entries[token.slot];
        if (!entry.occupied || entry.generation != token.generation) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        return entry.identity;
    }

private:
    struct Entry {
        Identity identity{};
        uint32_t generation = InitialGeneration;
        uint32_t references = 0;
        bool occupied       = false;
    };
    using Guard = runtime::MutexGuard;

    Entry _entries[Capacity];
    mutable runtime::Mutex _mutex;
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_RESOURCE_INTERNER_HPP_
