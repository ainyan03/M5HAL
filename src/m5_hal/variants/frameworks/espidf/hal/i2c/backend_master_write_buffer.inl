#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_WRITE_BUFFER_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_WRITE_BUFFER_INL

#include "../../../../../hal/v1/data.hpp"
#include "../../../../../hal/v1/error.hpp"
#include "../../../../../hal/v1/memory/allocator.hpp"

#include <algorithm>
#include <cstring>

namespace m5::variants::frameworks::espidf::hal::v1::i2c::detail {

class TempWriteBuffer {
public:
    TempWriteBuffer() = default;

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> build(::m5::hal::v1::data::ConstDataSpan prefix,
                                                                 ::m5::hal::v1::data::Source* tx)
    {
        ::m5::hal::v1::data::ConstDataSpan first{};
        if (tx != nullptr && !tx->eof()) {
            auto peeked = tx->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            first = peeked.value();
        }

        if (prefix.size > static_cast<size_t>(-1) - first.size) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        auto reserved = reserve(prefix.size + first.size);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }

        auto appended = append(prefix);
        if (!appended.has_value()) {
            return m5::stl::make_unexpected(appended.error());
        }
        appended = append(first);
        if (!appended.has_value()) {
            return m5::stl::make_unexpected(appended.error());
        }
        if (tx == nullptr) {
            return {};
        }
        if (first.size > 0) {
            auto adv = tx->advance(first.size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
        }

        while (!tx->eof()) {
            auto peeked = tx->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            auto span = peeked.value();
            if (span.size == 0) {
                break;
            }
            auto appended = append(span);
            if (!appended.has_value()) {
                return m5::stl::make_unexpected(appended.error());
            }
            auto adv = tx->advance(span.size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
        }
        return {};
    }

    uint8_t* data() const
    {
        return static_cast<uint8_t*>(_buffer.data());
    }

    size_t size() const
    {
        return _size;
    }

    bool empty() const
    {
        return _size == 0;
    }

private:
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> append(::m5::hal::v1::data::ConstDataSpan span)
    {
        if (span.size == 0) {
            return {};
        }
        if (span.data == nullptr) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        if (span.size > static_cast<size_t>(-1) - _size) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        auto reserved = reserve(_size + span.size);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }
        std::memcpy(data() + _size, span.data, span.size);
        _size += span.size;
        return {};
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> reserve(size_t required)
    {
        if (required <= _capacity) {
            return {};
        }

        const size_t block_size = ::m5::hal::v1::memory::Allocator::tempBlockSize();
        size_t new_capacity     = std::max(roundUp(required, block_size), block_size);
        if (_capacity > 0) {
            const size_t doubled = (_capacity > (static_cast<size_t>(-1) / 2u)) ? required : (_capacity * 2u);
            new_capacity         = std::max(new_capacity, doubled);
        }

        if (!_buffer) {
            _buffer = ::m5::hal::v1::memory::TempBuffer{::m5::hal::v1::memory::defaultAllocator(), new_capacity};
            if (!_buffer) {
                return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
            }
        } else if (!_buffer.reallocate(new_capacity)) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        _capacity = new_capacity;
        return {};
    }

    static size_t roundUp(size_t value, size_t alignment)
    {
        if (value > static_cast<size_t>(-1) - (alignment - 1u)) {
            return value;
        }
        return ((value + alignment - 1u) / alignment) * alignment;
    }

    ::m5::hal::v1::memory::TempBuffer _buffer;
    size_t _size     = 0;
    size_t _capacity = 0;
};

}  // namespace m5::variants::frameworks::espidf::hal::v1::i2c::detail

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_WRITE_BUFFER_INL
