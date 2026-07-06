// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_BLOCK_HPP_
#define M5_HAL_DATA_BLOCK_HPP_

#include "../data.hpp"
#include "../memory/allocator.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::data {

// A Source backed by a queue of fixed-size memory blocks (typically
// TempBuffer blocks of 256 bytes). Each block holds one frame built
// by frame::buildDataFrame(). Blocks are consumed front-to-back;
// when a block is fully read, it is automatically returned to the
// Allocator that owns it.
//
// Zero-copy: buildDataFrame writes directly into a block, and the
// BlockSource hands that same memory out via peek() — no intermediate
// buffer or memcpy.

class BlockSource : public Source {
public:
    static constexpr size_t kMaxBlocks = 16;

    BlockSource() = default;

    explicit BlockSource(memory::Allocator& alloc) : _alloc{&alloc}
    {
    }

    ~BlockSource()
    {
        releaseAll();
    }

    BlockSource(const BlockSource&)            = delete;
    BlockSource& operator=(const BlockSource&) = delete;
    BlockSource(BlockSource&&)                 = delete;
    BlockSource& operator=(BlockSource&&)      = delete;

    void setAllocator(memory::Allocator& alloc)
    {
        _alloc = &alloc;
    }

    bool addBlock(uint8_t* block, size_t valid_len)
    {
        if (_count >= kMaxBlocks || block == nullptr || valid_len == 0) {
            return false;
        }
        _blocks[_count] = {block, valid_len, 0};
        ++_count;
        return true;
    }

    size_t blockCount() const
    {
        return _count;
    }

    //! Monotonic count of consumed (freed) blocks over this source's lifetime.
    //! Feeds the credit notifier's "receiver made progress" signal; unlike an
    //! allocator-wide free counter it cannot be advanced by the sender's own
    //! outgoing frames (which would self-trigger an endless Credit loop).
    size_t releasedTotal() const
    {
        return _released_total;
    }

    size_t totalBuffered() const
    {
        size_t total = 0;
        for (size_t i = 0; i < _count; ++i) {
            total += _blocks[i].valid - _blocks[i].cursor;
        }
        return total;
    }

    void releaseAll()
    {
        for (size_t i = 0; i < _count; ++i) {
            if (_alloc != nullptr && _blocks[i].data != nullptr) {
                _alloc->deallocate(_blocks[i].data);
                ++_released_total;
            }
            _blocks[i] = {};
        }
        _count = 0;
    }

    // ---- Source interface ----

    result_t<ConstDataSpan> peek(size_t max_len) override
    {
        while (_count > 0 && _blocks[0].cursor >= _blocks[0].valid) {
            consumeFront();
        }
        if (_count == 0) {
            return ConstDataSpan{};
        }
        auto& front   = _blocks[0];
        size_t remain = front.valid - front.cursor;
        size_t take   = remain < max_len ? remain : max_len;
        return ConstDataSpan{front.data + front.cursor, take};
    }

    result_t<void> advance(size_t N) override
    {
        while (N > 0 && _count > 0) {
            auto& front   = _blocks[0];
            size_t remain = front.valid - front.cursor;
            if (N < remain) {
                front.cursor += N;
                N = 0;
            } else {
                N -= remain;
                consumeFront();
            }
        }
        return {};
    }

    bool eof() const override
    {
        for (size_t i = 0; i < _count; ++i) {
            if (_blocks[i].cursor < _blocks[i].valid) {
                return false;
            }
        }
        return true;
    }

private:
    struct Block {
        uint8_t* data = nullptr;
        size_t valid  = 0;
        size_t cursor = 0;
    };

    void consumeFront()
    {
        if (_count == 0) {
            return;
        }
        if (_alloc != nullptr && _blocks[0].data != nullptr) {
            _alloc->deallocate(_blocks[0].data);
            ++_released_total;
        }
        for (size_t i = 1; i < _count; ++i) {
            _blocks[i - 1] = _blocks[i];
        }
        _blocks[_count - 1] = {};
        --_count;
    }

    memory::Allocator* _alloc = nullptr;
    Block _blocks[kMaxBlocks] = {};
    size_t _count             = 0;
    size_t _released_total    = 0;
};

}  // namespace m5::hal::v2::data

#endif
