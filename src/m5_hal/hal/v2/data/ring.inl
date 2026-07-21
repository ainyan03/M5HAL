// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_DATA_RING_INL_
#define M5_HAL_HAL_V2_DATA_RING_INL_

#include "ring.hpp"

namespace m5::hal::v2::data {

RingFIFO::RingFIFO(uint8_t* buf, size_t capacity) : _buf{buf}, _capacity{capacity}
{
}

size_t RingFIFO::buffered() const
{
    return _buffered.load(std::memory_order_acquire);
}

size_t RingFIFO::free() const
{
    return _capacity - _buffered.load(std::memory_order_acquire);
}

size_t RingFIFO::capacity() const
{
    return _capacity;
}

void RingFIFO::reset()
{
    _head.store(0, std::memory_order_relaxed);
    _tail.store(0, std::memory_order_relaxed);
    _buffered.store(0, std::memory_order_relaxed);
    _sink.clearReserved();
}

void RingFIFO::setBuf(uint8_t* buf, size_t cap)
{
    _buf      = buf;
    _capacity = cap;
    reset();
}

Source& RingFIFO::source()
{
    return _source;
}

Sink& RingFIFO::sink()
{
    return _sink;
}

RingFIFO::SourceView::SourceView(RingFIFO& owner) : _owner{owner}
{
}

m5::hal::v2::result_t<ConstDataSpan> RingFIFO::SourceView::peek(size_t max_len)
{
    const size_t buf = _owner._buffered.load(std::memory_order_acquire);
    if (_owner._buf == nullptr || buf == 0) {
        return ConstDataSpan{};
    }
    const size_t tail = _owner._tail.load(std::memory_order_relaxed);
    size_t contig     = _owner._capacity - tail;
    if (contig > buf) {
        contig = buf;
    }
    size_t len = std::min(contig, max_len);
    return ConstDataSpan{_owner._buf + tail, len};
}

m5::hal::v2::result_t<void> RingFIFO::SourceView::advance(size_t N)
{
    // N == 0 must return before the modulo below: an unbound ring
    // (capacity 0) would otherwise reach `% 0`. Any N > 0 on an unbound
    // ring is rejected by the buffered check (buffered is always 0).
    if (N == 0) {
        return {};
    }
    const size_t buf = _owner._buffered.load(std::memory_order_relaxed);
    if (N > buf) {
        // Bounded-FIFO carve-out of the Source contract (data.hpp):
        // over-skips are rejected, not queued as StreamSource does.
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    const size_t tail = _owner._tail.load(std::memory_order_relaxed);
    _owner._tail.store((tail + N) % _owner._capacity, std::memory_order_relaxed);
    _owner._buffered.fetch_sub(N, std::memory_order_release);
    return {};
}

bool RingFIFO::SourceView::eof() const
{
    return _owner._buf == nullptr;
}

RingFIFO::SinkView::SinkView(RingFIFO& owner) : _owner{owner}
{
}

m5::hal::v2::result_t<DataSpan> RingFIFO::SinkView::reserve(size_t max_len)
{
    if (_owner._buf == nullptr) {
        return DataSpan{};
    }
    const size_t buf = _owner._buffered.load(std::memory_order_acquire);
    size_t avail     = _owner._capacity - buf;
    if (avail == 0) {
        return DataSpan{};
    }
    const size_t head = _owner._head.load(std::memory_order_relaxed);
    size_t contig     = _owner._capacity - head;
    if (contig > avail) {
        contig = avail;
    }
    size_t len = std::min(contig, max_len);
    _reserved  = len;
    return DataSpan{_owner._buf + head, len};
}

m5::hal::v2::result_t<void> RingFIFO::SinkView::commit(size_t N)
{
    // N == 0 ("wrote nothing") must return before the modulo below: an
    // unbound ring (capacity 0) would otherwise reach `% 0`. Any N > 0
    // on an unbound ring is rejected by the _reserved check (reserve()
    // never lends bytes while unbound).
    if (N == 0) {
        _reserved = 0;
        return {};
    }
    const size_t buf = _owner._buffered.load(std::memory_order_relaxed);
    if (N > _reserved || N > (_owner._capacity - buf)) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::BUFFER_OVERFLOW);
    }
    const size_t head = _owner._head.load(std::memory_order_relaxed);
    _owner._head.store((head + N) % _owner._capacity, std::memory_order_relaxed);
    _owner._buffered.fetch_add(N, std::memory_order_release);
    _reserved = 0;
    return {};
}

bool RingFIFO::SinkView::closed() const
{
    return _owner._buf == nullptr;
}

}  // namespace m5::hal::v2::data

#endif  // M5_HAL_HAL_V2_DATA_RING_INL_
