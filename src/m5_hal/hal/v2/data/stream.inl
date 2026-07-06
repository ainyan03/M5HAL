// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_DATA_STREAM_INL_
#define M5_HAL_HAL_V2_DATA_STREAM_INL_

#include "stream.hpp"

namespace m5::hal::v2::data {

m5::hal::v2::result_t<size_t> readUntil(StreamReader& reader, uint8_t delim, DataSpan dst)
{
    if (dst.data == nullptr || dst.size == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    size_t total = 0;
    while (total < dst.size) {
        uint8_t b = 0;
        auto r    = reader.read(DataSpan{&b, 1});
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        if (r.value() == 0) {
            break;  // the reader's timeout expired: a partial read is normal
        }
        dst.data[total++] = b;
        if (b == delim) {
            break;
        }
    }
    return total;
}

StreamSource::StreamSource(StreamReader& reader, DataSpan scratch) : _reader{&reader}, _scratch{scratch}
{
}

StreamSource::StreamSource(StreamReader* reader, DataSpan scratch) : _reader{reader}, _scratch{scratch}
{
}

m5::hal::v2::result_t<ConstDataSpan> StreamSource::peek(size_t max_len)
{
    if (_reader == nullptr) {
        return ConstDataSpan{};
    }
    if (_scratch.data == nullptr || _scratch.size == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    // A pending skip means the buffer is empty and the cursor sits
    // beyond data that has not arrived yet; it must be consumed
    // before any byte can be exposed.
    if (_pending_skip > 0) {
        auto drained = drainSkip(true);
        if (!drained.has_value()) {
            return m5::stl::make_unexpected(drained.error());
        }
        if (_pending_skip > 0) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
    }
    // Compaction only ever runs right after `advance` consumed a
    // prefix, so the bytes a previous `peek` lent out are gone and
    // the monotonic-prefix guarantee is preserved.
    if (_head > 0) {
        ::memmove(_scratch.data, _scratch.data + _head, _filled - _head);
        _filled -= _head;
        _head = 0;
    }
    // Top up whenever the request is not yet satisfied: one blocking
    // read, bounded by the reader's own timeout policy. After the
    // timeout the caller gets whatever did arrive (a short peek);
    // only a still-empty buffer reports TIMEOUT_ERROR. Callers that
    // must not block check `readableBytes()` first.
    const size_t want = std::min(max_len, _scratch.size);
    if (want > buffered() && _filled < _scratch.size) {
        auto got = _reader->read(DataSpan{_scratch.data + _filled, _scratch.size - _filled});
        if (!got.has_value()) {
            return m5::stl::make_unexpected(got.error());
        }
        _filled += got.value();
    }
    if (buffered() == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    }
    return ConstDataSpan{_scratch.data, std::min(buffered(), max_len)};
}

m5::hal::v2::result_t<void> StreamSource::advance(size_t N)
{
    if (_reader == nullptr) {
        return {};
    }
    const size_t take = std::min(N, buffered());
    _head += take;
    if (_head == _filled) {
        _head = _filled = 0;
    }
    if (N > take) {
        _pending_skip += N - take;
    }
    if (_pending_skip > 0) {
        return drainSkip(false);
    }
    return {};
}

bool StreamSource::eof() const
{
    return _reader == nullptr;
}

size_t StreamSource::buffered(void) const
{
    return _filled - _head;
}

size_t StreamSource::pendingSkip(void) const
{
    return _pending_skip;
}

void StreamSource::discardBuffered(void)
{
    _head = _filled = 0;
    _pending_skip   = 0;
}

m5::hal::v2::result_t<void> StreamSource::drainSkip(bool blocking)
{
    while (_pending_skip > 0) {
        size_t want = std::min(_pending_skip, _scratch.size);
        if (!blocking) {
            auto avail = _reader->readableBytes();
            if (!avail.has_value()) {
                return m5::stl::make_unexpected(avail.error());
            }
            want = std::min(want, avail.value());
            if (want == 0) {
                return {};
            }
        }
        auto got = _reader->read(DataSpan{_scratch.data, want});
        if (!got.has_value()) {
            return m5::stl::make_unexpected(got.error());
        }
        if (got.value() == 0) {
            return {};
        }
        _pending_skip -= got.value();
    }
    return {};
}

StreamSink::StreamSink(StreamWriter& writer, DataSpan scratch) : _writer{&writer}, _scratch{scratch}
{
}

StreamSink::StreamSink(StreamWriter* writer, DataSpan scratch) : _writer{writer}, _scratch{scratch}
{
}

m5::hal::v2::result_t<DataSpan> StreamSink::reserve(size_t max_len)
{
    if (_writer == nullptr) {
        return DataSpan{};
    }
    if (_scratch.data == nullptr || _scratch.size == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    return DataSpan{_scratch.data, std::min(max_len, _scratch.size)};
}

m5::hal::v2::result_t<void> StreamSink::commit(size_t N)
{
    if (_writer == nullptr) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::CLOSED);
    }
    if (N > _scratch.size) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::BUFFER_OVERFLOW);
    }
    if (N == 0) {
        return {};
    }
    auto wrote = _writer->write(ConstDataSpan{_scratch.data, N});
    if (!wrote.has_value()) {
        return m5::stl::make_unexpected(wrote.error());
    }
    if (wrote.value() != N) {
        // A short write's dominant cause is the writer's own write
        // timeout — classify it as TIMEOUT_ERROR (retryable), symmetric
        // with the read side, instead of a fatal-looking IO_ERROR.
        // NOTE: bytes may sit half-flushed on the wire; the caller
        // decides whether to resync/retransmit.
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    }
    return {};
}

bool StreamSink::closed() const
{
    return _writer == nullptr;
}

}  // namespace m5::hal::v2::data

#endif  // M5_HAL_HAL_V2_DATA_STREAM_INL_
