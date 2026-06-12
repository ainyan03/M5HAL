// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_STREAM_HPP_
#define M5_HAL_DATA_STREAM_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

// Minimal byte-stream interfaces (`StreamReader` / `StreamWriter`) and
// the adapters (`StreamSource` / `StreamSink`) that lift them into the
// Source / Sink model. Authoritative spec lives at spec/design/data_io.md
// (§Stream adapters). The parent namespace `m5::hal::v1::data` is
// declared in hal/data.hpp.
//
// Use cases:
//   - Feed a frame codec (or any Source consumer) from a UART RX
//     accessor: `StreamSource src{&dev.rx(), DataSpan{buf, sizeof buf}};`
//   - Let a producer (frame encoder, ...) push into a UART TX accessor
//     through the Sink contract: `StreamSink snk{&dev.tx(), ...};`
//   - Any future transport (TCP, remote, ...) that implements the two
//     reader methods / one writer method gets the same adapters for free.
//
// Why adapters instead of making accessors derive from Source/Sink:
// `Source::peek` is a borrowing API (repeated peeks keep the prefix
// stable), which requires a scratch buffer the underlying consume-only
// stream cannot provide. Keeping that buffer (and the non-trivial
// contract: timeout-vs-eof, skip reservations) in one decorator avoids
// per-transport re-implementations and keeps accessors buffer-free.
namespace m5::hal::v1::data {

/*!
  @brief Abstract pull-side byte stream (the raw capability behind
         `StreamSource`).

  - `read(dst)` blocks according to the implementation's own timeout
    policy (for UART accessors: first-byte / inter-byte timeouts from
    the access config) and returns the number of bytes stored. Zero
    means "nothing arrived in time"; it does NOT distinguish a timeout
    from a peer hang-up.
  - `readableBytes()` returns the number of bytes that can be read
    without blocking.
 */
struct StreamReader {
    virtual ~StreamReader() = default;

    virtual m5::hal::v1::result_t<size_t> read(DataSpan dst)  = 0;
    virtual m5::hal::v1::result_t<size_t> readableBytes(void) = 0;
};

/*!
  @brief Abstract push-side byte stream (the raw capability behind
         `StreamSink`).

  `write(src)` blocks according to the implementation's own timeout
  policy and returns the number of bytes accepted (possibly fewer than
  `src.size` on a write timeout).
 */
struct StreamWriter {
    virtual ~StreamWriter() = default;

    virtual m5::hal::v1::result_t<size_t> write(ConstDataSpan src) = 0;
};

/*!
  @brief Read from `reader` until a delimiter byte is stored.

  Fills `dst` one byte at a time until `delim` is stored, `dst` is
  full, or the reader's own timeout policy expires (a zero-byte read).
  Line-oriented protocols (NMEA sentences, AT command responses, ...)
  are the target: the loop, the bound, and the termination test live
  here once instead of in every sketch.

  Contract:
  - The return value is the number of bytes stored. **The delimiter is
    included** when it arrived, so completion is decided from the data
    alone: `n > 0 && dst.data[n - 1] == delim` = a complete line,
    anything else = a partial read (timeout or a full `dst`). This is
    deliberately unlike Arduino's `readBytesUntil`, which strips the
    delimiter and leaves complete and cut-off lines indistinguishable.
  - A timeout is NOT an error: the partial bytes are stored and their
    count is returned (possibly 0). Timeout pacing follows the
    reader's policy (for UART accessors: first-byte / inter-byte
    timeouts from the access config).
  - A hard reader error is returned as that error; bytes stored before
    the failure are in `dst` but the caller should not trust them.

  Works on any `StreamReader`; `uart::RxAccessor::readUntil` wraps this
  in a single RX channel-lock window.
 */
inline m5::hal::v1::result_t<size_t> readUntil(StreamReader& reader, uint8_t delim, DataSpan dst)
{
    if (dst.data == nullptr || dst.size == 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
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

/*!
  @brief Adapter that exposes a `StreamReader` as a `Source`.

  The caller supplies a non-owning scratch buffer; peeked-but-not-yet-
  advanced bytes live there, which is what makes the borrowing `peek`
  contract (monotonic non-decreasing prefix) possible on top of a
  consume-only stream. `peek` can return at most `scratch.size` bytes
  per call; consumers must be written against short peeks anyway.

  Blocking policy:
  - request not yet satisfied (`max_len` > buffered): one blocking
    `read` for the missing bytes, bounded by the reader's own timeout
    policy (UART: first-byte / inter-byte timeouts). After the timeout
    the caller gets what did arrive - a short peek - and a still-empty
    buffer reports `TIMEOUT_ERROR`. That error is a recoverable
    condition - the caller may simply peek again - and NOT
    end-of-stream (an empty span would mean EOF per the Source
    contract, which a serial line never reaches on its own).
  - request already buffered: returns immediately without touching the
    reader. Callers that must never block check `readableBytes()`
    first and peek no more than that.

  `advance` past the buffered bytes stores the excess as a skip
  reservation (spec/design/data_io.md §Source): readable bytes are
  discarded immediately, the rest is consumed automatically by later
  `peek` / `advance` calls.

  `eof()` is true only for a detached adapter (null reader), and
  `closed()` keeps its default (= eof()): an attached stream may always
  produce more bytes. Stream errors, including timeouts, are reported
  through the error path and never latch EOF.
 */
class StreamSource : public Source {
public:
    StreamSource(StreamReader& reader, DataSpan scratch) : _reader{&reader}, _scratch{scratch}
    {
    }
    StreamSource(StreamReader* reader, DataSpan scratch) : _reader{reader}, _scratch{scratch}
    {
    }

    m5::hal::v1::result_t<ConstDataSpan> peek(size_t max_len) override
    {
        if (_reader == nullptr) {
            return ConstDataSpan{};
        }
        if (_scratch.data == nullptr || _scratch.size == 0) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
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
                return m5::stl::make_unexpected(m5::hal::v1::error::error_t::TIMEOUT_ERROR);
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
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::TIMEOUT_ERROR);
        }
        return ConstDataSpan{_scratch.data, std::min(buffered(), max_len)};
    }

    m5::hal::v1::result_t<void> advance(size_t N) override
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

    /*! @brief True only when the adapter was constructed detached
        (null reader); a live stream never reaches EOF on its own. */
    bool eof() const override
    {
        return _reader == nullptr;
    }

    /*! @brief Bytes currently buffered in scratch (debug / verification). */
    size_t buffered(void) const
    {
        return _filled - _head;
    }

    /*! @brief Skip bytes still owed to `advance` (debug / verification). */
    size_t pendingSkip(void) const
    {
        return _pending_skip;
    }

    /*!
      @brief Drop everything buffered in the adapter (and any pending
             skip reservation).

      Connection-start flush: discards stale bytes (e.g. a peer's boot
      noise) so the next `peek` starts from fresh input. Only the
      adapter's own buffer is affected — bytes still queued in the
      underlying transport are NOT touched; flush those at the
      transport level (e.g. `tcflush`) when a full flush is needed.
      Any span borrowed from a previous `peek` is invalidated.
     */
    void discardBuffered(void)
    {
        _head = _filled = 0;
        _pending_skip   = 0;
    }

private:
    // Consume the skip reservation. `blocking` selects one reader-paced
    // blocking read per missing chunk (peek path) vs. draining only what
    // is already readable (advance path). The scratch buffer doubles as
    // the discard area: a reservation only exists while the buffer is
    // empty (`advance` always consumes buffered bytes first).
    m5::hal::v1::result_t<void> drainSkip(bool blocking)
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

    StreamReader* _reader = nullptr;
    DataSpan _scratch{};
    size_t _head         = 0;
    size_t _filled       = 0;
    size_t _pending_skip = 0;
};

/*!
  @brief Adapter that exposes a `StreamWriter` as a `Sink`.

  `reserve` lends out the caller-supplied scratch buffer (stable across
  repeated calls, capped at `scratch.size`); `commit(N)` pushes the
  first N scratch bytes through `StreamWriter::write`. A short write -
  the writer accepted fewer bytes than committed (write timeout, ...) -
  is reported as `IO_ERROR`.

  `closed()` is true only for a detached adapter (null writer);
  committing while detached returns `CLOSED` instead of silently
  dropping bytes.
 */
class StreamSink : public Sink {
public:
    StreamSink(StreamWriter& writer, DataSpan scratch) : _writer{&writer}, _scratch{scratch}
    {
    }
    StreamSink(StreamWriter* writer, DataSpan scratch) : _writer{writer}, _scratch{scratch}
    {
    }

    m5::hal::v1::result_t<DataSpan> reserve(size_t max_len) override
    {
        if (_writer == nullptr) {
            return DataSpan{};
        }
        if (_scratch.data == nullptr || _scratch.size == 0) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        return DataSpan{_scratch.data, std::min(max_len, _scratch.size)};
    }

    m5::hal::v1::result_t<void> commit(size_t N) override
    {
        if (_writer == nullptr) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::CLOSED);
        }
        if (N > _scratch.size) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::BUFFER_OVERFLOW);
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
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::TIMEOUT_ERROR);
        }
        return {};
    }

    bool closed() const override
    {
        return _writer == nullptr;
    }

private:
    StreamWriter* _writer = nullptr;
    DataSpan _scratch{};
};

}  // namespace m5::hal::v1::data

#endif
