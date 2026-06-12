// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_MEMORY_HPP_
#define M5_HAL_DATA_MEMORY_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

// Memory-backed Source / Sink concretes. Authoritative spec lives at
// spec/design/data_io.md (the Source contract / the Sink contract).
// The parent namespace `m5::hal::v1::data` is declared in hal/data.hpp;
// concrete derivations share that namespace so callers can refer to
// both `Source` / `Sink` and `MemorySource` / `MemorySink` without
// extra qualification.
namespace m5::hal::v1::data {

/*!
  @brief Source that yields a fixed memory range.

  Holds an internal cursor: `peek` is idempotent and `advance` moves
  the cursor. When `advance(N)` is asked to step past the end, the
  excess is discarded and the source transitions to end-of-stream
  (memory-backed sources do not queue skips — see the spec).

  Error path: this derivation currently never returns an error
  (`peek` / `advance` always carry a value). Callers MUST NOT use
  that as a license to skip the check — a generic API such as
  `Bus::transfer` may receive a future streaming derivation; see
  spec/design/data_io.md, error-path responsibilities.
 */
class MemorySource : public Source {
public:
    MemorySource() = default;
    explicit MemorySource(ConstDataSpan data) : _data{data}
    {
    }

    m5::hal::v1::result_t<ConstDataSpan> peek(size_t max_len) override
    {
        size_t remain = (_cursor < _data.size) ? (_data.size - _cursor) : 0;
        size_t take   = std::min(max_len, remain);
        return ConstDataSpan{_data.data + _cursor, take};
    }

    m5::hal::v1::result_t<void> advance(size_t N) override
    {
        size_t remain = (_cursor < _data.size) ? (_data.size - _cursor) : 0;
        _cursor += std::min(N, remain);
        return {};
    }

    bool eof() const override
    {
        return _cursor >= _data.size;
    }

    /*! @brief Always true: the content is fixed at construction, so the
        peekable bytes are final even while some remain unconsumed. */
    bool closed() const override
    {
        return true;
    }

private:
    ConstDataSpan _data;
    size_t _cursor = 0;
};

/*!
  @brief Sink that writes into a fixed memory range.

  `reserve` lends out the remaining free area as a borrowed `DataSpan`;
  `commit` advances the cursor. Contract violations (e.g.
  `commit(N)` with `N` greater than the previously reserved size) are
  undefined behavior per the spec, but for safety this implementation
  clamps the cursor to `_buf.size` so it cannot overrun the buffer.
  Derivations that want stricter checking can add a debug assert.

  Error path: like `MemorySource`, this derivation currently never
  returns an error. Callers MUST NOT skip the check — see
  spec/design/data_io.md, error-path responsibilities.
 */
class MemorySink : public Sink {
public:
    MemorySink() = default;
    explicit MemorySink(DataSpan buf) : _buf{buf}
    {
    }

    m5::hal::v1::result_t<DataSpan> reserve(size_t max_len) override
    {
        size_t remain = (_cursor < _buf.size) ? (_buf.size - _cursor) : 0;
        size_t take   = std::min(max_len, remain);
        return DataSpan{_buf.data + _cursor, take};
    }

    m5::hal::v1::result_t<void> commit(size_t N) override
    {
        size_t remain = (_cursor < _buf.size) ? (_buf.size - _cursor) : 0;
        _cursor += std::min(N, remain);
        return {};
    }

    bool closed() const override
    {
        return _cursor >= _buf.size;
    }

    /*! @brief Total bytes committed so far (the encoded length when used as a build target). */
    size_t written() const
    {
        return _cursor;
    }

private:
    DataSpan _buf;
    size_t _cursor = 0;
};

}  // namespace m5::hal::v1::data

#endif
