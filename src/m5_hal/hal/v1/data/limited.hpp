#ifndef M5_HAL_DATA_LIMITED_HPP_
#define M5_HAL_DATA_LIMITED_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

// Source / Sink decorators that cap the byte count. Authoritative spec
// lives at spec/design/data_io.md (rationale for the Limited decorators, partial transfer).
// The parent namespace `m5::hal::v1::data` is declared in hal/data.hpp.
//
// Use cases:
//   - Send only the first N bytes of a long `MemorySource` or an
//     unbounded streaming Source.
//   - Receive only N bytes into a Sink whose capacity is unknown or
//     unbounded (ring buffer, ...).
//
// Decorator shape: a base Source/Sink pointer plus a remaining-byte
// counter. The wrapper signals `eof()` / `closed()` as soon as the
// counter hits zero. If the base reaches end of stream first, the
// wrapper propagates that immediately (whichever side terminates
// first wins). A null base in the pointer ctor is treated as an
// empty Source/Sink.
//
// Error path: the wrapper generates no errors of its own; if the
// base is a streaming derivation it propagates whatever the base
// returns. Callers MUST still check `has_value()` regardless of the
// base type (see design/data_io.md, error-path responsibilities).
namespace m5::hal::v1::data {

/*!
  @brief Source decorator that exposes at most `max_bytes` from a base
         `Source`.

  After the cap is reached, `eof()` becomes true and further `peek`
  calls return an empty span. Errors from the base are propagated
  verbatim; the decorator never produces an error of its own. Callers
  MUST still check `has_value()` regardless of the base type.
 */
class LimitedSource : public Source {
public:
    LimitedSource(Source& base, size_t max_bytes) : _base{&base}, _remaining{max_bytes}
    {
    }
    LimitedSource(Source* base, size_t max_bytes) : _base{base}, _remaining{max_bytes}
    {
    }

    m5::stl::expected<ConstDataSpan, m5::hal::v1::error::error_t> peek(size_t max_len) override
    {
        size_t want = std::min(max_len, _remaining);
        if (want == 0 || _base == nullptr) {
            return ConstDataSpan{};
        }
        return _base->peek(want);
    }

    m5::stl::expected<void, m5::hal::v1::error::error_t> advance(size_t N) override
    {
        size_t step = std::min(N, _remaining);
        _remaining -= step;
        if (step == 0) {
            return {};
        }
        if (_base == nullptr) {
            return {};
        }
        return _base->advance(step);
    }

    /*! @brief True when the local cap is exhausted, or when the base
        is null / already at EOF. */
    bool eof() const override
    {
        return _remaining == 0 || _base == nullptr || _base->eof();
    }

    /*! @brief Bytes still allowed through the cap (debug / verification). */
    size_t remaining() const
    {
        return _remaining;
    }

private:
    Source* _base     = nullptr;
    size_t _remaining = 0;
};

/*!
  @brief Sink decorator that accepts at most `max_bytes` into a base
         `Sink`.

  After the cap is reached, `closed()` becomes true and further
  `reserve` calls return an empty span. If the base closes first the
  wrapper propagates immediately. Errors from the base are propagated
  verbatim; the decorator never produces an error of its own. Callers
  MUST still check `has_value()` regardless of the base type.
 */
class LimitedSink : public Sink {
public:
    LimitedSink(Sink& base, size_t max_bytes) : _base{&base}, _remaining{max_bytes}
    {
    }
    LimitedSink(Sink* base, size_t max_bytes) : _base{base}, _remaining{max_bytes}
    {
    }

    m5::stl::expected<DataSpan, m5::hal::v1::error::error_t> reserve(size_t max_len) override
    {
        size_t want = std::min(max_len, _remaining);
        if (want == 0 || _base == nullptr) {
            return DataSpan{};
        }
        return _base->reserve(want);
    }

    m5::stl::expected<void, m5::hal::v1::error::error_t> commit(size_t N) override
    {
        size_t step = std::min(N, _remaining);
        _remaining -= step;
        if (step == 0) {
            return {};
        }
        if (_base == nullptr) {
            return {};
        }
        return _base->commit(step);
    }

    bool closed() const override
    {
        return _remaining == 0 || _base == nullptr || _base->closed();
    }

    size_t remaining() const
    {
        return _remaining;
    }

private:
    Sink* _base       = nullptr;
    size_t _remaining = 0;
};

}  // namespace m5::hal::v1::data

#endif
