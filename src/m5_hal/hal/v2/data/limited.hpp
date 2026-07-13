// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_LIMITED_HPP_
#define M5_HAL_DATA_LIMITED_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

// Source / Sink decorators that cap the byte count. Authoritative spec
// lives at spec/design/data_io.md (rationale for the Limited decorators, partial transfer).
// The parent namespace `m5::hal::v2::data` is declared in hal/data.hpp.
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
// first wins).
//
// Two ctors, by intent:
//   - reference ctor (`Source&` / `Sink&`): the base is REQUIRED. Use
//     this when a real base must be present.
//   - pointer ctor (`Source*` / `Sink*`): the base is OPTIONAL. A null
//     base is a SUPPORTED, intentional way to express an EMPTY
//     Source/Sink — peek/reserve yield an empty span and eof()/closed()
//     are true immediately (a no-op stream). This is by design, not a
//     defensive fallback; pass a non-null pointer (or use the reference
//     ctor) when a base is mandatory.
//
// Error path: the wrapper generates no errors of its own; if the
// base is a streaming derivation it propagates whatever the base
// returns. Callers MUST still check `has_value()` regardless of the
// base type (see design/data_io.md, error-path responsibilities).
namespace m5::hal::v2::data {

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
    /*! @brief Required-base ctor. */
    LimitedSource(Source& base, size_t max_bytes) : _base{&base}, _remaining{max_bytes}
    {
    }
    /*! @brief Optional-base ctor: a null `base` is an intentional empty Source (immediate eof). */
    LimitedSource(Source* base, size_t max_bytes) : _base{base}, _remaining{max_bytes}
    {
    }

    m5::hal::v2::result_t<ConstDataSpan> peek(size_t max_len) override
    {
        size_t want = std::min(max_len, _remaining);
        if (want == 0 || _base == nullptr) {
            return ConstDataSpan{};
        }
        return _base->peek(want);
    }

    m5::hal::v2::result_t<void> advance(size_t N) override
    {
        size_t step = std::min(N, _remaining);
        if (step == 0 || _base == nullptr) {
            _remaining -= step;
            return {};
        }
        auto r = _base->advance(step);
        // Charge the cap only when the base accepted the advance: charging
        // up front would bill the budget again on a caller's retry after a
        // base error, silently shrinking the window by bytes that were
        // never consumed. Post-error cursor state is the base's own
        // documented contract; the decorator adds no consumption of its own.
        if (r.has_value()) {
            _remaining -= step;
        }
        return r;
    }

    /*! @brief True when the local cap is exhausted, or when the base
        is null / already at EOF. */
    bool eof() const override
    {
        return _remaining == 0 || _base == nullptr || _base->eof();
    }

    /*! @brief Producer closure: a spent cap is final, and a closed base
        stays closed through the decorator. */
    bool closed() const override
    {
        return _remaining == 0 || _base == nullptr || _base->closed();
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
    /*! @brief Required-base ctor. */
    LimitedSink(Sink& base, size_t max_bytes) : _base{&base}, _remaining{max_bytes}
    {
    }
    /*! @brief Optional-base ctor: a null `base` is an intentional empty Sink (immediately closed). */
    LimitedSink(Sink* base, size_t max_bytes) : _base{base}, _remaining{max_bytes}
    {
    }

    m5::hal::v2::result_t<DataSpan> reserve(size_t max_len) override
    {
        size_t want = std::min(max_len, _remaining);
        if (want == 0 || _base == nullptr) {
            return DataSpan{};
        }
        return _base->reserve(want);
    }

    m5::hal::v2::result_t<void> commit(size_t N) override
    {
        size_t step = std::min(N, _remaining);
        if (step == 0 || _base == nullptr) {
            _remaining -= step;
            return {};
        }
        auto r = _base->commit(step);
        // Charge the budget by what the base actually took: a partially
        // accepting base (StreamSink) may flush a prefix before failing,
        // and charging the full step would leak budget for bytes that
        // never went through.
        _remaining -= r.has_value() ? step : partialCommitAccepted();
        return r;
    }

    size_t partialCommitAccepted() const override
    {
        // Forward the base's accepted prefix so a caller driving this
        // decorator (e.g. remote::detail::drainToSink) advances its
        // upstream by what actually reached the base sink.
        return _base != nullptr ? _base->partialCommitAccepted() : 0;
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

}  // namespace m5::hal::v2::data

#endif
