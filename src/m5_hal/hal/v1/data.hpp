#ifndef M5_HAL_DATA_HPP_
#define M5_HAL_DATA_HPP_

#include "./error.hpp"
#include "./types.hpp"

#include <M5Utility.hpp>

#include <cstddef>
#include <cstdint>

// ============================================================================
// Source / Sink I/O model.
//
// The authoritative spec lives in spec/design/data_io.md. This header only
// declares the abstract bases; concretes live in hal/data/<name>.hpp
// (for example, hal/data/memory.hpp defines `MemorySource` / `MemorySink`).
//
// The parent namespace is `m5::hal::v1::data`. Following the file/namespace
// alignment convention, derivations live in the same namespace.
// ============================================================================

/*!
  @namespace m5::hal::v1::data
  @brief Stream I/O abstractions (Source / Sink) and their span wrappers.
 */
namespace m5::hal::v1::data {

/*!
  @brief Non-owning, read-only `(ptr, size)` wrapper.

  The caller owns the buffer lifetime.
 */
struct ConstDataSpan {
    const uint8_t* data = nullptr;
    size_t size         = 0;

    constexpr ConstDataSpan() = default;
    constexpr ConstDataSpan(const uint8_t* d, size_t s) : data{d}, size{s}
    {
    }
};

/*!
  @brief Non-owning, mutable `(ptr, size)` wrapper.

  Implicitly converts to `ConstDataSpan`. The caller owns the buffer
  lifetime.
 */
struct DataSpan {
    uint8_t* data = nullptr;
    size_t size   = 0;

    constexpr DataSpan() = default;
    constexpr DataSpan(uint8_t* d, size_t s) : data{d}, size{s}
    {
    }
    constexpr operator ConstDataSpan() const
    {
        return {data, size};
    }
};

/*!
  @brief Abstract base for a streaming data Source (TX side).

  Contract (see spec/design/data_io.md §Source for the full version):
  - `peek(max_len)` is idempotent and monotonically non-decreasing.
    Repeated calls without an intervening `advance` keep the prefix
    bytes stable and never shrink the size. The pointer may change
    between calls, but the content is consistent.
  - The span returned by `peek` is borrowed until the next `peek` or
    `advance` call.
  - `peek` returning a zero-length span signals end-of-stream
    (`eof()` also becomes true). Because of that, `peek(0)` is a
    contract violation (`max_len` must be >= 1); the behavior is
    derivation-defined.
  - `advance(N)` is an independent cursor operation, unrelated to the
    presence or size of any prior `peek`. Discarding bytes by repeated
    `advance` is a supported usage.
  - When `advance` is asked to skip past not-yet-arrived data,
    streaming derivations queue the request and consume it on arrival;
    memory-backed derivations drop the excess and move to end-of-stream.

  Error handling (see spec/design/data_io.md §error path):
  - `peek` / `advance` return `expected<..., error_t>`. This is the
    abstract contract; it anticipates future streaming derivations
    (TCP/UDP/network ring buffers, DMA, remote bus, ...) that produce
    real I/O errors. Today's synchronous memory-backed derivations
    (`MemorySource` / `LimitedSource`) do not return errors, but that
    is a property of the derivation, not the abstract contract.
  - Callers MUST check `has_value()` on both APIs. "I know the
    derivation doesn't fail" is not a license to skip the check.
  - Derivation authors MUST document, per derivation, the error
    conditions and the cursor state after an error.
 */
class Source {
public:
    virtual ~Source() = default;

    virtual m5::stl::expected<ConstDataSpan, m5::hal::v1::error::error_t> peek(size_t max_len) = 0;
    virtual m5::stl::expected<void, m5::hal::v1::error::error_t> advance(size_t N)             = 0;
    virtual bool eof() const                                                                   = 0;
};

/*!
  @brief Abstract base for a streaming data Sink (RX side).

  Contract (see spec/design/data_io.md §Sink for the full version):
  - `reserve(max_len)` is idempotent and monotonically non-decreasing
    (mirrors `Source::peek`).
  - The span returned by `reserve` is borrowed until the next
    `reserve` or `commit` call.
  - `reserve` returning a zero-length span signals end-of-writes
    (`closed()` also becomes true). Because of that, `reserve(0)` is a
    contract violation (`max_len` must be >= 1); the behavior is
    derivation-defined.
  - `commit(N)` reports the number of bytes written into the most
    recently reserved span. The caller MUST keep `N <= reserved size`.
  - Contract violations (oversized commit, commit without reserve,
    ...) are implementation-defined and generally undefined behavior.
    Derivations are not required to validate sizes (a debug assert is
    recommended but optional).

  Error handling (see spec/design/data_io.md §error path):
  - `reserve` / `commit` return `expected<..., error_t>`. As with
    `Source`, this is the abstract contract aimed at future streaming
    derivations (network ring buffers, DMA-backed sinks, remote bus,
    ...) that can produce real I/O errors. Today's synchronous
    memory-backed derivations (`MemorySink` / `LimitedSink`) do not
    return errors, but that is again a derivation property.
  - Callers MUST check `has_value()` on both APIs.
  - Derivation authors MUST document the error conditions and the
    post-error cursor state.
 */
class Sink {
public:
    virtual ~Sink() = default;

    virtual m5::stl::expected<DataSpan, m5::hal::v1::error::error_t> reserve(size_t max_len) = 0;
    virtual m5::stl::expected<void, m5::hal::v1::error::error_t> commit(size_t N)            = 0;
    virtual bool closed() const                                                              = 0;
};

}  // namespace m5::hal::v1::data

#endif
