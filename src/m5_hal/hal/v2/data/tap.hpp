// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_TAP_HPP_
#define M5_HAL_DATA_TAP_HPP_

#include "stream.hpp"

#include <cstddef>

// Observation-mirror decorators for StreamReader / StreamWriter. Authoritative
// spec lives at spec/design/data_io.md (§Tap decorators). The parent
// namespace `m5::hal::v2::data` is declared in hal/data.hpp.
//
// TapReader / TapWriter decorate a StreamReader / StreamWriter and replicate
// the bytes that actually flowed through the primary path into an observer
// (`StreamWriter* mirror`). The primary use case is host-side wire capture
// (posix `WireDumpWriter`), but the decorator itself is transport-agnostic:
// any StreamWriter can serve as a mirror.
namespace m5::hal::v2::data {

/*!
  @brief `StreamReader` decorator that mirrors bytes read through `inner`
         into an observer `StreamWriter`.

  Pure pass-through when `mirror == nullptr`. Mirrors only successful reads
  that actually returned bytes (`n > 0`); a timeout (`n == 0`) or an error
  is never forwarded. The mirror's own return value and errors are ignored
  entirely - it is an observer, not a consumer, and can never affect the
  primary read result. The mirror must not block: it sits on the hot
  transfer path (this is a documented contract, not enforced).
 */
class TapReader : public StreamReader {
public:
    TapReader(StreamReader& inner, StreamWriter* mirror) : _inner(inner), _mirror(mirror)
    {
    }

    m5::hal::v2::result_t<size_t> read(DataSpan dst) override
    {
        auto r = _inner.read(dst);
        if (_mirror != nullptr && r.has_value() && r.value() > 0) {
            (void)_mirror->write(ConstDataSpan{dst.data, r.value()});
        }
        return r;
    }

    m5::hal::v2::result_t<size_t> readableBytes() override
    {
        return _inner.readableBytes();
    }

private:
    StreamReader& _inner;
    StreamWriter* _mirror;
};

/*!
  @brief `StreamWriter` decorator that mirrors bytes accepted by `inner`
         into an observer `StreamWriter`.

  Pure pass-through when `mirror == nullptr`. Mirrors exactly the number of
  bytes `inner` accepted, so a short write (write timeout) replicates only
  the accepted prefix - never the requested count. The mirror's own return
  value and errors are ignored entirely - it is an observer, not a
  consumer, and can never affect the primary write result. The mirror must
  not block: it sits on the hot transfer path (this is a documented
  contract, not enforced).
 */
class TapWriter : public StreamWriter {
public:
    TapWriter(StreamWriter& inner, StreamWriter* mirror) : _inner(inner), _mirror(mirror)
    {
    }

    m5::hal::v2::result_t<size_t> write(ConstDataSpan src) override
    {
        auto r = _inner.write(src);
        if (_mirror != nullptr && r.has_value() && r.value() > 0) {
            (void)_mirror->write(ConstDataSpan{src.data, r.value()});
        }
        return r;
    }

private:
    StreamWriter& _inner;
    StreamWriter* _mirror;
};

}  // namespace m5::hal::v2::data

#endif
