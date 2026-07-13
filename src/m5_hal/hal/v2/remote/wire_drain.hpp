// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_WIRE_DRAIN_HPP_
#define M5_HAL_REMOTE_WIRE_DRAIN_HPP_

#include "../data.hpp"

#include <cstring>

namespace m5::hal::v2::remote::detail {

// ---- drainToSink — shared encoder-output -> wire-sink pump ----------------
//
// Copies bytes out of `out` (typically `MuxFrameEncoder::output()`) into
// `sink` (the physical wire transport) through the standard
// reserve/commit dance, `chunk_max` bytes at a time.
//
// Contract: `out` is advanced only by the number of bytes `sink` actually
// accepted, and draining stops there for this call (the remainder is
// retried, from where it left off, on the next call). A sink whose
// `commit()` is atomic (every concrete Sink except `StreamSink`) either
// accepts the whole chunk or none, so this is equivalent to the naive
// "advance on success, break on failure" loop for those. `StreamSink`'s
// `commit()` can itself perform a short write to the underlying
// transport and report that accepted prefix via
// `Sink::partialCommitAccepted()` even though `commit()` itself returned
// an error; draining this way means bytes that did reach the wire are
// never resent, and bytes that did not are never silently dropped.
inline void drainToSink(data::Source& out, data::Sink& sink, size_t chunk_max = 4096)
{
    while (!out.eof()) {
        auto p = out.peek(chunk_max);
        if (!p.has_value() || p.value().size == 0) {
            break;
        }
        auto rsv = sink.reserve(p.value().size);
        if (!rsv.has_value() || rsv.value().size == 0) {
            break;
        }
        size_t n = rsv.value().size < p.value().size ? rsv.value().size : p.value().size;
        ::memcpy(rsv.value().data, p.value().data, n);
        auto c          = sink.commit(n);
        size_t accepted = c.has_value() ? n : sink.partialCommitAccepted();
        if (accepted > n) {
            accepted = n;  // defensive: never advance past what we peeked
        }
        if (accepted > 0) {
            (void)out.advance(accepted);
        }
        if (!c.has_value()) {
            break;
        }
    }
}

}  // namespace m5::hal::v2::remote::detail

#endif  // M5_HAL_REMOTE_WIRE_DRAIN_HPP_
