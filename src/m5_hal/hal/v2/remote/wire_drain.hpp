// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_WIRE_DRAIN_HPP_
#define M5_HAL_REMOTE_WIRE_DRAIN_HPP_

#include "../data.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace m5::hal::v2::remote::detail {

// ---- drainToSink — shared encoder-output -> wire-sink pump ----------------
//
// Copies bytes out of `out` (typically `MuxFrameEncoder::output()`) into
// `sink` (the physical wire transport) through the standard
// reserve/commit dance, `chunk_max` bytes at a time.
//
// Contract: `out` is advanced only by the number of bytes `sink` actually
// accepted. Temporary lack of input/output space is reported as
// `DrainStop::WouldBlock`; Source/Sink errors are propagated unchanged. A
// sink whose
// `commit()` is atomic (every concrete Sink except `StreamSink`) either
// accepts the whole chunk or none, so this is equivalent to the naive
// "advance on success, break on failure" loop for those. `StreamSink`'s
// `commit()` can itself perform a short write to the underlying
// transport and report that accepted prefix via
// `Sink::partialCommitAccepted()` even though `commit()` itself returned
// an error; draining this way means bytes that did reach the wire are
// never resent, and bytes that did not are never silently dropped.
enum class DrainStop : uint8_t {
    Drained,
    WouldBlock,
};

struct DrainProgress {
    size_t accepted_bytes = 0;
    DrainStop stop        = DrainStop::Drained;
};

inline result_t<DrainProgress> drainToSink(data::Source& out, data::Sink& sink, size_t chunk_max = 4096)
{
    if (chunk_max == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    size_t accepted_total = 0;
    while (!out.eof()) {
        auto p = out.peek(chunk_max);
        if (!p.has_value()) {
            return m5::stl::make_unexpected(p.error());
        }
        if (p.value().size == 0) {
            return DrainProgress{accepted_total, out.eof() ? DrainStop::Drained : DrainStop::WouldBlock};
        }
        auto rsv = sink.reserve(p.value().size);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        if (rsv.value().size == 0) {
            if (sink.closed()) {
                return m5::stl::make_unexpected(error::error_t::CLOSED);
            }
            return DrainProgress{accepted_total, DrainStop::WouldBlock};
        }
        size_t n = rsv.value().size < p.value().size ? rsv.value().size : p.value().size;
        ::memcpy(rsv.value().data, p.value().data, n);
        auto c          = sink.commit(n);
        size_t accepted = c.has_value() ? n : sink.partialCommitAccepted();
        if (accepted > n) {
            accepted = n;  // defensive: never advance past what we peeked
        }
        if (accepted > 0) {
            auto advanced = out.advance(accepted);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            accepted_total += accepted;
        }
        if (!c.has_value()) {
            // StreamSink reports a transport short write as TIMEOUT_ERROR
            // and exposes the accepted prefix above. That is cooperative
            // backpressure, not a terminal wire failure: keep the unsent
            // tail queued for the next pump. Other errors are hard failures.
            if (c.error() == error::error_t::TIMEOUT_ERROR) {
                return DrainProgress{accepted_total, DrainStop::WouldBlock};
            }
            return m5::stl::make_unexpected(c.error());
        }
    }
    return DrainProgress{accepted_total, DrainStop::Drained};
}

}  // namespace m5::hal::v2::remote::detail

#endif  // M5_HAL_REMOTE_WIRE_DRAIN_HPP_
