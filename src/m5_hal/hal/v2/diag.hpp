// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DIAG_HPP
#define M5_HAL_DIAG_HPP

// M5HAL_DIAG — opt-in event-trace macro for M5HAL.
// Independent of M5HAL_ASSERT (contract violations): this is a
// developer-facing trace for following the shape of a run (state
// transitions, retries, ...), never a correctness signal. Gated by
// the build flag M5HAL_CONFIG_DIAG (default 0); when off the macro
// does not evaluate its arguments, so a disabled call site costs
// nothing (no format string, no argument evaluation, no call).
// Output format matches M5HAL_ASSERT's shape (see assert.hpp):
//     [D][file:line] func(): msg
//
// Known pitfall: on a remote server device where the console UART
// also carries the wire protocol, writing diagnostics to stderr can
// corrupt the wire itself. Targets in that shape must call setSink()
// to redirect diagnostics to a channel that does not alias the wire.

#include <cstdarg>
#include <cstdio>

#ifndef M5HAL_CONFIG_DIAG
#define M5HAL_CONFIG_DIAG 0
#endif

namespace m5::hal::v2::diag {

// Sink receives one diagnostic event (unformatted args). Replace with
// setSink() when stderr is not safe on the target.
using sink_t = void (*)(const char* file, int line, const char* func, const char* fmt, va_list ap);

namespace detail {

// Function-local static: the sink slot is initialized on first use,
// which sidesteps static-initialization-order issues across
// translation units (no global object depends on construction order).
inline sink_t& sinkSlot()
{
    static sink_t s = nullptr;
    return s;
}

// Default sink: stderr, flushed immediately, same message shape as
// M5HAL_ASSERT's assertFail (see assert.hpp).
inline void defaultSink(const char* file, int line, const char* func, const char* fmt, va_list ap)
{
    std::fprintf(stderr, "[D][%s:%d] %s(): ", file, line, func);
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    std::fflush(stderr);
}

}  // namespace detail

// Install `sink` as the diagnostic event sink. `nullptr` restores the
// default (stderr) sink. Not thread-safe against concurrent diagLog
// calls; install the sink once during startup.
inline void setSink(sink_t sink)
{
    detail::sinkSlot() = sink;
}

}  // namespace m5::hal::v2::diag

namespace m5::hal::v2::detail {

// Format and dispatch one diagnostic event to the current sink
// (installed sink, or the default stderr sink when none is set).
// Always compiled regardless of M5HAL_CONFIG_DIAG (only the
// M5HAL_DIAG macro expansion is gated) so tests can call this
// directly without rebuilding with the flag on.
inline void diagLog(const char* file, int line, const char* func, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    auto sink = ::m5::hal::v2::diag::detail::sinkSlot();
    if (sink != nullptr) {
        sink(file, line, func, fmt, ap);
    } else {
        ::m5::hal::v2::diag::detail::defaultSink(file, line, func, fmt, ap);
    }
    va_end(ap);
}

}  // namespace m5::hal::v2::detail

// Opt-in event trace. When M5HAL_CONFIG_DIAG is 0 (default), this is a
// no-op and none of the arguments are evaluated.
//   Example: M5HAL_DIAG("retry %u/%u", attempt, max_attempts);
#if M5HAL_CONFIG_DIAG
#define M5HAL_DIAG(...) ::m5::hal::v2::detail::diagLog(__FILE__, __LINE__, __func__, __VA_ARGS__)
#else
#define M5HAL_DIAG(...) ((void)0)
#endif

#endif
