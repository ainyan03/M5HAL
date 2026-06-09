#ifndef M5_HAL_ASSERT_HPP
#define M5_HAL_ASSERT_HPP

// M5HAL_ASSERT — contract-violation detection macro for M5HAL.
// Centralizes the design rule "contract violations crash in debug,
// are UB in release; recoverable errors travel through `expected`".
// Output is log-level independent (an assert is never silent). The
// format follows M5Utility's `M5_LIB_LOG` shape:
//     [A][file:line] func(): msg
// The native gtest death tests match the message on stderr.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace m5::hal::v1::detail {

// Report a contract violation on stderr (log-level independent) and abort.
[[noreturn]] inline void assertFail(const char* file, int line, const char* func, const char* fmt, ...)
{
    std::fprintf(stderr, "[A][%s:%d] %s(): ", file, line, func);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
    std::fflush(stderr);
    std::abort();
}

}  // namespace m5::hal::v1::detail

// Contract-violation assert. In debug builds (NDEBUG undefined),
// a false `cond` prints diagnostics and aborts. In release builds
// (NDEBUG), `cond` is not evaluated and the macro is a no-op
// (UB-style: violations are caller bugs to be caught in debug).
//   Example: M5HAL_ASSERT(pin < count, "pin %u out of range (count=%u)", pin, count);
#if !defined(NDEBUG)
#define M5HAL_ASSERT(cond, ...)                                                           \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            ::m5::hal::v1::detail::assertFail(__FILE__, __LINE__, __func__, __VA_ARGS__); \
        }                                                                                 \
    } while (0)
#else
// No-op without evaluating `cond`. The `sizeof` reference avoids
// `-Wall -Wextra` "unused variable" warnings for assert-only locals.
#define M5HAL_ASSERT(cond, ...) ((void)sizeof(cond))
#endif

#endif
