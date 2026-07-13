// SPDX-License-Identifier: MIT
#ifndef M5_HAL_TEST_V2_SUPPORT_GTEST_WATCHDOG_HPP
#define M5_HAL_TEST_V2_SUPPORT_GTEST_WATCHDOG_HPP

// Per-test hang watchdog for the native gtest suites (POSIX only).
//
// Why: neither googletest nor `pio test` has a per-test timeout, so a
// regression that turns a bounded wait into an unbounded one (e.g. an API
// called with its TIMEOUT_FOREVER default whose wake condition never comes)
// hangs the test process -- and with it the local-ci / push-gated pipeline --
// indefinitely instead of failing. This
// watchdog turns such a hang into a fast, attributed failure: the process
// exits 124 and prints WHICH test overran.
//
// This is the generic safety net, not the first line of defense: tests should
// still use finite or non-blocking timeouts (delegation-guardrails.md
// §テスト規約) so a regression fails at the exact assertion, not by budget.
//
// Usage (after InitGoogleTest, before RUN_ALL_TESTS):
//     ::testing::InitGoogleTest(&argc, argv);
//     m5hal_test_support::installGtestWatchdog();
//
// Budget: 30 s per test (the suites' slowest tests finish in ~1 s), 120 s
// under ThreadSanitizer. Override with M5HAL_GTEST_WATCHDOG_SEC; 0 disables.
//
// Constraints: uses the process-wide alarm(2)/SIGALRM slot, so tests must not
// use alarm() themselves (none do today). In a multi-threaded test SIGALRM
// may be delivered to any thread; the handler only write()s and _exit()s, so
// that is safe.

#include <gtest/gtest.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#if defined(__SANITIZE_THREAD__)
#define M5HAL_GTEST_WATCHDOG_DEFAULT_SEC 120u
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define M5HAL_GTEST_WATCHDOG_DEFAULT_SEC 120u
#else
#define M5HAL_GTEST_WATCHDOG_DEFAULT_SEC 30u
#endif
#else
#define M5HAL_GTEST_WATCHDOG_DEFAULT_SEC 30u
#endif

namespace m5hal_test_support {

// Snapshotted at OnTestStart so the SIGALRM handler stays async-signal-safe
// (write + _exit only; no allocation, no gtest calls).
inline char g_watchdog_test_name[256] = "<no test running>";

inline void watchdogAlarmHandler(int)
{
    // Both fds: `pio test` relays neither stream un-verbose (the exit-124
    // "Program errored" line is the visible signal there), but stdout shows
    // the attribution under `pio test -v` and stderr under a direct run.
    constexpr char head[] = "\n[gtest-watchdog] per-test time budget exceeded, aborting in: ";
    constexpr char tail[] = "\n[gtest-watchdog] (M5HAL_GTEST_WATCHDOG_SEC overrides the budget; 0 disables)\n";
    for (int fd : {STDOUT_FILENO, STDERR_FILENO}) {
        (void)!::write(fd, head, sizeof(head) - 1);
        (void)!::write(fd, g_watchdog_test_name, ::strlen(g_watchdog_test_name));
        (void)!::write(fd, tail, sizeof(tail) - 1);
    }
    ::_exit(124);
}

class WatchdogListener : public ::testing::EmptyTestEventListener {
public:
    explicit WatchdogListener(unsigned seconds) : _seconds{seconds}
    {
    }
    void OnTestStart(const ::testing::TestInfo& info) override
    {
        ::snprintf(g_watchdog_test_name, sizeof(g_watchdog_test_name), "%s.%s", info.test_suite_name(), info.name());
        ::alarm(_seconds);
    }
    void OnTestEnd(const ::testing::TestInfo&) override
    {
        ::alarm(0);
    }

private:
    unsigned _seconds;
};

inline void installGtestWatchdog()
{
    unsigned seconds   = M5HAL_GTEST_WATCHDOG_DEFAULT_SEC;
    const char* env    = ::getenv("M5HAL_GTEST_WATCHDOG_SEC");
    if (env != nullptr && *env != '\0') {
        seconds = static_cast<unsigned>(::strtoul(env, nullptr, 10));
    }
    if (seconds == 0) {
        return;  // explicitly disabled
    }
    struct sigaction sa;
    ::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = &watchdogAlarmHandler;
    // No `::` here: macOS's signal.h defines sigemptyset as a function-style
    // macro, and `::sigemptyset(...)` fails to parse after expansion.
    sigemptyset(&sa.sa_mask);
    ::sigaction(SIGALRM, &sa, nullptr);
    // gtest owns and deletes appended listeners.
    ::testing::UnitTest::GetInstance()->listeners().Append(new WatchdogListener(seconds));
}

}  // namespace m5hal_test_support

#endif
