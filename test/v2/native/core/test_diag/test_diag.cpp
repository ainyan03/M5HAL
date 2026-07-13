// SPDX-License-Identifier: MIT
//
// Native gtest for the M5HAL_DIAG event-trace macro (hal/v2/diag.hpp):
// the default build (M5HAL_CONFIG_DIAG undefined -> 0) compiles a call
// site to a no-op, and detail::diagLog / setSink work independently of
// the flag (so this suite never needs a rebuild with the flag on).

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/diag.hpp>

#include <cstdarg>
#include <cstdio>
#include <string>

namespace {

TEST(M5HalDiag, MacroCompilesToANoOpInTheDefaultBuild)
{
    // M5HAL_CONFIG_DIAG is 0 in this build (no -D override): the macro
    // must accept a printf-style call, but a disabled call site does NOT
    // evaluate its arguments, so a side-effecting argument must not run.
    int calls = 0;
    M5HAL_DIAG("x %d", ++calls);
    EXPECT_EQ(calls, 0);
}

struct CapturedEvent {
    std::string file;
    int line = 0;
    std::string func;
    std::string message;
};

CapturedEvent* g_captured = nullptr;

void captureSink(const char* file, int line, const char* func, const char* fmt, va_list ap)
{
    char buf[256];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    g_captured->file    = file;
    g_captured->line    = line;
    g_captured->func    = func;
    g_captured->message = buf;
}

TEST(M5HalDiag, SetSinkReceivesFileLineFuncAndFormattedArgs)
{
    CapturedEvent captured;
    g_captured = &captured;
    m5::hal::v2::diag::setSink(captureSink);

    // detail::diagLog is called directly (not through M5HAL_DIAG) so the
    // test does not depend on M5HAL_CONFIG_DIAG being enabled.
    const int line_of_call = __LINE__ + 1;
    m5::hal::v2::detail::diagLog(__FILE__, line_of_call, "SomeFunc", "value=%d name=%s", 42, "abc");

    EXPECT_EQ(captured.file, __FILE__);
    EXPECT_EQ(captured.line, line_of_call);
    EXPECT_EQ(captured.func, "SomeFunc");
    EXPECT_EQ(captured.message, "value=42 name=abc");

    // nullptr restores the default (stderr) sink; subsequent calls must
    // not reach the now-dangling test sink.
    m5::hal::v2::diag::setSink(nullptr);
    g_captured = nullptr;
    m5::hal::v2::detail::diagLog(__FILE__, __LINE__, __func__, "back to default sink");
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
