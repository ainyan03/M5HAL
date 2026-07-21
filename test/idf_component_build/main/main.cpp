// SPDX-License-Identifier: MIT
// Official ESP-IDF wrapper for the canonical public API compile fixture.
// Keep the IDF-specific backend reachability fence as well: the post-link IRAM
// check needs those backend symbols to survive section garbage collection.
#define app_main m5hal_build_test_app_main
#include "../../../examples/v2/BuildTest/BuildTest.cpp"
#undef app_main

#include "../../v2/build_check/build_check.hpp"

extern "C" void app_main(void)
{
    m5hal_build_test_app_main();
    m5hal_build_check::v2::compileApiSurface();
}
