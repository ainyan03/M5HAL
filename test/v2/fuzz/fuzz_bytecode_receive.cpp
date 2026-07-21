// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace {
void noDelay(uint32_t)
{
}
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    constexpr size_t kMaxInput = 4096;
    m5::hal::v2::bytecode::BytecodeRunner runner;
    runner.setDelayFn(&noDelay);

    // No buses or GPIO group are registered: decoded commands can only be
    // validated and rejected, never actuate hardware, buses, pins, or delays.
    (void)runner.run(m5::hal::v2::data::ConstDataSpan{data, std::min(size, kMaxInput)});
    return 0;
}
