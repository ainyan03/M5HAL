#ifndef M5_HAL_PLATFORMS_ESPRESSIF_ESP32_UNKNOWN_HEADER_HPP
#define M5_HAL_PLATFORMS_ESPRESSIF_ESP32_UNKNOWN_HEADER_HPP

// Generic fallback for any ESP32-family chip the v0 platform_checker does not
// name explicitly (e.g. esp32s2 / esp32c2 / esp32c5 / esp32c61 and future
// SoCs). v0's per-chip directories are identical boilerplate that all route to
// the shared esp32 gpio implementation, so an unrecognized chip routes here and
// compiles the same way. Mirrors the v1 platform variant, which is already
// chip-agnostic (any CONFIG_IDF_TARGET maps to the single esp32 variant).

#include "../../../interface/gpio.hpp"
// #include "../../../interface/bus.hpp"
#include "../gpio.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V0 namespace v0 {
namespace platforms {
namespace esp32 {
namespace gpio {
interface::gpio::GPIO* getGPIO(void);
}  // namespace gpio
namespace bus {
namespace i2c {
}  // namespace i2c
}  // namespace bus
}  // namespace esp32
}  // namespace platforms
}  // namespace v0
}  // namespace hal
}  // namespace m5

#endif
