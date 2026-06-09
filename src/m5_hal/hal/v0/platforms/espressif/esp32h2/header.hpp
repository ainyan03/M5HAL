#ifndef M5_HAL_PLATFORMS_ESPRESSIF_ESP32H2_HEADER_HPP
#define M5_HAL_PLATFORMS_ESPRESSIF_ESP32H2_HEADER_HPP

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
