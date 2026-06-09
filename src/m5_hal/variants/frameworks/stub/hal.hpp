#ifndef M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_HPP

// Per-kind hub for the stub framework variant. The stub provides no-op
// concretes for every HAL kind that needs an unconditional fallback.
// No impl.inl hub is needed because all stub concretes are inline.
#include "hal/gpio/gpio.hpp"

#endif
