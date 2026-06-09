#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_HPP

// Per-kind hub for the POSIX host framework variant. Offers UART only
// (host serial via termios). The subfile is self-contained: the
// M5HAL_FRAMEWORK_HAS_POSIX gating, namespace scaffolding, and the
// `using namespace ::m5::hal::v1;` resolver are local to the kind file.
#include "hal/uart/uart.hpp"

#endif
