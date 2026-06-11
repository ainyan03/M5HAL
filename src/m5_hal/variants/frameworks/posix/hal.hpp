#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_HPP

// Per-kind hub for the POSIX host framework variant. Offers UART only
// (host serial via termios); the tcp/ files are a remote-bus TRANSPORT,
// not a bus kind, so they appear here but not in _offer.hpp. Each
// subfile is self-contained: the M5HAL_FRAMEWORK_HAS_POSIX gating,
// namespace scaffolding, and the `using namespace ::m5::hal::v1;`
// resolver are local to the kind file.
#include "hal/uart/uart.hpp"
#include "hal/uart/ports.hpp"
#include "hal/uart/remote_connect.hpp"
#include "hal/tcp/tcp.hpp"
#include "hal/tcp/remote_connect.hpp"

#endif
