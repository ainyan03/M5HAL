// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_INL

#include "tcp.hpp"

#if defined(ESP_PLATFORM) && __has_include(<sys/socket.h>)

// Everything is the shared BSD-socket detail; this variant adds nothing
// on the implementation side (the aliases live in tcp.hpp).
#include "../../../../../_detail/bsd_tcp.inl"

#endif  // ESP_PLATFORM && <sys/socket.h>

#endif
