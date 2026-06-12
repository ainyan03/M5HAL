// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable. The EARLY runtime-only scan pass behind
// hal/v1/runtime/runtime.hpp: masks every non-runtime capability flag
// of the just-included _offer.hpp, then defers to the shared emitter
// (offer_all.inl).
//
// Why an early pass exists: bus::IBus embeds runtime::Mutex BY VALUE,
// so the runtime kind must be resolved before hal/v1/bus/bus.hpp —
// long before M5HAL_v1.hpp's main scan at the end of the umbrella
// header. Masking (instead of a dedicated emitter) keeps the runtime
// dispatch block in offer_all.inl identical to every other kind.
//
// The masked kinds lose nothing: the main scan re-includes the same
// _offer.hpp and emits them normally. The runtime first-hit markers
// burned here simply stop the main scan from re-selecting runtime;
// its pass over the runtime block is then a no-op.

#undef M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_
#undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_
#undef M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_
#undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_
#undef M5HAL_VARIANT_CURRENT_HAS_HAL_UART_
#include "./offer_all.inl"
// clang-format on
