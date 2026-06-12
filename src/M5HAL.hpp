// SPDX-License-Identifier: MIT
#ifndef M5HAL_HPP
#define M5HAL_HPP

// =============================================================================
// M5HAL compatibility shim.
//
// Thin wrapper kept for backwards compatibility. Existing code that
// writes `#include <M5HAL.hpp>` resolves through this shim to the
// **v0** entry header, honouring the "no touch needed" promise of
// the v0 / v1 coexistence strategy.
//
// New code should `#include <M5HAL_v0.hpp>` or `<M5HAL_v1.hpp>`
// directly to spell its intent. When the default switches to v1 in
// the future, only the include below changes — code that already
// spells the explicit header is unaffected.
//
// See README §v0 / v1 coexistence and
// spec/design/v0_v1_coexistence.md for the full picture.
// =============================================================================

#include "M5HAL_v0.hpp"

#endif  // M5HAL_HPP
