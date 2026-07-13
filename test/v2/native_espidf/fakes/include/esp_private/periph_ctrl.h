// SPDX-License-Identifier: MIT
#pragma once

// Fake stand-in for ESP-IDF's esp_private/periph_ctrl.h. Only has to exist:
// slave.inl's init() unconditionally probes __has_include(<esp_private/
// periph_ctrl.h>) and includes whichever of the two periph_ctrl headers is
// found, but with soc/soc_caps.h's SOC_RCC_IS_INDEPENDENT=1 (and
// SOC_PERIPH_CLK_CTRL_SHARED left undefined) the PERIPH_RCC_ATOMIC() block
// macro that header would normally provide is never invoked -- see
// soc/soc_caps.h's comment. Nothing to define here.
