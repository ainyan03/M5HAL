// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic ESP-IDF error-code fake (real values from
// esp_err_defs.h -- kept identical so a mapper like slave.inl's mapEspErr
// would see the same numbers, even though that non-LL branch is not
// exercised by this harness).

using esp_err_t = int;

constexpr esp_err_t ESP_OK                = 0;
constexpr esp_err_t ESP_FAIL              = -1;
constexpr esp_err_t ESP_ERR_NO_MEM        = 0x101;
constexpr esp_err_t ESP_ERR_INVALID_ARG   = 0x102;
constexpr esp_err_t ESP_ERR_INVALID_STATE = 0x103;
constexpr esp_err_t ESP_ERR_NOT_FOUND     = 0x105;
constexpr esp_err_t ESP_ERR_NOT_SUPPORTED = 0x106;
constexpr esp_err_t ESP_ERR_TIMEOUT       = 0x107;
