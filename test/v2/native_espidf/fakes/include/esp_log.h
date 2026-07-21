// SPDX-License-Identifier: MIT
#pragma once

#include "esp_err.h"

// Host fake lanes verify lifecycle semantics, not ESP-IDF logging. Keep the
// production call sites type-checked without emitting nondeterministic output.
inline const char* esp_err_to_name(esp_err_t)
{
    return "ESP_FAKE_ERROR";
}

#define ESP_LOGE(tag, format, ...) ((void)0)
