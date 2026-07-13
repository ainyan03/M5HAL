// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic interrupt-allocation fake. esp_intr_alloc() captures
// the (handler, arg) pair a backend registers for its ISR -- both in the
// returned intr_handle_t (so esp_intr_free can release it) AND in a
// process-wide "last captured" slot, so a test can fire the ISR
// synchronously without reaching into the backend's private members (there
// is no other seam: the real esp_intr_alloc never hands the handler back to
// application code). See ../../README.md "How ISR injection works".
//
// Scoped to ONE in-flight interrupt registration at a time (the "last
// captured" slot is overwritten on every esp_intr_alloc call) -- sufficient
// for a single-peripheral-instance harness. A future harness driving two
// interrupt sources concurrently (e.g. I2C slave + SPI slave in the same
// test) would need a keyed registry instead; see ../../README.md "Extending
// to another peripheral".

#include "esp_err.h"

using intr_handler_t = void (*)(void*);

struct intr_handle_data_t {
    intr_handler_t handler = nullptr;
    void* arg              = nullptr;
};
using intr_handle_t = intr_handle_data_t*;

#define ESP_INTR_FLAG_IRAM   (1 << 0)
#define ESP_INTR_FLAG_LEVEL3 (1 << 1)

namespace m5hal_hostharness {

struct CapturedIntr {
    intr_handler_t handler = nullptr;
    void* arg              = nullptr;
};

inline CapturedIntr& lastCapturedIntr()
{
    static CapturedIntr captured;
    return captured;
}

}  // namespace m5hal_hostharness

inline esp_err_t esp_intr_alloc(int /*source*/, int /*flags*/, intr_handler_t handler, void* arg,
                                intr_handle_t* ret_handle)
{
    if (ret_handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *ret_handle                           = new intr_handle_data_t{handler, arg};
    m5hal_hostharness::lastCapturedIntr() = {handler, arg};
    return ESP_OK;
}

inline esp_err_t esp_intr_free(intr_handle_t handle)
{
    delete handle;
    return ESP_OK;
}

namespace m5hal_hostharness {

// Test-facing: synchronously invoke the most recently esp_intr_alloc()'d
// handler, as if the interrupt fired right now. No-op if nothing was ever
// captured (or the backend already esp_intr_free()'d -- the captured
// function pointer/arg copy is independent of that call, so this stays
// callable, but firing after release() is a test-authoring bug: the arg
// (the backend instance) may already be destroyed).
inline void fireLastIsr()
{
    auto& c = lastCapturedIntr();
    if (c.handler != nullptr) {
        c.handler(c.arg);
    }
}

}  // namespace m5hal_hostharness
