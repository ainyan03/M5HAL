# native_espidf fake IDF header tree

Host-buildable regression harness for M5HAL's ESP-IDF framework backends: a
fake ESP-IDF header tree that lets an ISR-driven backend compile and run
**unmodified** (no product logic changes -- only the compile-time gates that
select the backend under `ESP_PLATFORM` are widened to also accept
`M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS`) on the native/host PlatformIO env, with a
deterministic, single-threaded test scripting fake ISR events instead of a
real bus.

First consumer: `src/m5_hal/variants/frameworks/espidf/hal/i2c/slave.inl`
(the I2C slave LL-stretch state machine), exercised by
`../test_espidf_i2c_slave/test_espidf_i2c_slave.cpp` under the
`test_native_espidf_fake` PlatformIO env.

## Layout

```
fakes/include/                  <- IDF header-path mirror; pass as -I root
  freertos/FreeRTOS.h, task.h, semphr.h   <- peripheral-agnostic
  esp_err.h, esp_attr.h, esp_intr_alloc.h <- peripheral-agnostic
  soc/soc_caps.h                          <- peripheral-agnostic capability flags
  hal/i2c_types.h, soc/i2c_struct.h,
  soc/i2c_periph.h, hal/i2c_ll.h          <- I2C-specific device model
  driver/gpio.h, esp_rom_gpio.h,
  esp_private/periph_ctrl.h               <- peripheral-agnostic, no-op HW routing
```

The split matters for reuse: everything under `freertos/`, the top-level
`esp_*.h` files, `soc/soc_caps.h`, `driver/gpio.h`, `esp_rom_gpio.h`, and
`esp_private/periph_ctrl.h` is **peripheral-agnostic** -- any ESP-IDF backend
hosted this way shares them unchanged. Only `hal/i2c_types.h`,
`soc/i2c_struct.h`, `soc/i2c_periph.h`, and `hal/i2c_ll.h` are I2C-specific
(the fake *device model*).

## Design: what the fake actually does

The fake's job is narrow: be an **event-snapshot presentation device**. It
does not model bus timing, arbitration, level-type interrupt re-assertion,
or a real scheduler. A test:

1. Pokes the fake device model (`i2c_dev_t` in `soc/i2c_struct.h` -- FIFOs,
   interrupt-pending bits, address-phase direction, stretch cause) directly,
   representing "what a real ISR snapshot would show right now".
2. Calls `m5hal_hostharness::fireLastIsr()` (`esp_intr_alloc.h`) to
   synchronously invoke the backend's captured ISR handler, exactly as a
   real interrupt would.
3. Drives the SAME public accessor API (`beginTransaction` / `read` /
   `write` / `endTransaction`) an application uses, interleaved with step 1-2,
   to reproduce a specific event ORDERING.

This is sufficient for every regression in the wave-1 test suite because
each of them is a pure **event-ordering** bug (a race between the ISR's view
of the wire and the accessor's consumption of it), not a timing or
electrical bug -- exactly the class of bug `slave.hpp`'s own header comment
already documents as "the transaction-window state machine is HW
-independent" (the shared logic layer the ESP32-S3 LL path and the software
backend both build on).

### How ISR injection works

`esp_intr_alloc()` (real signature: register a `(source, flags, handler,
arg)` interrupt) captures the `(handler, arg)` pair into a process-wide
"last captured" slot in addition to returning a heap handle for
`esp_intr_free`. `m5hal_hostharness::fireLastIsr()` calls it back
synchronously. This is the only way to reach the backend's ISR without
touching its private members (`SlaveBus_espidf::_intr` is private, and
rightly so -- adding a test-only public accessor would be a product-code
change beyond the gate-only contract this harness works under). It is
scoped to ONE in-flight interrupt registration; see "Extending to another
peripheral" below for what a multi-peripheral harness would need instead.

### How the fake device model works

`i2c_dev_t` (`soc/i2c_struct.h`) is not a register-layout overlay like the
real struct -- it directly holds the fake model's state (RX/TX FIFO bytes +
counts, `int_ena`/`int_st`, the address-phase direction latch, the stretch
cause + a `stretch_active` bool standing in for "real HW is physically
holding SCL low"). `hal/i2c_ll.h`'s fake `i2c_ll_*` functions are the ONLY
things that read/write these fields on the product-code side; a test pokes
them directly on the setup side. `I2C_LL_GET_HW(port)` resolves to a
process-wide singleton per port (`m5hal_hostharness::i2cDeviceFor`),
matching real hardware (I2C_NUM_0 is one physical peripheral). Each test's
`SlaveBus_espidf::init()` resets the fields the state machine depends on
(FIFO counts via `i2c_ll_txfifo_rst`/`rxfifo_rst`, interrupt mask, stretch),
so a fresh bus + `init()` per `TEST` is sufficient isolation -- see the
per-field notes in `soc/i2c_struct.h` for the ones that are NOT reset
(diagnostic-only counters).

## Scope and fidelity limits

- **No real bus timing.** Nothing here models clock frequency, setup/hold
  times, or how long an event takes -- only ordering.
- **No level-type interrupt re-assertion.** Real HW re-fires a level-type
  source (e.g. TXFIFO_WM) every cycle it stays asserted; the fake fires
  exactly once per `fireLastIsr()` call. A regression that depends on
  *repeated* re-assertion (rather than the state after one ISR pass) is out
  of this harness's reach.
- **No scheduler.** `freertos/task.h`'s `xTaskCreate` reports success but
  never invokes the task body; `freertos/semphr.h`'s `xSemaphoreTake` never
  actually blocks. The wave-1 scenarios never need the backend's responder
  task to run: `SlaveStreamAccessor::write()`'s release path resolves a held
  read stretch synchronously (composes + fills the fake TX FIFO + clears the
  stretch) whenever `_open == _current`, with no task involved -- which is
  also exactly the code path the cc133e89 regression lives in. A scenario
  that genuinely needs the task loop to run (e.g. the stretch-timeout
  fill-byte fallback in `requestTaskLoop`) is out of reach without teaching
  the harness to invoke the captured `TaskFunction_t` synchronously.
- **One in-flight ISR registration.** See "How ISR injection works" above.
- **LL stretch and LL BE paths are both reachable; the v2-driver fallback is
  not.** `soc/soc_caps.h`'s `SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE` defaults to
  1 (selects `M5HAL_ESPIDF_I2C_SLAVE_LL`, the `test_native_espidf_fake` env);
  defining `M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_NO_STRETCH_CAPABILITY` flips it to 0, which selects
  `M5HAL_ESPIDF_I2C_SLAVE_LL_BE` instead (the classic-ESP32 no-clock-stretch
  flavor, the `test_native_espidf_fake_be` env -- see
  `../test_espidf_i2c_slave_be/`). Both share this same fake header tree
  unmodified. The v2-driver fallback path (`driver/i2c_slave.h`) is still not
  modeled (it would need its own fake header plus both LL gates forced off to
  reach).
- **Warnings, not `-Werror`.** The `test_native_espidf_fake` env inherits
  `common_native`'s `-Wconversion -Wsign-conversion -Wshadow` etc. without
  `-Werror`; a fake header's stray warning does not fail the build.

## Extending to another peripheral

The split above is the extension seam: to host another ISR-driven ESP-IDF
backend (SPI slave, UART, ...) off-target,

1. Reuse `freertos/`, `esp_err.h`, `esp_attr.h`, `esp_intr_alloc.h`,
   `soc/soc_caps.h` (add that peripheral's capability macros alongside the
   I2C ones already there), `driver/gpio.h`, `esp_rom_gpio.h`,
   `esp_private/periph_ctrl.h` unchanged.
2. Add a device model: `hal/<kind>_types.h` + `soc/<kind>_struct.h` +
   `hal/<kind>_ll.h` (and a `soc/<kind>_periph.h` if that backend indexes a
   signal table the way I2C does), following the same shape as the I2C
   files -- a fake struct holding the model state, a `<KIND>_LL_GET_HW(port)`
   singleton accessor, and `<kind>_ll_*` functions that are either no-ops
   (init-only config) or operate on the model (the state-machine calls the
   backend's ISR/read/write paths actually use).
3. Widen that backend's own `defined(ESP_PLATFORM)` gates to
   `defined(ESP_PLATFORM) || defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)` (the same
   minimal, gate-only edit this harness's I2C consumer makes) -- do NOT
   reuse `M5HAL_CONFIG_ESPIDF_I2C_SLAVE_IRAM_ISR`-style per-kind build flags across
   kinds; each backend keeps its own.
4. Add a `test/v2/native_espidf/test_<kind>_slave/` (or similar) directory
   and a dedicated PlatformIO env analogous to `test_native_espidf_fake`,
   with `-Itest/v2/native_espidf/fakes/include` and that backend's own
   `-D` overrides.
5. If the new peripheral's ISR must coexist with I2C's in the SAME test
   binary (not just the same fake tree), `esp_intr_alloc.h`'s "last
   captured" single-slot model needs to become a keyed registry first (see
   "How ISR injection works" above) -- until then, keep one peripheral's
   ISR-driven backend per PlatformIO env / test binary.
