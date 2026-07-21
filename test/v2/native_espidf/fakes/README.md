# ESP-IDF host fake tree

This directory provides the minimum ESP-IDF headers and driver behavior needed
to run selected M5HAL ESP-IDF backends in native tests. Product backend code is
compiled through explicit host-harness gates; the fake supplies deterministic
events and driver outcomes.

## Scope

The current consumers are:

| PlatformIO env | Backend exercised | Test source |
|---|---|---|
| `test_native_espidf_fake` | I2C slave LL stretch path | [`../test_espidf_i2c_slave/`](../test_espidf_i2c_slave/) |
| `test_native_espidf_fake_be` | I2C slave LL best-effort path | [`../test_espidf_i2c_slave_be/`](../test_espidf_i2c_slave_be/) |
| `test_native_espidf_fake_spi_slave` | SPI slave driver lifecycle | [`../test_espidf_spi_slave_lifecycle/`](../test_espidf_spi_slave_lifecycle/) |

Run them from the repository root:

```sh
pio test -e test_native_espidf_fake
pio test -e test_native_espidf_fake_be
pio test -e test_native_espidf_fake_spi_slave
```

The env definitions and harness macros in [`../../../../platformio.ini`](../../../../platformio.ini)
are authoritative.

## Layout

```text
fakes/include/
  freertos/                 task and synchronization substitutes
  driver/                   fake public driver APIs
  hal/ and soc/             I2C device model and LL operations
  esp_private/              private driver hooks used by selected backends
  esp_*.h                   shared SDK declarations and helpers
```

Shared headers may contain harness-specific branches when peripheral behavior
differs. Peripheral state belongs in that peripheral's fake driver or device
model; it must not leak into unrelated tests.

## Fidelity boundary

The fake is suitable for deterministic state and ordering checks, including
injected ISR events, queue completion, timeout, cleanup, and error paths exposed
by its current drivers. It does not establish:

- physical bus timing, arbitration, signal quality, or DMA behavior;
- behavior of SDK APIs that are absent from the fake tree;
- scheduler or interrupt re-assertion behavior unless the selected fake models
  it explicitly;
- interaction between multiple real peripherals merely because their tests use
  the same header tree.

Use embedded or HIL fixtures for physical-peripheral claims. A fake test should
state the event or driver outcome it injects, not generalize that result to the
wire.

## Extending the fake

1. Identify the exact SDK headers and calls reached by the backend.
2. Reuse shared declarations only when their behavior is genuinely
   peripheral-independent; otherwise add a peripheral-specific fake.
3. Add an explicit host-harness selection gate and a dedicated PlatformIO env.
4. Keep one backend scenario per test binary unless the fake models concurrent
   registrations and scheduling needed by the combined case.
5. Add tests for both success and cleanup/error outcomes, then document the new
   fidelity boundary here.

Do not add test-only public accessors to product classes. Expose backend events
through the fake SDK/driver boundary used by the production implementation.
