# M5HAL

*Japanese: [README.ja.md](README.ja.md)*

<!-- pair: overview -->
## Overview

M5HAL is a hardware abstraction layer for M5 products. The stable **v0 API**
remains the default for existing ESP32 code. The **v2 API** is an opt-in,
clean-slate API under active development; select it explicitly with
`<M5HAL_v2.hpp>`.

<!-- pair: requirements -->
## Requirements

- v0 requires an ESP32-family board. v2 also has build-checked support for an
  allowlist of non-ESP Arduino cores; their runtime guarantees vary by core
  ([details](spec/design/variants.md#arduino-variant-の対応コア-build-gate)).
  Direct ESP-IDF component use requires ESP-IDF 5.0 or later. Arduino-ESP32
  support includes core 2.x, whose bundled SDK is ESP-IDF 4.4.
- A C++17 compiler.
- [M5Utility](https://github.com/m5stack/M5Utility). PlatformIO and the ESP-IDF
  component manager resolve it automatically; Arduino IDE users install it
  alongside M5HAL.

<!-- pair: installation -->
## Installation

- **Arduino IDE:** install "M5HAL" and "M5Utility" from Library Manager.
- **PlatformIO:** add the library to `platformio.ini`:

  ```ini
  lib_deps =
      m5stack/M5HAL
  ```

- **ESP-IDF component manager:** add it to `idf_component.yml`:

  ```yaml
  dependencies:
    m5stack/M5HAL: "*"
  ```

  ESP-IDF 5.0's bundled component manager predates the current manifest schema.
  In an activated ESP-IDF 5.0 environment, update it with
  `python -m pip install --upgrade idf-component-manager`.

<!-- pair: quickstart -->
## v2 quick start

M5HAL v2 requires C++17. PlatformIO `espressif32@6.x` with Arduino core 2.x
defaults to gnu++11, so add
`build_flags = -std=gnu++17` and `build_unflags = -std=gnu++11`.
Arduino-ESP32 3.x and Arduino IDE already default to C++17.

This minimal Arduino I2C shape uses the portable `BusConfig`; the selected
provider creates and initializes the backend:

```cpp
#include <M5HAL_v2.hpp>

#include <memory>

namespace m5hal = m5::hal::v2;

std::shared_ptr<m5hal::i2c::IBus> i2c_bus;

void setup()
{
    m5hal::i2c::BusConfig bus_cfg{
        m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};

    auto acquired = m5hal::M5_Hal.I2C.acquire(bus_cfg);
    if (!acquired) return;
    i2c_bus = acquired.value();

    m5hal::i2c::MasterAccessConfig dev_cfg;
    dev_cfg.i2c_addr = 0x76;
    dev_cfg.freq = 100000;

    m5hal::i2c::MasterAccessor dev{i2c_bus, dev_cfg};
    auto value = dev.readRegister(0x00);
    if (!value) return;
    // Use value.value().
}

void loop() {}
```

`BusConfig` contains portable bus semantics and wiring, not a framework
handle. Existing native objects use a separate ownership policy after the
caller initializes them; see the complete
[`I2C` example](examples/v2/HowToUse/I2C/) and the
[`Bus / Accessor specification`](spec/design/bus_accessor.md) for ownership,
locking, transfer, and lifetime contracts.

<!-- pair: generations -->
## API generations and entry headers

API generation and release version are separate concepts. v0 is the stable
legacy generation and stays the default through the migration period; v2 is
available by explicit opt-in. There is no v1 API generation.

| Header | API exposed | Use |
|---|---|---|
| `<M5HAL.hpp>` | v0 by default | Existing source compatibility |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (v0) | Explicit v0 selection |
| `<M5HAL_v2.hpp>` | `m5::hal::v2::*` | Explicit v2 selection |

Non-ESP Arduino targets are v2-only. On v0-supported targets, v0 and v2 entry
headers may coexist in one translation unit, though selecting one generation
per file is clearer. The complete switching and namespace contract is in
[`v0 / v2 coexistence`](spec/design/v0_v2_coexistence.md).

<!-- pair: navigation -->
## Where to go next

| Goal | Documentation |
|---|---|
| Run an example | [`v2 example index`](examples/v2/HowToUse/README.md) |
| Understand the public specification | [`spec/README.md`](spec/README.md) |
| Configure build-time behavior | [`configuration.md`](spec/design/configuration.md) |
| Understand bus ownership and accessors | [`bus_accessor.md`](spec/design/bus_accessor.md) |
| Select or port a backend | [`variants.md`](spec/design/variants.md), [`porting guide`](spec/porting_guide/README.md) |
| Use remote transports | [`remote.md`](spec/design/remote.md) |
