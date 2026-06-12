# M5HAL

*日本語: [README.ja.md](README.ja.md)*

## Overview
HAL (Hardware Abstraction Layer) for M5 Products.

The **v0 API is stable** and stays the default, so existing code keeps
working unchanged. The **v1 API is under active development** and is
opt-in — include `<M5HAL_v1.hpp>` explicitly to try it.

## Requirements

- An ESP32-family board. The published packages target the `espressif32`
  platform (Arduino-ESP32 or ESP-IDF >= 4.4).
- [M5Utility](https://github.com/m5stack/M5Utility) — PlatformIO and the
  ESP-IDF component manager pull it in automatically; in the Arduino IDE,
  install it alongside M5HAL.

## Documentation

- Confirmed specification documents live under [`spec/`](spec/README.md)
  (also bundled in the release packages).

## Where to start

| Reader | Start here |
|---|---|
| Existing v0 user | Keep using `<M5HAL.hpp>` or `<M5HAL_v0.hpp>`. Read [v0 / v1 coexistence](#v0--v1-coexistence) only if you need to understand the migration period. |
| Trying v1 in a sketch | Read [Trying the v1 API](#trying-the-v1-api), then open [`examples/v1/HowToUse/I2C`](examples/v1/HowToUse/I2C/), [`examples/v1/HowToUse/SPI`](examples/v1/HowToUse/SPI/), or [`examples/v1/HowToUse/UART`](examples/v1/HowToUse/UART/). |
| Implementing a backend or reviewing internals | Use [`spec/README.md`](spec/README.md) as the map. The main design files are `bus_accessor`, `i2c`, `spi`, `gpio`, and `variants`. |

## API generations (v0 / v1) and release numbering

M5HAL distinguishes between **API generation** (the spec lineage) and
**release version** (the library version number).

- **v0** — the legacy API generation, shipped as the `v0.0.x` releases.
- **v1** — the new API generation, designed as a clean-slate redesign.
- **v0.2.x** — the **migration-period release line** that ships both API
  generations side-by-side in a single library.

While the major release version is `0` (i.e. `v0.x.y`), **v0 stays the
default** so existing consumers keep working without code changes. From
the `v0.2` release onward, users who explicitly opt in can try the v1 API.
When the major version eventually reaches `1` (i.e. `v1.x.y`), v1 becomes
the default.

## v0 / v1 coexistence

M5HAL adopts a coexistence strategy so that existing v0 consumers keep
using their code unchanged while the v1 API lives side-by-side in the same
library. The entry headers are:

| Header | Exposes | For |
|---|---|---|
| `<M5HAL.hpp>` | (shim → v0 by default) | Backward compatibility — existing code that already includes `<M5HAL.hpp>` keeps working unchanged. New code should prefer one of the explicit headers below. |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (= v0, via `inline namespace v0`) | Code that explicitly opts into the v0 (legacy) API |
| `<M5HAL_v1.hpp>` | `m5::hal::v1::*` | Code that explicitly opts into the v1 API |

- **Both entries may share a translation unit.** Include guards and the
  platform-detection macros are generation-separated (v0 owns the
  unprefixed `M5HAL_TARGET_PLATFORM_*` names, v1 uses
  `M5HAL_V1_TARGET_PLATFORM_*`), so one `.cpp` may include both a v0 entry
  (`<M5HAL.hpp>` shim or `<M5HAL_v0.hpp>` direct) and `<M5HAL_v1.hpp>`,
  e.g. while migrating that file gradually. An intermediate library should
  still make its intended generation explicit per TU for readability.
- **Switching the default later.** `M5HAL_V0_INLINE` (defined in
  `src/m5_hal_config.hpp`, defaults to `1` while the major version is `0`)
  controls whether v0 is exposed as `inline namespace v0`. When the
  library bumps to major version `1`, the default flips to `0` and v1
  becomes `inline namespace v1` instead, at which point legacy code must
  qualify symbols as `m5::hal::v0::Foo` explicitly.
- **Forward-compatible layout.** The same pattern extends to future
  versions: `hal/<vN>/` for the implementation, `examples/<vN>/` for
  samples, and `M5HAL_<vN>.hpp` as the entry header. v2 / v3 can be added
  alongside v0 / v1 without touching existing consumers.

## Trying the v1 API

v1 is opt-in. Include `<M5HAL_v1.hpp>`. Mixing a v0 entry header into the
same translation unit is also supported (see
[v0 / v1 coexistence](#v0--v1-coexistence)), but one generation per file
reads better.

The current v1 bus API is centered on:

- **Bus** — the physical bus instance (`i2c::Bus`, `spi::Bus`, `uart::Bus`,
  `i2s::Bus`, or an explicit variant such as `spi::Bus_software`)
- **Accessor** — one target device on that bus, with per-device settings
  such as address, chip-select pin, baud rate, frequency, timeout, and SPI mode
- **TransferDesc** — per-transfer metadata such as an I2C register prefix
  or SPI command/address/dummy phases; UART does not need a transfer descriptor
- **Source / Sink** — streaming-friendly data input/output abstractions;
  span and raw pointer overloads are available

**You own the buses**: v1 has no hidden singleton bus inside the HAL.
Define the bus object yourself (usually static / global), init it, and
hand it to accessors. To publish board wiring as "slot n is this bus",
register it in the non-owning registries `M5_Hal.I2C` / `M5_Hal.SPI` /
... (`M5_Hal.SPI.addBus(&bus, 1)`, then `M5_Hal.SPI.getBus(1)`; the
same bus may sit in several slots — "SD and LCD share one bus").
`M5_Hal` itself bundles the GPIO/bus registries and the service
runner; you can ignore it unless you use expanders or a board-support
layer. for simple buffers

Minimal I2C shape:

```cpp
#include <M5HAL_v1.hpp>
#include <Wire.h>

namespace m5hal = m5::hal::v1;

m5hal::i2c::Bus i2c_bus;

void setup()
{
    m5hal::i2c::BusConfig bus_cfg;
    bus_cfg.wire    = &Wire;
    bus_cfg.pin_scl = 22;
    bus_cfg.pin_sda = 21;
    i2c_bus.init(bus_cfg);

    m5hal::i2c::AccessConfig dev_cfg;
    dev_cfg.i2c_addr        = 0x76;
    dev_cfg.freq            = 100000;
    dev_cfg.wire_timeout_ms = 100;
    // dev_cfg.register_address_bytes = 2;  // only for 2-byte register-address devices

    m5hal::i2c::MasterAccessor dev{i2c_bus, dev_cfg};
    auto id = dev.readRegister(0x00);
}
```

For a complete Arduino sketch, start with
[`examples/v1/HowToUse/I2C`](examples/v1/HowToUse/I2C/).
The example scans the bus, creates an accessor for the first responding
device, demonstrates register reads, and shows `ScopedAccess` for grouping
multiple transfers under one bus lock.

When you need a specific backend, use the suffixed variant type name.
For example, the software I2C backend can be selected by changing the bus
type in the example to:

```cpp
using ExampleBus = m5::hal::v1::i2c::Bus_software;
```

`#include <Wire.h>` is needed because the default Arduino I2C backend
(`i2c::Bus_arduino`) delegates to a `TwoWire` instance
(`bus_cfg.wire = &Wire`); it goes away when you pin the software or
ESP-IDF backend instead. The unsuffixed `i2c::Bus` / `i2c::BusConfig`
spellings are type aliases to the first backend the build environment
offered (`Bus_arduino`, ...), so the `BusConfig` field set (whether
there is a `wire`, ...) follows the selected variant.

SPI follows the same Bus / Accessor shape. Arduino SPI, ESP-IDF SPI, and
software SPI are available as v1 backends when the build environment exposes
the corresponding framework support. SPI transactions use
`beginTransaction()` / `endTransaction()` when CS must stay asserted across
multiple transfers. Start with
[`examples/v1/HowToUse/SPI`](examples/v1/HowToUse/SPI/) for a
logic-analyzer-friendly sketch that needs no SPI slave.

UART also follows the same Bus / Accessor shape. **The baud rate lives
on `uart::AccessConfig` (the accessor side), not on the bus** — the same
physical port can serve different peers with different settings. Pick
your accessor from three: TX-only (`TxAccessor`), RX-only
(`RxAccessor`), or the two-way facade (`Accessor`) — use the split pair
when separate tasks send and receive, the facade for simple
command-response code (the split accessors are the primary API; see
[`spec/design/uart.md`](spec/design/uart.md)). Start with
[`examples/v1/HowToUse/UART`](examples/v1/HowToUse/UART/) for an Arduino
sketch that uses USB Serial for logs and `Serial1` as the M5HAL UART bus.
Connect TX to RX to confirm loopback receive without another UART device.
[`examples/v1/HowToUse/UARTEcho`](examples/v1/HowToUse/UARTEcho/) goes one
step further: it echoes everything received back to the sender through the
`StreamSink` adapter, showing how the accessors compose with the
Source / Sink stream model.

[`examples/v1/HowToUse/I2SAudio`](examples/v1/HowToUse/I2SAudio/) plays a
sine wave through the built-in speaker using the `i2s::Bus` TX path
(I2S needs board-specific amplifier setup; the sketch covers M5Stack
Core2 V1.1, with CoreS3 wiring included but not yet verified).

[`examples/v1/HowToUse/Bytecode`](examples/v1/HowToUse/Bytecode/) drives
GPIO, I2C, and SPI from bytecode scripts written out as plain byte arrays
(the "init sequence as a const table" pattern), executed on the buttons of
an M5Stack Core BASIC.

An index of all examples, with wiring and expected output, is in
[`examples/v1/HowToUse/README.md`](examples/v1/HowToUse/README.md).
