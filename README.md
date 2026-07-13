# M5HAL

*日本語: [README.ja.md](README.ja.md)*

## Overview
HAL (Hardware Abstraction Layer) for M5 Products.

The **v0 API is stable** and stays the default, so existing code keeps
working unchanged. The **v2 API is under active development** and is
opt-in — include `<M5HAL_v2.hpp>` explicitly to try it.

## Requirements

- An ESP32-family board. The published packages target the `espressif32`
  platform (Arduino-ESP32 or ESP-IDF >= 4.4).
- A compiler with C++17 support.
- [M5Utility](https://github.com/m5stack/M5Utility) — PlatformIO and the
  ESP-IDF component manager pull it in automatically; in the Arduino IDE,
  install it alongside M5HAL.

## Installation

- **Arduino IDE**: install "M5HAL" from the Library Manager, plus
  "M5Utility" alongside it.
- **PlatformIO**: add to `platformio.ini`:
  ```ini
  lib_deps =
      m5stack/M5HAL
  ```
  M5Utility resolves automatically as a declared dependency.
- **ESP-IDF component manager**: add to your project's `idf_component.yml`:
  ```yaml
  dependencies:
    m5stack/M5HAL: "*"
  ```

## Documentation

- Confirmed specification documents live under [`spec/`](spec/README.md)
  (also bundled in the release packages).

## Where to start

| Reader | Start here |
|---|---|
| Existing v0 user | Keep using `<M5HAL.hpp>` or `<M5HAL_v0.hpp>`. Read [v0 / v2 coexistence](#v0--v2-coexistence) only if you need to understand the migration period. |
| Trying v2 in a sketch | Read [Trying the v2 API](#trying-the-v2-api), then open [`examples/v2/HowToUse/I2C`](examples/v2/HowToUse/I2C/), [`examples/v2/HowToUse/SPI`](examples/v2/HowToUse/SPI/), or [`examples/v2/HowToUse/UART`](examples/v2/HowToUse/UART/). |
| Implementing a backend or reviewing internals | Use [`spec/README.md`](spec/README.md) as the map. The main design files are `bus_accessor`, `i2c`, `spi`, `gpio`, and `variants`. |

## Trying the v2 API

v2 is opt-in. Include `<M5HAL_v2.hpp>`. Mixing a v0 entry header into the
same translation unit is also supported (see
[v0 / v2 coexistence](#v0--v2-coexistence)), but one generation per file
reads better.

The current v2 bus API is centered on:

- **Bus** — the physical bus instance (`i2c::Bus`, `spi::Bus`, `uart::Bus`,
  `i2s::Bus`, or an explicit variant such as `spi::Bus_software`)
- **Accessor** — one target device on that bus, with per-device settings
  such as address, chip-select pin, baud rate, frequency, timeout, and SPI mode
- **TransferDesc** — per-transfer metadata such as an I2C register prefix
  or SPI command/address/dummy phases; UART does not need a transfer descriptor
- **Source / Sink** — streaming-friendly data input/output abstractions;
  span and raw pointer overloads are available

**Acquire one shared bus per wiring**: v2 has no hidden singleton bus and
`M5_Hal` does not keep buses alive. Its per-kind registry interns weak references.
The recommended path is to acquire a bus by its wiring —
`M5_Hal.I2C.acquire(cfg)` returns a shared owner
interned by its pins, so a board-support layer and user code that name the same
pins get the *same* instance — one physical bus, one lock — instead of fighting
over the wire. Build an accessor straight from the handle and it co-owns the
bus. The bus remains alive until the last returned handle or co-owning accessor
is destroyed; then its backend is released and the weak registry entry becomes
reclaimable. `M5_Hal` bundles the GPIO/bus registries and the service runner.

Escape hatch: when you want to own a bus yourself, construct it directly
(`i2c::Bus bus; bus.init(cfg);`) and hand it to accessors by reference —
registry acquisition is just the recommended default. See
[`spec/design/bus_accessor.md`](spec/design/bus_accessor.md) for the
bus-ownership model and
[`examples/v2/HowToUse/I2CRegistry`](examples/v2/HowToUse/I2CRegistry/)
for the sharing + hardware-allocation demo.

Minimal I2C shape (**Arduino framework**):

> **PlatformIO consumers:** M5HAL v2 needs C++17. On `espressif32@6.x`
> (Arduino core 2.x) the default is gnu++11, so add to your
> `platformio.ini`: `build_flags = -std=gnu++17` and
> `build_unflags = -std=gnu++11`. arduino-esp32 3.x and the Arduino IDE
> already default to C++17. (If you forget, the header stops the build
> with a one-line `#error` instead of a wall of `constexpr` errors.)
>
> The example is written for the Arduino backend: `BusConfig`'s field set
> is **variant-dependent** (`bus_cfg.wire = &Wire` exists only on the
> Arduino I2C backend). On ESP-IDF or the software backend the config
> carries different fields — see the note after the example.

```cpp
#include <M5HAL_v2.hpp>
#include <Wire.h>

#include <memory>

namespace m5hal = m5::hal::v2;

std::shared_ptr<m5hal::i2c::IBus> i2c_bus;  // a shared owner

void setup()
{
    // Tag-typed pins: either order is correct (no swapped-pin accidents).
    m5hal::i2c::BusConfig bus_cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
    bus_cfg.wire = &Wire;

    // Acquire the interned bus; this shared_ptr owns its lifetime.
    auto acquired = m5hal::M5_Hal.I2C.acquire(bus_cfg);
    if (!acquired) return;
    i2c_bus = acquired.value();

    // Fill every field BEFORE constructing the accessor: the config is
    // copied by value at construction and then frozen. (For deferred
    // setup, default-construct the accessor and call setConfig() later.)
    m5hal::i2c::AccessConfig dev_cfg;   // field assignment; no tag ctor
    dev_cfg.i2c_addr        = 0x76;
    dev_cfg.freq            = 100000;
    dev_cfg.wire_timeout_ms = 100;
    // dev_cfg.register_address_bytes = 2;  // only for 2-byte register-address devices

    m5hal::i2c::MasterAccessor dev{i2c_bus, dev_cfg};  // co-owns the acquired bus

    // Every transfer returns result_t<T> — unwrap it, don't assign directly.
    auto id = dev.readRegister(0x00);   // result_t<uint8_t>, NOT uint8_t
    if (!id) return;                    // check before use (id.error() has the code)
    if (id.value() == 0x60) {
        // ... matched the expected WHO_AM_I ...
    }
}
```

For a complete Arduino sketch, start with
[`examples/v2/HowToUse/I2C`](examples/v2/HowToUse/I2C/).
The example scans the bus, creates an accessor for the first responding
device, demonstrates register reads, and shows `ScopedAccess` for grouping
multiple transfers under one bus lock.

When you need a specific backend, pass a suffixed CONFIG type to acquire.
For example, to drive the same pins with the software (bit-bang) I2C backend:

```cpp
m5hal::i2c::BusConfig_software bus_cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
auto i2c_bus = m5hal::M5_Hal.I2C.acquire(bus_cfg).value();
```

`#include <Wire.h>` is needed because the default Arduino I2C backend
(`i2c::BusConfig_arduino`) carries a `TwoWire` handle (`bus_cfg.wire = &Wire`);
it goes away when you pass the software or ESP-IDF config instead. The
unsuffixed `i2c::BusConfig` spelling is a type alias to the first backend the
build environment offered (`BusConfig_arduino`, ...), so its field set (whether
there is a `wire`, ...) follows the selected variant.

### Common mistakes (habits from other libraries that do NOT apply here)

If you (or a code-generating assistant) reach for one of these shapes,
it will not compile — M5HAL v2 does it differently on purpose:

- **Positional bus pins** — `BusConfig{22, 21}` is a compile error. Pins
  are tag-typed: `BusConfig{Scl{22}, Sda{21}}` (so a swapped pair cannot
  slip through). See the tag structs `Scl` / `Sda`.
- **Type-driven register width** — there is no `readRegister<uint16_t>`
  or `readRegister16`. The register-address width comes from the config
  field `register_address_bytes`, not from a template argument.
- **Treating `acquire()` as a raw `shared_ptr`** — `M5_Hal.I2C.acquire(cfg)`
  returns `result_t<shared_ptr<IBus>>`, not a bare `shared_ptr`. Check it
  (`if (!acquired) ...`) and then `acquired.value()`. The same goes for the
  register accessors: they return `result_t<T>`, so `.value()` after an
  error check, never a direct assignment.

SPI follows the same Bus / Accessor shape. Arduino SPI, ESP-IDF SPI, and
software SPI are available as v2 backends when the build environment exposes
the corresponding framework support. SPI transactions use
`beginTransaction()` / `endTransaction()` when CS must stay asserted across
multiple transfers. Start with
[`examples/v2/HowToUse/SPI`](examples/v2/HowToUse/SPI/) for a
logic-analyzer-friendly sketch that needs no SPI slave.

UART also follows the same Bus / Accessor shape. **The baud rate lives
on `uart::AccessConfig` (the accessor side), not on the bus** — the same
physical port can serve different peers with different settings. Pick
your accessor from three: TX-only (`TxAccessor`), RX-only
(`RxAccessor`), or the two-way facade (`Accessor`) — use the split pair
when separate tasks send and receive, the facade for simple
command-response code (the split accessors are the primary API; see
[`spec/design/uart.md`](spec/design/uart.md)). Start with
[`examples/v2/HowToUse/UART`](examples/v2/HowToUse/UART/) for an Arduino
sketch that uses USB Serial for logs and `Serial1` as the M5HAL UART bus.
Connect TX to RX to confirm loopback receive without another UART device.
[`examples/v2/HowToUse/UARTEcho`](examples/v2/HowToUse/UARTEcho/) goes one
step further: it echoes everything received back to the sender through the
`StreamSink` adapter, showing how the accessors compose with the
Source / Sink stream model.

I2S is a continuous stream bus. The local ESP-IDF gen5 backend supports
playback (TX), recording (RX), and full duplex: DOUT enables TX, DIN enables
RX, and both pins enable independent TX/RX DMA paths. Use `i2s::TxAccessor`
for playback, `i2s::RxAccessor` for capture, or `i2s::Accessor` when one
object should bundle both directions. The remote I2S proxy exposes TX
streaming with credit flow control and synchronous RX `read` /
`readableBytes` requests; see
[`examples/v2/HowToUse/RemoteI2S`](examples/v2/HowToUse/RemoteI2S/) for a
host-side example.

[`examples/v2/HowToUse/I2SAudio`](examples/v2/HowToUse/I2SAudio/) plays a
sine wave through the built-in speaker using the local `i2s::Bus` TX path
(I2S needs board-specific amplifier setup; the sketch covers M5Stack
Core2 V1.1, with CoreS3 wiring included but not yet verified).

Remote examples are available for both serial and TCP transports:
[`examples/v2/HowToUse/Remote`](examples/v2/HowToUse/Remote/) is the host
facade entry point,
[`examples/v2/RemoteServerTCP`](examples/v2/RemoteServerTCP/) exposes a
device over TCP, and [`examples/v2/RemoteTest`](examples/v2/RemoteTest/)
is the host-side protocol test harness.

A remote `Hal` owns one connection session and one RPC serialization gate;
every bus, GPIO proxy, compatibility session view, and backend operation for
that connection uses that same gate. Reconnecting closes the old session:
pre-reconnect bus proxies then return `CLOSED` and are never rebound to the new
peer. Retained GPIO objects remain memory-safe only while their owning `Hal`
lives; after reconnect they expose their final cache and ignore writes/mode
changes. Explicit `BusView::release(shared_ptr&)` requires the caller to be the
sole owner (destroy accessors and aliases first), consumes and clears the handle
on success, and leaves it intact on failure. See
[`spec/design/remote.md`](spec/design/remote.md) and
[`spec/design/bus_accessor.md`](spec/design/bus_accessor.md) for the complete
lifetime, callback, and quarantine contracts.

[`examples/v2/HowToUse/Bytecode`](examples/v2/HowToUse/Bytecode/) drives
GPIO, I2C, and SPI from bytecode scripts written out as plain byte arrays
(the "init sequence as a const table" pattern), executed on the buttons of
an M5Stack Core BASIC.

An index of all examples, with wiring and expected output, is in
[`examples/v2/HowToUse/README.md`](examples/v2/HowToUse/README.md).

## API generations (v0 / v2) and release numbering

M5HAL distinguishes between **API generation** (the spec lineage) and
**release version** (the library version number). The generation number
equals the major release version in which that generation becomes the
default — v0 ↔ `0.x`, v2 ↔ `2.x`. (There is no `v1` API generation: the
number is intentionally skipped so a generation number never collides with
a differently-meant major release number.)

- **v0** — the legacy API generation, shipped as the `v0.0.x` releases.
- **v2** — the new API generation, designed as a clean-slate redesign.
- **`1.x.x`** — the **migration-period release line** that ships both API
  generations side-by-side in a single library.

While the major release version is `0` (i.e. `v0.x.y`), **v0 stays the
default** so existing consumers keep working without code changes. From
the `1.0` release onward, users who explicitly opt in can try the v2 API.
When the major version eventually reaches `2` (i.e. `v2.x.y`), v2 becomes
the default.

## v0 / v2 coexistence

M5HAL adopts a coexistence strategy so that existing v0 consumers keep
using their code unchanged while the v2 API lives side-by-side in the same
library. The entry headers are:

| Header | Exposes | For |
|---|---|---|
| `<M5HAL.hpp>` | (shim → v0 by default) | Backward compatibility — existing code that already includes `<M5HAL.hpp>` keeps working unchanged. New code should prefer one of the explicit headers below. |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (= v0, via `inline namespace v0`) | Code that explicitly opts into the v0 (legacy) API |
| `<M5HAL_v2.hpp>` | `m5::hal::v2::*` | Code that explicitly opts into the v2 API |

- **Both entries may share a translation unit.** The include guards and
  platform-detection macros are generation-separated, so one `.cpp` may
  include both a v0 entry (`<M5HAL.hpp>` shim or `<M5HAL_v0.hpp>` direct)
  and `<M5HAL_v2.hpp>` — e.g. while migrating that file gradually. An
  intermediate library should still make its intended generation explicit
  per TU for readability.

For the inline-namespace default switch (`M5HAL_V0_INLINE`), the
generation-separated platform macros, and the forward-compatible
`hal/<vN>/` layout that lets future generations be added without touching
existing consumers, see
[`spec/design/v0_v2_coexistence.md`](spec/design/v0_v2_coexistence.md).
