# examples/v2/HowToUse — v2 API examples

Self-contained Arduino sketches for the v2 API. Each sketch is heavily
commented and runnable as-is in the Arduino IDE; the table below is the
quick map. Pin defaults assume M5Stack Core (Basic / Gray / Fire) style
wiring unless noted.

| Example | Hardware | What it shows | Wiring |
|---|---|---|---|
| [I2C](I2C/) | Any ESP32 board with an I2C device (scans the bus, uses the first responder) | Bus / Accessor basics: scan, `probe`, `readRegister`, burst read, `ScopedAccess` | SDA=21, SCL=22 |
| [I2CRegistry](I2CRegistry/) | Any ESP32 board; no I2C device required | `LogicalBusConfig`, intent-based `commitBuses()`, and querying `BackendKind` allocation results | shared bus SDA=21/SCL=22; intent buses internal SDA=21/SCL=22, PortA SDA=25/SCL=26, HAT SDA=18/SCL=19 |
| [SPI](SPI/) | Any ESP32 board; no SPI slave required | Plain write, command+data, dummy clocks, explicit `beginTransaction` / `endTransaction` — wire activity for a logic analyzer | SCLK=18, MOSI=23, MISO=19, D/C=27, CS=14 |
| [UART](UART/) | Any ESP32 board | UART Bus / Accessor basics; USB Serial for logs, `Serial1` as the M5HAL bus | TX=17, RX=16; jumper TX→RX for loopback |
| [UARTEcho](UARTEcho/) | Any ESP32 board + an external UART peer | Echo through the `StreamReader` / `StreamSink` adapters (Source / Sink stream model) | peer TX→RX=16, peer RX←TX=17, shared GND. Do **not** jumper TX to RX on the same board |
| [I2SAudio](I2SAudio/) | M5Stack Core2 V1.1 (verified); CoreS3 wiring included but unverified | 440 Hz sine playback through the local `i2s::Bus` TX path, including the board-specific amplifier setup | none (built-in speaker) |
| [Bytecode](Bytecode/) | M5Stack Core BASIC | GPIO / I2C / SPI driven from bytecode scripts stored as const byte arrays; buttons A/B/C run the scripts | none (uses on-board LCD / power IC) |
| [Remote](Remote/) | PC (POSIX native) + ESP32 with RemoteServer firmware | Remote I2C scan and GPIO read via the `Hal` facade: `connect(endpoint)`, `acquire`, `probe` | USB cable between PC and ESP32 |
| [RemoteI2S](RemoteI2S/) | PC (POSIX native) + ESP32 with RemoteServerTCP or RemoteServer firmware, Core2-class amplifier wiring | Streaming a sine-wave tone to the remote I2S speaker path over `BusStreamTransfer` via the `Hal` facade | USB serial or TCP between PC and ESP32 |

Every sketch prints its progress to USB Serial (115200). Most sketches acquire
an interned bus from `M5_Hal` (e.g. `M5_Hal.I2C.acquire(cfg)`) and own it through
the returned `shared_ptr`; the registry itself retains only a weak reference.
Bytecode keeps its buses as direct-constructed globals
(the escape hatch) so its script-driven accessors bind at startup. To force a
specific backend instead of the build's default, pass a suffixed config type
(`BusConfig_software` / `_arduino` / `_espidf`) to acquire (see the comments in
the sketch and `spec/design/variants.md` in the repository).

The I2S API itself supports local TX, local RX, and full duplex when the backend
exposes DIN/DOUT. `I2SAudio` is intentionally only the built-in-speaker playback
example; it is not the full I2S feature boundary.

`Remote` is a native (non-Arduino) POSIX host binary, not an Arduino sketch: it
connects from a PC to an ESP32 over USB serial. The device side runs the
RemoteServer firmware from `examples/v2/RemoteServer/` (flash any
`RemoteServer_*` PlatformIO env). The full protocol test harness is
`RemoteTest_host` (`examples/v2/RemoteTest/`); this example focuses on the high-level
`Hal` facade API (`connect(endpoint)` → `acquire` → `probe`).

Remote buses and GPIO objects are connection-scoped. Reconnecting closes old
bus proxies (`CLOSED`) rather than rebinding them to the new peer. Explicit
`release(shared_ptr&)` requires the sole remaining owner, so destroy accessors
and other aliases first; success clears the passed handle and failure preserves
it. Retained GPIO objects are safe only while their `Hal` is alive; after a
reconnect they retain their final cached reads and ignore writes/mode changes.
See `spec/design/remote.md` for the complete session and callback contract.
