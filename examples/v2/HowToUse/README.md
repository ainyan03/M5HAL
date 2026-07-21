# examples/v2/HowToUse — v2 API examples

This directory contains runnable examples, not the API contract. Follow each
example's source comments for setup details; use the linked specification for
design and lifetime rules. Arduino sketches print progress at 115200 baud.

| Example | Required hardware / wiring | Expected result |
|---|---|---|
| [I2C](I2C/) | ESP32 + an I2C device; SDA=21, SCL=22 | Scans the bus, selects the first responder, and reports register reads |
| [I2CRegistry](I2CRegistry/) | ESP32; no I2C device required | Reports shared-bus identity and allocation results for several wiring intents |
| [SPI](SPI/) | ESP32; no slave required; SCLK=18, MOSI=23, MISO=19, D/C=27, CS=14 | Emits writes and framed transfers for observation with a logic analyzer |
| [UART](UART/) | ESP32; jumper TX=17 to RX=16 | Sends periodic lines and receives them through loopback |
| [UARTEcho](UARTEcho/) | ESP32 + external UART peer; peer TX→16, peer RX←17, shared GND | Echoes peer bytes and reports the running byte count |
| [I2SAudio](I2SAudio/) | M5Stack Core2 V1.1 built-in speaker; CoreS3 wiring is included but unverified | Plays a 440 Hz sine wave |
| [Bytecode](Bytecode/) | M5Stack Core BASIC, no external wiring | Buttons A/B/C run GPIO, I2C, and SPI bytecode sequences |
| [Remote](Remote/) | POSIX PC + ESP32 running `examples/v2/RemoteServer/`; USB serial | Connects, scans remote I2C, reads GPIO, then prints `Done.` |
| [RemoteI2S](RemoteI2S/) | POSIX PC + ESP32 RemoteServer with Core2-class speaker wiring; USB serial or TCP | Connects, acquires remote I2S, and streams a tone |

The remote device can instead use
[`RemoteServerTCP`](../RemoteServerTCP/) for TCP. Protocol-level validation
lives in [`RemoteTest`](../RemoteTest/); it is a test harness rather than a
HowToUse example.

For API contracts, start at the [specification map](../../../spec/README.md).
The [Bus / Accessor specification](../../../spec/design/bus_accessor.md)
covers ownership and locking, [variants](../../../spec/design/variants.md)
covers provider selection, and [remote](../../../spec/design/remote.md) covers
connection lifetime.
