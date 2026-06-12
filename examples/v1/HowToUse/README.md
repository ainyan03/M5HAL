# examples/v1/HowToUse — v1 API examples

Self-contained Arduino sketches for the v1 API. Each sketch is heavily
commented and runnable as-is in the Arduino IDE; the table below is the
quick map. Pin defaults assume M5Stack Core (Basic / Gray / Fire) style
wiring unless noted.

| Example | Hardware | What it shows | Wiring |
|---|---|---|---|
| [I2C](I2C/) | Any ESP32 board with an I2C device (scans the bus, uses the first responder) | Bus / Accessor basics: scan, `probe`, `readRegister`, burst read, `ScopedAccess` | SDA=21, SCL=22 |
| [SPI](SPI/) | Any ESP32 board; no SPI slave required | Plain write, command+data, dummy clocks, explicit `beginTransaction` / `endTransaction` — wire activity for a logic analyzer | SCLK=18, MOSI=23, MISO=19, D/C=2, CS=5 |
| [UART](UART/) | Any ESP32 board | UART Bus / Accessor basics; USB Serial for logs, `Serial1` as the M5HAL bus | TX=17, RX=16; jumper TX→RX for loopback |
| [UARTEcho](UARTEcho/) | Any ESP32 board + an external UART peer | Echo through the `StreamReader` / `StreamSink` adapters (Source / Sink stream model) | peer TX→RX=16, peer RX←TX=17, shared GND. Do **not** jumper TX to RX on the same board |
| [I2SAudio](I2SAudio/) | M5Stack Core2 V1.1 (verified); CoreS3 wiring included but unverified | 440 Hz sine playback through `i2s::Bus` TX, including the board-specific amplifier setup | none (built-in speaker) |
| [Bytecode](Bytecode/) | M5Stack Core BASIC | GPIO / I2C / SPI driven from bytecode scripts stored as const byte arrays; buttons A/B/C run the scripts | none (uses on-board LCD / power IC) |

Every sketch prints its progress to USB Serial (115200). To force a
specific backend instead of the build's default, change the
`Example*Bus` type alias near the top of each sketch (see the comments
in the sketch and `spec/design/variants.md` in the repository).
