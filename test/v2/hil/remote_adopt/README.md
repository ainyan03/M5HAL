# Remote adopted-bus HIL

This fixture checks repeated remote use of externally initialized Arduino I2C
and SPI buses, plus GPIO output-read event delivery. The Core2 remote server is
driven by one POSIX host connection through three close/reacquire cycles.

## Hardware and wiring

- Remote server: M5Stack Core2, USB-UART at 115200 baud
- Responder: M5Stack CoreS3
- I2C: Core2 SDA=32/SCL=33 to CoreS3 SDA=2/SCL=1, address `0x42`
- SPI: Core2 CLK=13/MOSI=14/MISO=36/CS=26 to CoreS3
  CLK=18/MOSI=17/MISO=8/CS=9
- Common GND between the boards

Use explicit serial ports because enumeration can change when either board is
reconnected.

## Build the server and host

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
pio run -e v2_hil_remote_adopt_device_arduino_esp32 \
  -t upload --upload-port <Core2-port>
pio run -e v2_hil_remote_adopt_host
```

## GPIO phase

The responder is not used. Do not drive GPIO26 externally.

```sh
.pio/build/v2_hil_remote_adopt_host/program uart:<Core2-port> gpio
```

Acceptance is `PASS mode=gpio pin=26 sequence=0-1-0` with at least one rising
and one falling event.

## I2C phase

Flash the register-map responder. In a separate terminal, monitor the responder until its
`regmap slave fixture: ...` boot banner appears and confirm that no repeating `init failed` error follows:

```sh
pio run -e v2_hil_i2c_slave_device_esp32s3 \
  -t upload --upload-port <CoreS3-port>
pio device monitor --port <CoreS3-port> -b 115200

# After the responder is initialized, run this in the host terminal:
.pio/build/v2_hil_remote_adopt_host/program uart:<Core2-port> i2c
```

Acceptance is `PASS mode=i2c cycles=3 state_preserved=1`.

## SPI phase

Flash the SPI responder. In a separate terminal, monitor it and wait for
`ARMED SPI_SLAVE_M5HAL` before starting the host:

```sh
pio run -e v2_hil_spi_slave_echo_esp32s3 \
  -t upload --upload-port <CoreS3-port>
pio device monitor --port <CoreS3-port> -b 115200

# After ARMED appears, run this in the host terminal:
.pio/build/v2_hil_remote_adopt_host/program uart:<Core2-port> spi
```

Acceptance is `PASS mode=spi cycles=3 bytes=96`. Payload continuity belongs to
the dedicated [`../spi_slave/`](../spi_slave/) fixture; this phase uses completed
byte totals as its oracle.
