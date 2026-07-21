# ESP-IDF SPI slave HIL

This fixture runs the M5HAL ESP-IDF SPI slave backend on a CoreS3 and checks
receive continuity across repeated CS assertions.

## Hardware and wiring

| Signal | Core2 master | CoreS3 slave |
|---|---:|---:|
| CLK | GPIO13 | GPIO18 |
| MOSI | GPIO14 | GPIO17 |
| MISO | GPIO36 | GPIO8 |
| CS | GPIO26 | GPIO9 |
| GND | common | common |

Use SPI mode 1, 1 MHz, and 32 bytes per transaction.

## Run

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
pio run -e v2_hil_spi_slave_echo_esp32s3 \
  -t upload --upload-port <CoreS3-port>
pio device monitor --port <CoreS3-port> -b 115200
```

In that monitor, wait for `ARMED SPI_SLAVE_M5HAL`, then use an independent SPI
master with the pin mapping above. The public tree does not prescribe a specific
master firmware; it must perform this sequence:

- mode 1 at 1 MHz, full duplex, 32 bytes per CS assertion;
- 102 assertions, transmitting one continuous modulo-256 `+1` byte ramp across
  transaction boundaries;
- no intentional CS-low hold after each transfer, followed by a 20,000 us
  CS-high gap before the next assertion.

The gap must be between CS assertions; holding CS low is not equivalent.

## Acceptance

The CoreS3 must print:

```text
RESULT SPI_SLAVE_M5HAL PASS mode=1 len=32 transactions=101 received=101 intra_bad=0 inter_bad=0
```

The fixture treats the first observed zero-byte synchronization boundary
separately and checks 101 complete data frames. The CoreS3 result is the oracle;
the classic ESP32 master's final RX byte is not used for continuity acceptance.
