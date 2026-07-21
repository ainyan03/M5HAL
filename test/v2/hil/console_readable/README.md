# Console readableBytes HIL

This fixture checks that the ESP UART console reports buffered input through
`readableBytes()`. The device echoes only after that count becomes non-zero;
the `uart_echo` host supplies and verifies the payloads.

## Hardware

- Device: classic ESP32 with UART0 connected to its USB bridge
- Host connection: the same USB cable; no additional wiring

## Run

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
pio run -e v2_hil_console_readable_device_esp32 \
  -t upload --upload-port <device-port>
pio run -e v2_hil_uart_echo_host
M5HAL_POSIX_UART_PORT=<device-port> \
  .pio/build/v2_hil_uart_echo_host/program
```

## Acceptance

All three `UartEcho` cases must pass, including the binary payload and 512-byte
burst. This fixture covers the UART0 console transport; other console transports
are outside its runtime scope.
