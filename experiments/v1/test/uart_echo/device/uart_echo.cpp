// SPDX-License-Identifier: MIT
//
// HIL device firmware — UART echo for M5Stack Core BASIC (classic ESP32).
//
// Drives UART0 (GPIO1 = TX / GPIO3 = RX, i.e. the USB-serial bridge) through the
// M5HAL v1 UART Bus and echoes every received byte straight back. There is
// deliberately NO Arduino `Serial` console and NO logging: the USB channel is a
// clean echo pipe so the host driver can verify round-trips byte-exact.
//
// Pair with the host driver: experiments/v1/test/uart_echo/host/uart_echo.cpp.
// Run (or use experiments/v1/test/hil-run.sh uart_echo):
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/hil.ini.cli
//   pio run -e v1_hil_uart_echo_device_esp32 -t upload

#include <Arduino.h>
#include <M5HAL_v1.hpp>

namespace m5hal = m5::hal::v1;

namespace {

constexpr int PIN_UART_TX = 1;  // UART0 TX (USB bridge) on classic ESP32
constexpr int PIN_UART_RX = 3;  // UART0 RX (USB bridge) on classic ESP32

// Override at build time to match the host (e.g. for a high-baud HIL run):
//   PLATFORMIO_BUILD_FLAGS="-DM5HAL_HIL_ECHO_BAUD=3000000" pio run -e ... -t upload
// (The Core BASIC v2.7 USB bridge is a CH9102, good to ~3 Mbaud+; the older
//  CP2104 revision caps ~2 Mbaud. The M5HAL UART itself goes higher still.)
#ifndef M5HAL_HIL_ECHO_BAUD
#define M5HAL_HIL_ECHO_BAUD 115200
#endif
constexpr uint32_t BAUD = M5HAL_HIL_ECHO_BAUD;

m5hal::uart::Bus uart_bus;  // flat-injected = arduino UART variant on this build
m5hal::uart::UARTAccessConfig uart_cfg;

}  // namespace

void setup()
{
    // Bind M5HAL UART to UART0 (HardwareSerial 0 = `Serial` on classic ESP32).
    // We never call Serial.begin()/print() ourselves — the M5HAL arduino UART
    // variant begins/configures the port lazily on the first read/write below.
    //
    // Enlarge the RX/TX ring buffer (vs the Arduino default 256) so a high-baud
    // burst is absorbed while the echo loop drains it: at 3 Mbaud a 512-byte
    // burst arrives in ~1.7 ms and would overflow a 256-byte RX ring. The arduino
    // UART variant honors UARTBusConfig.{rx,tx}_buffer_size (applied before its
    // lazy begin()), so no direct Serial.setRxBufferSize() is needed.
    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.serial         = &Serial;
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 2048;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);

    uart_cfg.baud_rate             = BAUD;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;
}

void loop()
{
    m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};

    auto readable = uart_rx.readableBytes();
    if (!readable.has_value() || readable.value() == 0) {
        return;
    }

    uint8_t buf[1024];
    size_t want = readable.value();
    if (want > sizeof(buf)) {
        want = sizeof(buf);
    }
    auto got = uart_rx.read(buf, want);
    if (!got.has_value() || got.value() == 0) {
        return;
    }
    (void)uart_tx.write(buf, got.value());  // echo back verbatim
}
