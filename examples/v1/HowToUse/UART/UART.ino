// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseUART
//
// Minimal UART Bus / Accessor sketch. USB Serial is used for logs, while
// Serial1 is handed to M5HAL as the UART bus.
//
// Pin defaults match a common M5Stack Core Port-C style wiring:
//   TX=17, RX=16
//
// To verify both transmit and receive without an external UART device, connect
// TX to RX with a jumper. The sketch periodically writes a line and reports any
// bytes echoed back through the loopback.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>

#include <string.h>

namespace m5hal = m5::hal::v1;

constexpr int PIN_UART_TX = 17;
constexpr int PIN_UART_RX = 16;

#ifndef M5HAL_EXAMPLE_HOWTOUSEUART_BAUD
#define M5HAL_EXAMPLE_HOWTOUSEUART_BAUD 115200
#endif

// m5hal::uart::Bus resolves to the first backend the build offers
// (framework scan order; see spec/design/variants.md). Uncomment the
// variant::* alias below to force a specific backend instead.
using ExampleUARTBus = m5hal::uart::Bus;
// using ExampleUARTBus = m5hal::uart::variant::arduino::Bus;

ExampleUARTBus uart_bus;
m5hal::uart::UARTAccessConfig uart_cfg;
bool uart_ready         = false;
uint32_t tx_counter     = 0;
uint32_t last_send_msec = 0;

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %d\n", label, static_cast<int>(error));
}

static void writeLine(const char* text)
{
    m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    auto r = uart_tx.write(reinterpret_cast<const uint8_t*>(text), strlen(text));
    if (!r.has_value()) {
        printError("uart write", r.error());
        return;
    }
    Serial.printf("uart write: %u bytes\n", static_cast<unsigned>(r.value()));
}

// To consume received bytes as a `data::Source` (or feed the TX side as a
// `data::Sink`), wrap the accessor with the StreamSource / StreamSink
// adapters — see the UARTEcho example and spec/design/data_io.md.
static void readEcho(void)
{
    m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};
    auto readable = uart_rx.readableBytes();
    if (!readable.has_value()) {
        printError("uart readableBytes", readable.error());
        return;
    }
    if (readable.value() == 0) {
        Serial.println("uart read: no bytes (connect TX to RX for loopback)");
        return;
    }

    uint8_t rx[64] = {};
    size_t want    = readable.value();
    if (want > sizeof(rx)) {
        want = sizeof(rx);
    }

    auto r = uart_rx.read(rx, want);
    if (!r.has_value()) {
        printError("uart read", r.error());
        return;
    }

    Serial.printf("uart read: %u bytes: ", static_cast<unsigned>(r.value()));
    Serial.write(rx, r.value());
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("M5HAL HowToUseUART");
    Serial.printf("TX=%d RX=%d baud=%u\n", PIN_UART_TX, PIN_UART_RX,
                  static_cast<unsigned>(M5HAL_EXAMPLE_HOWTOUSEUART_BAUD));
    Serial.println("Connect TX to RX to see loopback bytes.");

    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.serial = &Serial1;
    bus_cfg.pin_tx = PIN_UART_TX;
    bus_cfg.pin_rx = PIN_UART_RX;

    auto init = uart_bus.init(bus_cfg);
    if (!init.has_value()) {
        printError("uart bus init", init.error());
        return;
    }

    uart_cfg.baud_rate             = M5HAL_EXAMPLE_HOWTOUSEUART_BAUD;
    uart_cfg.first_byte_timeout_ms = 20;
    uart_cfg.inter_byte_timeout_ms = 5;
    uart_cfg.write_timeout_ms      = 100;

    uart_ready = true;
    writeLine("M5HAL UART hello\r\n");
    delay(20);
    readEcho();
}

void loop()
{
    if (!uart_ready) {
        delay(1000);
        return;
    }

    const uint32_t now = millis();
    if (now - last_send_msec < 2000) {
        delay(10);
        return;
    }
    last_send_msec = now;

    char line[48] = {};
    snprintf(line, sizeof(line), "M5HAL UART count=%lu\r\n", static_cast<unsigned long>(tx_counter++));
    writeLine(line);
    delay(20);
    readEcho();
}
