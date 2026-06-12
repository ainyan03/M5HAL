// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseUARTEcho
//
// UART echo through the Stream adapters: everything received on RX is sent
// straight back out of TX. The whole echo path is one call —
//
//   StreamSink echo{uart_tx, DataSpan{scratch, sizeof(scratch)}};
//   uart_rx.read(echo, sizeof(scratch));
//
// `read(Sink&, len)` pushes received bytes into any Sink; `StreamSink`
// is the adapter that turns the TX accessor into such a Sink, so each
// received chunk is transmitted as soon as it is committed. The same
// adapters work for any byte transport that implements the small
// `data::StreamReader` / `data::StreamWriter` interfaces (see
// spec/design/data_io.md, "Stream adapters").
//
// Wiring: connect an external UART peer (USB-serial adapter, another
// board, ...) — peer TX -> RX=16, peer RX <- TX=17, GND shared. Type
// into the peer's terminal and the characters come back.
// Do NOT jumper TX to RX on this board: the echo would feed itself and
// loop forever.
//
// USB Serial is used for logs only.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>

namespace m5hal = m5::hal::v1;

constexpr int PIN_UART_TX = 17;
constexpr int PIN_UART_RX = 16;

#ifndef M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD
#define M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD 115200
#endif

// m5hal::uart::Bus resolves to the first backend the build offers
// (framework scan order; see spec/design/variants.md).
using ExampleUARTBus = m5hal::uart::Bus;
// using ExampleUARTBus = m5hal::uart::variant::arduino::Bus;

ExampleUARTBus uart_bus;
m5hal::uart::UARTAccessConfig uart_cfg;
bool uart_ready      = false;
uint32_t echo_total  = 0;

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %d\n", label, static_cast<int>(error));
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("M5HAL HowToUseUARTEcho");
    Serial.printf("TX=%d RX=%d baud=%u\n", PIN_UART_TX, PIN_UART_RX,
                  static_cast<unsigned>(M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD));
    Serial.println("Connect an external UART peer; its bytes are echoed back.");

    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.serial = &Serial1;
    bus_cfg.pin_tx = PIN_UART_TX;
    bus_cfg.pin_rx = PIN_UART_RX;

    auto init = uart_bus.init(bus_cfg);
    if (!init.has_value()) {
        printError("uart bus init", init.error());
        return;
    }

    uart_cfg.baud_rate             = M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD;
    uart_cfg.first_byte_timeout_ms = 20;
    uart_cfg.inter_byte_timeout_ms = 5;
    uart_cfg.write_timeout_ms      = 100;

    uart_ready = true;
}

void loop()
{
    if (!uart_ready) {
        delay(1000);
        return;
    }

    m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};
    m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};

    // The TX accessor implements data::StreamWriter, so StreamSink can
    // lift it into a Sink. scratch is the staging buffer the Sink lends
    // out to the receiver; each committed chunk goes out immediately.
    uint8_t scratch[64];
    m5hal::data::StreamSink echo{uart_tx, m5hal::data::DataSpan{scratch, sizeof(scratch)}};

    // RX -> TX in one call. Returns after up to sizeof(scratch) bytes,
    // or earlier when the line goes idle (first_byte_timeout_ms).
    auto n = uart_rx.read(echo, sizeof(scratch));
    if (!n.has_value()) {
        printError("uart echo", n.error());
        delay(100);
        return;
    }
    if (n.value() > 0) {
        echo_total += n.value();
        Serial.printf("echoed %u bytes (total %lu)\n", static_cast<unsigned>(n.value()),
                      static_cast<unsigned long>(echo_total));
    }
}
