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
#include <M5HAL_v2.hpp>

#include <memory>

namespace m5hal = m5::hal::v2;

constexpr int PIN_UART_TX = 17;
constexpr int PIN_UART_RX = 16;

#ifndef M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD
#define M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD 115200
#endif

// Borrowed handle, assigned in setup().
// For borrow model details, see HowToUse/I2C and README.
std::shared_ptr<m5hal::uart::IBus> uart_bus;
m5hal::uart::AccessConfig acc_cfg;

// Accessors constructed once, reused (preferred for frequent I/O).
m5hal::uart::TxAccessor uart_tx;
m5hal::uart::RxAccessor uart_rx;

bool uart_ready     = false;
uint32_t echo_total = 0;

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %s (%d)\n", label, m5hal::error::toString(error), static_cast<int>(error));
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

    // Tag-typed pins: either order is correct (no swapped-pin accidents).
    m5hal::uart::BusConfig bus_cfg{m5hal::uart::Tx{PIN_UART_TX}, m5hal::uart::Rx{PIN_UART_RX}};
    bus_cfg.setSerial(Serial1);

    // Borrow the bus from M5_Hal (it owns the instance; you hold a shared handle).
    auto acquired = m5hal::M5_Hal.UART.acquire(bus_cfg);
    if (!acquired.has_value()) {
        printError("uart bus acquire", acquired.error());
        return;
    }
    uart_bus = acquired.value();

    acc_cfg.baud_rate             = M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD;
    acc_cfg.first_byte_timeout_ms = 20;
    acc_cfg.inter_byte_timeout_ms = 5;
    acc_cfg.write_timeout_ms      = 100;

    // Bind the reused accessors to the borrowed bus once. bind / setConfig only
    // fail while an access window is open, which never happens here.
    const bool bound = uart_tx.bind(*uart_bus).has_value() && uart_tx.setConfig(acc_cfg).has_value() &&
                       uart_rx.bind(*uart_bus).has_value() && uart_rx.setConfig(acc_cfg).has_value();
    if (!bound) {
        Serial.println("uart accessor bind failed");
        return;
    }

    uart_ready = true;
}

void loop()
{
    if (!uart_ready) {
        delay(1000);
        return;
    }

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
