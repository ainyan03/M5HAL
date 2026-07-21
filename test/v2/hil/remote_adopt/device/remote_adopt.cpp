// SPDX-License-Identifier: MIT
//
// Arduino remote-server HIL for externally initialised I2C/SPI buses.
// The server borrows Wire and HSPI through ServerPhysicalBusPool::adopt*;
// repeated remote BusRelease must not end or reconfigure either object.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/remote/server_connection_wiring.hpp>
#include <m5_hal/hal/v2/remote/server_handler.hpp>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int kI2cSda  = 32;
constexpr int kI2cScl  = 33;
constexpr int kSpiClk  = 13;
constexpr int kSpiMosi = 14;
constexpr int kSpiMiso = 36;

SPIClass adopted_spi{HSPI};
m5hal::remote::ServerPhysicalBusPool physical_pool;
m5hal::remote::ServerBusPool bus_pool;
m5hal::remote::RemoteServerHandler handler;

uint8_t server_scratch[m5hal::remote::kMaxScriptSize];
m5hal::remote::Server server{m5hal::data::DataSpan{server_scratch, sizeof(server_scratch)}};

uint8_t rx_scratch[4096];
uint8_t tx_scratch[4096];
m5hal::uart::Bus_arduino transport_bus;
m5hal::remote::RemoteServerAdapter* adapter_ptr = nullptr;

void halt()
{
    for (;;) {
        delay(1000);
    }
}

}  // namespace

void setup()
{
    // Initialise both vendor objects before handing them to M5HAL. There must
    // be no textual Serial output: UART0 becomes the framed protocol transport.
    if (!Wire.begin(kI2cSda, kI2cScl, 400000)) {
        halt();
    }
    adopted_spi.begin(kSpiClk, kSpiMiso, kSpiMosi, -1);

    if (!physical_pool.adoptI2C(Wire, kI2cScl, kI2cSda).has_value() ||
        !physical_pool.adoptSPI(adopted_spi, kSpiClk, kSpiMosi, kSpiMiso).has_value()) {
        halt();
    }

    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.pin_tx         = 1;
    bus_cfg.pin_rx         = 3;
    bus_cfg.rx_buffer_size = 8192;
    bus_cfg.tx_buffer_size = 2048;
    if (!transport_bus.init(bus_cfg, m5hal::native::borrowed(Serial)).has_value()) {
        halt();
    }

    m5hal::uart::AccessConfig uart_cfg;
    uart_cfg.baud_rate             = 115200;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::TxAccessor uart_tx{transport_bus, uart_cfg};
    static m5hal::uart::RxAccessor uart_rx{transport_bus, uart_cfg};
    static m5hal::data::StreamSource wire_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink wire_sink{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::data::MuxFrameEncoder encoder{m5hal::memory::defaultAllocator()};
    static m5hal::data::MuxFrameDecoder decoder{m5hal::memory::defaultAllocator()};
    static m5hal::remote::RemoteServerAdapter adapter{encoder, decoder, wire_src, wire_sink};

    // Publish MCU GPIO so the same fixture can verify Arduino output readback
    // through the remote watcher. Keep the ESP32 flash pins inaccessible.
    (void)m5hal::M5_Hal.Gpio.setDenyMask(0, 0, 0x00000FC0u);
    server.setGPIOGroup(m5hal::M5_Hal.Gpio);

    m5hal::remote::ConnectionWiring wiring;
    wiring.phys_pool   = &physical_pool;
    wiring.hello_flags = 0x01;
    m5hal::remote::wireConnection(server, bus_pool, handler, adapter, wiring);
    adapter_ptr = &adapter;
}

void loop()
{
    for (int i = 0; i < 50; ++i) {
        (void)adapter_ptr->service();
    }
    delay(1);
}
