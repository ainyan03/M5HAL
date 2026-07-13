// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — RemoteServer
//
// Device-side firmware for the M5HAL remote bus protocol. Exposes I2C,
// SPI, and GPIO over a framed serial transport so a PC host can
// drive the hardware through the Hal facade API.
//
// PIO envs:
//   RemoteServer_esp32           UART0 3Mbaud / espidf  (Core2)
//   RemoteServer_esp32s3         USB-Serial-JTAG / espidf (CoreS3)
//   RemoteServer_esp32_arduino   UART0 / arduino
// =============================================================================

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/remote/server_bus_pool.hpp>

#if !M5HAL_FRAMEWORK_HAS_ARDUINO
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#endif

#include <m5_hal/hal/v2/remote/server_connection_wiring.hpp>
#include <m5_hal/hal/v2/remote/server_handler.hpp>

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------
#ifndef M5HAL_EXAMPLE_REMOTE_UART_BAUD_RATE
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#define M5HAL_EXAMPLE_REMOTE_UART_BAUD_RATE 115200
#else
#define M5HAL_EXAMPLE_REMOTE_UART_BAUD_RATE 3000000
#endif
#endif

#ifndef M5HAL_EXAMPLE_REMOTE_I2C_PIN_SCL
#define M5HAL_EXAMPLE_REMOTE_I2C_PIN_SCL 22
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_I2C_PIN_SDA
#define M5HAL_EXAMPLE_REMOTE_I2C_PIN_SDA 21
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_SPI_PIN_CLOCK
#define M5HAL_EXAMPLE_REMOTE_SPI_PIN_CLOCK 32
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_SPI_PIN_MOSI
#define M5HAL_EXAMPLE_REMOTE_SPI_PIN_MOSI 26
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_SPI_PIN_MISO
#define M5HAL_EXAMPLE_REMOTE_SPI_PIN_MISO 36
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_SPI_PIN_CS
#define M5HAL_EXAMPLE_REMOTE_SPI_PIN_CS 33
#endif

#ifndef M5HAL_EXAMPLE_REMOTE_STATIC_I2C
#define M5HAL_EXAMPLE_REMOTE_STATIC_I2C 1
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_SPI
#define M5HAL_EXAMPLE_REMOTE_SPI 1
#endif
#ifndef M5HAL_EXAMPLE_REMOTE_TRANSPORT_USB_JTAG
#define M5HAL_EXAMPLE_REMOTE_TRANSPORT_USB_JTAG 0
#endif

// ---------------------------------------------------------------------------
// Namespace aliases
// ---------------------------------------------------------------------------
namespace m5hal = m5::hal::v2;

// ---------------------------------------------------------------------------
// Pin constants
// ---------------------------------------------------------------------------
static constexpr int PIN_I2C_SCL  = M5HAL_EXAMPLE_REMOTE_I2C_PIN_SCL;
static constexpr int PIN_I2C_SDA  = M5HAL_EXAMPLE_REMOTE_I2C_PIN_SDA;
static constexpr int PIN_UART_TX  = 1;
static constexpr int PIN_UART_RX  = 3;
static constexpr int PIN_SPI_CLK  = M5HAL_EXAMPLE_REMOTE_SPI_PIN_CLOCK;
static constexpr int PIN_SPI_MOSI = M5HAL_EXAMPLE_REMOTE_SPI_PIN_MOSI;
static constexpr int PIN_SPI_MISO = M5HAL_EXAMPLE_REMOTE_SPI_PIN_MISO;
static constexpr int PIN_SPI_CS   = M5HAL_EXAMPLE_REMOTE_SPI_PIN_CS;

static constexpr uint8_t BUS_ID_I2C = 0;
static constexpr uint8_t BUS_ID_SPI = 2;

// ---------------------------------------------------------------------------
// Shared resources
// ---------------------------------------------------------------------------
static m5hal::i2c::Bus i2c_bus;
static m5hal::spi::Bus spi_bus;
static m5hal::spi::MasterAccessor* g_spi_acc = nullptr;
static m5hal::remote::ServerPhysicalBusPool g_phys_pool;
static m5hal::remote::ServerBusPool g_bus_pool;
static m5hal::remote::RemoteServerHandler g_handler;

static uint8_t server_scratch[m5hal::remote::kMaxScriptSize];
static m5hal::remote::Server g_srv{m5hal::data::DataSpan{server_scratch, sizeof(server_scratch)}};

// Wire I/O scratch (raw UART bytes for StreamSource/StreamSink)
static uint8_t rx_scratch[4096];
static uint8_t tx_scratch[4096];

// PortC debug markers
static constexpr int PIN_DBG_SPI = 13;
static constexpr int PIN_DBG_MUX = 14;

static void initDebugPins()
{
    auto p13 = m5hal::M5_Hal.Gpio.getPin(PIN_DBG_SPI);
    auto p14 = m5hal::M5_Hal.Gpio.getPin(PIN_DBG_MUX);
    if (p13.isValid()) {
        p13.setMode(m5hal::types::gpio_mode_t::Output);
        p13.writeHigh();
    }
    if (p14.isValid()) {
        p14.setMode(m5hal::types::gpio_mode_t::Output);
        p14.writeHigh();
    }
}

// ---------------------------------------------------------------------------
// Bus initialisation
// ---------------------------------------------------------------------------
static void initBuses()
{
#if M5HAL_EXAMPLE_REMOTE_STATIC_I2C
    // Static I2C + self-scan diagnostic. Note: this claims bus_id 0, so a
    // host-side dynamic I2C create for the same id is rejected with
    // INVALID_STATE; use env:RemoteServer_esp32_host_i2c when the host should
    // own the I2C bus instead (e.g. RemoteI2S amplifier init on Core2).
    m5hal::i2c::BusConfig i2c_cfg{m5hal::i2c::Scl{PIN_I2C_SCL}, m5hal::i2c::Sda{PIN_I2C_SDA}};
    auto i2c_init_r = i2c_bus.init(i2c_cfg);
    ESP_LOGI("MUX", "I2C init scl=%d sda=%d: %s", PIN_I2C_SCL, PIN_I2C_SDA, i2c_init_r.has_value() ? "OK" : "FAIL");

    if (i2c_init_r.has_value()) {
        for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
            auto p = i2c_bus.probe(addr, 100000, 50);
            if (p.has_value()) {
                ESP_LOGI("MUX", "  I2C 0x%02X: ACK", addr);
            }
        }
        ESP_LOGI("MUX", "I2C self-scan done");
    }

    m5hal::i2c::MasterAccessConfig i2c_acc_cfg;
    static m5hal::i2c::MasterAccessor i2c_acc{i2c_bus, i2c_acc_cfg};
    (void)g_srv.registerI2C(BUS_ID_I2C, i2c_acc);
#endif

#if M5HAL_EXAMPLE_REMOTE_SPI
    m5hal::spi::BusConfig spi_cfg;
    spi_cfg.pin_clk  = PIN_SPI_CLK;
    spi_cfg.pin_mosi = PIN_SPI_MOSI;
    spi_cfg.pin_miso = PIN_SPI_MISO;
    if (spi_bus.init(spi_cfg).has_value()) {
        static m5hal::spi::MasterAccessConfig spi_acc_cfg;
        spi_acc_cfg.pin_cs = PIN_SPI_CS;
        spi_acc_cfg.freq   = 1000000;
        static m5hal::spi::MasterAccessor spi_acc{spi_bus, spi_acc_cfg};
        (void)g_srv.registerSPI(BUS_ID_SPI, spi_acc);
        g_spi_acc = &spi_acc;
    }
#endif

    g_srv.setGPIOGroup(m5hal::M5_Hal.Gpio);
#if defined(CONFIG_IDF_TARGET_ESP32)
    (void)m5hal::M5_Hal.Gpio.setDenyMask(0, 0, 0x00000FC0u);  // GPIO 6-11 = Flash SPI
#endif
}

// ===========================================================================
// UART transport + mux endpoint
// ===========================================================================

#if M5HAL_FRAMEWORK_HAS_ARDUINO

// ---- arduino ----
static m5hal::uart::Bus uart_bus;

static m5hal::remote::RemoteServerAdapter* g_adapter = nullptr;

void setup()
{
    initBuses();

    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.setSerial(Serial);
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 8192;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);

    m5hal::uart::AccessConfig uart_cfg;
    uart_cfg.baud_rate             = M5HAL_EXAMPLE_REMOTE_UART_BAUD_RATE;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::TxAccessor uart_tx{uart_bus, uart_cfg};
    static m5hal::uart::RxAccessor uart_rx{uart_bus, uart_cfg};

    static m5hal::data::StreamSource wire_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink wire_snk{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::data::MuxFrameEncoder enc{m5hal::memory::defaultAllocator()};
    static m5hal::data::MuxFrameDecoder dec{m5hal::memory::defaultAllocator()};
    static m5hal::remote::RemoteServerAdapter adapter{enc, dec, wire_src, wire_snk};

    m5hal::remote::ConnectionWiring cfg;
    cfg.phys_pool         = &g_phys_pool;
    cfg.hello_flags       = 0x01;
    cfg.static_spi_acc    = g_spi_acc;
    cfg.static_spi_bus_id = BUS_ID_SPI;
    m5hal::remote::wireConnection(g_srv, g_bus_pool, g_handler, adapter, cfg);

    g_adapter = &adapter;
}

void loop()
{
    for (int i = 0; i < 50; ++i) {
        (void)g_adapter->service();
    }
    delay(1);
}

#else  // espidf

// ---- espidf ----

#if !M5HAL_EXAMPLE_REMOTE_TRANSPORT_USB_JTAG
static m5hal::uart::Bus_espidf uart_bus;
#endif

static m5hal::remote::RemoteServerAdapter* g_adapter = nullptr;

#if M5HAL_EXAMPLE_REMOTE_TRANSPORT_USB_JTAG
#include <driver/usb_serial_jtag.h>

static struct JtagReader : public m5hal::data::StreamReader {
    m5hal::result_t<size_t> read(m5hal::data::DataSpan dst) override
    {
        int n = usb_serial_jtag_read_bytes(dst.data, dst.size, pdMS_TO_TICKS(2));
        return static_cast<size_t>(n > 0 ? n : 0);
    }
    m5hal::result_t<size_t> readableBytes() override
    {
        return static_cast<size_t>(0);
    }
} g_jtag_reader;

static struct JtagWriter : public m5hal::data::StreamWriter {
    m5hal::result_t<size_t> write(m5hal::data::ConstDataSpan src) override
    {
        int n = usb_serial_jtag_write_bytes(src.data, src.size, pdMS_TO_TICKS(100));
        return static_cast<size_t>(n > 0 ? n : 0);
    }
} g_jtag_writer;
#endif

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    initDebugPins();
    initBuses();

#if M5HAL_EXAMPLE_REMOTE_TRANSPORT_USB_JTAG
    usb_serial_jtag_driver_config_t jtag_cfg = {.tx_buffer_size = 4096, .rx_buffer_size = 4096};
    (void)usb_serial_jtag_driver_install(&jtag_cfg);
    vTaskDelay(pdMS_TO_TICKS(100));

    static m5hal::data::StreamSource wire_src{g_jtag_reader, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink wire_snk{g_jtag_writer, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
#else
    m5hal::uart::BusConfig_espidf bus_cfg;
    bus_cfg.port_num       = 0;
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 8192;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);
    uart_flush(static_cast<uart_port_t>(bus_cfg.port_num));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush(static_cast<uart_port_t>(bus_cfg.port_num));

    m5hal::uart::AccessConfig uart_cfg;
    uart_cfg.baud_rate             = M5HAL_EXAMPLE_REMOTE_UART_BAUD_RATE;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::TxAccessor uart_tx{uart_bus, uart_cfg};
    static m5hal::uart::RxAccessor uart_rx{uart_bus, uart_cfg};

    static m5hal::data::StreamSource wire_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink wire_snk{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
#endif

    static m5hal::data::MuxFrameEncoder enc{m5hal::memory::defaultAllocator()};
    static m5hal::data::MuxFrameDecoder dec{m5hal::memory::defaultAllocator()};
    static m5hal::remote::RemoteServerAdapter adapter{enc, dec, wire_src, wire_snk};

    m5hal::remote::ConnectionWiring cfg;
    cfg.phys_pool         = &g_phys_pool;
    cfg.hello_flags       = 0x01;
    cfg.static_spi_acc    = g_spi_acc;
    cfg.static_spi_bus_id = BUS_ID_SPI;
    m5hal::remote::wireConnection(g_srv, g_bus_pool, g_handler, adapter, cfg);

    // Serve from a dedicated task: service() executes bus transfers (I2S DMA
    // setup, mono expansion bounce buffers, ...) whose stack depth exceeds the
    // default main-task stack. 8 KiB matches the Arduino loopTask headroom.
    auto serve = [](void*) {
        while (true) {
            for (int i = 0; i < 50; ++i) {
                (void)g_adapter->service();
                m5hal::M5_Hal.Services.runOnce();
            }
            vTaskDelay(1);
        }
    };
    g_adapter = &adapter;
    if (::xTaskCreate(serve, "m5hal_srv", 8192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("MUX", "server task create failed");
    }
}

#endif  // M5HAL_FRAMEWORK_HAS_ARDUINO
