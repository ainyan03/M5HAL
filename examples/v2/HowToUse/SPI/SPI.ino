// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseSPI
//
// Minimal SPI Bus / Accessor sketch. It does not require an SPI slave: plain
// write, command+data, dummy clocks, and a manually scoped transaction all
// produce wire activity that can be observed with a logic analyzer.
//
// Pin defaults match M5Stack Core BASIC LCD (ILI9342C) wiring:
//   SCLK=18, MOSI=23, MISO=19, D/C=27, CS=14
//
// When M5HAL_EXAMPLE_SPI_LCD_DEMO is defined (default on ESP32), the sketch
// also initialises the ILI9342C panel and paints colour bars — a visual
// confirmation that writeCommand / writeCommandData / writeCommandAddress
// actually reach the display. Define M5HAL_EXAMPLE_SPI_LCD_DEMO=0 to skip
// the LCD part (e.g. when running on a board without an ILI9342C).
//
// To force a backend, pass a suffixed config type (spi::BusConfig_software /
// _arduino / _espidf) to acquire. The default spi::BusConfig selects the first
// backend offered by the active build environment.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v2.hpp>
#include <SPI.h>

#include <memory>

namespace m5hal = m5::hal::v2;

constexpr int PIN_SPI_CLK  = 18;
constexpr int PIN_SPI_MOSI = 23;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_DC = 27;
constexpr int PIN_SPI_CS = 14;

#ifndef M5HAL_EXAMPLE_HOWTOUSESPI_FREQ
#define M5HAL_EXAMPLE_HOWTOUSESPI_FREQ 40000000
#endif

#ifndef M5HAL_EXAMPLE_SPI_LCD_DEMO
#if CONFIG_IDF_TARGET_ESP32 || defined(ESP32)
#define M5HAL_EXAMPLE_SPI_LCD_DEMO 1
#else
#define M5HAL_EXAMPLE_SPI_LCD_DEMO 0
#endif
#endif

#if M5HAL_EXAMPLE_SPI_LCD_DEMO
constexpr int PIN_LCD_RST = 33;
constexpr int PIN_LCD_BL  = 32;
constexpr int LCD_WIDTH   = 320;
constexpr int LCD_HEIGHT  = 240;
#endif

// Borrowed handle, assigned in setup().
// For borrow model details, see HowToUse/I2C and README.
std::shared_ptr<m5hal::spi::IBus> spi_bus;

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %s (%d)\n", label, m5hal::error::toString(error), static_cast<int>(error));
}

static void demoPlainWrite(m5hal::spi::MasterAccessor& dev)
{
    static constexpr uint8_t payload[] = {0xA5, 0x5A, 0x3C, 0xC3};
    auto r                             = dev.write(payload, sizeof(payload));
    if (!r.has_value()) {
        printError("plain write", r.error());
        return;
    }
    Serial.printf("plain write: %u bytes\n", static_cast<unsigned>(sizeof(payload)));
}

static void demoCommandData(m5hal::spi::MasterAccessor& dev)
{
    static constexpr uint8_t data[] = {0x11, 0x22, 0x33};
    auto r                          = dev.writeCommandData(0x9F, m5hal::data::ConstDataSpan{data, sizeof(data)});
    if (!r.has_value()) {
        printError("command+data", r.error());
        return;
    }
    Serial.println("command+data: sent");
}

static void demoDummyClock(m5hal::spi::MasterAccessor& dev)
{
    auto r = dev.sendDummyClock(16);
    if (!r.has_value()) {
        printError("dummy clock", r.error());
        return;
    }
    Serial.println("dummy clock: 16 cycles");
}

static void demoReadCommand(m5hal::spi::MasterAccessor& dev)
{
    uint8_t rx[4] = {};
    auto r        = dev.readCommandData(0x04, m5hal::data::DataSpan{rx, sizeof(rx)});
    if (!r.has_value()) {
        printError("read command (RDDID 0x04)", r.error());
        return;
    }
    Serial.printf("RDDID (0x04): %02X %02X %02X %02X (%u bytes)\n", rx[0], rx[1], rx[2], rx[3],
                  static_cast<unsigned>(r.value()));
}

static void demoManualTransaction(m5hal::spi::MasterAccessor& dev)
{
    auto bt = dev.beginTransaction();
    if (!bt.has_value()) {
        printError("beginTransaction", bt.error());
        return;
    }

    static constexpr uint8_t first[]  = {0x01, 0x02};
    static constexpr uint8_t second[] = {0x03, 0x04};
    auto a                            = dev.write(first, sizeof(first));
    auto b                            = dev.write(second, sizeof(second));

    auto et = dev.endTransaction();
    if (!a.has_value()) {
        printError("transaction write A", a.error());
        return;
    }
    if (!b.has_value()) {
        printError("transaction write B", b.error());
        return;
    }
    if (!et.has_value()) {
        printError("endTransaction", et.error());
        return;
    }
    Serial.println("manual transaction: two writes under one CS assertion");
}

#if M5HAL_EXAMPLE_SPI_LCD_DEMO
static void demoLcdDraw(m5hal::spi::MasterAccessor& dev)
{
    Serial.println("LCD demo: init + colour bars");

    pinMode(PIN_LCD_BL, OUTPUT);
    pinMode(PIN_LCD_RST, OUTPUT);
    digitalWrite(PIN_LCD_RST, LOW);
    delay(20);
    digitalWrite(PIN_LCD_RST, HIGH);
    delay(120);

    dev.writeCommand(0x01);
    delay(120);
    dev.writeCommand(0x11);
    delay(120);
    dev.writeCommand(0x21);
    uint8_t colmod = 0x55;
    dev.writeCommandData(0x3A, m5hal::data::ConstDataSpan{&colmod, 1});
    dev.writeCommand(0x29);

    digitalWrite(PIN_LCD_BL, HIGH);

    static constexpr uint16_t COLORS[] = {
        0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF, 0x0000,
    };
    constexpr int BAR_H = LCD_HEIGHT / 8;

    for (int bar = 0; bar < 8; ++bar) {
        int y0 = bar * BAR_H;
        int y1 = y0 + BAR_H - 1;
        uint8_t caset[] = {0, 0, static_cast<uint8_t>((LCD_WIDTH - 1) >> 8),
                           static_cast<uint8_t>((LCD_WIDTH - 1) & 0xFF)};
        uint8_t raset[] = {static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0 & 0xFF),
                           static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1 & 0xFF)};
        dev.writeCommandData(0x2A, m5hal::data::ConstDataSpan{caset, sizeof(caset)});
        dev.writeCommandData(0x2B, m5hal::data::ConstDataSpan{raset, sizeof(raset)});

        uint16_t c   = COLORS[bar];
        uint16_t cbe = static_cast<uint16_t>((c >> 8) | (c << 8));
        static uint16_t tile[LCD_WIDTH * BAR_H];
        for (int i = 0; i < LCD_WIDTH * BAR_H; ++i) {
            tile[i] = cbe;
        }
        auto bt = dev.beginTransaction();
        dev.writeCommand(0x2C);
        auto pixels = reinterpret_cast<const uint8_t*>(tile);
        for (int row = 0; row < BAR_H; ++row) {
            dev.write(pixels + row * LCD_WIDTH * 2, LCD_WIDTH * 2);
        }
        dev.endTransaction();
    }

    Serial.println("LCD demo: colour bars drawn");
}
#endif

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("M5HAL HowToUseSPI");
    Serial.printf("pins: SCLK=%d MOSI=%d MISO=%d DC=%d CS=%d freq=%u\n", PIN_SPI_CLK, PIN_SPI_MOSI, PIN_SPI_MISO,
                  PIN_SPI_DC, PIN_SPI_CS, static_cast<unsigned>(M5HAL_EXAMPLE_HOWTOUSESPI_FREQ));

    // Tag-typed core pins (CLK / MOSI / MISO): a swapped wiring will not compile.
    // BusConfig selects the build's default backend; use BusConfig_espidf or
    // BusConfig_arduino to force a specific one.
    m5hal::spi::BusConfig bus_cfg{m5hal::spi::Clk{PIN_SPI_CLK}, m5hal::spi::Mosi{PIN_SPI_MOSI},
                                  m5hal::spi::Miso{PIN_SPI_MISO}};

    auto acquired = m5hal::M5_Hal.SPI.acquire(bus_cfg);
    if (!acquired.has_value()) {
        printError("SPI bus acquire", acquired.error());
        return;
    }
    spi_bus = acquired.value();

    // setupWithDCPin: the display-class preset — sets the device D/C pin,
    // the matching data-path mode, and the 8-bit command phase that
    // writeCommand* expects, in one call.
    m5hal::spi::AccessConfig acc_cfg;
    acc_cfg.setupWithDCPin(PIN_SPI_DC).pin_cs = PIN_SPI_CS;
    acc_cfg.freq                              = M5HAL_EXAMPLE_HOWTOUSESPI_FREQ;
    acc_cfg.spi_mode                          = 0;  // (default; shown for clarity)
    acc_cfg.spi_order                         = 0;  // 0 = MSB first (default; shown for clarity)
    acc_cfg.spi_address_length                = 24;
    acc_cfg.spi_read_dummy_cycle              = 8;
    acc_cfg.spi_write_dummy_cycle             = 0;  // (default; shown for clarity)

    m5hal::spi::MasterAccessor dev{spi_bus, acc_cfg};  // co-owns the borrowed bus

    Serial.printf("backend: %s\n",
                  spi_bus->backendKind() == m5hal::types::backend_kind_t::Hardware ? "espidf (Hardware)" : "arduino (Software)");

    demoPlainWrite(dev);
    demoCommandData(dev);
    demoDummyClock(dev);
    demoReadCommand(dev);
    demoManualTransaction(dev);

#if M5HAL_EXAMPLE_SPI_LCD_DEMO
    demoLcdDraw(dev);
#endif

    Serial.println("HowToUseSPI done.");
}

void loop()
{
    delay(1000);
}
