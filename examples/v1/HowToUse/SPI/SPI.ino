// =============================================================================
// M5HAL — HowToUseSPI
//
// Minimal SPI Bus / Accessor sketch. It does not require an SPI slave: plain
// write, command+data, dummy clocks, and a manually scoped transaction all
// produce wire activity that can be observed with a logic analyzer.
//
// Pin defaults match M5Stack Core (Basic / Gray / Fire) style wiring:
//   SCLK=18, MOSI=23, MISO=19, D/C=2, CS=5
//
// To force a backend, change ExampleSPIBus below. The default m5hal::spi::Bus
// is the first backend offered by the active build environment.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <SPI.h>

namespace m5hal = m5::hal::v1;

constexpr int PIN_SPI_CLK  = 18;
constexpr int PIN_SPI_MOSI = 23;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_DC   = 2;
constexpr int PIN_SPI_CS   = 5;

#ifndef M5HAL_EXAMPLE_HOWTOUSESPI_FREQ
#define M5HAL_EXAMPLE_HOWTOUSESPI_FREQ 1000000
#endif

// m5hal::spi::Bus resolves to the first backend the build offers
// (framework scan order; see spec/design/variants.md). Uncomment a
// variant::* alias below to force a specific backend instead.
using ExampleSPIBus = m5hal::spi::Bus;
// using ExampleSPIBus = m5hal::spi::variant::arduino::Bus;
// using ExampleSPIBus = m5hal::spi::variant::espidf::Bus;
// using ExampleSPIBus = m5hal::spi::variant::software::Bus;

ExampleSPIBus spi_bus;

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %d\n", label, static_cast<int>(error));
}

static void demoPlainWrite(m5hal::spi::SPIMasterAccessor& dev)
{
    static constexpr uint8_t payload[] = {0xA5, 0x5A, 0x3C, 0xC3};
    auto r                             = dev.write(payload, sizeof(payload));
    if (!r.has_value()) {
        printError("plain write", r.error());
        return;
    }
    Serial.printf("plain write: %u bytes\n", static_cast<unsigned>(r.value()));
}

static void demoCommandData(m5hal::spi::SPIMasterAccessor& dev)
{
    static constexpr uint8_t data[] = {0x11, 0x22, 0x33};
    auto r                          = dev.writeCommandData(0x9F, m5hal::data::ConstDataSpan{data, sizeof(data)});
    if (!r.has_value()) {
        printError("command+data", r.error());
        return;
    }
    Serial.printf("command+data: %u bytes\n", static_cast<unsigned>(r.value()));
}

static void demoDummyClock(m5hal::spi::SPIMasterAccessor& dev)
{
    auto r = dev.sendDummyClock(16);
    if (!r.has_value()) {
        printError("dummy clock", r.error());
        return;
    }
    Serial.println("dummy clock: 16 cycles");
}

static void demoManualTransaction(m5hal::spi::SPIMasterAccessor& dev)
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

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("M5HAL HowToUseSPI");
    Serial.printf("pins: SCLK=%d MOSI=%d MISO=%d DC=%d CS=%d freq=%u\n", PIN_SPI_CLK, PIN_SPI_MOSI, PIN_SPI_MISO,
                  PIN_SPI_DC, PIN_SPI_CS, static_cast<unsigned>(M5HAL_EXAMPLE_HOWTOUSESPI_FREQ));

    m5hal::spi::BusConfig bus_cfg;
    bus_cfg.spi      = &SPI;
    bus_cfg.pin_clk  = PIN_SPI_CLK;
    bus_cfg.pin_mosi = PIN_SPI_MOSI;
    bus_cfg.pin_miso = PIN_SPI_MISO;
    bus_cfg.pin_dc   = PIN_SPI_DC;

    if (auto r = spi_bus.init(bus_cfg); !r.has_value()) {
        printError("SPI bus init", r.error());
        return;
    }

    m5hal::spi::SPIMasterAccessConfig dev_cfg;
    dev_cfg.pin_cs                = PIN_SPI_CS;
    dev_cfg.freq                  = M5HAL_EXAMPLE_HOWTOUSESPI_FREQ;
    dev_cfg.timeout_ms            = 100;
    dev_cfg.spi_mode              = 0;
    dev_cfg.spi_order             = 0;  // 0 = MSB first
    dev_cfg.spi_data_mode         = m5hal::spi::spi_data_mode_t::spi_halfduplex_with_dc_pin;
    dev_cfg.spi_command_length    = 8;
    dev_cfg.spi_address_length    = 24;
    dev_cfg.spi_read_dummy_cycle  = 8;
    dev_cfg.spi_write_dummy_cycle = 0;

    m5hal::spi::SPIMasterAccessor dev{spi_bus, dev_cfg};

    demoPlainWrite(dev);
    demoCommandData(dev);
    demoDummyClock(dev);
    demoManualTransaction(dev);

    Serial.println("HowToUseSPI done.");
}

void loop()
{
    delay(1000);
}
