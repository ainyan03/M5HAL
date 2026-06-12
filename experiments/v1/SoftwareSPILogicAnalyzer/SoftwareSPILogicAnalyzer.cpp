// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <M5HAL_v1.hpp>

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CLK
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CLK 18
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MOSI
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MOSI 23
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MISO
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MISO 19
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_DC
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_DC 2
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CS
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CS 5
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_FREQ
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_FREQ 1000000
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_MODE
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_MODE 0
#endif

#ifndef M5HAL_EXPERIMENT_SOFTWARE_SPI_ORDER
#define M5HAL_EXPERIMENT_SOFTWARE_SPI_ORDER 0
#endif

namespace {

using SoftwareSPIBus = ::m5::hal::v1::spi::variant::software::Bus;

SoftwareSPIBus bus;
::m5::hal::v1::spi::SPIMasterAccessConfig access_config;

void printConfig()
{
    Serial.println("SoftwareSPILogicAnalyzer");
    Serial.printf("CLK=%d MOSI=%d MISO=%d DC=%d CS=%d\n", M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CLK,
                  M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MOSI, M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MISO,
                  M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_DC, M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CS);
    Serial.printf("freq=%u Hz mode=%u order=%s\n", static_cast<unsigned>(M5HAL_EXPERIMENT_SOFTWARE_SPI_FREQ),
                  static_cast<unsigned>(M5HAL_EXPERIMENT_SOFTWARE_SPI_MODE),
                  M5HAL_EXPERIMENT_SOFTWARE_SPI_ORDER == 0 ? "MSB first" : "LSB first");
    Serial.println("Logic analyzer pattern:");
    Serial.println("  1. command/data split: cmd=0x9F data=0xA5 0x5A 0x00 0xFF");
    Serial.println("  2. write pattern: 0x00 0xFF 0x55 0xAA");
    Serial.println("  3. command/address/write dummy/data: cmd=0x02 addr=0x001234 dummy=4 write=2");
    Serial.println("  4. command/address/read dummy/data: cmd=0x0B addr=0x001234 dummy=8 read=4");
    Serial.println("  5. dummy clocks: 16 cycles");
}

void runPattern()
{
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, access_config};

    const uint8_t command_data[] = {0x9F, 0xA5, 0x5A, 0x00, 0xFF};
    auto cd = spi.writeCommandData(::m5::hal::v1::data::ConstDataSpan{command_data, sizeof(command_data)});
    if (!cd.has_value()) {
        Serial.printf("writeCommandData error=%d\n", static_cast<int>(cd.error()));
    }
    delay(10);

    const uint8_t write_pattern[] = {0x00, 0xFF, 0x55, 0xAA};
    auto wr = spi.write(write_pattern, sizeof(write_pattern));
    if (!wr.has_value()) {
        Serial.printf("write error=%d\n", static_cast<int>(wr.error()));
    }
    delay(10);

    const uint8_t address_write_pattern[] = {0xDE, 0xAD};
    auto aw = spi.writeCommandAddressData(
        0x02, 0x001234, ::m5::hal::v1::data::ConstDataSpan{address_write_pattern, sizeof(address_write_pattern)});
    if (!aw.has_value()) {
        Serial.printf("writeCommandAddressData error=%d\n", static_cast<int>(aw.error()));
    }
    delay(10);

    uint8_t rx_pattern[4] = {};
    auto rd = spi.readCommandAddressData(0x0B, 0x001234, ::m5::hal::v1::data::DataSpan{rx_pattern, sizeof(rx_pattern)});
    if (!rd.has_value()) {
        Serial.printf("readCommandAddressData error=%d\n", static_cast<int>(rd.error()));
    }
    delay(10);

    auto dummy = spi.sendDummyClock(16);
    if (!dummy.has_value()) {
        Serial.printf("dummy error=%d\n", static_cast<int>(dummy.error()));
    }
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);
    printConfig();

    ::m5::hal::v1::spi::SPIBusConfig bus_config;
    bus_config.pin_clk  = M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CLK;
    bus_config.pin_mosi = M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MOSI;
    bus_config.pin_miso = M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MISO;
    bus_config.pin_dc   = M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_DC;

    auto init = bus.init(bus_config);
    if (!init.has_value()) {
        Serial.printf("bus init error=%d\n", static_cast<int>(init.error()));
        return;
    }

    access_config.pin_cs             = M5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CS;
    access_config.freq               = M5HAL_EXPERIMENT_SOFTWARE_SPI_FREQ;
    access_config.spi_mode           = M5HAL_EXPERIMENT_SOFTWARE_SPI_MODE & 0x03;
    access_config.spi_order          = M5HAL_EXPERIMENT_SOFTWARE_SPI_ORDER ? 1 : 0;
    access_config.spi_command_length    = 8;
    access_config.spi_address_length    = 24;
    access_config.spi_read_dummy_cycle  = 8;
    access_config.spi_write_dummy_cycle = 4;

    Serial.println("Ready. Pattern repeats every 100 ms.");
}

void loop()
{
    runPattern();
    delay(100);
}
