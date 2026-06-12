// SPDX-License-Identifier: MIT
//
// On-target wire self-test for the software (bit-bang) SPI variant.
// Capture rig and assert helpers live in ../spi_wire_capture.hpp;
// expected clock counts assume the wire contract documented in
// spec/design/spi.md (dummy cycles clock like data bits, the clock
// parks at idle before CS deassert, DC rides the half-period grid).
#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <unity.h>

#include "../spi_wire_capture.hpp"

#ifndef M5HAL_TEST_SOFTWARE_SPI_PIN_CLK
#define M5HAL_TEST_SOFTWARE_SPI_PIN_CLK 18
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI
#define M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI 23
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_PIN_DC
#define M5HAL_TEST_SOFTWARE_SPI_PIN_DC 2
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_PIN_CS
#define M5HAL_TEST_SOFTWARE_SPI_PIN_CS 5
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK
#define M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK M5HAL_TEST_SOFTWARE_SPI_PIN_CLK
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI
#define M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC
#define M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC M5HAL_TEST_SOFTWARE_SPI_PIN_DC
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS
#define M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS M5HAL_TEST_SOFTWARE_SPI_PIN_CS
#endif

#ifndef M5HAL_TEST_SOFTWARE_SPI_FREQ
#define M5HAL_TEST_SOFTWARE_SPI_FREQ 2000
#endif

namespace {

namespace cap = ::wire_capture;

using SoftwareSPIBus = ::m5::hal::v1::spi::variant::software::Bus;

SoftwareSPIBus makeBus()
{
    SoftwareSPIBus bus;
    ::m5::hal::v1::spi::SPIBusConfig bus_config;
    bus_config.pin_clk  = M5HAL_TEST_SOFTWARE_SPI_PIN_CLK;
    bus_config.pin_mosi = M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI;
    bus_config.pin_miso = -1;
    bus_config.pin_dc   = M5HAL_TEST_SOFTWARE_SPI_PIN_DC;
    auto init           = bus.init(bus_config);
    TEST_ASSERT_TRUE_MESSAGE(init.has_value(), "software SPI bus init failed");
    return bus;
}

::m5::hal::v1::spi::SPIMasterAccessConfig makeAccessConfig(uint8_t mode = 0, uint8_t order = 0)
{
    ::m5::hal::v1::spi::SPIMasterAccessConfig cfg;
    cfg.pin_cs                = M5HAL_TEST_SOFTWARE_SPI_PIN_CS;
    cfg.freq                  = M5HAL_TEST_SOFTWARE_SPI_FREQ;
    cfg.spi_mode              = mode & 0x03;
    cfg.spi_order             = order ? 1 : 0;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    return cfg;
}

void configureCapturePins()
{
    if (M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK != M5HAL_TEST_SOFTWARE_SPI_PIN_CLK) {
        pinMode(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK, INPUT);
    }
    if (M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI != M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI) {
        pinMode(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI, INPUT);
    }
    if (M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC != M5HAL_TEST_SOFTWARE_SPI_PIN_DC) {
        pinMode(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC, INPUT);
    }
    if (M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS != M5HAL_TEST_SOFTWARE_SPI_PIN_CS) {
        pinMode(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS, INPUT_PULLUP);
    }
}

void printWiring()
{
    Serial.println("Software SPI wire self-test wiring:");
    Serial.printf("  CLK  out GPIO%d -> capture GPIO%d\n", M5HAL_TEST_SOFTWARE_SPI_PIN_CLK,
                  M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK);
    Serial.printf("  MOSI out GPIO%d -> capture GPIO%d\n", M5HAL_TEST_SOFTWARE_SPI_PIN_MOSI,
                  M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI);
    Serial.printf("  DC   out GPIO%d -> capture GPIO%d\n", M5HAL_TEST_SOFTWARE_SPI_PIN_DC,
                  M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC);
    Serial.printf("  CS   out GPIO%d -> capture GPIO%d\n", M5HAL_TEST_SOFTWARE_SPI_PIN_CS,
                  M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS);
    Serial.printf("  freq=%u Hz mode=0 order=MSB first\n", static_cast<unsigned>(M5HAL_TEST_SOFTWARE_SPI_FREQ));
}

void testWriteCommandAddressDataWirePhases()
{
    SoftwareSPIBus bus = makeBus();
    auto cfg           = makeAccessConfig();
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

    const uint8_t tx[] = {0xDE, 0xAD};
    cap::start();
    auto result = spi.writeCommandAddressData(0x02, 0x001234, ::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
    cap::stop();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "writeCommandAddressData failed");
    TEST_ASSERT_FALSE_MESSAGE(cap::capture_overflow, "capture buffer overflow");

    uint8_t edges[80]{};
    const size_t edge_count = cap::collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(52, edge_count, "unexpected write transfer clock count");

    const uint8_t command_address[] = {0x02, 0x00, 0x12, 0x34};
    cap::assertBits(edges, 0, command_address, 32);
    cap::assertBits(edges, 36, tx, 16);
    cap::assertDcRange(edges, 0, 8, false);
    cap::assertDcRange(edges, 8, 44, true);
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_SOFTWARE_SPI_FREQ);
}

void testReadCommandAddressDataWirePhases()
{
    SoftwareSPIBus bus = makeBus();
    auto cfg           = makeAccessConfig();
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

    uint8_t rx[4]{};
    cap::start();
    auto result = spi.readCommandAddressData(0x0B, 0x001234, ::m5::hal::v1::data::DataSpan{rx, sizeof(rx)});
    cap::stop();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "readCommandAddressData failed");
    TEST_ASSERT_FALSE_MESSAGE(cap::capture_overflow, "capture buffer overflow");

    uint8_t edges[96]{};
    const size_t edge_count = cap::collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(72, edge_count, "unexpected read transfer clock count");

    const uint8_t command_address[] = {0x0B, 0x00, 0x12, 0x34};
    cap::assertBits(edges, 0, command_address, 32);
    cap::assertDcRange(edges, 0, 8, false);
    cap::assertDcRange(edges, 8, 64, true);
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_SOFTWARE_SPI_FREQ);
}

void testWriteUsesLsbFirstBitOrder()
{
    SoftwareSPIBus bus = makeBus();
    auto cfg           = makeAccessConfig(0, 1);
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

    const uint8_t tx[] = {0x96};
    cap::start();
    auto result = spi.write(::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
    cap::stop();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "LSB first write failed");
    TEST_ASSERT_FALSE_MESSAGE(cap::capture_overflow, "capture buffer overflow");

    uint8_t edges[16]{};
    const size_t edge_count = cap::collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(8, edge_count, "unexpected LSB first clock count");
    cap::assertBits(edges, 0, tx, 8, true);
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_SOFTWARE_SPI_FREQ);
}

void testSpiModesSetExpectedClockEdges()
{
    const uint8_t tx[] = {0xA5};

    for (uint8_t mode = 0; mode < 4; ++mode) {
        SoftwareSPIBus bus = makeBus();
        auto cfg           = makeAccessConfig(mode, 0);
        ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

        cap::start();
        auto result = spi.write(::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
        cap::stop();

        TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "mode write failed");
        TEST_ASSERT_FALSE_MESSAGE(cap::capture_overflow, "capture buffer overflow");

        uint8_t edge_states[24]{};
        bool rising_edges[24]{};
        const size_t edge_count = cap::collectActiveClockEdges(edge_states, rising_edges, 24);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(16, edge_count, "unexpected mode clock edge count");

        const bool cpol = (mode & 0x02) != 0;
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(cpol ? 0 : 1, rising_edges[0] ? 1 : 0, "first active edge polarity mismatch");
        for (size_t i = 0; i < edge_count; ++i) {
            const bool expected_rising = ((i & 1u) == 0) ? !cpol : cpol;
            TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected_rising ? 1 : 0, rising_edges[i] ? 1 : 0,
                                            "clock edge alternation mismatch");
        }
        cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_SOFTWARE_SPI_FREQ);
    }
}

}  // namespace

void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(200);
    printWiring();
    cap::setPins(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK, M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI,
                 M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC, M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS);
    configureCapturePins();

    UNITY_BEGIN();
    RUN_TEST(testWriteCommandAddressDataWirePhases);
    RUN_TEST(testReadCommandAddressDataWirePhases);
    RUN_TEST(testWriteUsesLsbFirstBitOrder);
    RUN_TEST(testSpiModesSetExpectedClockEdges);
    UNITY_END();
}

void loop()
{
}
