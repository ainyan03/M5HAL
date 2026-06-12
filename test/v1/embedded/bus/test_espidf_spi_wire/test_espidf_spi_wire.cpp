// SPDX-License-Identifier: MIT
//
// On-target wire self-test for the espidf (hardware spi_master) SPI
// variant — the same protocol semantics the software variant pins
// (command/address/dummy/data phases, CS/DC windows, bit order, mode
// edge polarity), validated against the real controller through the
// shared GPIO self-capture rig (../spi_wire_capture.hpp).
//
// Runs from an Arduino-on-IDF build: the espidf variant is selectable
// via its alias from any ESP_PLATFORM build, and the Arduino layer
// supplies the capture rig (digitalRead / FreeRTOS task / micros).
//
// The wire contract asserted here is the spec one (spec/design/spi.md):
// dummy cycles must appear as real clock cycles on the wire, DC must
// be stable over each phase, CS active across all phases of one
// transfer. Clock counts are identical to the software-variant test —
// the two backends must be wire-compatible.
#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <unity.h>

#include "../spi_wire_capture.hpp"

#ifndef M5HAL_TEST_ESPIDF_SPI_PIN_CLK
#define M5HAL_TEST_ESPIDF_SPI_PIN_CLK 18
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_PIN_MOSI
#define M5HAL_TEST_ESPIDF_SPI_PIN_MOSI 23
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_PIN_MISO
#define M5HAL_TEST_ESPIDF_SPI_PIN_MISO 19
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_PIN_DC
#define M5HAL_TEST_ESPIDF_SPI_PIN_DC 2
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_PIN_CS
#define M5HAL_TEST_ESPIDF_SPI_PIN_CS 5
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_FREQ
#define M5HAL_TEST_ESPIDF_SPI_FREQ 2000
#endif

#ifndef M5HAL_TEST_ESPIDF_SPI_HOST
#if defined(SPI3_HOST)
#define M5HAL_TEST_ESPIDF_SPI_HOST SPI3_HOST
#else
#define M5HAL_TEST_ESPIDF_SPI_HOST SPI2_HOST
#endif
#endif

namespace {

namespace cap = ::wire_capture;

using EspidfSpiBus = ::m5::hal::v1::spi::Bus_espidf;

EspidfSpiBus& makeBus()
{
    // Static instance + re-init per test: a Unity assert failure
    // longjmps out of the test body, skipping local destructors - a
    // local Bus would leak the initialized SPI host and fail every
    // later init with INVALID_STATE. Bus::init releases the previous
    // ownership first, so re-init doubles as the cleanup path.
    static EspidfSpiBus bus;
    ::m5::hal::v1::spi::BusConfig_espidf bus_config;
    bus_config.host     = M5HAL_TEST_ESPIDF_SPI_HOST;
    bus_config.pin_clk  = M5HAL_TEST_ESPIDF_SPI_PIN_CLK;
    bus_config.pin_mosi = M5HAL_TEST_ESPIDF_SPI_PIN_MOSI;
    bus_config.pin_miso = M5HAL_TEST_ESPIDF_SPI_PIN_MISO;
    bus_config.pin_dc   = M5HAL_TEST_ESPIDF_SPI_PIN_DC;
    auto init           = bus.init(bus_config);
    TEST_ASSERT_TRUE_MESSAGE(init.has_value(), "espidf SPI bus init failed");
    return bus;
}

::m5::hal::v1::spi::MasterAccessConfig makeAccessConfig(uint8_t mode = 0, uint8_t order = 0)
{
    ::m5::hal::v1::spi::MasterAccessConfig cfg;
    cfg.pin_cs                = M5HAL_TEST_ESPIDF_SPI_PIN_CS;
    cfg.freq                  = M5HAL_TEST_ESPIDF_SPI_FREQ;
    cfg.spi_mode              = mode & 0x03;
    cfg.spi_order             = order ? 1 : 0;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    return cfg;
}

void printWiring()
{
    Serial.println("espidf SPI wire self-test (capture = output readback):");
    Serial.printf("  CLK=%d MOSI=%d MISO=%d DC=%d CS=%d host=%d\n", M5HAL_TEST_ESPIDF_SPI_PIN_CLK,
                  M5HAL_TEST_ESPIDF_SPI_PIN_MOSI, M5HAL_TEST_ESPIDF_SPI_PIN_MISO, M5HAL_TEST_ESPIDF_SPI_PIN_DC,
                  M5HAL_TEST_ESPIDF_SPI_PIN_CS, static_cast<int>(M5HAL_TEST_ESPIDF_SPI_HOST));
    Serial.printf("  freq=%u Hz\n", static_cast<unsigned>(M5HAL_TEST_ESPIDF_SPI_FREQ));
}

void testWriteCommandAddressDataWirePhases()
{
    EspidfSpiBus& bus = makeBus();
    auto cfg          = makeAccessConfig();
    ::m5::hal::v1::spi::MasterAccessor spi{bus, cfg};

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
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_ESPIDF_SPI_FREQ);
}

void testReadCommandAddressDataWirePhases()
{
    EspidfSpiBus& bus = makeBus();
    auto cfg          = makeAccessConfig();
    ::m5::hal::v1::spi::MasterAccessor spi{bus, cfg};

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
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_ESPIDF_SPI_FREQ);
}

void testWriteUsesLsbFirstBitOrder()
{
    EspidfSpiBus& bus = makeBus();
    auto cfg          = makeAccessConfig(0, 1);
    ::m5::hal::v1::spi::MasterAccessor spi{bus, cfg};

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
    cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_ESPIDF_SPI_FREQ);
}

void testSpiModesSetExpectedClockEdges()
{
    const uint8_t tx[] = {0xA5};

    for (uint8_t mode = 0; mode < 4; ++mode) {
        EspidfSpiBus& bus = makeBus();
        auto cfg          = makeAccessConfig(mode, 0);
        ::m5::hal::v1::spi::MasterAccessor spi{bus, cfg};

        cap::start();
        auto result = spi.write(::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
        cap::stop();

        char ctx[32];
        snprintf(ctx, sizeof(ctx), "mode %u", static_cast<unsigned>(mode));
        TEST_ASSERT_TRUE_MESSAGE(result.has_value(), ctx);
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
        cap::assertClockNeverFasterThanConfigured(M5HAL_TEST_ESPIDF_SPI_FREQ);
    }
}

}  // namespace

void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(200);
    printWiring();
    cap::setPins(M5HAL_TEST_ESPIDF_SPI_PIN_CLK, M5HAL_TEST_ESPIDF_SPI_PIN_MOSI, M5HAL_TEST_ESPIDF_SPI_PIN_DC,
                 M5HAL_TEST_ESPIDF_SPI_PIN_CS);

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
