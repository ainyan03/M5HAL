#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <unity.h>

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

using SoftwareSPIBus = ::m5::hal::v1::spi::variant::software::Bus;

constexpr uint8_t kClkBit  = 0x01;
constexpr uint8_t kMosiBit = 0x02;
constexpr uint8_t kDcBit   = 0x04;
constexpr uint8_t kCsBit   = 0x08;

struct Sample {
    uint32_t usec = 0;
    uint8_t state = 0;
};

constexpr size_t kMaxSamples = 2048;
Sample samples[kMaxSamples];
volatile size_t sample_count   = 0;
volatile bool capture_running  = false;
volatile bool capture_ready    = false;
volatile bool capture_done     = false;
volatile bool capture_overflow = false;

uint8_t readCaptureState()
{
    uint8_t state = 0;
    state |= digitalRead(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK) ? kClkBit : 0;
    state |= digitalRead(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI) ? kMosiBit : 0;
    state |= digitalRead(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC) ? kDcBit : 0;
    state |= digitalRead(M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS) ? kCsBit : 0;
    return state;
}

void captureTask(void*)
{
    uint8_t last     = readCaptureState();
    sample_count     = 0;
    capture_overflow = false;
    if (sample_count < kMaxSamples) {
        samples[sample_count++] = {micros(), last};
    }
    capture_ready = true;

    while (capture_running) {
        const uint8_t state = readCaptureState();
        if (state != last) {
            const size_t index = sample_count;
            if (index < kMaxSamples) {
                samples[index] = {micros(), state};
                sample_count   = index + 1;
            } else {
                capture_overflow = true;
            }
            last = state;
        }
    }

    capture_done = true;
    vTaskDelete(nullptr);
}

void startCapture()
{
    capture_running  = true;
    capture_ready    = false;
    capture_done     = false;
    capture_overflow = false;
    sample_count     = 0;
    xTaskCreatePinnedToCore(captureTask, "spi-capture", 4096, nullptr, 3, nullptr, 0);
    while (!capture_ready) {
        delay(1);
    }
}

void stopCapture()
{
    delay(2);
    capture_running = false;
    while (!capture_done) {
        delay(1);
    }
}

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

size_t collectActiveRisingEdges(uint8_t* states, size_t capacity)
{
    size_t count          = 0;
    const size_t captured = sample_count;
    for (size_t i = 1; i < captured; ++i) {
        const uint8_t prev = samples[i - 1].state;
        const uint8_t cur  = samples[i].state;
        const bool rising  = ((prev & kClkBit) == 0) && ((cur & kClkBit) != 0);
        const bool active  = (cur & kCsBit) == 0;
        if (rising && active) {
            if (count < capacity) {
                states[count] = cur;
            }
            ++count;
        }
    }
    return count;
}

size_t collectActiveClockEdges(uint8_t* states, bool* rising_edges, size_t capacity)
{
    size_t count          = 0;
    const size_t captured = sample_count;
    for (size_t i = 1; i < captured; ++i) {
        const uint8_t prev = samples[i - 1].state;
        const uint8_t cur  = samples[i].state;
        const bool changed = ((prev ^ cur) & kClkBit) != 0;
        const bool active  = (cur & kCsBit) == 0;
        if (changed && active) {
            if (count < capacity) {
                states[count]       = cur;
                rising_edges[count] = ((prev & kClkBit) == 0) && ((cur & kClkBit) != 0);
            }
            ++count;
        }
    }
    return count;
}

bool bitAt(const uint8_t* bytes, size_t bit_index, bool lsb_first = false)
{
    const uint8_t byte = bytes[bit_index >> 3];
    const uint8_t bit  = static_cast<uint8_t>(bit_index & 7u);
    const uint8_t mask = lsb_first ? static_cast<uint8_t>(0x01u << bit) : static_cast<uint8_t>(0x80u >> bit);
    return (byte & mask) != 0;
}

void assertBits(const uint8_t* states, size_t start_bit, const uint8_t* bytes, size_t bit_count, bool lsb_first = false)
{
    for (size_t i = 0; i < bit_count; ++i) {
        const bool expected = bitAt(bytes, i, lsb_first);
        const bool actual   = (states[start_bit + i] & kMosiBit) != 0;
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected ? 1 : 0, actual ? 1 : 0, "MOSI bit mismatch");
    }
}

void assertDcRange(const uint8_t* states, size_t start_bit, size_t bit_count, bool expected_high)
{
    for (size_t i = 0; i < bit_count; ++i) {
        const bool actual = (states[start_bit + i] & kDcBit) != 0;
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected_high ? 1 : 0, actual ? 1 : 0, "DC phase mismatch");
    }
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
    startCapture();
    auto result = spi.writeCommandAddressData(0x02, 0x001234, ::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
    stopCapture();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "writeCommandAddressData failed");
    TEST_ASSERT_FALSE_MESSAGE(capture_overflow, "capture buffer overflow");

    uint8_t edges[80]{};
    const size_t edge_count = collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(52, edge_count, "unexpected write transfer clock count");

    const uint8_t command_address[] = {0x02, 0x00, 0x12, 0x34};
    assertBits(edges, 0, command_address, 32);
    assertBits(edges, 36, tx, 16);
    assertDcRange(edges, 0, 8, false);
    assertDcRange(edges, 8, 44, true);
}

void testReadCommandAddressDataWirePhases()
{
    SoftwareSPIBus bus = makeBus();
    auto cfg           = makeAccessConfig();
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

    uint8_t rx[4]{};
    startCapture();
    auto result = spi.readCommandAddressData(0x0B, 0x001234, ::m5::hal::v1::data::DataSpan{rx, sizeof(rx)});
    stopCapture();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "readCommandAddressData failed");
    TEST_ASSERT_FALSE_MESSAGE(capture_overflow, "capture buffer overflow");

    uint8_t edges[96]{};
    const size_t edge_count = collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(72, edge_count, "unexpected read transfer clock count");

    const uint8_t command_address[] = {0x0B, 0x00, 0x12, 0x34};
    assertBits(edges, 0, command_address, 32);
    assertDcRange(edges, 0, 8, false);
    assertDcRange(edges, 8, 64, true);
}

void testWriteUsesLsbFirstBitOrder()
{
    SoftwareSPIBus bus = makeBus();
    auto cfg           = makeAccessConfig(0, 1);
    ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

    const uint8_t tx[] = {0x96};
    startCapture();
    auto result = spi.write(::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
    stopCapture();

    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "LSB first write failed");
    TEST_ASSERT_FALSE_MESSAGE(capture_overflow, "capture buffer overflow");

    uint8_t edges[16]{};
    const size_t edge_count = collectActiveRisingEdges(edges, sizeof(edges));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(8, edge_count, "unexpected LSB first clock count");
    assertBits(edges, 0, tx, 8, true);
}

void testSpiModesSetExpectedClockEdges()
{
    const uint8_t tx[] = {0xA5};

    for (uint8_t mode = 0; mode < 4; ++mode) {
        SoftwareSPIBus bus = makeBus();
        auto cfg           = makeAccessConfig(mode, 0);
        ::m5::hal::v1::spi::SPIMasterAccessor spi{bus, cfg};

        startCapture();
        auto result = spi.write(::m5::hal::v1::data::ConstDataSpan{tx, sizeof(tx)});
        stopCapture();

        TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "mode write failed");
        TEST_ASSERT_FALSE_MESSAGE(capture_overflow, "capture buffer overflow");

        uint8_t edge_states[24]{};
        bool rising_edges[24]{};
        const size_t edge_count = collectActiveClockEdges(edge_states, rising_edges, 24);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(16, edge_count, "unexpected mode clock edge count");

        const bool cpol = (mode & 0x02) != 0;
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(cpol ? 0 : 1, rising_edges[0] ? 1 : 0, "first active edge polarity mismatch");
        for (size_t i = 0; i < edge_count; ++i) {
            const bool expected_rising = ((i & 1u) == 0) ? !cpol : cpol;
            TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected_rising ? 1 : 0, rising_edges[i] ? 1 : 0,
                                            "clock edge alternation mismatch");
        }
    }
}

}  // namespace

void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(200);
    printWiring();
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
