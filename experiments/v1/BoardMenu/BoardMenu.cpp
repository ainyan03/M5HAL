// SPDX-License-Identifier: MIT
// M5HAL v1 board menu for M5Stack BASIC.
//
// This is an experiment harness, not a beginner example. It keeps the
// M5Stack BASIC pinout fixed and lets a developer choose small I2C / SPI /
// GPIO operations with the front buttons while observing Serial output or
// a logic analyzer.

#include <M5HAL_v1.hpp>

#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#else
#include <cstdarg>
#include <cstdio>

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr int LOW    = 0;
constexpr int HIGH   = 1;
constexpr int INPUT  = 0;
constexpr int OUTPUT = 1;

struct SerialCompat {
    void begin(unsigned long) const
    {
    }

    void printf(const char* format, ...) const
    {
        va_list args;
        va_start(args, format);
        (void)vprintf(format, args);
        va_end(args);
    }

    void println(const char* text = "") const
    {
        std::puts(text);
    }
};

SerialCompat Serial;

void pinMode(int pin, int mode)
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(pin);
    cfg.mode         = (mode == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    (void)gpio_config(&cfg);
}

int digitalRead(int pin)
{
    return gpio_get_level(static_cast<gpio_num_t>(pin));
}

void digitalWrite(int pin, int value)
{
    (void)gpio_set_level(static_cast<gpio_num_t>(pin), value ? 1 : 0);
}

void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t millis()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
#endif

namespace {

namespace m5hal = ::m5::hal::v1;

using SoftwareI2cBus = ::m5::hal::v1::i2c::Bus_software;
#if M5HAL_FRAMEWORK_HAS_ARDUINO
using ArduinoSpiBus = ::m5::hal::v1::spi::Bus_arduino;
#endif
using SoftwareSpiBus = ::m5::hal::v1::spi::Bus_software;
#if M5HAL_ESPIDF_I2C_HAS_MASTER
using EspidfI2cBus = ::m5::hal::v1::i2c::Bus_espidf;
#endif
#if M5HAL_ESPIDF_SPI_HAS_MASTER
using EspidfSpiBus = ::m5::hal::v1::spi::Bus_espidf;
#endif

constexpr int PIN_BTN_A     = 39;
constexpr int PIN_BTN_B     = 38;
constexpr int PIN_BTN_C     = 37;
constexpr int PIN_BACKLIGHT = 32;

constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

constexpr int PIN_SPI_CLK  = 18;
constexpr int PIN_SPI_MOSI = 23;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_DC   = 2;
constexpr int PIN_SPI_CS   = 5;

constexpr int PIN_LCD_CS  = 14;
constexpr int PIN_LCD_DC  = 27;
constexpr int PIN_LCD_RST = 33;

constexpr uint32_t I2C_FREQ     = 100000;
constexpr uint32_t SPI_FREQ     = 1000000;
constexpr uint32_t LCD_SPI_FREQ = 40000000;

constexpr int LCD_WIDTH  = 320;
constexpr int LCD_HEIGHT = 240;
constexpr int LCD_TILE_W = 80;
constexpr int LCD_TILE_H = 60;

constexpr uint16_t I2C_ADDR_NONE = 0xFFFF;
constexpr uint8_t REG_PROBE      = 0x00;

struct ButtonState {
    int pin;
    bool pressed = false;
    bool event   = false;

    void begin() const
    {
        pinMode(pin, INPUT);
    }

    void update()
    {
        const bool now_pressed = (digitalRead(pin) == LOW);
        event                  = now_pressed && !pressed;
        pressed                = now_pressed;
    }
};

ButtonState button_a{PIN_BTN_A};
ButtonState button_b{PIN_BTN_B};
ButtonState button_c{PIN_BTN_C};

bool backlight_on = true;
uint16_t lcd_tile[LCD_TILE_W * LCD_TILE_H]{};
uint8_t lcd_hue_offset = 0;

enum class IBusVariant : uint8_t {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
    Arduino,
#endif
    Software,
#if M5HAL_ESPIDF_I2C_HAS_MASTER
    ESPIDF,
#endif
};

enum class IBusVariant : uint8_t {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
    Arduino,
#endif
    Software,
#if M5HAL_ESPIDF_SPI_HAS_MASTER
    ESPIDF,
#endif
};

#if M5HAL_FRAMEWORK_HAS_ARDUINO
IBusVariant current_i2c_variant = IBusVariant::Arduino;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER
IBusVariant current_i2c_variant = IBusVariant::ESPIDF;
#else
IBusVariant current_i2c_variant = IBusVariant::Software;
#endif
IBusVariant current_spi_variant = IBusVariant::Software;

const char* i2cVariantName(IBusVariant variant)
{
    switch (variant) {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
        case IBusVariant::Arduino:
            return "Arduino";
#endif
        case IBusVariant::Software:
            return "Software";
#if M5HAL_ESPIDF_I2C_HAS_MASTER
        case IBusVariant::ESPIDF:
            return "ESP-IDF";
#endif
    }
    return "?";
}

const char* spiVariantName(IBusVariant variant)
{
    switch (variant) {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
        case IBusVariant::Arduino:
            return "Arduino";
#endif
        case IBusVariant::Software:
            return "Software";
#if M5HAL_ESPIDF_SPI_HAS_MASTER
        case IBusVariant::ESPIDF:
            return "ESP-IDF";
#endif
    }
    return "?";
}

void setBacklight(bool on)
{
    backlight_on = on;
    digitalWrite(PIN_BACKLIGHT, on ? HIGH : LOW);
}

void printResult(const char* label, m5hal::error::error_t err)
{
    Serial.printf("%s error=%d\n", label, static_cast<int>(err));
}

template <typename Bus>
bool initI2cBus(Bus& bus)
{
    m5hal::i2c::BusConfig_software cfg;
    cfg.pin_scl = PIN_I2C_SCL;
    cfg.pin_sda = PIN_I2C_SDA;
    auto result = bus.init(cfg);
    if (!result.has_value()) {
        printResult("I2C bus init", result.error());
        return false;
    }
    return true;
}

#if M5HAL_FRAMEWORK_HAS_ARDUINO
bool initI2cBus(m5hal::i2c::Bus_arduino& bus)
{
    m5hal::i2c::BusConfig_arduino cfg;
    cfg.wire    = &Wire;
    cfg.pin_scl = PIN_I2C_SCL;
    cfg.pin_sda = PIN_I2C_SDA;
    auto result = bus.init(cfg);
    if (!result.has_value()) {
        printResult("I2C bus init", result.error());
        return false;
    }
    return true;
}
#endif

#if M5HAL_ESPIDF_I2C_HAS_MASTER
bool initI2cBus(EspidfI2cBus& bus)
{
    m5hal::i2c::BusConfig_espidf cfg;
    cfg.pin_scl = PIN_I2C_SCL;
    cfg.pin_sda = PIN_I2C_SDA;
    auto result = bus.init(cfg);
    if (!result.has_value()) {
        printResult("I2C bus init", result.error());
        return false;
    }
    return true;
}
#endif

#if M5HAL_FRAMEWORK_HAS_ARDUINO
void runUseArduinoI2C()
{
    current_i2c_variant = IBusVariant::Arduino;
    Serial.println("\n[I2C bus] Arduino");
}
#endif

void runUseSoftwareI2C()
{
    current_i2c_variant = IBusVariant::Software;
    Serial.println("\n[I2C bus] Software");
}

#if M5HAL_ESPIDF_I2C_HAS_MASTER
void runUseESPIDFI2C()
{
    current_i2c_variant = IBusVariant::ESPIDF;
    Serial.println("\n[I2C bus] ESP-IDF");
}
#endif

uint16_t scanI2C(m5hal::i2c::IBus& bus, const char* label)
{
    Serial.printf("\n[%s] SDA=%d SCL=%d freq=%u\n", label, PIN_I2C_SDA, PIN_I2C_SCL, static_cast<unsigned>(I2C_FREQ));
    uint16_t first = I2C_ADDR_NONE;
    uint8_t count  = 0;

    for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
        if (bus.probe(addr, I2C_FREQ, 50).has_value()) {
            Serial.printf("  found 0x%02X\n", addr);
            if (first == I2C_ADDR_NONE) {
                first = addr;
            }
            ++count;
        }
    }

    if (count == 0) {
        Serial.println("  no devices found");
    } else {
        Serial.printf("  total=%u first=0x%02X\n", static_cast<unsigned>(count), first);
    }
    return first;
}

template <typename Bus, typename Action>
void withI2cBus(Action action)
{
    Bus bus;
    if (!initI2cBus(bus)) {
        return;
    }
    action(bus);
    (void)bus.release();
}

template <typename Action>
void withSelectedIBus(Action action)
{
    switch (current_i2c_variant) {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
        case IBusVariant::Arduino:
            withI2cBus<m5hal::i2c::Bus>(action);
            return;
#endif
        case IBusVariant::Software:
            withI2cBus<SoftwareI2cBus>(action);
            return;
#if M5HAL_ESPIDF_I2C_HAS_MASTER
        case IBusVariant::ESPIDF:
            withI2cBus<EspidfI2cBus>(action);
            return;
#endif
    }
}

void runI2CScan()
{
    withSelectedIBus([](m5hal::i2c::IBus& bus) {
        char label[32];
        snprintf(label, sizeof(label), "%s I2C scan", i2cVariantName(current_i2c_variant));
        (void)scanI2C(bus, label);
    });
}

void runI2CRegisterRead()
{
    withSelectedIBus([](m5hal::i2c::IBus& bus) {
        char label[40];
        snprintf(label, sizeof(label), "%s I2C register read", i2cVariantName(current_i2c_variant));
        const uint16_t addr = scanI2C(bus, label);
        if (addr == I2C_ADDR_NONE) {
            return;
        }

        m5hal::i2c::MasterAccessConfig cfg;
        cfg.i2c_addr        = addr;
        cfg.freq            = I2C_FREQ;
        cfg.wire_timeout_ms = 100;
        m5hal::i2c::MasterAccessor dev{bus, cfg};

        auto value = dev.readRegister(REG_PROBE);
        if (!value.has_value()) {
            printResult("readRegister(0x00)", value.error());
        } else {
            Serial.printf("  0x%02X[0x00] = 0x%02X\n", addr, value.value());
        }
    });
}

void configureSPIAccess(m5hal::spi::MasterAccessConfig& access, int cs_pin, uint32_t freq, uint8_t write_dummy_cycle)
{
    access.pin_cs                = cs_pin;
    access.freq                  = freq;
    access.spi_mode              = 0;
    access.spi_order             = 0;
    access.spi_command_length    = 8;
    access.spi_address_length    = 24;
    access.spi_read_dummy_cycle  = 8;
    access.spi_write_dummy_cycle = write_dummy_cycle;
}

void setOutputHigh(int pin)
{
    if (pin >= 0) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
}

template <typename Bus>
bool initSpiBus(Bus& bus, m5hal::spi::MasterAccessConfig& access, int dc_pin, int cs_pin, uint32_t freq,
                uint8_t write_dummy_cycle)
{
    setOutputHigh(cs_pin);

    m5hal::spi::IBusConfig bus_cfg;
    bus_cfg.pin_clk  = PIN_SPI_CLK;
    bus_cfg.pin_mosi = PIN_SPI_MOSI;
    bus_cfg.pin_miso = PIN_SPI_MISO;
    bus_cfg.pin_dc   = dc_pin;

    auto init = bus.init(bus_cfg);
    if (!init.has_value()) {
        printResult("SPI bus init", init.error());
        return false;
    }

    configureSPIAccess(access, cs_pin, freq, write_dummy_cycle);
    return true;
}

#if M5HAL_FRAMEWORK_HAS_ARDUINO
bool initSpiBus(ArduinoSpiBus& bus, m5hal::spi::MasterAccessConfig& access, int dc_pin, int cs_pin, uint32_t freq,
                uint8_t write_dummy_cycle)
{
    setOutputHigh(cs_pin);

    m5hal::spi::BusConfig_arduino bus_cfg;
    bus_cfg.spi      = &SPI;
    bus_cfg.pin_clk  = PIN_SPI_CLK;
    bus_cfg.pin_mosi = PIN_SPI_MOSI;
    bus_cfg.pin_miso = PIN_SPI_MISO;
    bus_cfg.pin_dc   = dc_pin;

    auto init = bus.init(bus_cfg);
    if (!init.has_value()) {
        printResult("SPI bus init", init.error());
        return false;
    }

    configureSPIAccess(access, cs_pin, freq, write_dummy_cycle);
    return true;
}
#endif

#if M5HAL_ESPIDF_SPI_HAS_MASTER
bool initSpiBus(EspidfSpiBus& bus, m5hal::spi::MasterAccessConfig& access, int dc_pin, int cs_pin, uint32_t freq,
                uint8_t write_dummy_cycle)
{
    setOutputHigh(cs_pin);

    m5hal::spi::BusConfig_espidf bus_cfg;
    bus_cfg.pin_clk  = PIN_SPI_CLK;
    bus_cfg.pin_mosi = PIN_SPI_MOSI;
    bus_cfg.pin_miso = PIN_SPI_MISO;
    bus_cfg.pin_dc   = dc_pin;

    auto init = bus.init(bus_cfg);
    if (!init.has_value()) {
        printResult("SPI bus init", init.error());
        return false;
    }

    configureSPIAccess(access, cs_pin, freq, write_dummy_cycle);
    return true;
}
#endif

#if M5HAL_FRAMEWORK_HAS_ARDUINO
void runUseArduinoSPI()
{
    current_spi_variant = IBusVariant::Arduino;
    Serial.println("\n[SPI bus] Arduino");
}
#endif

void runUseSoftwareSPI()
{
    current_spi_variant = IBusVariant::Software;
    Serial.println("\n[SPI bus] Software");
}

#if M5HAL_ESPIDF_SPI_HAS_MASTER
void runUseESPIDFSPI()
{
    current_spi_variant = IBusVariant::ESPIDF;
    Serial.println("\n[SPI bus] ESP-IDF");
}
#endif

template <typename Action>
void withSelectedIBus(m5hal::spi::MasterAccessConfig& access, int dc_pin, int cs_pin, uint32_t freq, Action action)
{
    switch (current_spi_variant) {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
        case IBusVariant::Arduino: {
            ArduinoSpiBus bus;
            if (!initSpiBus(bus, access, dc_pin, cs_pin, freq, 8)) {
                return;
            }
            action(bus);
            (void)bus.release();
            return;
        }
#endif
        case IBusVariant::Software: {
            SoftwareSpiBus bus;
            if (!initSpiBus(bus, access, dc_pin, cs_pin, freq, 4)) {
                return;
            }
            action(bus);
            (void)bus.release();
            return;
        }
#if M5HAL_ESPIDF_SPI_HAS_MASTER
        case IBusVariant::ESPIDF: {
            EspidfSpiBus bus;
            if (!initSpiBus(bus, access, dc_pin, cs_pin, freq, 8)) {
                return;
            }
            action(bus);
            (void)bus.release();
            return;
        }
#endif
    }
}

void runSPIWrite()
{
    Serial.printf("\n[%s SPI write] CLK=%d MOSI=%d MISO=%d DC=%d CS=%d freq=%u\n", spiVariantName(current_spi_variant),
                  PIN_SPI_CLK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_DC, PIN_SPI_CS, static_cast<unsigned>(SPI_FREQ));

    m5hal::spi::MasterAccessConfig access;
    withSelectedIBus(access, PIN_SPI_DC, PIN_SPI_CS, SPI_FREQ, [&](m5hal::spi::IBus& bus) {
        m5hal::spi::MasterAccessor spi{bus, access};
        const uint8_t payload[] = {0x00, 0xFF, 0x55, 0xAA};
        auto result             = spi.write(payload, sizeof(payload));
        if (!result.has_value()) {
            printResult("SPI write", result.error());
        } else {
            Serial.printf("  wrote %u bytes\n", static_cast<unsigned>(result.value()));
        }
    });
}

void runSPICommandPattern()
{
    Serial.printf("\n[%s SPI command pattern] CLK=%d MOSI=%d MISO=%d DC=%d CS=%d freq=%u\n",
                  spiVariantName(current_spi_variant), PIN_SPI_CLK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_DC, PIN_SPI_CS,
                  static_cast<unsigned>(SPI_FREQ));

    m5hal::spi::MasterAccessConfig access;
    withSelectedIBus(access, PIN_SPI_DC, PIN_SPI_CS, SPI_FREQ, [&](m5hal::spi::IBus& bus) {
        m5hal::spi::MasterAccessor spi{bus, access};
        const uint8_t command_data[] = {0xA5, 0x5A, 0x00, 0xFF};
        auto cd = spi.writeCommandData(0x9F, m5hal::data::ConstDataSpan{command_data, sizeof(command_data)});
        if (!cd.has_value()) {
            printResult("writeCommandData", cd.error());
        } else {
            Serial.printf("  command/data bytes=%u\n", static_cast<unsigned>(cd.value()));
        }

        const uint8_t address_data[] = {0xDE, 0xAD};
        auto aw =
            spi.writeCommandAddressData(0x02, 0x001234, m5hal::data::ConstDataSpan{address_data, sizeof(address_data)});
        if (!aw.has_value()) {
            printResult("writeCommandAddressData", aw.error());
        } else {
            Serial.printf("  command/address/write bytes=%u\n", static_cast<unsigned>(aw.value()));
        }

        auto dummy = spi.sendDummyClock(16);
        if (!dummy.has_value()) {
            printResult("sendDummyClock", dummy.error());
        } else {
            Serial.println("  dummy clocks=16");
        }
    });
}

void runSPIDummyClocks()
{
    // Sub-byte dummy clock check. Each sendDummyClock call is its own CS-low
    // window (the accessor wraps every transfer in begin/endTransaction), so on
    // a logic analyzer each burst shows exactly N SCLK pulses with no MOSI/data
    // activity -- count pulses per CS-low window. The non-multiple-of-8 counts
    // (1/3/7/13/31/33) exercise the bit-granular path; 33 also crosses the
    // 32-bit transferBits chunk boundary. On the Arduino variant these now work
    // on ESP32 (transferBits); pre-change they returned INVALID_ARGUMENT.
    Serial.printf("\n[%s SPI dummy clocks] SCLK=%d CS=%d freq=%u -- count pulses per CS window\n",
                  spiVariantName(current_spi_variant), PIN_SPI_CLK, PIN_SPI_CS, static_cast<unsigned>(SPI_FREQ));

    static const uint16_t counts[] = {1, 3, 7, 8, 13, 16, 31, 33};

    m5hal::spi::MasterAccessConfig access;
    withSelectedIBus(access, PIN_SPI_DC, PIN_SPI_CS, SPI_FREQ, [&](m5hal::spi::IBus& bus) {
        m5hal::spi::MasterAccessor spi{bus, access};
        for (uint16_t n : counts) {
            auto r = spi.sendDummyClock(n);
            if (r.has_value()) {
                Serial.printf("  dummy %2u clocks -> OK\n", static_cast<unsigned>(n));
            } else {
                Serial.printf("  dummy %2u clocks -> error=%d\n", static_cast<unsigned>(n),
                              static_cast<int>(r.error()));
            }
            delay(2);  // visual gap between CS windows on the analyzer
        }
    });
}

void configureLcdAccess(m5hal::spi::MasterAccessConfig& access)
{
    access.spi_address_length    = 32;
    access.spi_write_dummy_cycle = 0;
    access.spi_read_dummy_cycle  = 0;
}

void resetLcdPanel()
{
    setOutputHigh(PIN_LCD_CS);
    setOutputHigh(PIN_LCD_DC);

    pinMode(PIN_LCD_RST, OUTPUT);
    digitalWrite(PIN_LCD_RST, LOW);
    delay(10);
    digitalWrite(PIN_LCD_RST, HIGH);
    delay(120);
}

bool lcdWriteCommand(m5hal::spi::MasterAccessor& lcd, uint32_t command, const char* label)
{
    auto result = lcd.writeCommand(command);
    if (!result.has_value()) {
        printResult(label, result.error());
        return false;
    }
    return true;
}

bool lcdInitSequence(m5hal::spi::MasterAccessor& lcd)
{
    uint8_t colmod = 0x55;
    if (!lcdWriteCommand(lcd, 0x01, "lcd sleep out reset")) {
        return false;
    }
    delay(120);
    if (!lcdWriteCommand(lcd, 0x11, "lcd sleep out")) {
        return false;
    }
    delay(120);
    if (!lcdWriteCommand(lcd, 0x21, "lcd invert on")) {
        return false;
    }
    if (!lcdWriteCommand(lcd, 0x29, "lcd display on")) {
        return false;
    }
    auto result = lcd.writeCommandData(0x3A, m5hal::data::ConstDataSpan{&colmod, 1});
    if (!result.has_value()) {
        printResult("lcd pixel format", result.error());
        return false;
    }
    return true;
}

uint32_t lcdRange(int start, int end)
{
    return static_cast<uint32_t>((static_cast<uint32_t>(start) << 16) | static_cast<uint32_t>(end));
}

bool lcdSetWindow(m5hal::spi::MasterAccessor& lcd, int x, int y, int w, int h)
{
    auto col = lcd.writeCommandAddress(0x2A, lcdRange(x, x + w - 1));
    if (!col.has_value()) {
        printResult("lcd column address", col.error());
        return false;
    }
    auto row = lcd.writeCommandAddress(0x2B, lcdRange(y, y + h - 1));
    if (!row.has_value()) {
        printResult("lcd row address", row.error());
        return false;
    }
    return true;
}

void makeLcdTile()
{
    lcd_hue_offset = static_cast<uint8_t>(lcd_hue_offset + 37);
    for (int y = 0; y < LCD_TILE_H; ++y) {
        for (int x = 0; x < LCD_TILE_W; ++x) {
            const uint8_t r = static_cast<uint8_t>((x + lcd_hue_offset) * 0x41 >> 4);
            const uint8_t g = static_cast<uint8_t>(y * 4 + lcd_hue_offset);
            const uint8_t b = static_cast<uint8_t>((x ^ y) + lcd_hue_offset);
            const uint16_t rgb565 =
                static_cast<uint16_t>((static_cast<uint16_t>(r >> 3) << 11) | (static_cast<uint16_t>(g >> 2) << 5) |
                                      static_cast<uint16_t>(b >> 3));
            lcd_tile[y * LCD_TILE_W + x] = static_cast<uint16_t>((rgb565 >> 8) | (rgb565 << 8));
        }
    }
}

bool lcdDrawTile(m5hal::spi::MasterAccessor& lcd, int bx, int by)
{
    const int x = bx * LCD_TILE_W;
    const int y = by * LCD_TILE_H;
    if (!lcdSetWindow(lcd, x, y, LCD_TILE_W, LCD_TILE_H)) {
        return false;
    }

    const auto* pixels = reinterpret_cast<const uint8_t*>(lcd_tile);
    auto result        = lcd.writeCommandData(0x2C, m5hal::data::ConstDataSpan{pixels, sizeof(lcd_tile)});
    if (!result.has_value()) {
        printResult("lcd memory write", result.error());
        return false;
    }
    return true;
}

void runLcdInit()
{
    Serial.printf("\n[LCD init via %s SPI] CLK=%d MOSI=%d DC=%d CS=%d RST=%d freq=%u\n",
                  spiVariantName(current_spi_variant), PIN_SPI_CLK, PIN_SPI_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                  static_cast<unsigned>(LCD_SPI_FREQ));

    setBacklight(true);
    resetLcdPanel();

    m5hal::spi::MasterAccessConfig access;
    withSelectedIBus(access, PIN_LCD_DC, PIN_LCD_CS, LCD_SPI_FREQ, [&](m5hal::spi::IBus& bus) {
        configureLcdAccess(access);
        m5hal::spi::MasterAccessor lcd{bus, access};
        if (lcdInitSequence(lcd)) {
            Serial.println("  LCD initialized");
        }
    });
}

void runLcdFillPattern()
{
    Serial.printf("\n[LCD fill pattern via %s SPI] %dx%d tiles, freq=%u\n", spiVariantName(current_spi_variant),
                  LCD_TILE_W, LCD_TILE_H, static_cast<unsigned>(LCD_SPI_FREQ));

    setBacklight(true);
    m5hal::spi::MasterAccessConfig access;
    withSelectedIBus(access, PIN_LCD_DC, PIN_LCD_CS, LCD_SPI_FREQ, [&](m5hal::spi::IBus& bus) {
        configureLcdAccess(access);
        m5hal::spi::MasterAccessor lcd{bus, access};
        if (!lcdInitSequence(lcd)) {
            return;
        }

        makeLcdTile();
        const uint32_t start_ms = millis();
        for (int by = 0; by < LCD_HEIGHT / LCD_TILE_H; ++by) {
            for (int bx = 0; bx < LCD_WIDTH / LCD_TILE_W; ++bx) {
                if (!lcdDrawTile(lcd, bx, by)) {
                    return;
                }
            }
        }
        const uint32_t elapsed_ms = millis() - start_ms;
        Serial.printf("  filled %dx%d in %u ms\n", LCD_WIDTH, LCD_HEIGHT, static_cast<unsigned>(elapsed_ms));
    });
}

void runFastTickSnapshot()
{
    const auto hz = m5hal::service::fastTickFrequencyHz();
    const auto a  = m5hal::service::fastTick();
    delay(10);
    const auto b = m5hal::service::fastTick();

    Serial.println("\n[fastTick snapshot]");
    Serial.printf("  frequency=%u Hz\n", static_cast<unsigned>(hz));
    Serial.printf("  ticks in 10ms=%u\n", static_cast<unsigned>(b - a));
}

void runBacklightToggle()
{
    setBacklight(!backlight_on);
    Serial.printf("\n[Backlight] %s\n", backlight_on ? "on" : "off");
}

struct Operation {
    const char* name;
    void (*run)();
};

Operation operations[] = {
#if M5HAL_FRAMEWORK_HAS_ARDUINO
    {"Use Arduino I2C", runUseArduinoI2C},
#endif
    {"Use Software I2C", runUseSoftwareI2C},
#if M5HAL_ESPIDF_I2C_HAS_MASTER
    {"Use ESP-IDF I2C", runUseESPIDFI2C},
#endif
    {"I2C scan", runI2CScan},
    {"I2C register read", runI2CRegisterRead},
#if M5HAL_FRAMEWORK_HAS_ARDUINO
    {"Use Arduino SPI", runUseArduinoSPI},
#endif
    {"Use Software SPI", runUseSoftwareSPI},
#if M5HAL_ESPIDF_SPI_HAS_MASTER
    {"Use ESP-IDF SPI", runUseESPIDFSPI},
#endif
    {"SPI write", runSPIWrite},
    {"SPI command pattern", runSPICommandPattern},
    {"SPI dummy clocks", runSPIDummyClocks},
    {"LCD init", runLcdInit},
    {"LCD fill pattern", runLcdFillPattern},
    {"fastTick snapshot", runFastTickSnapshot},
    {"Backlight toggle", runBacklightToggle},
};

constexpr size_t OPERATION_COUNT = sizeof(operations) / sizeof(operations[0]);
size_t current_operation         = 0;

void printMenu()
{
    Serial.println("\nM5HAL BoardMenu - M5Stack BASIC");
    Serial.printf("I2C=%s  SPI=%s\n", i2cVariantName(current_i2c_variant), spiVariantName(current_spi_variant));
    Serial.println("BtnA: previous  BtnC: next  BtnB: run");
    for (size_t i = 0; i < OPERATION_COUNT; ++i) {
        Serial.printf("  %c %u. %s\n", (i == current_operation) ? '>' : ' ', static_cast<unsigned>(i + 1),
                      operations[i].name);
    }
}

void selectOperation(int delta)
{
    const auto next   = static_cast<int>(current_operation) + delta + static_cast<int>(OPERATION_COUNT);
    current_operation = static_cast<size_t>(next) % OPERATION_COUNT;
    printMenu();
}

void updateButtons()
{
    button_a.update();
    button_b.update();
    button_c.update();
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_BACKLIGHT, OUTPUT);
    setBacklight(true);
    button_a.begin();
    button_b.begin();
    button_c.begin();

    printMenu();
}

void loop()
{
    updateButtons();
    if (button_a.event) {
        selectOperation(-1);
    }
    if (button_c.event) {
        selectOperation(1);
    }
    if (button_b.event) {
        Serial.printf("\nRunning: %s\n", operations[current_operation].name);
        operations[current_operation].run();
        Serial.println("Done.");
        printMenu();
    }
    delay(10);
}

#if !M5HAL_FRAMEWORK_HAS_ARDUINO
extern "C" void app_main()
{
    setup();
    while (true) {
        loop();
    }
}
#endif
