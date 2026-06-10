// =============================================================================
// M5HAL — HowToUseBytecode
//
// Drives GPIO, I2C, and SPI from bytecode scripts (spec/design/bytecode.md)
// that are written out as plain byte arrays in this file - the same way a
// device init sequence can be stored as a const table and replayed with
//
//   runner.run(ConstDataSpan{script, sizeof(script)});
//
// Target hardware: M5Stack Core BASIC (no external wiring needed).
//   Button A (GPIO39) : GPIO script  - blinks the LCD backlight (GPIO32)
//   Button B (GPIO38) : I2C script   - reads an IP5306 power IC register
//                       (addr 0x75, reg 0x70) into store slot 1
//   Button C (GPIO37) : SPI script   - first press resets + wakes the LCD
//                       (ILI9342C: SWRESET/SLPOUT/DISPON), later presses
//                       toggle display inversion (INVON/INVOFF)
//
// Every script is annotated instruction by instruction. Wire format:
//   [LenVar size][opcode][payload]  /  0x00 terminates the script.
// Opcode and operand bytes are spelled with named constants derived from
// the library enums (bytecode::OpCode etc.), so the tables stay readable
// and cannot drift from the implementation. USB Serial shows the result
// of each run (read data, errors + offset).
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <SPI.h>
#include <Wire.h>

namespace m5hal = m5::hal::v1;
namespace bytecode = m5hal::bytecode;

// ---- M5Stack Core BASIC pin map ---------------------------------------------
constexpr int PIN_BTN_A = 39;
constexpr int PIN_BTN_B = 38;
constexpr int PIN_BTN_C = 37;

constexpr uint16_t PIN_LCD_BL  = 32;  // backlight (GPIO script target)
constexpr uint16_t PIN_LCD_RST = 33;  // LCD reset (driven by the SPI script)

constexpr int PIN_I2C_SCL = 22;  // internal I2C (IP5306 power IC)
constexpr int PIN_I2C_SDA = 21;

constexpr int PIN_SPI_CLK  = 18;  // LCD SPI
constexpr int PIN_SPI_MOSI = 23;
constexpr int PIN_SPI_MISO = 19;
constexpr int PIN_SPI_DC   = 27;
// LCD CS (14) travels inside the SPI script's bus_configure payload below.

// ---- named bytes for the script tables ---------------------------------------
// All values come straight from the library enums - single source of truth.

constexpr uint8_t OP_DELAY_MS        = static_cast<uint8_t>(bytecode::OpCode::delay_ms);
constexpr uint8_t OP_BUS_CONFIGURE   = static_cast<uint8_t>(bytecode::OpCode::bus_configure);
constexpr uint8_t OP_BUS_TRANSFER    = static_cast<uint8_t>(bytecode::OpCode::bus_transfer);
constexpr uint8_t OP_GPIO_SET_MODE   = static_cast<uint8_t>(bytecode::OpCode::gpio_set_mode);
constexpr uint8_t OP_GPIO_WRITE_HIGH = static_cast<uint8_t>(bytecode::OpCode::gpio_write_high);
constexpr uint8_t OP_GPIO_WRITE_LOW  = static_cast<uint8_t>(bytecode::OpCode::gpio_write_low);

constexpr uint8_t KIND_I2C = static_cast<uint8_t>(m5hal::types::bus_kind_t::I2C);
constexpr uint8_t KIND_SPI = static_cast<uint8_t>(m5hal::types::bus_kind_t::SPI);

constexpr uint8_t BC_GPIO_OUTPUT = static_cast<uint8_t>(m5hal::types::gpio_mode_t::Output);
constexpr uint8_t SPI_MODE_HALFDUPLEX_DC =
    static_cast<uint8_t>(m5hal::spi::spi_data_mode_t::spi_halfduplex_with_dc_pin);

constexpr uint8_t STORE_DISCARD = bytecode::kDiscardStoreId;  // 0xFF: read and drop
constexpr uint8_t END_OF_SCRIPT = 0x00;                       // LenVar 0 terminator

// ---- bytecode building blocks (keep the arrays readable) ----------------------

// delay_ms instruction: [5][OP_DELAY_MS][ms:u32 LE]
#define BC_DELAY_MS(ms)                                                                                      \
    0x05, OP_DELAY_MS, static_cast<uint8_t>((ms) & 0xFF), static_cast<uint8_t>(((ms) >> 8) & 0xFF),          \
        static_cast<uint8_t>(((ms) >> 16) & 0xFF), static_cast<uint8_t>(((ms) >> 24) & 0xFF)

// gpio_number_t as the u16 LE the gpio_* instructions carry.
#define BC_PIN_U16(pin) static_cast<uint8_t>((pin) & 0xFF), static_cast<uint8_t>(((pin) >> 8) & 0xFF)

// Single-pin gpio writes: [3][opcode][pin:u16].
#define BC_GPIO_HIGH(pin) 0x03, OP_GPIO_WRITE_HIGH, BC_PIN_U16(pin)
#define BC_GPIO_LOW(pin) 0x03, OP_GPIO_WRITE_LOW, BC_PIN_U16(pin)

// One LCD command byte on SPI bus 0: bus_transfer with a 15-byte SPI
// meta (command=cmd, command_bytes=1, D/C levels left to the variant's
// dc-pin handling), no tx data, no rx. [0x15][OP_BUS_TRANSFER][payload:20].
#define BC_LCD_CMD(cmd)                                                                                  \
    0x15, OP_BUS_TRANSFER,  /* bus_transfer */                                                           \
        KIND_SPI, 0x00,     /* kind=SPI, bus_id=0 */                                                     \
        STORE_DISCARD, 0x00, /* store_id=discard, rx_len=0 */                                            \
        0x0F,               /* meta_size=15 */                                                           \
        0x00,               /* meta: flags (no fixed dc level) */                                        \
        0xFF, 0xFF, 0xFF,   /* meta: cmd/addr/data dc = -1 (variant default) */                          \
        static_cast<uint8_t>(cmd), 0x00, 0x00, 0x00, /* meta: command u32 LE */                          \
        0x00, 0x00, 0x00, 0x00,                      /* meta: address u32 LE */                          \
        0x01, 0x00, 0x00 /* meta: command_bytes=1, address_bytes=0, dummy=0 */

// ---- scripts (the actual byte arrays) ----------------------------------------

// Button A: blink the LCD backlight three times via gpio_* instructions.
static constexpr uint8_t SCRIPT_GPIO_BLINK[] = {
    0x04, OP_GPIO_SET_MODE, BC_GPIO_OUTPUT,  // gpio_set_mode: Output,
    BC_PIN_U16(PIN_LCD_BL),                    //   pin 32 (backlight)
    BC_GPIO_LOW(PIN_LCD_BL),                   // backlight off
    BC_DELAY_MS(150),                          //
    BC_GPIO_HIGH(PIN_LCD_BL),                  // backlight on
    BC_DELAY_MS(350),                          //
    BC_GPIO_LOW(PIN_LCD_BL),                   // 2nd blink
    BC_DELAY_MS(150),                          //
    BC_GPIO_HIGH(PIN_LCD_BL),                  //
    BC_DELAY_MS(350),                          //
    BC_GPIO_LOW(PIN_LCD_BL),                   // 3rd blink
    BC_DELAY_MS(150),                          //
    BC_GPIO_HIGH(PIN_LCD_BL),                  //
    END_OF_SCRIPT,
};

// Button B: configure I2C bus 0 for the IP5306 (0x75) and read register
// 0x70 (charge control / state) into store slot 1.
static constexpr uint8_t SCRIPT_I2C_READ[] = {
    0x0F, OP_BUS_CONFIGURE,  // bus_configure (payload 14)
    KIND_I2C, 0x00,          //   kind=I2C, bus_id=0
    0x80, 0x1A, 0x06, 0x00,  //   freq      = 400000
    0x32, 0x00, 0x00, 0x00,  //   timeout   = 50 ms
    0x75, 0x00,              //   i2c_addr  = 0x75 (IP5306)
    0x02,                    //   flags     = use_restart
    0x01,                    //   register_address_bytes = 1
    0x08, OP_BUS_TRANSFER,   // bus_transfer (payload 7)
    KIND_I2C, 0x00,          //   kind=I2C, bus_id=0
    0x01,                    //   store_id = 1
    0x01,                    //   rx_len   = 1
    0x02,                    //   meta_size = 2
    0x01, 0x70,              //   meta: prefix_len=1, register 0x70
    END_OF_SCRIPT,
};

// Button C (first press): reset and wake the ILI9342C panel. Shows a
// mixed-kind script: GPIO reset pulse + backlight, then SPI configure
// and the command sequence. The panel RAM is not cleared, so expect
// noise pixels - INVON at the end makes the wake-up visibly obvious.
static constexpr uint8_t SCRIPT_LCD_INIT[] = {
    0x06, OP_GPIO_SET_MODE, BC_GPIO_OUTPUT,  // gpio_set_mode: Output,
    BC_PIN_U16(PIN_LCD_RST),                   //   pin 33 (LCD RST)
    BC_PIN_U16(PIN_LCD_BL),                    //   pin 32 (backlight)
    BC_GPIO_HIGH(PIN_LCD_BL),                  // backlight on
    BC_GPIO_LOW(PIN_LCD_RST),                  // RST low
    BC_DELAY_MS(20),                           //
    BC_GPIO_HIGH(PIN_LCD_RST),                 // RST high
    BC_DELAY_MS(120),                          //
    0x13, OP_BUS_CONFIGURE,                    // bus_configure (payload 18)
    KIND_SPI, 0x00,                            //   kind=SPI, bus_id=0
    0x0E, 0x00,                                //   pin_cs = 14
    0x80, 0x96, 0x98, 0x00,                    //   freq   = 10 MHz
    0x64, 0x00, 0x00, 0x00,                    //   timeout = 100 ms
    SPI_MODE_HALFDUPLEX_DC,                    //   data_mode = halfduplex_with_dc_pin
    0x00,                                      //   spi_mode 0, MSB first
    0x08, 0x00,                                //   command 8 bit, no address phase
    0x00, 0x00,                                //   no dummy cycles
    BC_LCD_CMD(0x01), BC_DELAY_MS(150),        // SWRESET
    BC_LCD_CMD(0x11), BC_DELAY_MS(120),        // SLPOUT
    BC_LCD_CMD(0x29),                          // DISPON
    BC_LCD_CMD(0x21),                          // INVON
    END_OF_SCRIPT,
};

// Button C (later presses): toggle inversion - two one-command scripts.
static constexpr uint8_t SCRIPT_LCD_INVERT_ON[]  = {BC_LCD_CMD(0x21), END_OF_SCRIPT};
static constexpr uint8_t SCRIPT_LCD_INVERT_OFF[] = {BC_LCD_CMD(0x20), END_OF_SCRIPT};

// ---- HAL objects + runner -----------------------------------------------------

m5hal::i2c::Bus i2c_bus;
m5hal::spi::Bus spi_bus;
m5hal::i2c::I2CMasterAccessConfig i2c_initial_cfg;  // overwritten by bus_configure scripts
m5hal::spi::SPIMasterAccessConfig spi_initial_cfg;
m5hal::i2c::I2CMasterAccessor i2c_dev{i2c_bus, i2c_initial_cfg};
m5hal::spi::SPIMasterAccessor spi_dev{spi_bus, spi_initial_cfg};

bytecode::BytecodeRunner runner;

bool hal_ready   = false;
bool lcd_started = false;
bool lcd_inverted = true;  // SCRIPT_LCD_INIT leaves the panel inverted

// Edge-detected active-low button (Core BASIC buttons have external pull-ups).
struct Button {
    int pin;
    bool last = true;

    bool pressed(void)
    {
        const bool now = digitalRead(pin) != LOW;
        const bool hit = last && !now;
        last           = now;
        return hit;
    }
};

Button btn_a{PIN_BTN_A};
Button btn_b{PIN_BTN_B};
Button btn_c{PIN_BTN_C};

static void runScript(const char* label, const uint8_t* script, size_t len)
{
    auto result = runner.run(m5hal::data::ConstDataSpan{script, len});
    if (!result.has_value()) {
        Serial.printf("%s: error %d at offset %u\n", label, static_cast<int>(result.error()),
                      static_cast<unsigned>(runner.lastOffset()));
        return;
    }
    Serial.printf("%s: ok (%u bytes executed)\n", label, static_cast<unsigned>(result.value()));
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("M5HAL HowToUseBytecode (M5Stack Core BASIC)");
    Serial.println("A: GPIO backlight blink / B: I2C IP5306 read / C: SPI LCD init + invert");

    pinMode(PIN_BTN_A, INPUT);
    pinMode(PIN_BTN_B, INPUT);
    pinMode(PIN_BTN_C, INPUT);

    m5hal::i2c::BusConfig i2c_cfg{&Wire, PIN_I2C_SCL, PIN_I2C_SDA};
    if (auto r = i2c_bus.init(i2c_cfg); !r.has_value()) {
        Serial.printf("i2c bus init failed: %d\n", static_cast<int>(r.error()));
        return;
    }

    m5hal::spi::BusConfig spi_cfg;
    spi_cfg.spi      = &SPI;
    spi_cfg.pin_clk  = PIN_SPI_CLK;
    spi_cfg.pin_mosi = PIN_SPI_MOSI;
    spi_cfg.pin_miso = PIN_SPI_MISO;
    spi_cfg.pin_dc   = PIN_SPI_DC;
    if (auto r = spi_bus.init(spi_cfg); !r.has_value()) {
        Serial.printf("spi bus init failed: %d\n", static_cast<int>(r.error()));
        return;
    }

    // Everything the scripts may address gets registered up front; the
    // scripts themselves carry the per-target configuration.
    (void)runner.registerI2C(0, i2c_dev);
    (void)runner.registerSPI(0, spi_dev);
    runner.setGPIOGroup(m5hal::M5_Hal.Gpio);

    hal_ready = true;
    Serial.println("ready.");
}

void loop()
{
    if (!hal_ready) {
        delay(1000);
        return;
    }

    if (btn_a.pressed()) {
        runScript("GPIO blink", SCRIPT_GPIO_BLINK, sizeof(SCRIPT_GPIO_BLINK));
    }

    if (btn_b.pressed()) {
        runScript("I2C read", SCRIPT_I2C_READ, sizeof(SCRIPT_I2C_READ));
        auto stored = runner.storedData(1);
        if (stored.size > 0) {
            Serial.printf("  IP5306 reg 0x70 = 0x%02X\n", stored.data[0]);
        }
    }

    if (btn_c.pressed()) {
        if (!lcd_started) {
            runScript("LCD init", SCRIPT_LCD_INIT, sizeof(SCRIPT_LCD_INIT));
            lcd_started  = true;
            lcd_inverted = true;
        } else if (lcd_inverted) {
            runScript("LCD invert off", SCRIPT_LCD_INVERT_OFF, sizeof(SCRIPT_LCD_INVERT_OFF));
            lcd_inverted = false;
        } else {
            runScript("LCD invert on", SCRIPT_LCD_INVERT_ON, sizeof(SCRIPT_LCD_INVERT_ON));
            lcd_inverted = true;
        }
    }

    delay(10);  // light debounce for the edge detector
}
