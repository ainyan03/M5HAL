// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseI2C
//
// Minimal I2C Bus / Accessor sketch. It does not assume a specific sensor:
// the sketch scans the bus, uses the first responding device, then demonstrates
// probe, readRegister, burst read, and ScopedAccess.
//
// Pin defaults match M5Stack Core (Basic / Gray / Fire) style wiring:
//   SDA=21, SCL=22
//
// To force a backend, change ExampleBus below. The default m5hal::i2c::Bus
// is the first backend offered by the active build environment.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <Wire.h>

namespace m5hal = m5::hal::v1;

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// Register constants keep the intended register-address width visible.
static constexpr uint8_t REG_PROBE_R  = 0x00;
static constexpr uint8_t REG_PROBE_R2 = 0x01;

#ifndef M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ
#define M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ 100000
#endif

// m5hal::i2c::Bus resolves to the first backend the build offers
// (framework scan order; see spec/design/variants.md). Uncomment a
// suffixed variant type below to force a specific backend instead.
using ExampleBus = m5hal::i2c::Bus;
// using ExampleBus = m5hal::i2c::Bus_arduino;
// using ExampleBus = m5hal::i2c::Bus_espidf;
// using ExampleBus = m5hal::i2c::Bus_software;

ExampleBus i2c_bus;

// -------------------------------------------------------------------------

static uint16_t scanFirst(m5hal::i2c::IBus& bus)
{
    Serial.println("I2C scan:");
    uint16_t found = 0xFFFF;
    for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
        if (bus.probe(addr).has_value()) {
            Serial.printf("  found device at 0x%02X\n", addr);
            if (found == 0xFFFF) {
                found = addr;
            }
        }
    }
    return found;
}

// -------------------------------------------------------------------------

static void demoReadRegister(m5hal::i2c::MasterAccessor& dev)
{
    auto v = dev.readRegister(0x00);  // literal sugar: 1-byte register address by default
    if (!v.has_value()) {
        Serial.printf("readRegister(0x00) failed: %d\n", (int)v.error());
        return;
    }
    Serial.printf("register 0x00 = 0x%02X\n", v.value());
}

// -------------------------------------------------------------------------

static void demoBurstRead(m5hal::i2c::MasterAccessor& dev)
{
    uint8_t buf[4] = {};
    auto r         = dev.readRegister(REG_PROBE_R, buf, sizeof(buf));
    if (!r.has_value()) {
        Serial.printf("burst read failed: %d\n", (int)r.error());
        return;
    }
    Serial.printf("registers 0x00..0x03 = %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3]);
}

// -------------------------------------------------------------------------

static void demoScopedAccess(m5hal::i2c::MasterAccessor& dev)
{
    // Pass an explicit lock budget when you can handle the timeout;
    // omitting it means "wait forever" (handy, but be deliberate).
    m5hal::bus::ScopedAccess scope{dev, 100};
    if (scope.has_error()) {
        Serial.printf("ScopedAccess failed: %d\n", (int)scope.error());
        return;
    }
    // These two transfers run while the bus lock is held.
    auto a = dev.readRegister(REG_PROBE_R);
    auto b = dev.readRegister(REG_PROBE_R2);
    if (a.has_value() && b.has_value()) {
        Serial.printf("atomic read: 0x%02X 0x%02X\n", a.value(), b.value());
    }
}  // scope destructor unlocks the bus

// -------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("M5HAL HowToUseI2C");
    Serial.printf("pins: SDA=%d SCL=%d freq=%u\n", PIN_SDA, PIN_SCL,
                  static_cast<unsigned>(M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ));

    m5hal::i2c::BusConfig bus_cfg;
    bus_cfg.wire    = &Wire;
    bus_cfg.pin_scl = PIN_SCL;
    bus_cfg.pin_sda = PIN_SDA;

    if (auto r = i2c_bus.init(bus_cfg); !r) {
        Serial.printf("Bus init failed: %d\n", (int)r.error());
        return;
    }

    auto addr = scanFirst(i2c_bus);
    if (addr == 0xFFFF) {
        Serial.println("No I2C device found, abort.");
        return;
    }
    Serial.printf("Using device at 0x%02X for demo.\n", addr);

    m5hal::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = addr;
    acc_cfg.freq       = M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ;
    acc_cfg.wire_timeout_ms = 100;
    m5hal::i2c::MasterAccessor dev{i2c_bus, acc_cfg};

    demoReadRegister(dev);
    demoBurstRead(dev);
    demoScopedAccess(dev);

    Serial.println("HowToUseI2C done.");
}

void loop()
{
    delay(1000);
}
