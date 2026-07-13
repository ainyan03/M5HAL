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
// m5hal::i2c::Bus is a runtime FACADE: the bus type is always i2c::Bus, and
// init() picks the concrete backend from the CONFIG type you pass. The default
// i2c::BusConfig maps to the build's winner backend; an explicit suffixed
// config forces a specific one (see the bus declaration and setup() below).
// Build with -DM5HAL_EXAMPLE_FORCE_SOFTWARE_I2C to drive the same pins with the
// software (bit-bang) backend.
//
// Shared-owner model: acquire() interns the wiring and returns an owning
// shared_ptr. The registry retains only a weak reference. Two acquires of the
// same pins return the SAME instance (one
// physical bus, one lock), so a board-support layer and user code cooperate
// instead of fighting over the wire. The backend is chosen by the config TYPE
// passed to acquire (default i2c::BusConfig = the build's winner; _arduino /
// _espidf / _software select a specific one), so the handle is always
// shared_ptr<i2c::IBus>. (You can still own a bus yourself with
// `i2c::Bus bus; bus.init(cfg)` and pass it by reference — see README.)
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v2.hpp>
#include <Wire.h>

#include <memory>

namespace m5hal = m5::hal::v2;

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// Register constants keep the intended register-address width visible.
static constexpr uint8_t REG_PROBE_R  = 0x00;
static constexpr uint8_t REG_PROBE_R2 = 0x01;

#ifndef M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ
#define M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ 100000
#endif

// Shared owner, assigned in setup().
std::shared_ptr<m5hal::i2c::IBus> i2c_bus;

// -------------------------------------------------------------------------

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %s (%d)\n", label, m5hal::error::toString(error), static_cast<int>(error));
}

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
    auto v = dev.readRegister(REG_PROBE_R);  // 1-byte register-address default
    if (!v.has_value()) {
        printError("readRegister(REG_PROBE_R)", v.error());
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
        printError("burst read", r.error());
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
        printError("ScopedAccess", scope.error());
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

    // Tag-typed pins: either order is correct (no swapped-pin accidents). The
    // config TYPE selects the backend: the default BusConfig maps to the winner
    // backend (and carries the Arduino TwoWire handle), while BusConfig_software
    // selects the bit-bang backend on the same pins.
#ifdef M5HAL_EXAMPLE_FORCE_SOFTWARE_I2C
    m5hal::i2c::BusConfig_software bus_cfg{m5hal::i2c::Scl{PIN_SCL}, m5hal::i2c::Sda{PIN_SDA}};
#else
    m5hal::i2c::BusConfig bus_cfg{m5hal::i2c::Scl{PIN_SCL}, m5hal::i2c::Sda{PIN_SDA}};
    bus_cfg.wire = &Wire;
#endif

    // Acquire the interned bus. This shared handle owns its lifetime.
    auto acquired = m5hal::M5_Hal.I2C.acquire(bus_cfg);
    if (!acquired) {
        printError("Bus acquire", acquired.error());
        return;
    }
    i2c_bus = acquired.value();

    auto addr = scanFirst(*i2c_bus);
    if (addr == 0xFFFF) {
        Serial.println("No I2C device found, abort.");
        return;
    }
    Serial.printf("Using device at 0x%02X for demo.\n", addr);

    m5hal::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = addr;
    acc_cfg.freq            = M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ;
    acc_cfg.wire_timeout_ms = 100;
    m5hal::i2c::MasterAccessor dev{i2c_bus, acc_cfg};  // co-owns the acquired bus

    demoReadRegister(dev);
    demoBurstRead(dev);
    demoScopedAccess(dev);

    Serial.println("HowToUseI2C done.");
}

void loop()
{
    delay(1000);
}
