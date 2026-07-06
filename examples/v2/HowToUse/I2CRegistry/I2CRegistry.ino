// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseI2CRegistry
//
// HowToUse/I2C shows the basic borrow (M5_Hal.I2C.acquire(cfg) -> one accessor).
// This sketch layers the two ADR 034 features that borrowing unlocks:
//
//  1. M5_Hal.I2C.acquire(cfg): get a bus by its WIRING (pins). The same pins
//     always return the SAME shared instance -- one physical bus, one lock --
//     so a board-support layer and user code that name the same pins cooperate
//     instead of fighting over the wire with two separate locks.
//
//  2. acquire(LogicalBusConfig{pins, intent}) + commitBuses(): declare HOW you
//     want each bus driven (must be hardware / prefer hardware / either / force
//     software) and let M5HAL bin-pack the limited hardware controllers across
//     all your buses, then query the result on the bus instance. This is how a
//     board with more I2C buses than hardware controllers (e.g. M5StickC's
//     internal + PortA + HAT on a 2-controller ESP32) decides which run in
//     hardware and which fall back to software.
//
// Nothing here needs a real device: probes just NACK and the allocation/query
// is what the demo shows.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v2.hpp>

#include <memory>

namespace m5hal = m5::hal::v2;

// -------------------------------------------------------------------------

static void printError(const char* label, m5hal::error::error_t error)
{
    Serial.printf("%s failed: %s (%d)\n", label, m5hal::error::toString(error), static_cast<int>(error));
}

// --- 1. acquire: one wiring, one shared instance --------------------------

static void demoAcquireSharing()
{
    Serial.println("== acquire: same wiring -> one shared bus ==");

    // The bit-bang backend keeps this demo from holding a hardware controller,
    // leaving both for the intent demo below.
    m5hal::i2c::BusConfig_software cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};

    auto a = m5hal::M5_Hal.I2C.acquire(cfg);
    if (!a) {
        printError("acquire", a.error());
        return;
    }
    // A second acquire of the SAME pins hands back the SAME instance, so two
    // independent code paths can each hold their shared_ptr and share the lock.
    auto b = m5hal::M5_Hal.I2C.acquire(cfg);
    Serial.printf("  two acquires of (SCL22,SDA21) -> %s instance\n",
                  (a.value().get() == b.value().get()) ? "the SAME" : "DIFFERENT");

    // Use it like any bus (a NACK just means nothing answered at that address).
    Serial.printf("  probe 0x68 over the shared bus: %s\n", a.value()->probe(0x68).has_value() ? "ACK" : "no device");

    // The bus lives while any shared_ptr holds it; both a and b drop at return,
    // so the bus is released and its registry slot reclaimed.
}

// --- 2. intent: declare HOW, then commit + query --------------------------

static void report(const char* name, const std::shared_ptr<m5hal::i2c::IBus>& bus)
{
    const bool hw = bus->backendKind() == m5hal::types::BackendKind::Hardware;
    Serial.printf("  %-14s -> %-8s controller=%2d maxFreq=%luHz\n", name, hw ? "HARDWARE" : "software",
                  (int)bus->controllerId(), (unsigned long)bus->maxFrequency());
}

static void demoIntent()
{
    Serial.println("== intent: declare HOW, then commit + query ==");
    namespace i2c = m5hal::i2c;

    // M5StickC-style: three I2C buses, but an ESP32 has only two I2C controllers,
    // so one must run in software. Declare the wiring and the intent together in
    // one LogicalBusConfig, built with an intent helper: must-be-hardware /
    // hardware-if-free / either.
    auto internal =
        m5hal::M5_Hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}, i2c::requireHardware()});
    auto portA = m5hal::M5_Hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{26}, i2c::Sda{25}, i2c::preferHardware()});
    auto hat   = m5hal::M5_Hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{19}, i2c::Sda{18}, i2c::automatic()});
    if (!internal || !portA || !hat) {
        Serial.println("  acquire failed");
        return;
    }

    // Resolve the whole set at once: require/prefer-hardware buses take the
    // controllers first, the rest fall back to software. commitBuses() returns
    // OUT_OF_RESOURCE only if a require-hardware bus cannot get a controller.
    if (auto r = m5hal::M5_Hal.I2C.commitBuses(); !r) {
        printError("commitBuses", r.error());
        return;
    }

    report("internal(Req)", internal.value());
    report("portA(Prefer)", portA.value());
    report("hat(Auto)", hat.value());

    // To change the plan later, re-acquire a bus with a stronger intent and
    // commit again: the factory hot-swaps backends under the bus lock so a
    // lower-priority bus yields its controller (see spec/design/i2c.md).
}

// -------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("M5HAL HowToUseI2CRegistry");

    demoAcquireSharing();
    demoIntent();

    Serial.println("HowToUseI2CRegistry done.");
}

void loop()
{
    delay(1000);
}
