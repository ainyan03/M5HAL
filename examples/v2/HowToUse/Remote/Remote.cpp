// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseRemote
//
// Minimal PC host-side example: connect to an ESP32 running the
// RemoteServer firmware and drive its I2C / GPIO through the Hal
// facade — the same API used for local hardware.
//
// Build & run (PlatformIO):
//   pio run -e HowToUse_Remote_host
//   .pio/build/HowToUse_Remote_host/program [uart:<path>|tcp:<host>:<port>]
//
// If no endpoint argument is given, the program picks the first USB serial
// candidate (name heuristic, not a hello probe) and connects to uart:<path>.
// initUart() remains available as the typed UART-only API; this example uses
// connect(endpoint) as the common path.
//
// Device firmware (3 Mbaud, matching the default DeviceConfig):
//     pio run -e RemoteServer_esp32    -t upload  (Core2 / UART0 3Mbaud)
//     pio run -e RemoteServer_esp32s3  -t upload  (CoreS3 / USB-JTAG)
//
// This example is POSIX-only (macOS / Linux). It is not an Arduino
// sketch; the device side uses RemoteServer, not this file.
// =============================================================================

#if !defined(ARDUINO)

#include <M5HAL_v2.hpp>

#include <stdio.h>
#include <stdlib.h>

namespace m5hal      = m5::hal::v2;
namespace posix_uart = m5::variants::frameworks::posix::hal::v2::uart;

// ---------------------------------------------------------------------------
// Auto-discover: pick the best USB serial candidate by name heuristic.
// This does NOT probe (hello); connect() in main() does the handshake.
// ---------------------------------------------------------------------------
static bool autoDiscover(const char** out_path)
{
    static posix_uart::SerialPortInfo ports[8];
    size_t n = posix_uart::listSerialPorts(ports, sizeof(ports) / sizeof(ports[0]));
    if (n == 0) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        if (ports[i].rank <= 1) {
            *out_path = ports[i].path;
            return true;
        }
    }
    *out_path = ports[0].path;
    return true;
}

// ---------------------------------------------------------------------------
// I2C bus scan: probe addresses 0x08..0x77.
// ---------------------------------------------------------------------------
static void scanI2C(m5hal::Hal& hal)
{
    auto cfg = hal.I2C.createBusConfig(m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21});
    auto bus = hal.I2C.acquire(cfg);
    if (!bus.has_value()) {
        ::printf("  I2C acquire failed: %s (%d)\n", m5hal::error::toString(bus.error()), static_cast<int>(bus.error()));
        return;
    }

    // Hal::capabilities() describes the connection advertisement. This
    // snapshot describes this acquired I2C Bus instance and its session.
    const auto instance_caps = bus.value()->capabilities();
    ::printf("I2C instance: master=%s tx=%s rx=%s generation=%u\n",
             instance_caps.supports(m5hal::bus::BusFeature::MasterTransfer) ? "yes" : "no",
             instance_caps.supports(m5hal::bus::BusFeature::Transmit) ? "yes" : "no",
             instance_caps.supports(m5hal::bus::BusFeature::Receive) ? "yes" : "no",
             static_cast<unsigned>(instance_caps.generation()));

    ::printf("I2C scan (SCL=22, SDA=21):\n");
    int found_count     = 0;
    uint16_t first_addr = 0xFFFF;
    for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
        if (bus.value()->probe(addr).has_value()) {
            ::printf("  0x%02X  ACK\n", addr);
            if (first_addr == 0xFFFF) {
                first_addr = addr;
            }
            ++found_count;
        }
    }
    ::printf("  %d device(s) found.\n", found_count);

    if (first_addr == 0xFFFF) {
        return;
    }

    // Read register 0x00 from the first device. Target address and clock live
    // in the accessor config; readRegister wraps its own bus transaction (the
    // same pattern as the local I2C example).
    {
        m5hal::i2c::MasterAccessConfig acc_cfg;
        acc_cfg.i2c_addr = first_addr;
        acc_cfg.freq     = 100000;
        m5hal::i2c::MasterAccessor acc{bus.value(), acc_cfg};

        auto reg = acc.readRegister(0x00);  // 1-byte register-address default
        if (reg.has_value()) {
            ::printf("  [0x%02X] reg 0x00 = 0x%02X\n", first_addr, reg.value());
        } else {
            ::printf("  [0x%02X] readRegister failed: %s\n", first_addr, m5hal::error::toString(reg.error()));
        }
    }

    // Explicitly close the bus. For remote backends this sends
    // BusRelease to the peer so the device frees the hardware resource. All
    // accessors must be gone first; close consumes and clears this owner.
    auto rel = hal.I2C.close(bus.value());
    if (rel.has_value()) {
        ::printf("  I2C bus closed.\n");
    } else {
        ::printf("  I2C close failed: %s\n", m5hal::error::toString(rel.error()));
    }
}

// ---------------------------------------------------------------------------
// GPIO port read (if the remote device exposes GPIO).
// Current implementation returns host-side cached values seeded at
// connection time. Live push updates use GpioSubscribe / EvtGpioState
// (see spec/design/remote.md, push events) and are not demonstrated here.
// ---------------------------------------------------------------------------
static void readGPIO(m5hal::Hal& hal)
{
    if (!hal.hasRemoteGpio()) {
        ::printf("GPIO: not available on this device.\n");
        return;
    }

    auto slot       = hal.remoteGpioSlot();
    const auto* igp = hal.Gpio.getGPIO(slot);
    if (igp == nullptr) {
        return;
    }

    ::printf("GPIO (slot %d, %u pin(s), %u port(s)):\n", static_cast<int>(slot), igp->getPinCount(),
             igp->getPortCount());

    for (uint8_t p = 0; p < igp->getPortCount(); ++p) {
        auto pa = hal.Gpio.getPort(slot, p);
        if (!pa.has_value()) {
            continue;
        }
        uint32_t val = pa.value().port->readPort();
        ::printf("  port[%u] = 0x%08X  (", p, val);
        for (int b = 31; b >= 0; --b) {
            ::putchar((val & (1u << b)) ? '1' : '0');
            if (b > 0 && (b % 8) == 0) {
                ::putchar('_');
            }
        }
        ::printf(")\n");
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    const char* endpoint = nullptr;
    char auto_endpoint[512];

    if (argc > 1) {
        endpoint = argv[1];
    } else {
        const char* port = nullptr;
        ::printf("No serial port specified; auto-discovering...\n");
        if (!autoDiscover(&port)) {
            ::fprintf(stderr, "No serial port found. Pass an endpoint as argument.\n");
            return 1;
        }
        int n = ::snprintf(auto_endpoint, sizeof(auto_endpoint), "uart:%s", port);
        if (n < 0 || static_cast<size_t>(n) >= sizeof(auto_endpoint)) {
            ::fprintf(stderr, "Discovered serial endpoint is too long: %s\n", port);
            return 1;
        }
        endpoint = auto_endpoint;
        ::printf("Trying %s\n", endpoint);
    }

    // ---- Connect ----
    m5hal::Hal remote;
    auto r = remote.connect(endpoint);
    if (!r.has_value()) {
        ::fprintf(stderr, "connect(%s) failed: %s (%d)\n", endpoint, m5hal::error::toString(r.error()),
                  static_cast<int>(r.error()));
        return 1;
    }

    // Print capabilities.
    const auto* caps = remote.capabilities();
    if (caps != nullptr) {
        ::printf("Connected to %s  (GPIO: %s, buses: %u)\n", endpoint, caps->has_gpio ? "yes" : "no",
                 static_cast<unsigned>(caps->bus_count));
    } else {
        ::printf("Connected to %s\n", endpoint);
    }

    // ---- Demo operations ----
    scanI2C(remote);
    readGPIO(remote);

    ::printf("Done.\n");
    return 0;
}

#endif  // !ARDUINO
