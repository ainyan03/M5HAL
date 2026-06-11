// SPDX-License-Identifier: MIT
//
// M5HAL — HowToUseRemoteBus (host side, PC)
//
// Drives the I2C bus of a board running the RemoteBus.ino sketch as if
// it were a local bus, over the USB serial cable. Connection setup is
// two calls (spec/design/remote.md §接続ユーティリティ):
//
//   SerialRemoteEndpoint ep{baud};       // posix Bus + accessors + RemoteLink
//   connectRemoteSerial(ep, opt);        // enumerate ports x hello until a peer answers
//
// With no port given, candidates are probed best-first and the peer is
// the first port that answers hello (name heuristics only propose;
// the protocol decides). Opening a port DTR-resets a classic ESP32, so
// strong candidates get a few retries while the board boots.
//
// Build & run (from the M5HAL repository root):
//   pio run -e HowToUse_RemoteBusHost_native
//   .pio/build/HowToUse_RemoteBusHost_native/program              # auto-discover the port
//   .pio/build/HowToUse_RemoteBusHost_native/program <port> [baud]
//
// (This file lives in a subfolder of the sketch, so the Arduino IDE does
//  not try to compile it for the device.)

#if !defined(ARDUINO)

#include <M5HAL_v1.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace m5hal      = m5::hal::v1;
namespace posix_uart = m5hal::uart::variant::posix;  // the posix UART variant alias

int main(int argc, char** argv)
{
    // Keep progress visible even when stdout is a pipe (e.g. running
    // through `pio run -t upload`, which block-buffers it otherwise:
    // nothing would appear until the program exits).
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    const char* port    = argc > 1 ? argv[1] : ::getenv("M5HAL_POSIX_UART_PORT");
    const uint32_t baud = argc > 2 ? static_cast<uint32_t>(::strtoul(argv[2], nullptr, 10)) : 115200;

    // Connect: explicit port when given, auto-discovery otherwise. The
    // utility hardware-resets each candidate into run mode before the
    // hello probes (ConnectOptions::hardware_reset, default on).
    posix_uart::SerialRemoteEndpoint ep{baud};
    posix_uart::ConnectOptions opt;
    opt.path       = port;  // nullptr -> walk the listSerialPorts candidates
    opt.on_attempt = [](void*, const char* p) { ::printf("trying %s ...\n", p); };
    auto caps      = posix_uart::connectRemoteSerial(ep, opt);
    if (!caps.has_value()) {
        ::fprintf(stderr, "no RemoteBus device found: %d (is RemoteBus.ino running?)\n",
                  static_cast<int>(caps.error()));
        return 1;
    }
    ::printf("using %s @ %u baud\n", ep.devicePath(), static_cast<unsigned>(baud));

    // Capability list from the hello exchange.
    ::printf("remote proto v%u, %u bus(es)%s\n", caps.value().proto_ver,
             static_cast<unsigned>(caps.value().bus_count), caps.value().has_gpio ? ", gpio" : "");
    for (size_t i = 0; i < caps.value().bus_count; ++i) {
        ::printf("  bus %u: kind=%u\n", caps.value().buses[i].bus_id,
                 static_cast<unsigned>(caps.value().buses[i].kind));
    }

    // The proxy is a normal i2c::I2CBus; every accessor sugar just works.
    m5hal::remote::RemoteI2CBus remote_bus{ep.link.session(), 0};
    m5hal::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.timeout_ms = 100;
    m5hal::i2c::I2CMasterAccessor dev{remote_bus, acc_cfg};

    ::printf("scanning the remote I2C bus...\n");
    size_t found = 0;
    for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
        acc_cfg.i2c_addr = addr;
        if (!dev.setConfig(acc_cfg).has_value()) {
            continue;
        }
        if (dev.probe().has_value()) {
            ::printf("  found device at 0x%02X\n", addr);
            ++found;
        }
    }
    if (found == 0) {
        ::printf("  no devices answered\n");
        return 1;
    }

    // On a Core BASIC the IP5306 power chip sits at 0x75: read a register
    // through the remote bus exactly like a local readRegister call.
    acc_cfg.i2c_addr = 0x75;
    if (dev.setConfig(acc_cfg).has_value() && dev.probe().has_value()) {
        auto reg = dev.readRegister(uint8_t{0x00});
        if (reg.has_value()) {
            ::printf("IP5306 reg 0x00 = 0x%02X (read remotely)\n", reg.value());
        }
    }

    // Remote GPIO, event-driven (spec/design/remote.md §push イベント):
    // subscribe to the buttons once, then the DEVICE pushes an
    // evt_gpio_state script whenever one changes — no per-pin polling
    // RPCs. The host just pumps session.poll() and the registered
    // handler logs each transition.
    if (caps.value().has_gpio) {
        m5hal::remote::RemoteGPIO remote_gpio{ep.link.session(), 40, 0};  // remote slot 0
        m5hal::gpio::GPIOGroup remote_pins;
        if (remote_pins.addGPIO(&remote_gpio, 0).has_value()) {
            const uint8_t buttons[] = {39, 38, 37};  // Core BASIC A / B / C
            const char* names[]     = {"A", "B", "C"};

            // Pin handles still work for setup and the initial snapshot.
            for (size_t i = 0; i < 3; ++i) {
                auto pin = remote_pins.tryGetPin(m5hal::types::makeGpioNumber(0, buttons[i]));
                if (!pin.has_value()) {
                    ::fprintf(stderr, "button %s unavailable\n", names[i]);
                    return 1;
                }
                pin.value().setMode(m5hal::types::gpio_mode_t::Input);
                ::printf("%s%s=%d", i == 0 ? "initial (pressed=0): " : " ", names[i], pin.value().read() ? 1 : 0);
            }
            ::printf("\n");

            // The event handler receives the REMOTE gpio_number space.
            struct Ctx {
                const uint8_t* buttons;
                const char* const* names;
            } ctx{buttons, names};
            ep.link.session().runner().setGpioEventHandler(
                [](void* c, m5hal::types::gpio_number_t pin, bool level) {
                    auto* x             = static_cast<Ctx*>(c);
                    const uint8_t local = static_cast<uint8_t>(m5hal::types::extractLocalPin(pin));
                    for (size_t i = 0; i < 3; ++i) {
                        if (x->buttons[i] == local) {
                            ::printf("button %s %s\n", x->names[i], level ? "released" : "pressed");
                            return;
                        }
                    }
                },
                &ctx);

            for (size_t i = 0; i < 3; ++i) {
                auto s = remote_gpio.subscribe(buttons[i]);
                if (!s.has_value()) {
                    ::fprintf(stderr, "subscribe %s failed: %d\n", names[i], static_cast<int>(s.error()));
                    return 1;
                }
            }
            ::printf("watching buttons A/B/C via push events (Ctrl+C to exit)\n");

            // poll() blocks for at most the rx stream's own timeout when
            // idle, so this loop self-paces; ping ~ every 2 s guards the
            // link (events are best-effort and carry no liveness signal).
            for (unsigned cycle = 0;; ++cycle) {
                (void)ep.link.session().poll();
                if (cycle % 20 == 19 && !ep.link.session().ping().has_value()) {
                    ::fprintf(stderr, "connection lost\n");
                    return 1;
                }
            }
        }
    }

    ::printf("done\n");
    return 0;
}

#endif  // !ARDUINO
