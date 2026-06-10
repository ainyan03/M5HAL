// =============================================================================
// M5HAL — HowToUseRemoteBus (device side)
//
// Turns an M5Stack Core BASIC into a remote bus server: a PC connected
// over the USB serial cable can drive this board's I2C bus as if it were
// local. The counterpart host program lives in host/remote_bus_host.cpp
// (built on the PC with the posix UART variant; see the comment there).
//
// How it works (spec/design/remote.md):
//   - UART0 (the USB bridge, GPIO1/3) carries framed remote messages.
//     There is deliberately NO Serial console logging: the USB channel
//     belongs to the protocol.
//   - A remote::Server executes the received bytecode scripts on a
//     BytecodeRunner. Registration = allowlist: this sketch registers
//     ONE I2C accessor (the internal bus, SDA=21 / SCL=22) and a GPIO
//     group filtered down to the three buttons, so that is all a remote
//     peer can reach.
//   - GPIO is NOT published wholesale: handing the platform IGPIO to the
//     server would expose every pin, including the flash-connected ones
//     (spec/design/remote.md, safety boundary). AllowlistGPIO below
//     forwards only the listed pins.
//   - remote::RemoteServerService polls the link cooperatively from a
//     service::ServiceRunner in loop().
//
// On a Core BASIC the internal I2C carries the IP5306 power chip at
// 0x75, so the host's bus scan finds at least one device with no extra
// wiring, and the buttons A/B/C (GPIO 39/38/37) are readable remotely.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>

#include <Wire.h>

namespace m5hal = m5::hal::v1;

namespace {

constexpr int PIN_UART_TX = 1;   // UART0 TX (USB bridge) on classic ESP32
constexpr int PIN_UART_RX = 3;   // UART0 RX (USB bridge)
constexpr int PIN_I2C_SCL = 22;  // Core BASIC internal I2C
constexpr int PIN_I2C_SDA = 21;

// The only GPIO pins a remote peer may touch (Core BASIC buttons A/B/C).
constexpr uint8_t PUBLISHED_PINS[] = {39, 38, 37};

// Forwards an IGPIO but resolves only the allowlisted pins: everything
// else looks unconnected to the remote peer. This is the safety-boundary
// pattern from spec/design/remote.md — publish a dedicated, filtered
// view instead of the whole pin bank.
class AllowlistGPIO : public m5hal::gpio::IGPIO {
public:
    AllowlistGPIO(const m5hal::gpio::IGPIO* inner, const uint8_t* pins, size_t count)
        : _inner{inner}, _pins{pins}, _count{count}
    {
    }

    m5hal::gpio::IPort* portForPin(m5hal::types::gpio_local_pin_t pin_index) const override
    {
        return allowed(pin_index) ? _inner->portForPin(pin_index) : nullptr;
    }
    m5hal::gpio::IPort* getPort(uint8_t) const override
    {
        return nullptr;  // no whole-port access through the allowlist
    }
    uint16_t getPinCount() const override
    {
        return _inner->getPinCount();  // numbering stays the physical one
    }
    uint8_t getPortCount() const override
    {
        return 0;
    }
    bool isValid(m5hal::types::gpio_local_pin_t pin_index) const override
    {
        return allowed(pin_index) && _inner->isValid(pin_index);
    }

private:
    bool allowed(m5hal::types::gpio_local_pin_t pin_index) const
    {
        for (size_t i = 0; i < _count; ++i) {
            if (_pins[i] == pin_index) {
                return true;
            }
        }
        return false;
    }

    const m5hal::gpio::IGPIO* _inner = nullptr;
    const uint8_t* _pins             = nullptr;
    size_t _count                    = 0;
};

#ifndef M5HAL_EXAMPLE_HOWTOUSEREMOTEBUS_BAUD
#define M5HAL_EXAMPLE_HOWTOUSEREMOTEBUS_BAUD 115200
#endif

// --- the link (UART0 over USB) ---
m5hal::uart::Bus uart_bus;
m5hal::uart::UARTAccessConfig uart_cfg;
m5hal::uart::UARTTxAccessor* uart_tx = nullptr;
m5hal::uart::UARTRxAccessor* uart_rx = nullptr;

// --- the published bus (internal I2C) ---
m5hal::i2c::Bus i2c_bus;
m5hal::i2c::I2CMasterAccessConfig i2c_acc_cfg;
m5hal::i2c::I2CMasterAccessor* i2c_acc = nullptr;

// --- the server ---
uint8_t server_scratch[m5hal::remote::kMaxMessageSize];
uint8_t rx_scratch[m5hal::frame::kMaxWireSize];
uint8_t tx_scratch[m5hal::frame::kMaxWireSize];
m5hal::remote::Server* server                 = nullptr;
m5hal::remote::RemoteServerService* remote_sv = nullptr;
m5hal::service::ServiceRunner runner;

}  // namespace

void setup()
{
    // UART0 through M5HAL (no Serial.begin(); the variant begins lazily).
    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.serial         = &Serial;
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 1024;
    bus_cfg.tx_buffer_size = 1024;
    (void)uart_bus.init(bus_cfg);

    uart_cfg.baud_rate = M5HAL_EXAMPLE_HOWTOUSEREMOTEBUS_BAUD;
    // Tiny read timeouts: the server polls cooperatively, and a
    // StreamSource peek blocks up to first_byte_timeout when the line
    // is idle (spec/design/remote.md, server execution model).
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::UARTTxAccessor tx_acc{uart_bus, uart_cfg};
    static m5hal::uart::UARTRxAccessor rx_acc{uart_bus, uart_cfg};
    uart_tx = &tx_acc;
    uart_rx = &rx_acc;

    // The internal I2C bus this board publishes to the remote peer.
    m5hal::i2c::BusConfig i2c_cfg{&Wire, PIN_I2C_SCL, PIN_I2C_SDA};
    (void)i2c_bus.init(i2c_cfg);
    static m5hal::i2c::I2CMasterAccessor acc{i2c_bus, i2c_acc_cfg};
    i2c_acc = &acc;

    // Server + cooperative poll service. Registering the accessor as
    // bus_id 0 is what makes it reachable (and nothing else is).
    static m5hal::remote::Server srv{m5hal::data::DataSpan{server_scratch, sizeof(server_scratch)}};
    (void)srv.registerI2C(0, *i2c_acc);

    // Publish the buttons (and nothing else) through the allowlist view.
    static AllowlistGPIO published_gpio{m5hal::gpio::getGPIO(), PUBLISHED_PINS,
                                        sizeof(PUBLISHED_PINS) / sizeof(PUBLISHED_PINS[0])};
    static m5hal::gpio::GPIOGroup published_group;
    if (published_group.addGPIO(&published_gpio, 0).has_value()) {
        srv.setGPIOGroup(published_group);
    }
    server = &srv;

    static m5hal::data::StreamSource link_src{*uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink link_snk{*uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::remote::RemoteServerService sv{srv, link_src, link_snk};
    remote_sv = &sv;
    runner.add(sv);
}

void loop()
{
    runner.runOnce();
}
