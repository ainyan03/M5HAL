// SPDX-License-Identifier: MIT
//
// POSIX host half of the remote adopted-bus HIL. Run one mode per target
// responder: `program uart:/dev/cu.usbserial-... i2c|spi`.

#include <M5HAL_v2.hpp>

#include <array>
#include <cstdio>
#include <cstring>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int kI2cSda  = 32;
constexpr int kI2cScl  = 33;
constexpr int kSpiClk  = 13;
constexpr int kSpiMosi = 14;
constexpr int kSpiMiso = 36;
constexpr int kSpiCs   = 26;
constexpr int kGpioPin = 26;

struct GpioWatchState {
    m5hal::types::gpio_number_t pin = -1;
    size_t rising                   = 0;
    size_t falling                  = 0;
    bool last_level                 = false;
};

void onGpioWatch(void* raw, m5hal::types::gpio_number_t pin, bool level, m5hal::gpio::GPIOGroup::Edge edge)
{
    auto* state = static_cast<GpioWatchState*>(raw);
    if (state == nullptr || pin != state->pin) {
        return;
    }
    if (edge == m5hal::gpio::GPIOGroup::Edge::Rising) {
        ++state->rising;
    } else {
        ++state->falling;
    }
    state->last_level = level;
}

bool pumpFor(m5hal::Hal& remote, uint32_t duration_ms)
{
    const uint32_t start = m5hal::runtime::millis();
    while (m5hal::runtime::millis() - start < duration_ms) {
        m5hal::remote::PumpConfig cfg;
        cfg.keepalive = true;
        if (!remote.pumpRemote(cfg).has_value()) {
            return false;
        }
        m5hal::runtime::delayMs(1);
    }
    return true;
}

bool runI2c(m5hal::Hal& remote)
{
    constexpr uint8_t kAddress = 0x42;
    constexpr uint8_t kReg     = 0x80;
    constexpr std::array<uint8_t, 3> kValues{{0xA5, 0xB6, 0xC7}};
    bool have_previous = false;
    uint8_t previous   = 0;

    for (size_t cycle = 0; cycle < kValues.size(); ++cycle) {
        auto cfg = remote.I2C.createBusConfig(m5hal::i2c::Scl{kI2cScl}, m5hal::i2c::Sda{kI2cSda});
        auto bus = remote.I2C.acquire(cfg);
        if (!bus.has_value()) {
            ::fprintf(stderr, "FAIL mode=i2c cycle=%zu step=acquire error=%d\n", cycle + 1,
                      static_cast<int>(bus.error()));
            return false;
        }

        bool transfer_ok = true;
        {
            m5hal::i2c::MasterAccessConfig access_cfg;
            access_cfg.i2c_addr = kAddress;
            access_cfg.freq     = 100000;
            m5hal::i2c::MasterAccessor target{bus.value(), access_cfg};

            if (have_previous) {
                auto before = target.readRegister(kReg);
                transfer_ok = before.has_value() && before.value() == previous;
            }
            if (transfer_ok) {
                auto written = target.writeRegister(kReg, kValues[cycle]);
                auto after   = target.readRegister(kReg);
                transfer_ok =
                    written.has_value() && written.value() == 1 && after.has_value() && after.value() == kValues[cycle];
            }
        }
        if (!transfer_ok) {
            ::fprintf(stderr, "FAIL mode=i2c cycle=%zu step=transfer\n", cycle + 1);
            return false;
        }

        auto closed = remote.I2C.close(bus.value());
        if (!closed.has_value()) {
            ::fprintf(stderr, "FAIL mode=i2c cycle=%zu step=close error=%d\n", cycle + 1,
                      static_cast<int>(closed.error()));
            return false;
        }
        have_previous = true;
        previous      = kValues[cycle];
        ::printf("CYCLE mode=i2c index=%zu value=0x%02X released=1\n", cycle + 1, previous);
    }
    ::printf("PASS mode=i2c cycles=%zu state_preserved=1\n", kValues.size());
    return true;
}

bool runSpi(m5hal::Hal& remote)
{
    constexpr size_t kLength = 32;
    constexpr size_t kCycles = 3;

    for (size_t cycle = 0; cycle < kCycles; ++cycle) {
        auto cfg = remote.SPI.createBusConfig(m5hal::spi::Clk{kSpiClk}, m5hal::spi::Mosi{kSpiMosi},
                                              m5hal::spi::Miso{kSpiMiso});
        auto bus = remote.SPI.acquire(cfg);
        if (!bus.has_value()) {
            ::fprintf(stderr, "FAIL mode=spi cycle=%zu step=acquire error=%d\n", cycle + 1,
                      static_cast<int>(bus.error()));
            return false;
        }

        bool transfer_ok   = false;
        int transfer_error = 0;
        {
            m5hal::spi::MasterAccessConfig access_cfg;
            access_cfg.pin_cs        = kSpiCs;
            access_cfg.freq          = 1000000;
            access_cfg.spi_mode      = 1;
            access_cfg.spi_data_mode = m5hal::spi::spi_data_mode_t::FullDuplex;
            m5hal::spi::MasterAccessor target{bus.value(), access_cfg};

            std::array<uint8_t, kLength> tx{};
            std::array<uint8_t, kLength> rx{};
            for (size_t i = 0; i < tx.size(); ++i) {
                tx[i] = static_cast<uint8_t>(0x30u + cycle * 0x20u + i);
            }
            m5hal::spi::TransferDesc desc;
            auto begun = target.beginAccess();
            if (!begun.has_value()) {
                transfer_error = static_cast<int>(begun.error());
            } else {
                auto transferred = target.transfer(desc, m5hal::data::ConstDataSpan{tx.data(), tx.size()},
                                                   m5hal::data::DataSpan{rx.data(), rx.size()});
                auto ended       = target.endAccess();
                transfer_ok      = transferred.has_value() && ended.has_value() && transferred->tx == kLength &&
                              transferred->rx == kLength;
                if (!transferred.has_value()) {
                    transfer_error = static_cast<int>(transferred.error());
                } else if (!ended.has_value()) {
                    transfer_error = static_cast<int>(ended.error());
                }
            }
        }
        if (!transfer_ok) {
            ::fprintf(stderr, "FAIL mode=spi cycle=%zu step=transfer error=%d\n", cycle + 1, transfer_error);
            return false;
        }

        auto closed = remote.SPI.close(bus.value());
        if (!closed.has_value()) {
            ::fprintf(stderr, "FAIL mode=spi cycle=%zu step=close error=%d\n", cycle + 1,
                      static_cast<int>(closed.error()));
            return false;
        }
        ::printf("CYCLE mode=spi index=%zu bytes=%zu released=1\n", cycle + 1, kLength);
    }
    ::printf("PASS mode=spi cycles=%zu bytes=%zu\n", kCycles, kCycles * kLength);
    return true;
}

bool runGpio(m5hal::Hal& remote)
{
    if (!remote.hasRemoteGpio()) {
        ::fprintf(stderr, "FAIL mode=gpio step=capabilities gpio=0\n");
        return false;
    }

    const auto pin_number = m5hal::types::makeGpioNumber(remote.remoteGpioSlot(), kGpioPin);
    auto pin_result       = remote.Gpio.tryGetPin(pin_number);
    if (!pin_result.has_value()) {
        ::fprintf(stderr, "FAIL mode=gpio step=get_pin error=%d\n", static_cast<int>(pin_result.error()));
        return false;
    }
    auto pin = pin_result.value();

    GpioWatchState state{pin_number};
    auto sink = remote.Gpio.setWatchSink(&onGpioWatch, &state, 1000);
    if (!sink.has_value()) {
        ::fprintf(stderr, "FAIL mode=gpio step=set_sink error=%d\n", static_cast<int>(sink.error()));
        return false;
    }
    auto watched = remote.Gpio.watch(pin_number);
    if (!watched.has_value()) {
        (void)remote.Gpio.setWatchSink(nullptr, nullptr);
        ::fprintf(stderr, "FAIL mode=gpio step=watch error=%d\n", static_cast<int>(watched.error()));
        return false;
    }

    pin.setMode(m5hal::types::gpio_mode_t::Output);
    pin.write(false);
    const bool low1 = pumpFor(remote, 50) && !pin.read();
    pin.write(true);
    const bool high = pumpFor(remote, 50) && pin.read();
    pin.write(false);
    const bool low2 = pumpFor(remote, 50) && !pin.read();

    (void)remote.Gpio.unwatch(pin_number);
    (void)remote.Gpio.setWatchSink(nullptr, nullptr);

    if (!low1 || !high || !low2 || state.rising == 0 || state.falling == 0 || state.last_level) {
        ::fprintf(stderr, "FAIL mode=gpio step=observe low1=%d high=%d low2=%d rising=%zu falling=%zu last=%d\n",
                  low1 ? 1 : 0, high ? 1 : 0, low2 ? 1 : 0, state.rising, state.falling, state.last_level ? 1 : 0);
        return false;
    }
    ::printf("PASS mode=gpio pin=%d sequence=0-1-0 rising=%zu falling=%zu\n", kGpioPin, state.rising, state.falling);
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3 ||
        (::strcmp(argv[2], "i2c") != 0 && ::strcmp(argv[2], "spi") != 0 && ::strcmp(argv[2], "gpio") != 0)) {
        ::fprintf(stderr, "usage: %s uart:/dev/cu.usbserial-... i2c|spi|gpio\n", argv[0]);
        return 2;
    }

    m5hal::Hal remote;
    m5hal::remote::DeviceConfig connection_cfg;
    connection_cfg.baud_rate           = 115200;
    connection_cfg.response_timeout_ms = 5000;
    auto connected                     = remote.connect(argv[1], connection_cfg);
    if (!connected.has_value()) {
        ::fprintf(stderr, "FAIL mode=%s step=connect error=%d\n", argv[2], static_cast<int>(connected.error()));
        return 1;
    }

    const auto* caps = remote.capabilities();
    if (caps == nullptr || (::strcmp(argv[2], "gpio") != 0 && !caps->supports_bus_create)) {
        ::fprintf(stderr, "FAIL mode=%s step=capabilities bus_create=0\n", argv[2]);
        return 1;
    }
    const bool passed = ::strcmp(argv[2], "i2c") == 0   ? runI2c(remote)
                        : ::strcmp(argv[2], "spi") == 0 ? runSpi(remote)
                                                        : runGpio(remote);
    return passed ? 0 : 1;
}
