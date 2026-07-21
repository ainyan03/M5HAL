// SPDX-License-Identifier: MIT

#include "spi.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V0 namespace v0 {
namespace bus {
namespace spi {

m5::stl::expected<SPIBus*, m5::hal::error::error_t> getBus(const SPIBusConfig& config)
{
    // Fixed-slot registry keyed by the full pin set: distinct pin sets get
    // distinct bus instances instead of aliasing one shared static, and a
    // failed init() propagates instead of handing back a half-configured bus.
    static constexpr size_t kMaxBuses = 4;
    static constexpr size_t kPins     = sizeof(config.pins) / sizeof(config.pins[0]);
    static SoftwareSPIBus buses[kMaxBuses];
    static const interface::gpio::Pin* key_pins[kMaxBuses][kPins];
    static bool used[kMaxBuses];

    for (size_t i = 0; i < kMaxBuses; ++i) {
        if (!used[i]) {
            continue;
        }
        // SPIBusConfig carries only the pin set, so a key match means the
        // already-initialized instance is fully equivalent: no re-init.
        bool match = true;
        for (size_t p = 0; p < kPins; ++p) {
            if (key_pins[i][p] != config.pins[p]) {
                match = false;
                break;
            }
        }
        if (match) {
            return &buses[i];
        }
    }
    for (size_t i = 0; i < kMaxBuses; ++i) {
        if (!used[i]) {
            const auto err = buses[i].init(config);
            if (err != error::error_t::OK) {
                return m5::stl::make_unexpected(err);
            }
            for (size_t p = 0; p < kPins; ++p) {
                key_pins[i][p] = config.pins[p];
            }
            used[i] = true;
            return &buses[i];
        }
    }
    return m5::stl::make_unexpected(m5::hal::error::error_t::OUT_OF_RESOURCE);
}

error::error_t SoftwareSPIBus::init(const BusConfig& config)
{
    if (config.getBusType() != types::bus_type_t::SPI) {
        M5_LIB_LOGE("SoftwareSPI::init: error %s", __PRETTY_FUNCTION__);
        return error::error_t::INVALID_ARGUMENT;
    }
    _config = static_cast<const SPIBusConfig&>(config);
    M5_LIB_LOGV("SoftwareSPI::init: ok %s", __PRETTY_FUNCTION__);
    if (_config.pin_clk) {
        _config.pin_clk->setMode(m5::hal::types::gpio_mode_t::Output);
    }
    if (_config.pin_dc) {
        _config.pin_dc->setMode(m5::hal::types::gpio_mode_t::Output);
    }
    if (_config.pin_mosi) {
        _config.pin_mosi->setMode(m5::hal::types::gpio_mode_t::Output);
    }
    if (_config.pin_miso) {
        _config.pin_miso->setMode(m5::hal::types::gpio_mode_t::Input);
    }
    return error::error_t::OK;
}

m5::stl::expected<Accessor*, m5::hal::error::error_t> SoftwareSPIBus::beginAccess(const AccessConfig& access_config)
{
    /// @TODO ここで排他制御＆ロック処理を行うこと。
    if (_Accessor.get() != nullptr) {
        M5_LIB_LOGE("SoftwareSPI::beginAccess: error %s", __PRETTY_FUNCTION__);
        return m5::stl::make_unexpected(m5::hal::error::error_t::INVALID_ARGUMENT);
    }
    if (access_config.getBusType() != getBusType()) {
        M5_LIB_LOGE("SoftwareSPI::beginAccess: error %s", __PRETTY_FUNCTION__);
        return m5::stl::make_unexpected(m5::hal::error::error_t::INVALID_ARGUMENT);
    }
    auto result = new SoftwareSPIMasterAccessor(*this, static_cast<const SPIMasterAccessConfig&>(access_config));
    _Accessor.reset(result);
    M5_LIB_LOGV("SoftwareSPI::beginAccess: ok %s", __PRETTY_FUNCTION__);
    return result;
}

m5::stl::expected<void, m5::hal::error::error_t> SPIMasterAccessor::sendDummyClock(size_t count)
{
    if (count) {
        auto bc   = static_cast<const SPIBusConfig&>(getBusConfig());
        auto ac   = static_cast<const SPIMasterAccessConfig&>(getAccessConfig());
        auto cpol = (ac.spi_mode & 2);
        auto sclk = bc.pin_clk;
        do {
            sclk->write(!cpol);
            sclk->write(cpol);
        } while (--count);
    }
    return {};
}

m5::stl::expected<size_t, m5::hal::error::error_t> SPIMasterAccessor::writeCommand(const uint8_t* data, size_t len)
{
    auto bc     = static_cast<const SPIBusConfig&>(getBusConfig());
    auto pin_dc = bc.pin_dc;
    if (pin_dc) pin_dc->write(false);
    auto res = write(data, len);
    if (pin_dc) pin_dc->write(true);
    return res;
}

m5::stl::expected<size_t, m5::hal::error::error_t> SPIMasterAccessor::writeCommandData(const uint8_t* data, size_t len)
{
    auto bc      = static_cast<const SPIBusConfig&>(getBusConfig());
    auto pin_dc  = bc.pin_dc;
    auto ac      = static_cast<const SPIMasterAccessConfig&>(getAccessConfig());
    auto cmd_len = static_cast<size_t>(ac.spi_command_length >> 3);
    if (cmd_len) {
        if (pin_dc) pin_dc->write(false);
        auto res = write(data, cmd_len);
        data += cmd_len;
        auto datalen = len - cmd_len;
        if (pin_dc) pin_dc->write(true);
        if (!res || datalen == 0) return res;
        res = write(data, datalen);
        if (!res) {
            return res;
        }
        return len;
    }
    return write(data, len);
}

m5::stl::expected<void, m5::hal::error::error_t> SPIMasterAccessor::cs_control(bool cs_level)
{
    auto pin_cs = static_cast<const SPIMasterAccessConfig&>(getAccessConfig()).pin_cs;
    if (pin_cs) pin_cs->write(cs_level);
    return {};
}

m5::stl::expected<size_t, m5::hal::error::error_t> SoftwareSPIMasterAccessor::read(uint8_t* data, size_t len)
{
    (void)data;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::stl::expected<size_t, m5::hal::error::error_t> SoftwareSPIMasterAccessor::write(const uint8_t* data, size_t len)
{
    auto bc = static_cast<const SPIBusConfig&>(getBusConfig());
    auto ac = static_cast<const SPIMasterAccessConfig&>(getAccessConfig());

    bool cpol = (ac.spi_mode & 2);
    bool cpha = (ac.spi_mode & 1);
    bool flip = cpol ^ cpha;
    auto sclk = bc.pin_clk;
    auto mosi = bc.pin_mosi;

    size_t result = 0;
    do {
        uint_fast8_t mask = 0x80;
        uint_fast8_t d    = *data++;
        do {
            mosi->write(d & mask);
            sclk->write(flip);
            sclk->write(!flip);
        } while (mask >>= 1);
    } while (++result < len);
    sclk->write(cpol);

    return result;
}

}  // namespace spi
}  // namespace bus
}  // namespace v0
}  // namespace hal
}  // namespace m5
