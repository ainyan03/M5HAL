// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_BUS_POOL_INL_
#define M5_HAL_REMOTE_SERVER_BUS_POOL_INL_

#include "server_bus_pool.hpp"

#include "../diag.hpp"

#include <cstring>
#include <new>

#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#endif
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#endif

// Arduino only counts as an active bus provider when it is
// arduino-esp32: this remote server implementation assumes ESP-IDF-family
// SPI/Wire/task APIs underneath the Arduino surface (SPIClass layout,
// pdMS_TO_TICKS/vTaskDelay/esp_restart). Other Arduino cores (RP2040 /
// SAMD51, see _checker.hpp's variant allowlist) fall through to the
// "!HAS_ACTIVE_PROVIDER_" paths below instead.
#if ((defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO) && defined(ESP_PLATFORM)) || \
    (defined(M5HAL_FRAMEWORK_HAS_ESPIDF) && M5HAL_FRAMEWORK_HAS_ESPIDF) ||                              \
    (defined(M5HAL_FRAMEWORK_HAS_POSIX) && M5HAL_FRAMEWORK_HAS_POSIX)
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_PROVIDER_ 1
#else
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_PROVIDER_ 0
#endif

#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO && defined(ESP_PLATFORM)
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_ 1
#else
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_ 0
#endif

#if defined(M5HAL_FRAMEWORK_HAS_ESPIDF) && M5HAL_FRAMEWORK_HAS_ESPIDF
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ 1
#else
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ 0
#endif

namespace m5::hal::v2::remote {

int16_t detail::readI16LE(const uint8_t* p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

void detail::writeI16LE(uint8_t* p, int16_t value)
{
    const auto v = static_cast<uint16_t>(value);
    p[0]         = static_cast<uint8_t>(v & 0xFFu);
    p[1]         = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

namespace detail {

result_t<void> validatePinConfig(data::ConstDataSpan pin_config, size_t min_len)
{
    if (pin_config.size < min_len || pin_config.size > kServerPinConfigMax || pin_config.data == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    return {};
}

template <typename Slot>
Slot* findFreeBinding(Slot* slots)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (!slots[i].used) {
            return &slots[i];
        }
    }
    return nullptr;
}

bool sameConfigBytes(const uint8_t* cfg, uint8_t cfg_len, data::ConstDataSpan pin_config)
{
    if (cfg_len != pin_config.size) {
        return false;
    }
    if (cfg_len == 0) {
        return true;
    }
    return ::memcmp(cfg, pin_config.data, cfg_len) == 0;
}

template <typename PhysSlot>
void storeConfig(PhysSlot& slot, data::ConstDataSpan pin_config)
{
    if (pin_config.size > 0) {
        ::memcpy(slot.cfg, pin_config.data, pin_config.size);
    }
    slot.cfg_len = static_cast<uint8_t>(pin_config.size);
}

void appendClaimedPin(types::gpio_number_t* pins, size_t& count, types::gpio_number_t pin)
{
    if (pin < 0) {
        return;
    }
    pins[count++] = pin;
}

void notifyPinsClaimed(pins_claimed_fn_t fn, void* ctx, const types::gpio_number_t* pins, size_t count)
{
    if (fn != nullptr && count != 0) {
        fn(ctx, pins, count);
    }
}

bool sameI2CPins(data::ConstDataSpan cfg, types::gpio_number_t scl, types::gpio_number_t sda)
{
    return cfg.size >= 4 && static_cast<types::gpio_number_t>(readI16LE(cfg.data)) == scl &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 2)) == sda;
}

bool sameSPIPins(data::ConstDataSpan cfg, types::gpio_number_t clk, types::gpio_number_t mosi,
                 types::gpio_number_t miso)
{
    return cfg.size >= 6 && static_cast<types::gpio_number_t>(readI16LE(cfg.data)) == clk &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 2)) == mosi &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 4)) == miso;
}

bool sameUARTPins(data::ConstDataSpan cfg, types::gpio_number_t tx, types::gpio_number_t rx)
{
    return cfg.size >= 4 && static_cast<types::gpio_number_t>(readI16LE(cfg.data)) == tx &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 2)) == rx;
}

bool sameI2SPins(data::ConstDataSpan cfg, types::gpio_number_t bclk, types::gpio_number_t ws, types::gpio_number_t dout,
                 types::gpio_number_t din)
{
    return cfg.size >= 8 && static_cast<types::gpio_number_t>(readI16LE(cfg.data)) == bclk &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 2)) == ws &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 4)) == dout &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 6)) == din;
}

bool samePDMPins(data::ConstDataSpan cfg, types::gpio_number_t clk, types::gpio_number_t din)
{
    return cfg.size >= 4 && static_cast<types::gpio_number_t>(readI16LE(cfg.data)) == clk &&
           static_cast<types::gpio_number_t>(readI16LE(cfg.data + 2)) == din;
}

#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
bool isArduinoWireInUse(const ServerPhysicalBusPool& phys, ::TwoWire* wire)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used && phys.i2c[i].arduino_wire == wire) {
            return true;
        }
    }
    return false;
}

bool isArduinoSPIInUse(const ServerPhysicalBusPool& phys, ::SPIClass* spi)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (phys.spi[i].used && phys.spi[i].arduino_spi == spi) {
            return true;
        }
    }
    return false;
}

::TwoWire* acquireArduinoWireProvider(const ServerPhysicalBusPool& phys)
{
    if (!isArduinoWireInUse(phys, &Wire)) {
        return &Wire;
    }
#if defined(SOC_I2C_NUM) && SOC_I2C_NUM > 1
    if (!isArduinoWireInUse(phys, &Wire1)) {
        return &Wire1;
    }
#endif
    return nullptr;
}

::SPIClass* secondaryArduinoSPIProvider()
{
#if defined(SOC_SPI_PERIPH_NUM) && SOC_SPI_PERIPH_NUM > 2 && defined(HSPI)
    alignas(::SPIClass) static uint8_t storage[sizeof(::SPIClass)];
    static ::SPIClass* spi = nullptr;
    if (spi == nullptr) {
        spi = new (storage)::SPIClass(HSPI);
    }
    return spi;
#else
    return nullptr;
#endif
}

::SPIClass* acquireArduinoSPIProvider(const ServerPhysicalBusPool& phys)
{
    if (!isArduinoSPIInUse(phys, &SPI)) {
        return &SPI;
    }
    auto* secondary = secondaryArduinoSPIProvider();
    if (secondary != nullptr && !isArduinoSPIInUse(phys, secondary)) {
        return secondary;
    }
    return nullptr;
}
#endif

void releasePhysI2C(ServerPhysicalBusPool& phys, int8_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= kServerBusPoolSlots) {
        return;
    }
    auto& slot = phys.i2c[static_cast<size_t>(index)];
    if (!slot.used || slot.refcount == 0) {
        return;
    }
    --slot.refcount;
    M5HAL_DIAG("phys i2c release idx=%d refcount=%u", static_cast<int>(index), static_cast<unsigned>(slot.refcount));
    if (slot.refcount == 0) {
        if (slot.adopted) {
            return;
        }
        (void)slot.bus.close();
        slot.used    = false;
        slot.adopted = false;
        slot.cfg_len = 0;
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        slot.arduino_wire = nullptr;
#endif
    }
}

void releasePhysSPI(ServerPhysicalBusPool& phys, int8_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= kServerBusPoolSlots) {
        return;
    }
    auto& slot = phys.spi[static_cast<size_t>(index)];
    if (!slot.used || slot.refcount == 0) {
        return;
    }
    --slot.refcount;
    if (slot.refcount == 0) {
        if (slot.adopted) {
            return;
        }
        (void)slot.bus.close();
        slot.used    = false;
        slot.adopted = false;
        slot.cfg_len = 0;
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        slot.arduino_spi = nullptr;
#endif
    }
}

void releasePhysUART(ServerPhysicalBusPool& phys, int8_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= kServerBusPoolSlots) {
        return;
    }
    auto& slot = phys.uart[static_cast<size_t>(index)];
    if (!slot.used || slot.refcount == 0) {
        return;
    }
    --slot.refcount;
    if (slot.refcount == 0) {
        (void)slot.bus.close();
        slot.used    = false;
        slot.cfg_len = 0;
    }
}

void releasePhysI2S(ServerPhysicalBusPool& phys, int8_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= kServerBusPoolSlots) {
        return;
    }
    auto& slot = phys.i2s[static_cast<size_t>(index)];
    if (!slot.used || slot.refcount == 0) {
        return;
    }
    --slot.refcount;
    if (slot.refcount == 0) {
        (void)slot.bus.close();
        slot.used    = false;
        slot.cfg_len = 0;
    }
}

void releasePhysPDM(ServerPhysicalBusPool& phys, int8_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= kServerBusPoolSlots) {
        return;
    }
    auto& slot = phys.pdm[static_cast<size_t>(index)];
    if (!slot.used || slot.refcount == 0) {
        return;
    }
    if (--slot.refcount == 0) {
        (void)slot.bus.close();
        slot.used    = false;
        slot.cfg_len = 0;
    }
}

result_t<int8_t> acquirePhysI2C(ServerPhysicalBusPool& phys, data::ConstDataSpan pin_config, types::gpio_number_t scl,
                                types::gpio_number_t sda)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.i2c[i];
        if (!slot.used || !sameI2CPins({slot.cfg, slot.cfg_len}, scl, sda)) {
            continue;
        }
        if (!sameConfigBytes(slot.cfg, slot.cfg_len, pin_config)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (slot.refcount == 0xFFu) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        M5HAL_DIAG("phys i2c reuse idx=%d refcount=%u", static_cast<int>(i), static_cast<unsigned>(slot.refcount + 1));
        ++slot.refcount;
        return static_cast<int8_t>(i);
    }

    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.i2c[i];
        if (slot.used) {
            continue;
        }
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        auto* wire = acquireArduinoWireProvider(phys);
        if (wire == nullptr) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        i2c::BusConfig cfg{i2c::Scl{scl}, i2c::Sda{sda}};
#else
        i2c::BusConfig cfg{i2c::Scl{scl}, i2c::Sda{sda}};
#endif

#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        std::unique_ptr<i2c::IBus> backend{new (std::nothrow) i2c::Bus_arduino()};
        if (!backend) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        auto r = static_cast<i2c::Bus_arduino*>(backend.get())->init(cfg, native::borrowed(*wire));
        if (r.has_value()) {
            r = slot.bus.adoptPortableBackend(std::move(backend), cfg);
        }
#else
        auto r = slot.bus.init(cfg);
#endif
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        storeConfig(slot, pin_config);
        slot.adopted  = false;
        slot.refcount = 1;
        slot.used     = true;
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        slot.arduino_wire = wire;
#endif
        M5HAL_DIAG("phys i2c create idx=%d scl=%d sda=%d", static_cast<int>(i), static_cast<int>(scl),
                   static_cast<int>(sda));
        return static_cast<int8_t>(i);
    }
    return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
}

result_t<int8_t> acquirePhysSPI(ServerPhysicalBusPool& phys, data::ConstDataSpan pin_config, types::gpio_number_t clk,
                                types::gpio_number_t mosi, types::gpio_number_t miso)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.spi[i];
        if (!slot.used || !sameSPIPins({slot.cfg, slot.cfg_len}, clk, mosi, miso)) {
            continue;
        }
        if (!sameConfigBytes(slot.cfg, slot.cfg_len, pin_config)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (slot.refcount == 0xFFu) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        ++slot.refcount;
        return static_cast<int8_t>(i);
    }

    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.spi[i];
        if (slot.used) {
            continue;
        }
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        auto* spi = acquireArduinoSPIProvider(phys);
        if (spi == nullptr) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        spi::BusConfig cfg{spi::Clk{clk}, spi::Mosi{mosi}, spi::Miso{miso}};
#else
        spi::BusConfig cfg{spi::Clk{clk}, spi::Mosi{mosi}, spi::Miso{miso}};
#endif
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        std::unique_ptr<spi::IBus> backend{new (std::nothrow) spi::Bus_arduino()};
        if (!backend) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        auto r = static_cast<spi::Bus_arduino*>(backend.get())->init(cfg, native::borrowed(*spi));
        if (r.has_value()) {
            r = slot.bus.adoptPortableBackend(std::move(backend), cfg);
        }
#else
        auto r = slot.bus.init(cfg);
#endif
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        storeConfig(slot, pin_config);
        slot.adopted  = false;
        slot.refcount = 1;
        slot.used     = true;
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
        slot.arduino_spi = spi;
#endif
        return static_cast<int8_t>(i);
    }
    return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
}

#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_
result_t<int8_t> acquirePhysUART(ServerPhysicalBusPool& phys, data::ConstDataSpan pin_config, types::gpio_number_t tx,
                                 types::gpio_number_t rx, uint8_t port, uint16_t rxsz, uint16_t txsz)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.uart[i];
        if (!slot.used || !sameUARTPins({slot.cfg, slot.cfg_len}, tx, rx)) {
            continue;
        }
        if (!sameConfigBytes(slot.cfg, slot.cfg_len, pin_config)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (slot.refcount == 0xFFu) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        ++slot.refcount;
        return static_cast<int8_t>(i);
    }

    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.uart[i];
        if (slot.used) {
            continue;
        }
        uart::BusConfig cfg{uart::Tx{tx}, uart::Rx{rx}};
        cfg.rx_buffer_size = rxsz > 0 ? rxsz : 2048;
        cfg.tx_buffer_size = txsz > 0 ? txsz : 512;
        // The wire cannot express "any port" explicitly; hosts always send 0.
        // Port 0 usually carries the device console, so treat 0 as auto and
        // probe the spare controllers first. An explicit nonzero port is
        // honored as-is.
        const uint8_t candidates_auto[]  = {1, 2, 0};
        const uint8_t candidates_fixed[] = {port};
        const uint8_t* candidates        = (port == 0) ? candidates_auto : candidates_fixed;
        const size_t candidate_count     = (port == 0) ? sizeof(candidates_auto) : sizeof(candidates_fixed);
        result_t<void> r                 = m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        for (size_t c = 0; c < candidate_count; ++c) {
            std::unique_ptr<uart::IBus> backend{new (std::nothrow) uart::Bus_espidf()};
            if (!backend) {
                return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
            }
            r = static_cast<uart::Bus_espidf*>(backend.get())
                    ->init(cfg, native::managed(uart::NativePort{static_cast<::uart_port_t>(candidates[c])}));
            if (r.has_value()) {
                r = slot.bus.adoptPortableBackend(std::move(backend), cfg);
            }
            if (r.has_value()) {
                break;
            }
        }
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        storeConfig(slot, pin_config);
        slot.refcount = 1;
        slot.used     = true;
        return static_cast<int8_t>(i);
    }
    return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
}
#endif

// Needs the std I2S driver (IDF5) — mirror createI2S's guard, not just the
// espidf one, or arduino-core-2.x builds hit ESP-IDF provider unavailable.
#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ && defined(M5HAL_ESPIDF_I2S_HAS_STD) && M5HAL_ESPIDF_I2S_HAS_STD
result_t<int8_t> acquirePhysI2S(ServerPhysicalBusPool& phys, data::ConstDataSpan pin_config, types::gpio_number_t bclk,
                                types::gpio_number_t ws, types::gpio_number_t dout, types::gpio_number_t din,
                                uint8_t role, uint8_t txkb, uint8_t rxkb)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.i2s[i];
        if (!slot.used || !sameI2SPins({slot.cfg, slot.cfg_len}, bclk, ws, dout, din)) {
            continue;
        }
        if (!sameConfigBytes(slot.cfg, slot.cfg_len, pin_config)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (slot.refcount == 0xFFu) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        ++slot.refcount;
        return static_cast<int8_t>(i);
    }

    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.i2s[i];
        if (slot.used) {
            continue;
        }
        i2s::BusConfig cfg;
        cfg.pin_bclk       = bclk;
        cfg.pin_ws         = ws;
        cfg.pin_dout       = dout;
        cfg.pin_din        = din;
        cfg.role           = role ? i2s::IBusConfig::Role::Slave : i2s::IBusConfig::Role::Master;
        cfg.tx_buffer_size = txkb > 0 ? static_cast<size_t>(txkb) * 1024u : 8192;
        cfg.rx_buffer_size = rxkb > 0 ? static_cast<size_t>(rxkb) * 1024u : 8192;
        auto r             = slot.bus.init(cfg);
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        storeConfig(slot, pin_config);
        slot.refcount = 1;
        slot.used     = true;
        return static_cast<int8_t>(i);
    }
    return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
}
#endif

#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ && defined(M5HAL_ESPIDF_PDM_HAS_RX_PCM) && \
    M5HAL_ESPIDF_PDM_HAS_RX_PCM
result_t<int8_t> acquirePhysPDM(ServerPhysicalBusPool& phys, data::ConstDataSpan pin_config, types::gpio_number_t clk,
                                types::gpio_number_t din, uint8_t rxkb)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.pdm[i];
        if (!slot.used || !samePDMPins({slot.cfg, slot.cfg_len}, clk, din)) {
            continue;
        }
        if (!sameConfigBytes(slot.cfg, slot.cfg_len, pin_config)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (slot.refcount == 0xFFu) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        ++slot.refcount;
        return static_cast<int8_t>(i);
    }
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        auto& slot = phys.pdm[i];
        if (slot.used) {
            continue;
        }
        pdm::BusConfig cfg{pdm::Clk{clk}, pdm::Din{din}};
        cfg.rx_buffer_size = rxkb > 0 ? static_cast<size_t>(rxkb) * 1024u : 8192;
        auto initialized   = slot.bus.init(cfg);
        if (!initialized.has_value()) {
            return m5::stl::make_unexpected(initialized.error());
        }
        storeConfig(slot, pin_config);
        slot.refcount = 1;
        slot.used     = true;
        return static_cast<int8_t>(i);
    }
    return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
}
#endif

}  // namespace detail

#if M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
result_t<void> ServerPhysicalBusPool::adoptI2C(::TwoWire& wire, types::gpio_number_t scl, types::gpio_number_t sda)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (i2c[i].used && detail::sameI2CPins({i2c[i].cfg, i2c[i].cfg_len}, scl, sda)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (i2c[i].used && i2c[i].arduino_wire == &wire) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
    }
    auto* slot = detail::findFreeBinding(i2c);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    std::unique_ptr<i2c::IBus> backend{new (std::nothrow) i2c::Bus_arduino()};
    if (!backend) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    i2c::IBusConfig adopted_config{i2c::Scl{scl}, i2c::Sda{sda}};
    auto initialized = static_cast<i2c::Bus_arduino*>(backend.get())->init(adopted_config, native::borrowed(wire));
    if (!initialized.has_value()) {
        return m5::stl::make_unexpected(initialized.error());
    }
    auto adopted_backend =
        slot->bus.adoptBackend(std::move(backend), i2c::LogicalBusConfig{i2c::Scl{scl}, i2c::Sda{sda}});
    if (!adopted_backend.has_value()) {
        return m5::stl::make_unexpected(adopted_backend.error());
    }

    uint8_t cfg[4];
    detail::writeI16LE(cfg, static_cast<int16_t>(scl));
    detail::writeI16LE(cfg + 2, static_cast<int16_t>(sda));
    detail::storeConfig(*slot, {cfg, sizeof(cfg)});
    slot->adopted      = true;
    slot->refcount     = 0;
    slot->used         = true;
    slot->arduino_wire = &wire;
    M5HAL_DIAG("adopt i2c scl=%d sda=%d", static_cast<int>(scl), static_cast<int>(sda));
    return {};
}

result_t<void> ServerPhysicalBusPool::adoptSPI(::SPIClass& spi, types::gpio_number_t clk, types::gpio_number_t mosi,
                                               types::gpio_number_t miso)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (this->spi[i].used && detail::sameSPIPins({this->spi[i].cfg, this->spi[i].cfg_len}, clk, mosi, miso)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (this->spi[i].used && this->spi[i].arduino_spi == &spi) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
    }
    auto* slot = detail::findFreeBinding(this->spi);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    std::unique_ptr<spi::IBus> backend{new (std::nothrow) spi::Bus_arduino()};
    if (!backend) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    const spi::IBusConfig adopted_config{spi::Clk{clk}, spi::Mosi{mosi}, spi::Miso{miso}};
    auto initialized = static_cast<spi::Bus_arduino*>(backend.get())->init(adopted_config, native::borrowed(spi));
    if (!initialized.has_value()) {
        return m5::stl::make_unexpected(initialized.error());
    }
    auto adopted_backend = slot->bus.adoptBackend(
        std::move(backend), spi::LogicalBusConfig{spi::Clk{clk}, spi::Mosi{mosi}, spi::Miso{miso}});
    if (!adopted_backend.has_value()) {
        return m5::stl::make_unexpected(adopted_backend.error());
    }

    uint8_t cfg[6];
    detail::writeI16LE(cfg, static_cast<int16_t>(clk));
    detail::writeI16LE(cfg + 2, static_cast<int16_t>(mosi));
    detail::writeI16LE(cfg + 4, static_cast<int16_t>(miso));
    detail::storeConfig(*slot, {cfg, sizeof(cfg)});
    slot->adopted     = true;
    slot->refcount    = 0;
    slot->used        = true;
    slot->arduino_spi = &spi;
    return {};
}
#endif

result_t<void> createI2C(I2CSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx)
{
#if !M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_PROVIDER_
    (void)slots;
    (void)phys;
    (void)bus_id;
    (void)pin_config;
    (void)server;
    (void)pins_claimed_fn;
    (void)pins_claimed_ctx;
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
#else
    auto valid = detail::validatePinConfig(pin_config, 4);
    if (!valid.has_value()) {
        return valid;
    }
    I2CSlot* slot = detail::findFreeBinding(slots);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }

    const auto scl = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data));
    const auto sda = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 2));

    auto phys_index = detail::acquirePhysI2C(phys, pin_config, scl, sda);
    if (!phys_index.has_value()) {
        return m5::stl::make_unexpected(phys_index.error());
    }

    slot->acc =
        new (slot->acc_buf) i2c::MasterAccessor{phys.i2c[static_cast<size_t>(phys_index.value())].bus, slot->acc_cfg};
    slot->bus_id = bus_id;
    slot->phys   = phys_index.value();
    slot->used   = true;

    auto reg = server.registerI2C(bus_id, *slot->acc);
    if (!reg.has_value()) {
        slot->acc->~MasterAccessor();
        slot->acc  = nullptr;
        slot->used = false;
        slot->phys = -1;
        detail::releasePhysI2C(phys, phys_index.value());
        return reg;
    }
    types::gpio_number_t claimed[2];
    size_t claimed_count = 0;
    detail::appendClaimedPin(claimed, claimed_count, scl);
    detail::appendClaimedPin(claimed, claimed_count, sda);
    detail::notifyPinsClaimed(pins_claimed_fn, pins_claimed_ctx, claimed, claimed_count);
    return {};
#endif
}

void releaseI2C(I2CSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (slots[i].used && slots[i].bus_id == bus_id) {
            const int8_t phys_index = slots[i].phys;
            runner.unregisterI2C(bus_id);
            if (slots[i].acc != nullptr) {
                slots[i].acc->~MasterAccessor();
                slots[i].acc = nullptr;
            }
            slots[i].phys = -1;
            slots[i].used = false;
            detail::releasePhysI2C(phys, phys_index);
            return;
        }
    }
}

// ---- SPI pool ---------------------------------------------------------------

result_t<void> createSPI(SPISlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx)
{
#if !M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_PROVIDER_
    (void)slots;
    (void)phys;
    (void)bus_id;
    (void)pin_config;
    (void)server;
    (void)pins_claimed_fn;
    (void)pins_claimed_ctx;
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
#else
    auto valid = detail::validatePinConfig(pin_config, 6);
    if (!valid.has_value()) {
        return valid;
    }
    SPISlot* slot = detail::findFreeBinding(slots);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }

    const auto clk  = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data));
    const auto mosi = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 2));
    const auto miso = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 4));

    auto phys_index = detail::acquirePhysSPI(phys, pin_config, clk, mosi, miso);
    if (!phys_index.has_value()) {
        return m5::stl::make_unexpected(phys_index.error());
    }

    slot->acc_cfg.pin_cs = -1;
    slot->acc_cfg.freq   = 1000000;
    slot->acc =
        new (slot->acc_buf) spi::MasterAccessor{phys.spi[static_cast<size_t>(phys_index.value())].bus, slot->acc_cfg};
    slot->bus_id = bus_id;
    slot->phys   = phys_index.value();
    slot->used   = true;

    auto reg = server.registerSPI(bus_id, *slot->acc);
    if (!reg.has_value()) {
        slot->acc->~MasterAccessor();
        slot->acc  = nullptr;
        slot->used = false;
        slot->phys = -1;
        detail::releasePhysSPI(phys, phys_index.value());
        return reg;
    }
    types::gpio_number_t claimed[3];
    size_t claimed_count = 0;
    detail::appendClaimedPin(claimed, claimed_count, clk);
    detail::appendClaimedPin(claimed, claimed_count, mosi);
    detail::appendClaimedPin(claimed, claimed_count, miso);
    detail::notifyPinsClaimed(pins_claimed_fn, pins_claimed_ctx, claimed, claimed_count);
    return {};
#endif
}

void releaseSPI(SPISlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (slots[i].used && slots[i].bus_id == bus_id) {
            const int8_t phys_index = slots[i].phys;
            runner.unregisterSPI(bus_id);
            if (slots[i].acc != nullptr) {
                slots[i].acc->~MasterAccessor();
                slots[i].acc = nullptr;
            }
            slots[i].phys = -1;
            slots[i].used = false;
            detail::releasePhysSPI(phys, phys_index);
            return;
        }
    }
}

// ---- UART pool --------------------------------------------------------------

result_t<void> createUART(UARTSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                          Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx)
{
#if !M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_
    (void)slots;
    (void)phys;
    (void)bus_id;
    (void)pin_config;
    (void)server;
    (void)pins_claimed_fn;
    (void)pins_claimed_ctx;
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
#else
    auto valid = detail::validatePinConfig(pin_config, 7);
    if (!valid.has_value()) {
        return valid;
    }
    UARTSlot* slot = detail::findFreeBinding(slots);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    const auto tx       = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data));
    const auto rx       = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 2));
    const uint8_t port  = pin_config.data[4];
    const uint16_t rxsz = static_cast<uint16_t>(pin_config.data[5]) * 256u;
    const uint16_t txsz = static_cast<uint16_t>(pin_config.data[6]) * 256u;

    auto phys_index = detail::acquirePhysUART(phys, pin_config, tx, rx, port, rxsz, txsz);
    if (!phys_index.has_value()) {
        return m5::stl::make_unexpected(phys_index.error());
    }

    slot->acc_cfg.baud_rate             = 115200;
    slot->acc_cfg.first_byte_timeout_ms = 100;
    slot->acc_cfg.inter_byte_timeout_ms = 5;
    slot->acc_cfg.write_timeout_ms      = 200;
    slot->acc =
        new (slot->acc_buf) uart::Accessor{phys.uart[static_cast<size_t>(phys_index.value())].bus, slot->acc_cfg};
    slot->bus_id = bus_id;
    slot->phys   = phys_index.value();
    slot->used   = true;

    auto reg = server.registerUART(bus_id, *slot->acc);
    if (!reg.has_value()) {
        slot->acc->~Accessor();
        slot->acc  = nullptr;
        slot->used = false;
        slot->phys = -1;
        detail::releasePhysUART(phys, phys_index.value());
        return reg;
    }
    types::gpio_number_t claimed[2];
    size_t claimed_count = 0;
    detail::appendClaimedPin(claimed, claimed_count, tx);
    detail::appendClaimedPin(claimed, claimed_count, rx);
    detail::notifyPinsClaimed(pins_claimed_fn, pins_claimed_ctx, claimed, claimed_count);
    return {};
#endif
}

void releaseUART(UARTSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (slots[i].used && slots[i].bus_id == bus_id) {
            const int8_t phys_index = slots[i].phys;
            runner.unregisterUART(bus_id);
            if (slots[i].acc != nullptr) {
                slots[i].acc->~Accessor();
                slots[i].acc = nullptr;
            }
            slots[i].phys = -1;
            slots[i].used = false;
            detail::releasePhysUART(phys, phys_index);
            return;
        }
    }
}

// ---- I2S pool ---------------------------------------------------------------

result_t<void> createI2S(I2SSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx)
{
    auto valid = detail::validatePinConfig(pin_config, 11);
    if (!valid.has_value()) {
        return valid;
    }
    const auto bclk = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data));
    const auto ws   = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 2));
    const auto dout = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 4));
    const auto din  = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 6));
    if (bclk < 0 || ws < 0 || (dout < 0 && din < 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

#if !M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ || !defined(M5HAL_ESPIDF_I2S_HAS_STD) || \
    !M5HAL_ESPIDF_I2S_HAS_STD
    (void)slots;
    (void)phys;
    (void)bus_id;
    (void)server;
    (void)pins_claimed_fn;
    (void)pins_claimed_ctx;
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
#else
    I2SSlot* slot = detail::findFreeBinding(slots);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }

    const auto role = pin_config.data[8];
    const auto txkb = pin_config.data[9];
    const auto rxkb = pin_config.data[10];

    auto phys_index = detail::acquirePhysI2S(phys, pin_config, bclk, ws, dout, din, role, txkb, rxkb);
    if (!phys_index.has_value()) {
        return m5::stl::make_unexpected(phys_index.error());
    }

    // Fallback stream parameters. The remote I2S proxy sends BusConfigure in the
    // same script immediately before each BusStreamTransfer, so caller
    // AccessConfig values override these defaults for real transfers.
    slot->acc_cfg.sample_rate_hz   = 16000;
    slot->acc_cfg.bits_per_sample  = 16;
    slot->acc_cfg.channels         = 2;
    slot->acc_cfg.write_timeout_ms = 200;
    slot->acc_cfg.read_timeout_ms  = 200;

    auto& bus = phys.i2s[static_cast<size_t>(phys_index.value())].bus;
    if (dout >= 0 && din >= 0) {
        slot->acc = new (slot->acc_buf) i2s::Accessor{bus, slot->acc_cfg};
        auto reg  = server.registerI2S(bus_id, *slot->acc);
        if (!reg.has_value()) {
            slot->acc->~Accessor();
            slot->acc = nullptr;
            detail::releasePhysI2S(phys, phys_index.value());
            return reg;
        }
    } else if (dout >= 0) {
        slot->tx = new (slot->tx_buf) i2s::TxAccessor{bus, slot->acc_cfg};
        auto reg = server.registerI2S(bus_id, *slot->tx);
        if (!reg.has_value()) {
            slot->tx->~TxAccessor();
            slot->tx = nullptr;
            detail::releasePhysI2S(phys, phys_index.value());
            return reg;
        }
    } else if (din >= 0) {
        slot->rx = new (slot->rx_buf) i2s::RxAccessor{bus, slot->acc_cfg};
        auto reg = server.registerI2S(bus_id, *slot->rx);
        if (!reg.has_value()) {
            slot->rx->~RxAccessor();
            slot->rx = nullptr;
            detail::releasePhysI2S(phys, phys_index.value());
            return reg;
        }
    }

    slot->bus_id = bus_id;
    slot->phys   = phys_index.value();
    slot->used   = true;
    types::gpio_number_t claimed[4];
    size_t claimed_count = 0;
    detail::appendClaimedPin(claimed, claimed_count, bclk);
    detail::appendClaimedPin(claimed, claimed_count, ws);
    detail::appendClaimedPin(claimed, claimed_count, dout);
    detail::appendClaimedPin(claimed, claimed_count, din);
    detail::notifyPinsClaimed(pins_claimed_fn, pins_claimed_ctx, claimed, claimed_count);
    return {};
#endif
}

void releaseI2S(I2SSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (slots[i].used && slots[i].bus_id == bus_id) {
            const int8_t phys_index = slots[i].phys;
            runner.unregisterI2S(bus_id);
            if (slots[i].acc) {
                slots[i].acc->~Accessor();
                slots[i].acc = nullptr;
            } else if (slots[i].tx) {
                slots[i].tx->~TxAccessor();
                slots[i].tx = nullptr;
            }
            if (slots[i].rx) {
                slots[i].rx->~RxAccessor();
                slots[i].rx = nullptr;
            }
            slots[i].phys = -1;
            slots[i].used = false;
            detail::releasePhysI2S(phys, phys_index);
            return;
        }
    }
}

result_t<void> createPDM(PDMSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx)
{
    auto valid = detail::validatePinConfig(pin_config, 5);
    if (!valid.has_value()) {
        return valid;
    }
    const auto clk = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data));
    const auto din = static_cast<types::gpio_number_t>(detail::readI16LE(pin_config.data + 2));
    if (clk < 0 || din < 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
#if !M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_ || !defined(M5HAL_ESPIDF_PDM_HAS_RX_PCM) || \
    !M5HAL_ESPIDF_PDM_HAS_RX_PCM
    (void)slots;
    (void)phys;
    (void)bus_id;
    (void)server;
    (void)pins_claimed_fn;
    (void)pins_claimed_ctx;
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
#else
    PDMSlot* slot = detail::findFreeBinding(slots);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    auto phys_index = detail::acquirePhysPDM(phys, pin_config, clk, din, pin_config.data[4]);
    if (!phys_index.has_value()) {
        return m5::stl::make_unexpected(phys_index.error());
    }
    slot->acc_cfg.sample_rate_hz  = 16000;
    slot->acc_cfg.bits_per_sample = 16;
    slot->acc_cfg.channels        = 1;
    slot->acc_cfg.read_timeout_ms = 200;
    auto& bus                     = phys.pdm[static_cast<size_t>(phys_index.value())].bus;
    slot->rx                      = new (slot->rx_buf) pdm::RxAccessor{bus, slot->acc_cfg};
    auto registered               = server.registerPDM(bus_id, *slot->rx);
    if (!registered.has_value()) {
        slot->rx->~RxAccessor();
        slot->rx = nullptr;
        detail::releasePhysPDM(phys, phys_index.value());
        return registered;
    }
    slot->bus_id                         = bus_id;
    slot->phys                           = phys_index.value();
    slot->used                           = true;
    const types::gpio_number_t claimed[] = {clk, din};
    detail::notifyPinsClaimed(pins_claimed_fn, pins_claimed_ctx, claimed, 2);
    return {};
#endif
}

void releasePDM(PDMSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner)
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (slots[i].used && slots[i].bus_id == bus_id) {
            const int8_t phys_index = slots[i].phys;
            runner.unregisterPDM(bus_id);
            slots[i].rx->~RxAccessor();
            slots[i].rx   = nullptr;
            slots[i].phys = -1;
            slots[i].used = false;
            detail::releasePhysPDM(phys, phys_index);
            return;
        }
    }
}

void ServerBusPool::releaseAll()
{
    if (server == nullptr || phys == nullptr) {
        return;
    }
    auto& runner = server->runner();
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (i2c[i].used) {
            releaseI2C(i2c, *phys, i2c[i].bus_id, runner);
        }
    }
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (spi[i].used) {
            releaseSPI(spi, *phys, spi[i].bus_id, runner);
        }
    }
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (uart[i].used) {
            releaseUART(uart, *phys, uart[i].bus_id, runner);
        }
    }
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (i2s[i].used) {
            releaseI2S(i2s, *phys, i2s[i].bus_id, runner);
        }
    }
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (pdm[i].used) {
            releasePDM(pdm, *phys, pdm[i].bus_id, runner);
        }
    }
}

spi::MasterAccessor* ServerBusPool::findSPIAccessor(uint8_t bus_id) const
{
    for (size_t i = 0; i < kServerBusPoolSlots; ++i) {
        if (spi[i].used && spi[i].bus_id == bus_id) {
            return spi[i].acc;
        }
    }
    return nullptr;
}

result_t<void> ServerBusPool::handler(void* ctx, bool create, types::bus_kind_t kind, uint8_t bus_id,
                                      data::ConstDataSpan pin_config)
{
    auto* pool = static_cast<ServerBusPool*>(ctx);
    if (pool == nullptr || pool->server == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    if (pool->phys == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    M5HAL_DIAG("bus %s kind=%u bus_id=%u", create ? "create" : "release", static_cast<unsigned>(kind),
               static_cast<unsigned>(bus_id));
    if (create) {
        switch (kind) {
            case types::bus_kind_t::I2C:
                return createI2C(pool->i2c, *pool->phys, bus_id, pin_config, *pool->server, pool->pins_claimed_fn,
                                 pool->pins_claimed_ctx);
            case types::bus_kind_t::SPI:
                return createSPI(pool->spi, *pool->phys, bus_id, pin_config, *pool->server, pool->pins_claimed_fn,
                                 pool->pins_claimed_ctx);
            case types::bus_kind_t::UART:
                return createUART(pool->uart, *pool->phys, bus_id, pin_config, *pool->server, pool->pins_claimed_fn,
                                  pool->pins_claimed_ctx);
            case types::bus_kind_t::I2S:
                return createI2S(pool->i2s, *pool->phys, bus_id, pin_config, *pool->server, pool->pins_claimed_fn,
                                 pool->pins_claimed_ctx);
            case types::bus_kind_t::PDM:
                return createPDM(pool->pdm, *pool->phys, bus_id, pin_config, *pool->server, pool->pins_claimed_fn,
                                 pool->pins_claimed_ctx);
            default:
                return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
    }
    auto& runner = pool->server->runner();
    switch (kind) {
        case types::bus_kind_t::I2C:
            releaseI2C(pool->i2c, *pool->phys, bus_id, runner);
            return {};
        case types::bus_kind_t::SPI:
            releaseSPI(pool->spi, *pool->phys, bus_id, runner);
            return {};
        case types::bus_kind_t::UART:
            releaseUART(pool->uart, *pool->phys, bus_id, runner);
            return {};
        case types::bus_kind_t::I2S:
            releaseI2S(pool->i2s, *pool->phys, bus_id, runner);
            return {};
        case types::bus_kind_t::PDM:
            releasePDM(pool->pdm, *pool->phys, bus_id, runner);
            return {};
        default:
            return {};
    }
}

}  // namespace m5::hal::v2::remote

#undef M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_PROVIDER_
#undef M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_PROVIDER_
#undef M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_PROVIDER_

#endif  // M5_HAL_REMOTE_SERVER_BUS_POOL_INL_
