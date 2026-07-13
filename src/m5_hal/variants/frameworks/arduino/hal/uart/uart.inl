// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ARDUINO)

#include "../../../../../hal/v2/diag.hpp"

namespace m5::hal::v2::uart {

namespace {
namespace impl_arduino {

#if !defined(ESP_PLATFORM) && !defined(ARDUINO_ARCH_ESP8266) && !defined(SERIAL_8N1)
#error "This Arduino core does not define the SERIAL_* uart config macros; extend serialConfig() for it."
#endif

result_t<uint32_t> serialConfig(const uart::AccessConfig& cfg)
{
    if (cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP8266)
    // arduino-esp32 and the ESP8266 core define the SERIAL_* configs as
    // SerialConfig enum values (not macros), with every 8-bit combination
    // available.
    switch (cfg.parity) {
        case uart::parity_t::None:
            return cfg.stop_bits == 1 ? SERIAL_8N1 : SERIAL_8N2;
        case uart::parity_t::Even:
            return cfg.stop_bits == 1 ? SERIAL_8E1 : SERIAL_8E2;
        case uart::parity_t::Odd:
            return cfg.stop_bits == 1 ? SERIAL_8O1 : SERIAL_8O2;
        default:
            break;
    }
#else
    // The non-ESP allowlisted cores define their SERIAL_* configs as macros,
    // and the set a core defines tracks the UART peripheral's real capability
    // (e.g. the Adafruit nRF52 core omits SERIAL_8O1/8O2 entirely — nRF52
    // UARTE has no odd-parity support). Guard each constant so a missing
    // combination rejects at runtime with INVALID_ARGUMENT (the spec's code
    // for 未対応値, uart.md) instead of failing the whole variant at compile
    // time. The #error above trips if a future core ships no SERIAL_8N1
    // macro at all (either a different constant style — like arduino-esp32's
    // enum — or none; both need explicit handling here, not silent
    // all-reject).
    switch (cfg.parity) {
        case uart::parity_t::None:
            if (cfg.stop_bits == 1) {
                return SERIAL_8N1;
            }
#if defined(SERIAL_8N2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8N2;
            }
#endif
            break;
        case uart::parity_t::Even:
#if defined(SERIAL_8E1)
            if (cfg.stop_bits == 1) {
                return SERIAL_8E1;
            }
#endif
#if defined(SERIAL_8E2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8E2;
            }
#endif
            break;
        case uart::parity_t::Odd:
#if defined(SERIAL_8O1)
            if (cfg.stop_bits == 1) {
                return SERIAL_8O1;
            }
#endif
#if defined(SERIAL_8O2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8O2;
            }
#endif
            break;
        default:
            break;
    }
#endif
    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
}

bool sameConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

// RAII unlock for a runtime::Mutex critical section (mirrors the pattern in
// service.inl's ControlUnlock) so an early return can never leak the lock.
struct MutexUnlock {
    runtime::Mutex* m;
    ~MutexUnlock()
    {
        m->unlock();
    }
};

}  // namespace impl_arduino
}  // namespace

result_t<void> Bus_arduino::init(const BusConfig_arduino& config)
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};

    _config    = config;
    _serial    = config.serial;
    _hw_serial = config._hw_serial;
    if (_serial == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _attached = false;
    _begun    = false;
    return {};
}

result_t<void> Bus_arduino::release(void)
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};

    if (_serial != nullptr && _begun && !_attached && _hw_serial) {
        static_cast<::HardwareSerial*>(_serial)->end();
    }
    _serial   = nullptr;
    _begun    = false;
    _attached = false;
    return {};
}

uint32_t Bus_arduino::reconfigSkips()
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return 0;
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};
    return _reconfig_skips;
}

error::error_t Bus_arduino::attach(::HardwareSerial& serial)
{
    (void)release();  // release() takes its own _state_mutex critical section
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return error::error_t::TIMEOUT_ERROR;
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};
    _serial    = &serial;
    _hw_serial = true;
    _attached  = true;
    _begun     = false;
    return error::error_t::OK;
}

error::error_t Bus_arduino::attach(::Stream& stream)
{
    (void)release();
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return error::error_t::TIMEOUT_ERROR;
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};
    _serial    = &stream;
    _hw_serial = false;
    _attached  = true;
    _begun     = false;
    return error::error_t::OK;
}

result_t<void> Bus_arduino::applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg)
{
    if (_serial == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_arduino::MutexUnlock state_unlock{&_state_mutex};

    if (!_begun) {
        // First apply: no other owner can be mid-transfer yet
        // (spec/design/uart.md), so no quiescence gate is needed.
        return applyConfigLocked(cfg);
    }
    if (impl_arduino::sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    if (owner == nullptr) {
        // A reconfigure without an accessor identity cannot prove quiescence.
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig skipped: no accessor identity (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};
    }

    // Reconfigure: apply only when the opposite channel is quiescent for
    // `owner` (spec/design/uart.md; IBus::tryAcquireOppositeChannel is the
    // sanctioned exception to the channel-lock -> state-mutex ordering).
    // Applies uniformly regardless of `_hw_serial` (see applyConfigLocked).
    auto& ibus = static_cast<IBus&>(owner->getBus());
    auto grant = ibus.tryAcquireOppositeChannel(owner, entered);
    if (!grant.granted) {
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig skipped: opposite channel busy (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};  // keep serving the currently applied config
    }
    auto applied = applyConfigLocked(cfg);
    ibus.releaseOppositeChannel(owner, grant);
    return applied;
}

result_t<void> Bus_arduino::applyConfigLocked(const uart::AccessConfig& cfg)
{
    if (_hw_serial) {
        if (cfg.baud_rate == 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        auto native_cfg = impl_arduino::serialConfig(cfg);
        if (!native_cfg.has_value()) {
            return m5::stl::make_unexpected(native_cfg.error());
        }
        auto* hw = static_cast<::HardwareSerial*>(_serial);
#if defined(ESP_PLATFORM)
        if (!_begun) {
            if (_config.rx_buffer_size > 0) {
                hw->setRxBufferSize(_config.rx_buffer_size);
            }
            if (_config.tx_buffer_size > 0) {
                hw->setTxBufferSize(_config.tx_buffer_size);
            }
        }
        hw->begin(cfg.baud_rate, native_cfg.value(), static_cast<int>(_config.pin_rx), static_cast<int>(_config.pin_tx),
                  cfg.invert);
#elif defined(ARDUINO_ARCH_ESP8266)
        // The ESP8266 core has an RX buffer setter (TX is FIFO+blocking
        // there, so tx_buffer_size has nothing to apply to) and its begin()
        // exposes signal inversion; mode and TX pin keep the overload's own
        // defaults (SERIAL_FULL, GPIO1 — pin selection is not honored on
        // this core, see variants.md).
        if (!_begun && _config.rx_buffer_size > 0) {
            hw->setRxBufferSize(_config.rx_buffer_size);
        }
        hw->begin(cfg.baud_rate, static_cast<decltype(SERIAL_8N1)>(native_cfg.value()), SERIAL_FULL, 1, cfg.invert);
        if (!*hw) {
            // Before begin(), setRxBufferSize() only records the size; the
            // core's uart_init() does the actual RX buffer malloc inside
            // begin() and leaves the port unusable on failure (operator
            // bool() == false). Surface that as the bounded-resource error
            // (uart.md) instead of recording a successful apply.
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
#else
        // Every other allowlisted core provides a begin(baud, config)
        // overload (uint8_t on STM32, uint16_t on the ArduinoCore-API
        // cores) — decltype(SERIAL_8N1) matches each core's constant type,
        // so the validated config is actually applied instead of silently
        // staying at the core-default 8N1. None of these cores can invert
        // the signal; reject instead of silently ignoring (the uart.md
        // contract for unsupported values).
        if (cfg.invert) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        hw->begin(cfg.baud_rate, static_cast<decltype(SERIAL_8N1)>(native_cfg.value()));
#endif
    }
    _serial->setTimeout(cfg.first_byte_timeout_ms);
    _applied_cfg = cfg;
    _begun       = true;
    return {};
}

result_t<size_t> Bus_arduino::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return _serial->write(data, len);
}

result_t<size_t> Bus_arduino::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    const uint32_t start = millis();
    while (_serial->available() <= 0) {
        if (millis() - start >= timeout_ms) {
            return static_cast<size_t>(0);
        }
        delay(1);
    }
    size_t count = 0;
    while (count < len && _serial->available() > 0) {
        const int value = _serial->read();
        if (value < 0) {
            break;
        }
        buf[count++] = static_cast<uint8_t>(value);
    }
    return count;
}

result_t<size_t> Bus_arduino::rawReadableBytes()
{
    return static_cast<size_t>(_serial->available());
}

result_t<size_t> Bus_arduino::write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src, size_t len)
{
    auto applied = applyConfig(owner, Channel::Tx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    auto result = Bus_streaming::write(owner, cfg, src, len);
    _serial->flush();
    return result;
}

result_t<size_t> Bus_arduino::read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len)
{
    auto applied = applyConfig(owner, Channel::Rx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::read(owner, cfg, dst, len);
}

result_t<size_t> Bus_arduino::readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg)
{
    auto applied = applyConfig(owner, Channel::Rx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::readableBytes(owner, cfg);
}

}  // namespace m5::hal::v2::uart

#endif

#endif
