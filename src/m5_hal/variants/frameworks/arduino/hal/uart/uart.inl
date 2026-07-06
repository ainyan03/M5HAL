// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ARDUINO)

namespace m5::hal::v2::uart {

namespace {
namespace impl_arduino {

result_t<uint32_t> serialConfig(const uart::AccessConfig& cfg)
{
    if (cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    switch (cfg.parity) {
        case uart::parity_t::None:
            return cfg.stop_bits == 1 ? SERIAL_8N1 : SERIAL_8N2;
        case uart::parity_t::Even:
            return cfg.stop_bits == 1 ? SERIAL_8E1 : SERIAL_8E2;
        case uart::parity_t::Odd:
            return cfg.stop_bits == 1 ? SERIAL_8O1 : SERIAL_8O2;
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
}

bool sameConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

}  // namespace impl_arduino
}  // namespace

result_t<void> Bus_arduino::init(const BusConfig_arduino& config)
{
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
    if (_serial != nullptr && _begun && !_attached && _hw_serial) {
        static_cast<::HardwareSerial*>(_serial)->end();
    }
    _serial   = nullptr;
    _begun    = false;
    _attached = false;
    return {};
}

error::error_t Bus_arduino::attach(::HardwareSerial& serial)
{
    (void)release();
    _serial    = &serial;
    _hw_serial = true;
    _attached  = true;
    _begun     = false;
    return error::error_t::OK;
}

error::error_t Bus_arduino::attach(::Stream& stream)
{
    (void)release();
    _serial    = &stream;
    _hw_serial = false;
    _attached  = true;
    _begun     = false;
    return error::error_t::OK;
}

result_t<void> Bus_arduino::applyConfig(const uart::AccessConfig& cfg)
{
    if (_serial == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_begun && impl_arduino::sameConfig(_applied_cfg, cfg)) {
        return {};
    }
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
#else
        hw->begin(cfg.baud_rate);
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
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    auto result = Bus_streaming::write(owner, cfg, src, len);
    _serial->flush();
    return result;
}

result_t<size_t> Bus_arduino::read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len)
{
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::read(owner, cfg, dst, len);
}

result_t<size_t> Bus_arduino::readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg)
{
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::readableBytes(owner, cfg);
}

}  // namespace m5::hal::v2::uart

#endif

#endif
