#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ARDUINO)

namespace m5::variants::frameworks::arduino::hal::v1::uart {

namespace {

m5::stl::expected<uint32_t, ::m5::hal::v1::error::error_t> serialConfig(
    const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    if (cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    switch (cfg.parity) {
        case ::m5::hal::v1::uart::parity_t::none:
            return cfg.stop_bits == 1 ? SERIAL_8N1 : SERIAL_8N2;
        case ::m5::hal::v1::uart::parity_t::even:
            return cfg.stop_bits == 1 ? SERIAL_8E1 : SERIAL_8E2;
        case ::m5::hal::v1::uart::parity_t::odd:
            return cfg.stop_bits == 1 ? SERIAL_8O1 : SERIAL_8O2;
        default:
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
}

bool sameConfig(const ::m5::hal::v1::uart::UARTAccessConfig& lhs, const ::m5::hal::v1::uart::UARTAccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

}  // namespace

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::init(const BusConfig& config)
{
    _config = config;
    _serial = config.serial;
    if (_serial == nullptr) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _attached = false;
    _begun    = false;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::release(void)
{
    if (_serial != nullptr && _begun && !_attached) {
        _serial->end();
    }
    _serial   = nullptr;
    _begun    = false;
    _attached = false;
    return {};
}

::m5::hal::v1::error::error_t Bus::attach(::HardwareSerial& serial)
{
    (void)release();
    _serial   = &serial;
    _attached = true;
    // _begun stays false: the first access MUST apply the accessor
    // config (begin + timeout), exactly like init() does. The old
    // `_begun = true` made the first apply a no-op whenever the
    // accessor happened to ask for the default config — whether the
    // attached serial got (re)configured depended on the config VALUE,
    // which is the same class of trap as the i2c `_last_freq` sentinel
    // this mirrors (S16 D10). Callers who configured the serial
    // themselves should pass the same parameters in the accessor
    // config; attach does not try to preserve an unknown foreign setup.
    _begun = false;
    return ::m5::hal::v1::error::error_t::OK;
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::applyConfig(
    const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    if (_serial == nullptr || cfg.baud_rate == 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (_begun && sameConfig(_applied_cfg, cfg)) {
        return {};
    }
    auto native_cfg = serialConfig(cfg);
    if (!native_cfg.has_value()) {
        return m5::stl::make_unexpected(native_cfg.error());
    }
#if defined(ESP_PLATFORM)
    // RX/TX ring buffer sizes must be set before begin() and only take effect on
    // the first begin. Honors UARTBusConfig.{rx,tx}_buffer_size (a value of 0
    // leaves the Arduino default; rx default 256 == the Arduino default). A
    // larger RX ring matters for high-baud bursts (see the uart_echo HIL test).
    if (!_begun) {
        if (_config.rx_buffer_size > 0) {
            _serial->setRxBufferSize(_config.rx_buffer_size);
        }
        if (_config.tx_buffer_size > 0) {
            _serial->setTxBufferSize(_config.tx_buffer_size);
        }
    }
    _serial->begin(cfg.baud_rate, native_cfg.value(), static_cast<int8_t>(_config.pin_rx),
                   static_cast<int8_t>(_config.pin_tx), cfg.invert);
#else
    (void)native_cfg;
    (void)cfg;
    _serial->begin(cfg.baud_rate);
#endif
    _serial->setTimeout(cfg.first_byte_timeout_ms);
    _applied_cfg = cfg;
    _begun       = true;
    return {};
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::write(::m5::hal::v1::bus::Accessor* owner,
                                                                    const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                    ::m5::hal::v1::data::Source* tx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    size_t done = 0;
    while (tx != nullptr && !tx->eof() && done < len) {
        auto span = tx->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        const size_t written = _serial->write(span.value().data, span.value().size);
        if (written == 0) {
            break;
        }
        auto advanced = tx->advance(written);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += written;
    }
    _serial->flush();
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::read(::m5::hal::v1::bus::Accessor* owner,
                                                                   const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                   ::m5::hal::v1::data::Sink* rx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    size_t done             = 0;
    uint32_t last_byte_msec = millis();
    const uint32_t start    = last_byte_msec;
    while (rx != nullptr && !rx->closed() && done < len) {
        if (_serial->available() <= 0) {
            const uint32_t now     = millis();
            const uint32_t timeout = done == 0 ? cfg.first_byte_timeout_ms : cfg.inter_byte_timeout_ms;
            const uint32_t base    = done == 0 ? start : last_byte_msec;
            if (now - base >= timeout) {
                break;
            }
            delay(1);
            continue;
        }
        auto span = rx->reserve(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        size_t count = 0;
        while (count < span.value().size && done + count < len && _serial->available() > 0) {
            const int value = _serial->read();
            if (value < 0) {
                break;
            }
            span.value().data[count++] = static_cast<uint8_t>(value);
            last_byte_msec             = millis();
        }
        auto committed = rx->commit(count);
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        done += count;
    }
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::readableBytes(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return static_cast<size_t>(_serial->available());
}

}  // namespace m5::variants::frameworks::arduino::hal::v1::uart

#endif

#endif
