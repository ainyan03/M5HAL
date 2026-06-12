// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include "../../../../../hal/v1/bus/bus.hpp"
#include <M5Utility.hpp>

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v1::i2c {

namespace {
namespace impl_arduino {

// Map TwoWire::endTransmission() return code to M5HAL error_t.
// 0 = success, 1 = data too long, 2 = NACK on address, 3 = NACK on data,
// 4 = other error, 5 = timeout (ESP32 extension). Other codes are
// collapsed into the generic bus error.
::m5::hal::v1::error::error_t mapWireEndTransmission(uint8_t code)
{
    switch (code) {
        case 0:
            return ::m5::hal::v1::error::error_t::OK;
        case 2:
        case 3:
            return ::m5::hal::v1::error::error_t::I2C_NO_ACK;
        case 5:
            return ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
        default:
            return ::m5::hal::v1::error::error_t::I2C_BUS_ERROR;
    }
}

}  // namespace impl_arduino
}  // namespace

::m5::hal::v1::error::error_t Bus_arduino::attach(::TwoWire& wire)
{
    if (_wire) {
        (void)release();
    }
    _wire            = &wire;
    _owns_wire       = false;
    _last_freq       = 0;
    _last_timeout_ms = 0xFFFFFFFFu;
    return ::m5::hal::v1::error::error_t::OK;
}

::m5::hal::v1::result_t<void> Bus_arduino::init(const BusConfig_arduino& config)
{
    _config = config;
    if (_wire) {
        (void)release();
    }
    auto* wire = config.wire;
    if (wire == nullptr) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    if (_config.pin_sda >= 0 && _config.pin_scl >= 0) {
        wire->begin(static_cast<int>(_config.pin_sda), static_cast<int>(_config.pin_scl));
    } else {
        wire->begin();
    }

    auto err = attach(*wire);
    if (::m5::hal::v1::error::isError(err)) {
        wire->end();
        return m5::stl::make_unexpected(err);
    }
    _owns_wire = true;
    return {};
}

::m5::hal::v1::result_t<void> Bus_arduino::release(void)
{
    if (_wire && _owns_wire) {
        _wire->end();
    }
    _wire            = nullptr;
    _owns_wire       = false;
    _last_freq       = 0;
    _last_timeout_ms = 0xFFFFFFFFu;
    return {};
}

::m5::hal::v1::result_t<size_t> Bus_arduino::transfer(::m5::hal::v1::bus::IAccessor* owner,
                                                      const ::m5::hal::v1::i2c::MasterAccessConfig& cfg,
                                                      const ::m5::hal::v1::i2c::TransferDesc& desc,
                                                      ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    // `owner` is reserved for future lock / unlock semantics
    // (the M5UU lock migration is currently on hold); ignored here.
    (void)owner;
    if (_wire == nullptr) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    // `Wire` drives 7-bit addressing only: reject a 10-bit request
    // instead of silently truncating the address (which would address
    // the wrong device). Same degradation as the espidf gen4 backend.
    if (cfg.address_is_10bit || cfg.i2c_addr > 0x007Fu) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    // Apply per-accessor parameters lazily: only call setClock / setTimeOut
    // when the requested value differs from what was last issued. This mirrors
    // the _last_freq pattern to avoid redundant Wire API calls.
    if (cfg.freq != _last_freq) {
        _wire->setClock(cfg.freq);
        _last_freq = cfg.freq;
    }
    // arduino-esp32 TwoWire::setTimeOut() takes milliseconds, same unit as
    // cfg.wire_timeout_ms.
    if (cfg.wire_timeout_ms != _last_timeout_ms) {
        _wire->setTimeOut(static_cast<uint32_t>(cfg.wire_timeout_ms));
        _last_timeout_ms = cfg.wire_timeout_ms;
    }

    // Pass `desc.prefix` into the legacy code path (header.data /
    // header.size) as a local `ConstDataSpan`. `desc` lives as a
    // const reference parameter so its lifetime covers this entire
    // function body.
    ::m5::hal::v1::data::ConstDataSpan header{desc.prefix, desc.prefix_len};

    size_t total     = 0;
    bool write_phase = (header.size > 0) || (tx != nullptr && !tx->eof());
    bool read_phase  = (rx != nullptr);

    // Probe contract (see i2c.hpp): an all-empty descriptor sends
    // only address+W and inspects the ACK. We implement this with
    // `beginTransmission` -> `endTransmission(true)` (`Wire` still
    // emits the address+W byte even at zero payload, and
    // `endTransmission`'s return value tells us ACK vs NACK).
    if (!write_phase && !read_phase) {
        _wire->beginTransmission(static_cast<uint8_t>(cfg.i2c_addr));
        auto err = impl_arduino::mapWireEndTransmission(_wire->endTransmission(true));
        if (::m5::hal::v1::error::isError(err)) {
            return m5::stl::make_unexpected(err);
        }
        return size_t{0};
    }

    if (write_phase) {
        _wire->beginTransmission(static_cast<uint8_t>(cfg.i2c_addr));
        if (header.size) {
            size_t w = _wire->write(header.data, header.size);
            if (w != header.size) {
                // Wire only buffers here; a short write means the local TX
                // buffer is full, not a wire fault. Wire has no abort, so the
                // release below transmits the partial buffer (best effort).
                (void)_wire->endTransmission(true);
                return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::BUFFER_OVERFLOW);
            }
            // Prefix bytes are NOT counted: the return value is the data
            // phase only (tx + rx), matching SPI.
        }
        if (tx) {
            // Drain Source via peek/advance loop. SIZE_MAX requests "all
            // remaining bytes"; memory-backed sources return them in one
            // peek, stream-backed sources may chunk.
            while (!tx->eof()) {
                auto peeked = tx->peek(SIZE_MAX);
                if (!peeked.has_value()) {
                    _wire->endTransmission(true);
                    return m5::stl::make_unexpected(peeked.error());
                }
                auto span = peeked.value();
                if (span.size == 0) {
                    break;  // explicit end-of-stream
                }
                size_t w = _wire->write(span.data, span.size);
                if (w != span.size) {
                    // Local TX buffer full (see the header-write comment).
                    (void)_wire->endTransmission(true);
                    return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::BUFFER_OVERFLOW);
                }
                auto adv = tx->advance(w);
                if (!adv.has_value()) {
                    _wire->endTransmission(true);
                    return m5::stl::make_unexpected(adv.error());
                }
                total += w;
            }
        }
        bool send_stop = (rx == nullptr) || !cfg.use_restart;
        auto err       = impl_arduino::mapWireEndTransmission(_wire->endTransmission(send_stop));
        if (::m5::hal::v1::error::isError(err)) {
            return m5::stl::make_unexpected(err);
        }
    }

    if (rx) {
        // Reserve "everything the sink can hold for this transfer". The
        // returned span size is what we request over the wire; len is no
        // longer carried by the signature.
        auto rsv = rx->reserve(SIZE_MAX);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        auto rx_span = rsv.value();
        if (rx_span.size > 0) {
// TwoWire::requestFrom silently truncates requests that exceed its internal
// RX buffer. Detect and reject oversized requests before touching the wire
// so callers get INVALID_ARGUMENT rather than a short / wrong read.
#if defined(I2C_BUFFER_LENGTH)
            constexpr size_t kWireRxBufLen = I2C_BUFFER_LENGTH;
#else
            // Fall back to the Arduino default (Wire.h hard-codes 32 on most
            // cores; 128 is the safe conservative ceiling used when no macro
            // is available).
            constexpr size_t kWireRxBufLen = 128u;
#endif
            if (rx_span.size > kWireRxBufLen) {
                return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
            }
            size_t got = _wire->requestFrom(static_cast<uint8_t>(cfg.i2c_addr), static_cast<size_t>(rx_span.size),
                                            static_cast<size_t>(true));
            if (got != rx_span.size) {
                // Zero bytes back means the address byte was NACKed (no
                // device answered); a partial count is a transfer that broke
                // mid-read. Wire exposes no finer diagnostics than this.
                return m5::stl::make_unexpected(got == 0 ? ::m5::hal::v1::error::error_t::I2C_NO_ACK
                                                         : ::m5::hal::v1::error::error_t::I2C_BUS_ERROR);
            }
            for (size_t i = 0; i < rx_span.size; ++i) {
                int b = _wire->read();
                if (b < 0) {
                    return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::I2C_BUS_ERROR);
                }
                rx_span.data[i] = static_cast<uint8_t>(b);
            }
            auto com = rx->commit(rx_span.size);
            if (!com.has_value()) {
                return m5::stl::make_unexpected(com.error());
            }
            total += rx_span.size;
        }
    }

    return total;
}

}  // namespace m5::hal::v1::i2c

#endif

#endif
