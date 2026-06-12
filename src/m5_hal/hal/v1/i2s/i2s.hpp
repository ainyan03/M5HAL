// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2S_I2S_HPP_
#define M5_HAL_I2S_I2S_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::i2s {

/// I2S is a DMA-driven continuous stream with no transaction boundaries like I2C/SPI.
/// The API mirrors uart (write / writableBytes). Only master TX (playback) is supported
/// for now with the Philips standard slot format fixed. RX (recording) is reserved for
/// future extension via pin_din.

struct I2SBusConfig : public bus::BusConfig {
    types::gpio_number_t pin_bclk = -1;
    types::gpio_number_t pin_ws   = -1;
    types::gpio_number_t pin_dout = -1;
    types::gpio_number_t pin_din  = -1;  ///< RX (recording) — reserved, unused for now.
    types::gpio_number_t pin_mclk = -1;  ///< External MCLK output for codecs that require it; -1 = disabled.
    size_t tx_buffer_size = 8192;        ///< Approximate DMA buffer size; the backend rounds to descriptor granularity.

    constexpr I2SBusConfig(void) : bus::BusConfig{types::bus_kind_t::I2S}
    {
    }
};

struct I2SAccessConfig : public bus::AccessConfig {
    uint32_t sample_rate_hz   = 44100;
    uint32_t timeout_ms       = 1000;  ///< Bus lock timeout.
    uint32_t write_timeout_ms = 1000;  ///< write が DMA の空きを待つ上限。0 = non-blocking
    uint8_t bits_per_sample   = 16;    ///< Currently only 16 is supported.
    uint8_t channels          = 2;     ///< 1 = mono / 2 = stereo.

    constexpr I2SAccessConfig(void) : bus::AccessConfig{types::bus_kind_t::I2S}
    {
    }
};

struct I2SBus;

struct I2STxAccessor : public bus::Accessor, public data::StreamWriter {
    I2STxAccessor(I2SBus& bus, const I2SAccessConfig& access_config);

    const I2SAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    I2SBus& getI2SBus(void) const;

    result_t<void> setConfig(const I2SAccessConfig& cfg);

    result_t<size_t> write(data::ConstDataSpan tx_bytes) override;
    result_t<size_t> write(data::Source& tx, size_t len);
    result_t<size_t> write(const uint8_t* tx, size_t len);

    /// Bytes that can be accepted by write() right now without blocking (DMA buffer free space).
    result_t<size_t> writableBytes(void);

protected:
    I2SAccessConfig _access_config;
};

struct I2SBus : public bus::Bus {
    const I2SBusConfig& getConfig(void) const override
    {
        return _config;
    }

    /// Returns the byte count accepted within write_timeout_ms; a short return is normal.
    /// Underrun is not an error: the DMA outputs silence and resumes on the next write.
    virtual result_t<size_t> write(bus::Accessor* owner, const I2SAccessConfig& cfg, data::Source* tx, size_t len);
    virtual result_t<size_t> writableBytes(bus::Accessor* owner, const I2SAccessConfig& cfg);

protected:
    I2SBusConfig _config;
};

}  // namespace m5::hal::v1::i2s

#endif
