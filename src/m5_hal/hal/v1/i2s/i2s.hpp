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

struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_bclk = -1;
    types::gpio_number_t pin_ws   = -1;
    types::gpio_number_t pin_dout = -1;
    types::gpio_number_t pin_din  = -1;  ///< RX (recording) — reserved, unused for now.
    types::gpio_number_t pin_mclk = -1;  ///< External MCLK output for codecs that require it; -1 = disabled.
    size_t tx_buffer_size = 8192;        ///< Approximate DMA buffer size; the backend rounds to descriptor granularity.

    constexpr IBusConfig(void) : bus::IBusConfig{types::bus_kind_t::I2S}
    {
    }
};

struct AccessConfig : public bus::IAccessConfig {
    uint32_t sample_rate_hz   = 44100;
    uint32_t write_timeout_ms = 1000;  ///< Max wait for DMA free space during write, in ms; 0 = non-blocking.
    uint8_t bits_per_sample   = 16;    ///< Currently only 16 is supported.
    uint8_t channels          = 2;     ///< 1 = mono / 2 = stereo.

    constexpr AccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::I2S}
    {
    }
};

struct IBus;

struct TxAccessor : public bus::IAccessor, public data::StreamWriter {
    TxAccessor(IBus& bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (gate: `beginAccess` inside the sugars). @{ */
    TxAccessor(void) = default;
    explicit TxAccessor(const AccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to an I2S bus; rejected while an access window is open. */
    m5::hal::v1::result_t<void> bind(IBus& bus);
    /*! @} */

    const AccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    IBus& getBus(void) const;

    result_t<void> setConfig(const AccessConfig& cfg);

    result_t<size_t> write(data::ConstDataSpan tx_bytes) override;
    result_t<size_t> write(data::Source& tx, size_t len);
    result_t<size_t> write(const uint8_t* tx, size_t len);

    /// Bytes that can be accepted by write() right now without blocking (DMA buffer free space).
    result_t<size_t> writableBytes(void);

protected:
    AccessConfig _access_config;
};

struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    /// Returns the byte count accepted within write_timeout_ms; a short return is normal.
    /// Underrun is not an error: the DMA outputs silence and resumes on the next write.
    virtual result_t<size_t> write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* tx, size_t len);
    virtual result_t<size_t> writableBytes(bus::IAccessor* owner, const AccessConfig& cfg);

protected:
    IBusConfig _config;
};

//-------------------------------------------------------------------------
// bind() is defined below the concrete IBus: at the accessor's point of
// declaration the kind IBus is still an incomplete type, so the
// derived-to-base conversion _bindBus needs is not visible yet.
inline m5::hal::v1::result_t<void> TxAccessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _bindBus(bus);
    return {};
}

/*!
  @brief Non-owning I2S bus registry (`M5_Hal.I2S`); see `bus::BusGroup`.
 */
using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v1::i2s

#endif
