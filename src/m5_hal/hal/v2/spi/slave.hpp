// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SPI_SLAVE_HPP_
#define M5_HAL_HAL_V2_SPI_SLAVE_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::spi {

/*!
  @brief Slave-side SPI bus wiring + per-bus defaults.

  SPI slave is master-clocked, so there is no clock rate here (the master owns
  SCLK): only the four wires, the SPI mode, the bit order, and an optional
  controller (host) index. `timeout_ms` is the default cap serve() waits for a
  master transaction before returning empty.
 */
struct SlaveBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_clk  = -1;
    types::gpio_number_t pin_mosi = -1;
    types::gpio_number_t pin_miso = -1;
    types::gpio_number_t pin_cs   = -1;
    uint8_t spi_mode              = 0;                       ///< SPI mode 0..3.
    uint8_t spi_order             = 0;                       ///< 0 = MSB first.
    int8_t host                   = -1;                      ///< Controller index; -1 = backend default.
    uint8_t tx_fill_byte          = 0x00;                    ///< MISO byte clocked once `tx` is exhausted.
    uint32_t timeout_ms           = types::TIMEOUT_FOREVER;  ///< Default serve() wait for a transaction.

    constexpr SlaveBusConfig(void) : bus::IBusConfig{types::bus_kind_t::SPI}
    {
    }
};

/*!
  @brief SPI slave bus.

  Unlike I2C there is no addressing and no clock stretching: the master clocks a
  full-duplex transaction unconditionally, so the slave must have a buffer queued
  before the master starts and cannot stall mid-transaction. The whole
  interaction is therefore ONE primitive — serve() exchanges a single CS-delimited
  transaction. (The half-duplex register-map model of `spi_slave_hd` — HW v2 only
  — is intentionally out of scope here; this is the all-chip full-duplex backend.)
 */
struct ISlaveBus : public bus::IBus {
    const SlaveBusConfig &getConfig(void) const override
    {
        return _config;
    }

    virtual result_t<void> init(const SlaveBusConfig &cfg) = 0;
    virtual result_t<void> release(void) override          = 0;

    /*!
      @brief Serve ONE full-duplex transaction.

      Up to `len` bytes are pulled from `tx` (the slave's MISO data; once `tx` is
      exhausted or null the remainder is the config `tx_fill_byte`) and clocked
      out while the master's MOSI bytes are captured into `rx` (a null Sink
      discards them). SPI is full-duplex, so a single `len` bounds BOTH directions
      (the shared clock-cycle count) — the same reason the SPI master `transfer()`
      takes one `len`. Blocks until the master completes a transaction (CS
      deassert / `len` reached) or `timeout_ms` elapses with no transaction.
      Returns the number of bytes actually exchanged (the master may clock fewer
      than `len`, ending early on CS deassert).
     */
    virtual result_t<size_t> serve(bus::IAccessor *owner, data::Source *tx, data::Sink *rx, size_t len,
                                   uint32_t timeout_ms) = 0;

protected:
    SlaveBusConfig _config;
};

/*!
  @brief Thin accessor exposing serve() to the application.

  Mirrors the role of the I2C `SlaveStreamAccessor`, but SPI's single-primitive
  shape: there is no separate read / write or begin / endTransaction — one
  serve() is one master-driven transaction. Bind with the ctor (or default-
  construct and `bind()`), then call serve() per transaction in a resident loop.
 */
class SpiSlaveAccessor : public bus::IAccessor {
public:
    SpiSlaveAccessor(ISlaveBus &bus) : bus::IAccessor{bus}
    {
    }

    const bus::IAccessConfig &getConfig(void) const override
    {
        return _access_config;
    }
    ISlaveBus &getBus(void) const
    {
        return static_cast<ISlaveBus &>(bus::IAccessor::getBus());
    }

    /*! @brief Source/Sink serve (streaming callers). */
    result_t<size_t> serve(data::Source *tx, data::Sink *rx, size_t len, uint32_t timeout_ms = types::TIMEOUT_FOREVER)
    {
        if (!isBound()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        return getBus().serve(this, tx, rx, len, timeout_ms);
    }

    /*!
      @brief Span serve sugar.

      Clocks out `tx` while capturing into `rx`; `len` is the larger of the two
      span sizes (full-duplex). Pass an empty span for a one-way direction (the
      other direction sends `tx_fill_byte` / is discarded).
     */
    result_t<size_t> serve(data::ConstDataSpan tx, data::DataSpan rx, uint32_t timeout_ms = types::TIMEOUT_FOREVER)
    {
        data::MemorySource src{tx};
        data::MemorySink sink{rx};
        const size_t len = (tx.size > rx.size) ? tx.size : rx.size;
        return serve(&src, &sink, len, timeout_ms);
    }

private:
    struct AccessConfig : public bus::IAccessConfig {
        constexpr AccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::SPI}
        {
        }
    } _access_config;
};

}  // namespace m5::hal::v2::spi

#endif  // M5_HAL_HAL_V2_SPI_SLAVE_HPP_
