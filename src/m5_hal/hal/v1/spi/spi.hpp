// SPDX-License-Identifier: MIT
#ifndef M5_HAL_SPI_SPI_HPP_
#define M5_HAL_SPI_SPI_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../types.hpp"

/*!
  @namespace m5::hal::v1::spi
  @brief SPI bus, master accessor, transfer descriptor, command/address sugars.
 */
namespace m5::hal::v1::spi {

/*!
  @brief Bus-level configuration for SPI.

  Covers plain SPI as well as QSPI / OSPI configurations. Every pin
  is a `gpio_number_t` (`int16_t`); the default `-1` is the invalid
  sentinel. Variants resolve a non-negative value during `init()`
  via `m5::hal::v1::M5_Hal.Gpio.getPin(num)` (same singleton
  `GPIOGroup` convention as `IBusConfig`).

  QSPI / OSPI use `pin_mosi` / `pin_miso` / `pin_d2..d7` as the data
  lines. They are kept as named fields rather than a union with an
  anonymous array because the aliasing rules around such a union
  multiply edge cases. If indexed access becomes useful, add a helper
  (`gpio_number_t dataPin(uint8_t idx) const`) instead.
 */
struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_clk = -1;
    types::gpio_number_t pin_dc  = -1;  ///< Bus-wide D/C default; `MasterAccessConfig::pin_dc` overrides it per device.
    types::gpio_number_t pin_mosi = -1;  ///< data0
    types::gpio_number_t pin_miso = -1;  ///< data1
    types::gpio_number_t pin_d2   = -1;  ///< data2
    types::gpio_number_t pin_d3   = -1;  ///< data3
    types::gpio_number_t pin_d4   = -1;  ///< data4
    types::gpio_number_t pin_d5   = -1;  ///< data5
    types::gpio_number_t pin_d6   = -1;  ///< data6
    types::gpio_number_t pin_d7   = -1;  ///< data7

    constexpr IBusConfig(void) : bus::IBusConfig{types::bus_kind_t::SPI}
    {
    }
};

/*!
  @brief Data-path mode of an SPI transfer.
 */
enum class SpiDataMode {
    halfduplex,              ///< Half duplex.
    fullduplex,              ///< Full duplex.
    halfduplex_with_dc_pin,  ///< Half duplex with a separate D/C pin.
    fullduplex_with_dc_pin,  ///< Full duplex with a separate D/C pin.
    halfduplex_with_dc_bit,  ///< Half duplex with an in-band D/C bit (9-bit SPI).
    fullduplex_with_dc_bit,  ///< Full duplex with an in-band D/C bit (9-bit SPI).
    dual_output,
    dual_io,
    quad_output,
    quad_io,
    octal_output,
    octal_io,
};
typedef SpiDataMode spi_data_mode_t;

/*!
  @brief Accessor-level configuration for an SPI master target.
 */
struct MasterAccessConfig : public bus::IAccessConfig {
    types::gpio_number_t pin_cs = -1;
    /*!
      @brief Per-device D/C pin override.

      `-1` (the default) falls back to the bus-level `pin_dc` — the
      common single-display wiring. Set a non-negative pin to give this
      device its own D/C line, so two display-class devices with
      different D/C wiring can share one bus.
     */
    types::gpio_number_t pin_dc   = -1;
    uint32_t freq                 = 1000000;                      ///< Bus clock frequency in Hz. Default 1 MHz.
    spi_data_mode_t spi_data_mode = spi_data_mode_t::fullduplex;  ///< Default full duplex.
    struct {
        uint8_t spi_mode : 2;   ///< SPI mode 0..3, zero-initialized by the ctor.
        uint8_t spi_order : 1;  ///< 0 = MSB first, zero-initialized by the ctor.
    };
    uint8_t spi_command_length    = 0;  ///< Command-phase length in bits.
    uint8_t spi_address_length    = 0;  ///< Address-phase length in bits.
    uint8_t spi_read_dummy_cycle  = 0;  ///< Dummy cycles inserted before the read data phase.
    uint8_t spi_write_dummy_cycle = 0;  ///< Dummy cycles inserted before the write data phase.

    // Anonymous bitfield structs don't accept default member initializers
    // in C++17 (relaxed in C++20), so the anonymous-member names are
    // visible in the outer scope and zero-initialized via the mem-
    // initializer list.
    constexpr MasterAccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::SPI}, spi_mode{0}, spi_order{0}
    {
    }

    /*!
      @name Display-style setup presets.

      Field-assignment sugar for the display-class workload: each method
      sets the fields that the `writeCommand*` sugars depend on, named
      after how the D/C (data/command) distinction travels. The
      datasheet vocabulary maps as: "4-wire / 4-line serial" =
      `setupWithDCPin`, "3-wire / 3-line serial" = `setupWithDCBit`
      (those wire-count names collide with the sensor-world meaning of
      3/4-wire SPI, so the methods name the D/C transport instead —
      matching the `halfduplex_with_dc_pin` / `_with_dc_bit` enumerators
      they select).

      Both return `*this` so the call chains with further assignments:
      @code
      spi::AccessConfig cfg;
      cfg.setupWithDCPin(PIN_DC).pin_cs = PIN_CS;
      cfg.freq = 40000000;
      @endcode
      @{
     */
    /*! @brief D/C on a dedicated pin (datasheet: 4-wire / 4-line serial).
               Sets the per-device D/C pin, the matching data-path mode,
               and the 8-bit command phase `writeCommand` expects. */
    MasterAccessConfig& setupWithDCPin(types::gpio_number_t pin_dc_)
    {
        pin_dc             = pin_dc_;
        spi_data_mode      = spi_data_mode_t::halfduplex_with_dc_pin;
        spi_command_length = 8;
        return *this;
    }
    /*! @brief D/C as the 9th in-band bit (datasheet: 3-wire / 3-line
               serial). No D/C pin; the data-path mode carries the bit. */
    MasterAccessConfig& setupWithDCBit(void)
    {
        pin_dc             = -1;
        spi_data_mode      = spi_data_mode_t::halfduplex_with_dc_bit;
        spi_command_length = 8;
        return *this;
    }
    /*! @} */
};

/*!
  @brief Primary short name for the master access config.

  Master is the overwhelmingly common role, so it owns the short name;
  spell out `MasterAccessConfig` only where the contrast with a slave
  configuration matters.
 */
using AccessConfig = MasterAccessConfig;

/*!
  @brief Per-call SPI transfer descriptor.

  Inherits the empty `bus::ITransferDesc` marker and adds SPI-specific
  per-call directives: D/C pin levels, command / address phases, and
  dummy clock counts. Concrete variants decide which directives they
  can honor, but software SPI already consumes this full vocabulary.

  Portability of `dummy_cycles`: it counts clocks (bits), not bytes.
  Bit-granular paths honor any value: software bit-bang, ESP-IDF
  `SPI_TRANS_VARIABLE_DUMMY`, and the Arduino variant on ESP32 (which
  clocks the remainder via `SPIClass::transferBits`). Stock byte-oriented
  Arduino `SPIClass` (non-ESP32) only honors multiples of 8 and rejects
  the remainder with `INVALID_ARGUMENT`. Specify multiples of 8 to stay
  portable across every backend.
 */
struct TransferDesc : public bus::ITransferDesc {
    bool dc_level_valid     = false;
    bool dc_level           = true;
    uint32_t command        = 0;
    uint32_t address        = 0;
    uint8_t command_bytes   = 0;
    uint8_t address_bytes   = 0;
    uint8_t dummy_cycles    = 0;  ///< Dummy clocks (bits) before data; see portability note above.
    int8_t command_dc_level = -1;
    int8_t address_dc_level = -1;
    int8_t data_dc_level    = -1;
};

struct IBus;

/*!
  @brief Master-side accessor for an SPI bus.

  Holds per-target configuration (CS pin, frequency, mode, dummy
  cycles), wraps `beginTransaction` / `endTransaction` for CS scope,
  and exposes the `write` / `read` / `writeCommand*` / `read*` sugars
  that all funnel into a single `transfer` call.
 */
struct MasterAccessor : public bus::IAccessor {
    MasterAccessor(IBus& bus, const MasterAccessConfig& access_config);

    /*!
      @name Unbound construction + typed bind.

      Same contract as the I2C accessor: the unbound gate sits on the
      window openers (`beginAccess` / `beginTransaction`).
      @{
     */
    MasterAccessor(void) = default;
    explicit MasterAccessor(const MasterAccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to an SPI bus; rejected while a window is open. */
    m5::hal::v1::result_t<void> bind(IBus& bus);
    /*! @} */

    const MasterAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    /*! @brief Return the underlying bus as `IBus&` (kind fixed at ctor). */
    IBus& getBus(void) const;

    /*!
      @brief Replace the per-target configuration.

      Fails with `INVALID_ARGUMENT` while a transaction or access
      window is open — swapping the config mid-transfer would leave
      the active transaction undefined (same contract as the I2C
      accessor).
     */
    m5::hal::v1::result_t<void> setConfig(const MasterAccessConfig& cfg);

    /*!
      @brief Sole span-based transfer sugar.
     */
    m5::hal::v1::result_t<size_t> transfer(const TransferDesc& desc, data::ConstDataSpan tx_bytes,
                                           data::DataSpan rx_bytes);
    /*!
      @brief Source/Sink overload for streaming callers.
      @param len  Cap on the bytes transferred this call, even if the
                  underlying `Source` / `Sink` could continue further.
     */
    m5::hal::v1::result_t<size_t> transfer(const TransferDesc& desc, data::Source* tx, data::Sink* rx, size_t len);
    /*!
      @name CS scope (begin / end transaction).
      @{
     */
    m5::hal::v1::result_t<void> beginTransaction(void);
    m5::hal::v1::result_t<void> endTransaction(void);
    /*! @} */

    /*!
      @name Plain write / read sugars (span, Source/Sink, raw pointer).
      @{
     */
    m5::hal::v1::result_t<size_t> write(data::ConstDataSpan tx_bytes);
    m5::hal::v1::result_t<size_t> write(data::Source& tx, size_t len);
    m5::hal::v1::result_t<size_t> read(data::DataSpan rx_bytes);
    m5::hal::v1::result_t<size_t> read(data::Sink& rx, size_t len);
    m5::hal::v1::result_t<size_t> write(const uint8_t* tx, size_t len);
    m5::hal::v1::result_t<size_t> read(uint8_t* dst, size_t len);
    /*! @} */

    /*!
      @name Command / address sugars for display-like SPI peripherals.
      @{
     */
    m5::hal::v1::result_t<size_t> writeCommand(data::ConstDataSpan tx_bytes);
    m5::hal::v1::result_t<size_t> writeCommand(uint32_t command);
    m5::hal::v1::result_t<size_t> writeCommandAddress(uint32_t command, uint32_t address);
    m5::hal::v1::result_t<size_t> writeCommandData(data::ConstDataSpan tx_bytes);
    m5::hal::v1::result_t<size_t> writeCommandData(uint32_t command, data::ConstDataSpan tx_bytes);
    m5::hal::v1::result_t<size_t> writeCommandData(uint32_t command, data::Source& tx, size_t len);
    m5::hal::v1::result_t<size_t> writeCommandAddressData(uint32_t command, uint32_t address,
                                                          data::ConstDataSpan tx_bytes);
    m5::hal::v1::result_t<size_t> writeCommandAddressData(uint32_t command, uint32_t address, data::Source& tx,
                                                          size_t len);
    m5::hal::v1::result_t<size_t> readCommandData(uint32_t command, data::DataSpan rx_bytes);
    m5::hal::v1::result_t<size_t> readCommandData(uint32_t command, data::Sink& rx, size_t len);
    m5::hal::v1::result_t<size_t> readCommandAddressData(uint32_t command, uint32_t address, data::DataSpan rx_bytes);
    m5::hal::v1::result_t<size_t> readCommandAddressData(uint32_t command, uint32_t address, data::Sink& rx,
                                                         size_t len);
    /*! @brief Drive `count` dummy clock cycles (`count` <= 255,
               larger values fail with `INVALID_ARGUMENT`). */
    m5::hal::v1::result_t<void> sendDummyClock(size_t count);
    /*! @} */

protected:
    MasterAccessConfig _access_config;
    uint32_t _transaction_depth = 0;
};

//-------------------------------------------------------------------------

/*!
  @brief Concrete SPI bus base.

  Signature is aligned with `IBus` so accessor sugars converge here.
  Default implementations return `NOT_IMPLEMENTED` until a concrete
  variant overrides them.
 */
struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual m5::hal::v1::result_t<void> beginTransaction(bus::IAccessor* owner, const MasterAccessConfig& cfg);
    virtual m5::hal::v1::result_t<void> endTransaction(bus::IAccessor* owner, const MasterAccessConfig& cfg);
    /*!
      @brief Core SPI transfer entry point.

      Return-value contract (matches `IBus::transfer`):
      the bus returns the byte count of the DATA phase only (`tx + rx`
      as driven on the wire). The command / address phases encoded by
      `desc` are NOT included here. Variant implementations MUST follow
      this split, or the accessor layer double-counts.

      Accessor sugars report "bytes of the caller's data argument that
      were processed": `writeCommandData(span)` therefore adds the
      command bytes back (the span includes them), while plain
      `write`/`read`/`transfer` report the data phase as-is.

      `tx` / `rx` are nullable (`nullptr` = no data for that direction).
      The default implementation returns `NOT_IMPLEMENTED`.
     */
    virtual m5::hal::v1::result_t<size_t> transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg,
                                                   const TransferDesc& desc, data::Source* tx, data::Sink* rx);

protected:
    IBusConfig _config;
};

//-------------------------------------------------------------------------
/*!
  @brief RAII helper that wraps `MasterAccessor::beginTransaction` /
         `endTransaction` (the CS assert/deassert scope).

  Display-init style code with many early returns kept leaking
  `endTransaction` on the error paths; the scope closes the transaction
  on every exit. Polarity follows `bus::ScopedAccess`: success =
  `!scope.has_error()`, deliberately no `operator bool`.

  The destructor cannot report an `endTransaction` failure. When the
  release error must be observed (strict bring-up code), use
  `bus::guarded` instead — its policy keeps a body success from hiding
  a broken release (spec/design/bus_accessor.md §guarded).

  The bus lock under the transaction is taken with the infinite default
  budget. To bound it, hold an outer `bus::ScopedAccess{dev, budget}` —
  the depth counter folds the inner lock into the outer one.
 */
class ScopedTransaction {
public:
    explicit ScopedTransaction(MasterAccessor& accessor) : _accessor{&accessor}
    {
        auto r = _accessor->beginTransaction();
        if (!r.has_value()) {
            _error    = r.error();
            _accessor = nullptr;  // dtor will not call endTransaction
        }
    }
    ~ScopedTransaction()
    {
        if (_accessor != nullptr) {
            (void)_accessor->endTransaction();
        }
    }
    ScopedTransaction(const ScopedTransaction&)            = delete;
    ScopedTransaction& operator=(const ScopedTransaction&) = delete;
    ScopedTransaction(ScopedTransaction&&)                 = delete;
    ScopedTransaction& operator=(ScopedTransaction&&)      = delete;

    bool has_error(void) const
    {
        return _accessor == nullptr;
    }
    m5::hal::v1::error::error_t error(void) const
    {
        return _error;
    }

private:
    MasterAccessor* _accessor          = nullptr;
    m5::hal::v1::error::error_t _error = m5::hal::v1::error::error_t::OK;
};

//-------------------------------------------------------------------------
// bind() is defined below the concrete IBus: at the accessor's point of
// declaration the kind IBus is still an incomplete type, so the
// derived-to-base conversion _bindBus needs is not visible yet.
inline m5::hal::v1::result_t<void> MasterAccessor::bind(IBus& bus)
{
    if (inAccess() || _transaction_depth != 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _bindBus(bus);
    return {};
}

/*!
  @brief Non-owning SPI bus registry (`M5_Hal.SPI`); see `bus::BusGroup`.
 */
using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v1::spi

#endif
