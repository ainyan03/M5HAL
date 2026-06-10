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
  `GPIOGroup` convention as `I2CBusConfig`).

  QSPI / OSPI use `pin_mosi` / `pin_miso` / `pin_d2..d7` as the data
  lines. They are kept as named fields rather than a union with an
  anonymous array because the aliasing rules around such a union
  multiply edge cases. If indexed access becomes useful, add a helper
  (`gpio_number_t dataPin(uint8_t idx) const`) instead.
 */
struct SPIBusConfig : public bus::BusConfig {
    types::gpio_number_t pin_clk  = -1;
    types::gpio_number_t pin_dc   = -1;
    types::gpio_number_t pin_mosi = -1;  ///< data0
    types::gpio_number_t pin_miso = -1;  ///< data1
    types::gpio_number_t pin_d2   = -1;  ///< data2
    types::gpio_number_t pin_d3   = -1;  ///< data3
    types::gpio_number_t pin_d4   = -1;  ///< data4
    types::gpio_number_t pin_d5   = -1;  ///< data5
    types::gpio_number_t pin_d6   = -1;  ///< data6
    types::gpio_number_t pin_d7   = -1;  ///< data7

    constexpr SPIBusConfig(void) : bus::BusConfig{types::bus_kind_t::SPI}
    {
    }
};

/*!
  @brief Data-path mode of an SPI transfer.
 */
enum class SpiDataMode {
    spi_halfduplex,              ///< Half duplex.
    spi_fullduplex,              ///< Full duplex.
    spi_halfduplex_with_dc_pin,  ///< Half duplex with a separate D/C pin.
    spi_fullduplex_with_dc_pin,  ///< Full duplex with a separate D/C pin.
    spi_halfduplex_with_dc_bit,  ///< Half duplex with an in-band D/C bit (9-bit SPI).
    spi_fullduplex_with_dc_bit,  ///< Full duplex with an in-band D/C bit (9-bit SPI).
    spi_dual_output,
    spi_dual_io,
    spi_quad_output,
    spi_quad_io,
    spi_octal_output,
    spi_octal_io,
};
typedef SpiDataMode spi_data_mode_t;

/*!
  @brief Accessor-level configuration for an SPI master target.
 */
struct SPIMasterAccessConfig : public bus::AccessConfig {
    types::gpio_number_t pin_cs   = -1;
    uint32_t freq                 = 1000000;  ///< Bus clock frequency in Hz. Default 1 MHz.
    uint32_t timeout_ms           = 1000;
    spi_data_mode_t spi_data_mode = spi_data_mode_t::spi_fullduplex;  ///< Default full duplex.
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
    constexpr SPIMasterAccessConfig(void) : bus::AccessConfig{types::bus_kind_t::SPI}, spi_mode{0}, spi_order{0}
    {
    }
};

/*!
  @brief Per-call SPI transfer descriptor.

  Inherits the empty `bus::TransferDesc` marker and adds SPI-specific
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
struct TransferDesc : public bus::TransferDesc {
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

struct SPIBus;

/*!
  @brief Master-side accessor for an SPI bus.

  Holds per-target configuration (CS pin, frequency, mode, dummy
  cycles), wraps `beginTransaction` / `endTransaction` for CS scope,
  and exposes the `write` / `read` / `writeCommand*` / `read*` sugars
  that all funnel into a single `transfer` call.
 */
struct SPIMasterAccessor : public bus::Accessor {
    SPIMasterAccessor(SPIBus& bus, const SPIMasterAccessConfig& access_config);

    const SPIMasterAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    /*! @brief Return the underlying bus as `SPIBus&` (kind fixed at ctor). */
    SPIBus& getSPIBus(void) const;

    /*! @brief Replace the per-target configuration; same contract as I2C. */
    m5::stl::expected<void, m5::hal::v1::error::error_t> setConfig(const SPIMasterAccessConfig& cfg);

    /*!
      @brief Sole span-based transfer sugar.
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(const TransferDesc& desc,
                                                                    data::ConstDataSpan tx_bytes,
                                                                    data::DataSpan rx_bytes);
    /*!
      @brief Source/Sink overload for streaming callers.
      @param len  Cap on the bytes transferred this call, even if the
                  underlying `Source` / `Sink` could continue further.
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(const TransferDesc& desc, data::Source* tx,
                                                                    data::Sink* rx, size_t len);
    /*!
      @name CS scope (begin / end transaction).
      @{
     */
    m5::stl::expected<void, m5::hal::v1::error::error_t> beginTransaction(void);
    m5::stl::expected<void, m5::hal::v1::error::error_t> endTransaction(void);
    /*! @} */

    /*!
      @name Plain write / read sugars (span, Source/Sink, raw pointer).
      @{
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(data::Source& tx, size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(data::DataSpan rx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(data::Sink& rx, size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(const uint8_t* tx, size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(uint8_t* dst, size_t len);
    /*! @} */

    /*!
      @name Command / address sugars for display-like SPI peripherals.
      @{
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommand(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommand(uint32_t command);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandAddress(uint32_t command, uint32_t address);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandData(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandData(uint32_t command,
                                                                            data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandData(uint32_t command, data::Source& tx,
                                                                            size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandAddressData(uint32_t command, uint32_t address,
                                                                                   data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeCommandAddressData(uint32_t command, uint32_t address,
                                                                                   data::Source& tx, size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readCommandData(uint32_t command, data::DataSpan rx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readCommandData(uint32_t command, data::Sink& rx,
                                                                           size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readCommandAddressData(uint32_t command, uint32_t address,
                                                                                  data::DataSpan rx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readCommandAddressData(uint32_t command, uint32_t address,
                                                                                  data::Sink& rx, size_t len);
    /*! @brief Drive `count` dummy clock cycles. */
    m5::stl::expected<void, m5::hal::v1::error::error_t> sendDummyClock(size_t count);
    /*! @} */

protected:
    SPIMasterAccessConfig _access_config;
    uint32_t _transaction_depth = 0;
};

//-------------------------------------------------------------------------

/*!
  @brief Concrete SPI bus base.

  Signature is aligned with `I2CBus` so accessor sugars converge here.
  Default implementations return `NOT_IMPLEMENTED` until a concrete
  variant overrides them.
 */
struct SPIBus : public bus::Bus {
    const SPIBusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual m5::stl::expected<void, m5::hal::v1::error::error_t> beginTransaction(bus::Accessor* owner,
                                                                                  const SPIMasterAccessConfig& cfg);
    virtual m5::stl::expected<void, m5::hal::v1::error::error_t> endTransaction(bus::Accessor* owner,
                                                                                const SPIMasterAccessConfig& cfg);
    virtual m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(bus::Accessor* owner,
                                                                            const SPIMasterAccessConfig& cfg,
                                                                            const TransferDesc& desc, data::Source* tx,
                                                                            data::Sink* rx);

protected:
    SPIBusConfig _config;
};

}  // namespace m5::hal::v1::spi

#endif
