#ifndef M5_HAL_I2C_I2C_HPP_
#define M5_HAL_I2C_I2C_HPP_

#include "../assert.hpp"
#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../gpio/gpio.hpp"

#include <type_traits>

/*!
  @namespace m5::hal::v1::i2c
  @brief I2C bus, master accessor, transfer descriptor, and register sugar.
 */
namespace m5::hal::v1::i2c {

/*!
  @brief Bus-level configuration for I2C.

  Pin fields are global `gpio_number_t` values (`int16_t` opaque IDs).
  The default `-1` is the invalid sentinel; variants resolve a non-
  negative value through the singleton `GPIOGroup`
  (e.g. `m5::hal::v1::M5_Hal.Gpio.getPin(num)`).

  For MCU-internal pins, callers can still pass plain literals such as
  `PIN_SCL = 21` because slot 0 is reserved for the MCU GPIO and its
  high bits are zero.

  To route SCL / SDA through an I/O expander, register the expander's
  `IGPIO` to a slot on `M5_Hal.Gpio` and build the global
  `gpio_number_t` with `makeGpioNumber(slot, local)`:
  @code
  constexpr m5::hal::v1::types::gpio_slot_t EXPANDER_SLOT = 1;
  m5::hal::v1::M5_Hal.Gpio.addGPIO(&expander, EXPANDER_SLOT);
  I2CBusConfig bus_cfg{
      m5::hal::v1::types::makeGpioNumber(EXPANDER_SLOT, 0),
      m5::hal::v1::types::makeGpioNumber(EXPANDER_SLOT, 1)};
  @endcode

  Usage patterns:
  - Two-argument ctor (typical): `I2CBusConfig{SCL, SDA}`
  - Field assignment: `bus_cfg.pin_scl = SCL; bus_cfg.pin_sda = SDA;`
 */
struct I2CBusConfig : public bus::BusConfig {
    types::gpio_number_t pin_scl = -1;  ///< Global GPIO number for SCL.
    types::gpio_number_t pin_sda = -1;  ///< Global GPIO number for SDA.

    constexpr I2CBusConfig(void) : bus::BusConfig{types::bus_kind_t::I2C}
    {
    }
    constexpr I2CBusConfig(types::gpio_number_t scl, types::gpio_number_t sda)
        : bus::BusConfig{types::bus_kind_t::I2C}, pin_scl{scl}, pin_sda{sda}
    {
    }
};

/*!
  @brief Accessor-level configuration for an I2C master target.
 */
struct I2CMasterAccessConfig : public bus::AccessConfig {
    uint32_t freq       = 100000;  ///< Bus clock frequency in Hz.
    uint32_t timeout_ms = 1000;    ///< Per-transfer timeout in milliseconds.
    /*!
      @brief I2C slave address (target).

      The default `0` is also the memory-zero initial value. Most callers
      should assign this field before issuing a transfer; a deliberate
      general-call transfer may intentionally keep it at `0`.
     */
    uint16_t i2c_addr     = 0;
    bool address_is_10bit = false;
    /*!
      @brief Register-address width used by signed literal sugar.

      `0` and `1` both mean the default 1-byte register address.
      Specify `2` only for devices with 2-byte register addresses.
      Other values are outside the HAL sugar contract: debug builds
      assert, release builds return `INVALID_ARGUMENT`.

      Typed register constants (`uint8_t` / `uint16_t`) still carry
      their own width and do not consult this field.
     */
    uint8_t register_address_bytes = 0;
    /*!
      @brief Whether to emit a repeated start before the next transfer.

      Defaults to `true` so existing call sites keep their behavior.
      Concrete bus implementations are free to ignore this flag when
      the underlying hardware always emits a restart.
     */
    bool use_restart = true;

    constexpr I2CMasterAccessConfig(void) : bus::AccessConfig{types::bus_kind_t::I2C}
    {
    }
};

/*!
  @brief Per-call I2C transfer descriptor.

  Holds an inline prefix buffer that carries the register address (or
  any leading byte sequence). The inline buffer spares callers the
  need to build a local array and pass a pointer. A prefix longer
  than `PREFIX_CAPACITY` is out of contract — push the excess into
  the tx `Source` instead.
 */
struct TransferDesc : public bus::TransferDesc {
    static constexpr size_t PREFIX_CAPACITY = 8;
    uint8_t prefix[PREFIX_CAPACITY]         = {};
    uint8_t prefix_len                      = 0;

    /*! @brief No-prefix descriptor for `probe` and the `write` / `read` sugars. */
    constexpr TransferDesc() = default;

    /*!
      @brief Build a prefix from an unsigned integral register address.

      The address is serialized into `prefix` MSB first (big-endian
      wire convention). `sizeof(T) <= 4` and unsigned-integral are
      enforced by SFINAE. Callers MUST spell the type explicitly:
      `TransferDesc{0x12}` is rejected because `0x12` is an `int`;
      use `TransferDesc{uint8_t{0x12}}`.

      Examples:
      @code
      TransferDesc{uint8_t{0xD0}}        // 1-byte prefix
      TransferDesc{uint16_t{0x1234}}     // 2-byte prefix [0x12, 0x34]
      TransferDesc{uint32_t{0xDEADBEEF}} // 4-byte prefix [0xDE, 0xAD, 0xBE, 0xEF]
      @endcode
     */
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value && std::is_unsigned<T>::value && sizeof(T) <= 4,
                                      int>::type = 0>
    constexpr explicit TransferDesc(T reg) : prefix{}, prefix_len{static_cast<uint8_t>(sizeof(T))}
    {
        // Big-endian (MSB first): for N = sizeof(T),
        //   prefix[i] = (reg >> ((N - 1 - i) * 8)) & 0xFF
        constexpr size_t N = sizeof(T);
        for (size_t i = 0; i < N; ++i) {
            prefix[i] = static_cast<uint8_t>((reg >> ((N - 1 - i) * 8)) & 0xFF);
        }
    }

    /*!
      @brief Per-byte prefix ctors for callers that want to compose
             bytes manually (e.g. little-endian register addresses, or
             bit patterns that no integral type expresses cleanly).
     */
    constexpr TransferDesc(uint8_t b0, uint8_t b1) : prefix{b0, b1}, prefix_len{2}
    {
    }
    constexpr TransferDesc(uint8_t b0, uint8_t b1, uint8_t b2) : prefix{b0, b1, b2}, prefix_len{3}
    {
    }
    constexpr TransferDesc(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) : prefix{b0, b1, b2, b3}, prefix_len{4}
    {
    }
};

// Forward declaration — I2CMasterAccessor takes I2CBus& in its ctor;
// the full I2CBus definition lives below.
struct I2CBus;

/*!
  @brief Master-side accessor for an I2C bus.

  Holds per-target configuration (slave address, frequency, timeout)
  and serializes transfers against the underlying bus via
  lock / unlock. Convenience sugars (`write`, `read`, register
  helpers, `probe`) all wrap a single `transfer` call.
 */
struct I2CMasterAccessor : public bus::Accessor {
    /*!
      @brief Construct an accessor bound to an I2C bus.

      The ctor takes `I2CBus&` directly so kind mismatch is rejected
      at compile time; the previous `bus::Bus&` signature would have
      let an SPI bus through.
     */
    I2CMasterAccessor(I2CBus& bus, const I2CMasterAccessConfig& access_config);

    /*! @brief Covariant override: every accessor returns its concrete config. */
    const I2CMasterAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    /*!
      @brief Return the underlying bus as `I2CBus&`.

      The kind is fixed at ctor time, so the static_cast is safe. This
      replaces ad-hoc `static_cast<I2CBus*>(&getBus())` at call sites.
     */
    I2CBus& getI2CBus(void) const;

    /*!
      @brief Replace the per-target configuration.

      Sugar for the "build one accessor, swap its config in a loop"
      scan pattern: callers no longer rebuild an `AccessConfig` and a
      fresh `Accessor` per address (e.g. an I2C scan goes from 112
      accessor constructions to 1 + 112 `setConfig` calls).

      Rejected with `INVALID_ARGUMENT` while an access window is open
      (`inAccess() == true`), because changing the config mid-transfer
      would leave callers observing an undefined snapshot. Call it
      outside any `ScopedAccess` / explicit `beginAccess`.

      The arduino variant re-issues `setClock` on transfer when
      `cfg.freq != _last_freq`, so this entry point needs no extra
      invalidation logic — transparent to the caller.
     */
    m5::stl::expected<void, m5::hal::v1::error::error_t> setConfig(const I2CMasterAccessConfig& cfg);

    /*!
      @brief Sole transfer sugar: wrap one transfer with
             `beginAccess` / `endAccess`.

      Internally constructs `MemorySource` / `MemorySink` and dispatches
      to `bus->transfer(TransferDesc, Source*, Sink*)`. Callers write
      register addresses (or any leading bytes) directly into
      `desc.prefix`.
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(const TransferDesc& desc,
                                                                    data::ConstDataSpan tx_bytes,
                                                                    data::DataSpan rx_bytes);

    /*!
      @brief Source/Sink overload for streaming callers.

      Mirrors the bus entry point (and the SPI accessor's equivalent
      overload). `tx` / `rx` are nullable; the span sugar above
      delegates here.
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(const TransferDesc& desc, data::Source* tx,
                                                                    data::Sink* rx);

    /*!
      @name Span-based write / read sugars.

      Internally call `transfer` with an empty `TransferDesc`, so the
      `beginAccess` / `endAccess` wrap is shared.
      @{
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(data::ConstDataSpan tx_bytes);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(data::DataSpan rx_bytes);
    /*! @} */

    /*!
      @name Raw `uint8_t* + size_t` overloads.

      Save the caller a span construction when handing in a C array.
      These forward to the span overload. There is no overload-resolution
      ambiguity because `uint8_t*` and the `data::DataSpan` /
      `data::ConstDataSpan` value types are distinct.
      @{
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(const uint8_t* tx, size_t len);
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(uint8_t* dst, size_t len);
    /*! @} */

    /*!
      @name Register read / write sugars.

      Read or write N bytes anchored at a register address. SFINAE
      restricts `TReg` to **unsigned integral** with `sizeof(TReg) <= 2`,
      matching M5UU's `M5UnitComponent::writeRegister` / `readRegister`.
      The single name covers both single-byte-address devices (the
      majority) and 2-byte-address devices (some EEPROMs / sensors).

      Byte order: addresses 2 bytes or wider are transmitted MSB
      first (big-endian) on the wire. This follows the I2C industry
      convention and is fixed — callers pass a host-endian `uint16_t`,
      the library serializes it big-endian.

      Caller style: typed constants (`static constexpr uint8_t REG_X =
      0xD0;`) are preferred because the width is explicit. Bare signed
      literals such as `readRegister(0xD0)` are also accepted by the
      overloads below; their width comes from
      `I2CMasterAccessConfig::register_address_bytes` (`0` / `1` =
      1 byte, `2` = 2 bytes).

      Value-side size and byte order are the caller's responsibility
      (compose N bytes with `data::ConstDataSpan` / `data::DataSpan`).
      Big- / little-endian helpers belong in M5UU's
      `M5UnitComponent` layer, not in HAL.
      @{
     */
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(TReg reg, data::ConstDataSpan value)
    {
        return transfer(TransferDesc{reg}, value, data::DataSpan{});
    }
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(TReg reg, uint8_t value)
    {
        return writeRegister(reg, data::ConstDataSpan{&value, 1});
    }
    /*!
      @brief Raw-pointer overload for writing N bytes from a C array.

      Saves a span construction in the typical pattern
      `static constexpr uint8_t REG = 0xF4; uint8_t buf[3] = {...};`.
     */
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(TReg reg, const uint8_t* tx, size_t len)
    {
        return writeRegister(reg, data::ConstDataSpan{tx, len});
    }
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readRegister(TReg reg, data::DataSpan dst)
    {
        return transfer(TransferDesc{reg}, data::ConstDataSpan{}, dst);
    }
    /*! @brief Raw-pointer overload for reading N bytes into a C array. */
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readRegister(TReg reg, uint8_t* dst, size_t len)
    {
        return readRegister(reg, data::DataSpan{dst, len});
    }
    /*! @brief Convenience read of a 1-byte register (returns the byte). */
    template <typename TReg,
              typename std::enable_if<
                  std::is_integral<TReg>::value && std::is_unsigned<TReg>::value && sizeof(TReg) <= 2, int>::type = 0>
    m5::stl::expected<uint8_t, m5::hal::v1::error::error_t> readRegister(TReg reg)
    {
        uint8_t v;
        auto r = readRegister(reg, data::DataSpan{&v, 1});
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return v;
    }
    /*! @} */

    /*!
      @name Signed-literal register read / write sugars.

      These overloads exist for Arduino-style calls such as
      `readRegister(0x00)`. The register-address byte width is taken
      from `I2CMasterAccessConfig::register_address_bytes`: `0` and
      `1` serialize one byte, `2` serializes two bytes MSB first.
      Unsupported widths assert in debug builds and return
      `INVALID_ARGUMENT` in release builds.
      @{
     */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(int reg, data::ConstDataSpan value)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        return transfer(desc.value(), value, data::DataSpan{});
    }
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(int reg, uint8_t value)
    {
        return writeRegister(reg, data::ConstDataSpan{&value, 1});
    }
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> writeRegister(int reg, const uint8_t* tx, size_t len)
    {
        return writeRegister(reg, data::ConstDataSpan{tx, len});
    }
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readRegister(int reg, data::DataSpan dst)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        return transfer(desc.value(), data::ConstDataSpan{}, dst);
    }
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readRegister(int reg, uint8_t* dst, size_t len)
    {
        return readRegister(reg, data::DataSpan{dst, len});
    }
    m5::stl::expected<uint8_t, m5::hal::v1::error::error_t> readRegister(int reg)
    {
        uint8_t v;
        auto r = readRegister(reg, data::DataSpan{&v, 1});
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return v;
    }
    /*! @} */

    /*!
      @brief Probe for the presence of the configured slave.

      Sends a zero-byte write: `OK` if the target ACKs, `I2C_NO_ACK`
      (or another bus error) otherwise. Promotes the legacy
      "empty-write" idiom to a named API.
     */
    m5::stl::expected<void, m5::hal::v1::error::error_t> probe(void);

protected:
    m5::stl::expected<TransferDesc, m5::hal::v1::error::error_t> makeLiteralRegisterDesc(int reg) const
    {
        if (reg < 0) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        const uint8_t width = _access_config.register_address_bytes == 0 ? 1 : _access_config.register_address_bytes;
        if (width == 1) {
            if (reg > 0xFF) {
                return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
            }
            return TransferDesc{static_cast<uint8_t>(reg)};
        }
        if (width == 2) {
            if (reg > 0xFFFF) {
                return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
            }
            return TransferDesc{static_cast<uint16_t>(reg)};
        }
        M5HAL_ASSERT(false, "I2CMasterAccessConfig::register_address_bytes supports only 0, 1, or 2");
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    I2CMasterAccessConfig _access_config;
};

//-------------------------------------------------------------------------

/*!
  @brief Concrete I2C bus base.

  Variants derive from `I2CBus` and override `transfer`; the inline
  `probe` sugar uses a stack-local accessor so callers can check for
  a device without building one themselves.
 */
struct I2CBus : public bus::Bus {
    const I2CBusConfig& getConfig(void) const override
    {
        return _config;
    }

    /*!
      @brief Probe a single device without building an accessor.

      Internally constructs a stack-allocated `I2CMasterAccessor` as a
      sentinel and uses it as the lock owner. The sentinel's lifetime
      is the function scope, so this is thread-safe. One probe
      includes (sentinel ctor + `beginAccess` + transfer probe path +
      `endAccess` + sentinel dtor); the ctor is inline and trivially
      cheap.

      Callers that want to customize `freq` / `timeout_ms` should
      build an `I2CMasterAccessor` and use `setConfig` instead. The
      default `timeout_ms = 50` targets I2C scan loops (waiting one
      second per NACK is impractical) and is intentionally smaller
      than `I2CMasterAccessConfig`'s default of 1000 ms.
     */
    m5::stl::expected<void, m5::hal::v1::error::error_t> probe(uint16_t i2c_addr, uint32_t freq = 100000,
                                                               uint32_t timeout_ms = 50);

    /*!
      @brief Core I2C transfer entry point.

      Per-call metadata travels through `TransferDesc`. Leading bytes
      (register address, ...) live in the inline `desc.prefix` /
      `desc.prefix_len` buffer, sparing callers a local array and a
      pointer hand-off.

      `tx` and `rx` are nullable: `nullptr` means "no data for this
      segment" (write-only or read-only).

      Probe (empty transfer) contract: when `desc.prefix_len == 0`,
      `tx == nullptr` (or `tx` is already EOF), and `rx == nullptr`,
      this issues a single address+W on the wire and inspects the ACK
      bit. ACK -> `OK`; NACK -> `I2C_NO_ACK`. Every variant
      implementation MUST honor this path because `Accessor::probe`
      depends on it.

      `owner` is the calling accessor's identity (for lock-owner
      verification). Sugar paths through `Accessor` forward `this`
      automatically; low-level callers MUST keep an accessor alive
      and pass `&accessor`.

      The return value is the total transferred byte count
      (`prefix + tx + rx`) or an `error_t`. The probe path returns 0
      on a successful zero-byte transfer. The default implementation
      returns `NOT_IMPLEMENTED`; every variant overrides it.
     */
    virtual m5::stl::expected<size_t, m5::hal::v1::error::error_t> transfer(bus::Accessor* owner,
                                                                            const I2CMasterAccessConfig& cfg,
                                                                            const TransferDesc& desc, data::Source* tx,
                                                                            data::Sink* rx);

protected:
    I2CBusConfig _config;
};

}  // namespace m5::hal::v1::i2c

#endif
