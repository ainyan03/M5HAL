// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2C_I2C_HPP_
#define M5_HAL_I2C_I2C_HPP_

#include "../assert.hpp"
#include "../bus/bus.hpp"
#include "../bus/bus_view.hpp"
#include "../bus/hal_backend.hpp"
#include "../bus/hw_pool.hpp"
#include "../bus/managed_bus.hpp"
#include "../bus/managed_facade.hpp"
#include "../bus/registry.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../gpio/gpio.hpp"
#include "./master_clock_limit.hpp"

#include <atomic>
#include <memory>
#include <new>
#include <type_traits>

/*!
  @namespace m5::hal::v2::i2c
  @brief I2C bus, master accessor, transfer descriptor, and register sugar.

  ## Getting a bus (entry point)

  Buses are obtained from a `Hal` facade instance — `M5_Hal` (the local
  device) or a user-constructed remote `Hal` — never by naming a concrete
  backend type yourself. Remote instances are connected via
  `Hal::connect(endpoint)`; see m5_hal.hpp.

  @code
  m5hal::i2c::BusConfig cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
  auto bus = m5hal::M5_Hal.I2C.acquire(cfg);  // result_t<shared_ptr<IBus>>
  if (!bus) {
      // handle bus.error()
  }
  @endcode

  Note: `i2c::BusConfig` is an alias supplied by the active build variant —
  this abstract header defines only the `IBusConfig` base and the pin tags.
  Include the v2 entry header (`<M5HAL_v2.hpp>`) and the alias resolves
  to the variant's concrete config type.

  Acquiring the same wiring twice returns the same instance (identity
  acquire; see spec/design/bus_accessor.md). The concrete backend is
  selected by the build variant through the config type. Direct
  construction (`i2c::Bus bus; bus.init(cfg);`) remains the advanced path.
 */
namespace m5::hal::v2::i2c {

/*!
  @brief Strong-typed SCL pin for one-line bus-config construction.

  Wraps a global `gpio_number_t` so the constructor argument carries
  its role in the type. The constructor is `explicit` on purpose: a
  plain integer never converts into a tag, so an untagged positional
  call like `BusConfig{22, 21}` stays a compile error.
 */
struct Scl {
    constexpr explicit Scl(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*! @brief Strong-typed SDA pin. See @ref Scl. */
struct Sda {
    constexpr explicit Sda(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*!
  @brief Bus-level configuration for I2C.

  Pin fields are global `gpio_number_t` values (`int16_t` opaque IDs).
  The default `-1` is the invalid sentinel; variants resolve a non-
  negative value through the singleton `GPIOGroup` using a checked lookup
  (e.g. `m5::hal::v2::M5_Hal.Gpio.tryGetPin(num)`).

  For MCU-internal pins, callers can still pass plain literals such as
  `PIN_SCL = 21` because slot 0 is reserved for the MCU GPIO and its
  high bits are zero.

  To route SCL / SDA through an I/O expander, register the expander's
  `IGPIO` to a slot on `M5_Hal.Gpio` and build the global
  `gpio_number_t` with `makeGpioNumber(slot, local)`:
  @code
  constexpr m5::hal::v2::types::gpio_slot_t EXPANDER_SLOT = 1;
  m5::hal::v2::M5_Hal.Gpio.addGPIO(&expander, EXPANDER_SLOT);
  i2c::BusConfig bus_cfg;
  bus_cfg.pin_scl = m5::hal::v2::types::makeGpioNumber(EXPANDER_SLOT, 0);
  bus_cfg.pin_sda = m5::hal::v2::types::makeGpioNumber(EXPANDER_SLOT, 1);
  @endcode

  One-line construction passes the strong-typed pin tags — either order
  lands on the right field:
  @code
  i2c::BusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};
  @endcode
  Programmatic setup assigns the fields (`bus_cfg.pin_scl = SCL;`).
  There is deliberately no positional (untagged) pin constructor: SCL
  and SDA share one integer type, so a swapped argument order would
  compile and fail only on the wire — the tags carry the role in the
  type instead.
 */
struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_scl = -1;  ///< Global GPIO number for SCL.
    types::gpio_number_t pin_sda = -1;  ///< Global GPIO number for SDA.

    constexpr IBusConfig(void) : bus::IBusConfig{types::bus_kind_t::I2C}
    {
    }

    /*! @brief One-line pin construction (tag order is free). */
    constexpr IBusConfig(Scl scl, Sda sda)
        : bus::IBusConfig{types::bus_kind_t::I2C}, pin_scl{scl.value}, pin_sda{sda.value}
    {
    }
    constexpr IBusConfig(Sda sda, Scl scl) : IBusConfig{scl, sda}
    {
    }
};

/*!
  @brief Pin + intent acquire request for the logical acquire path.

  `acquire<CfgT>(cfg)` pins a specific backend by config TYPE -- the
  explicit route. The logical acquire states the WIRING plus an
  `AllocationIntent` (built through the kind helpers below, e.g.
  `i2c::requireHardware()`) and lets the factory pick hardware vs software at
  commit time. Identity is still the pins alone; the intent's `max_freq` hint
  is NOT an identity field, so the same wiring at a different frequency is
  still one bus.
 */
struct LogicalBusConfig {
    types::gpio_number_t pin_scl = -1;
    types::gpio_number_t pin_sda = -1;
    types::AllocationIntent intent{};  ///< Capability-based allocation request (HW/SW tier + controller).

    constexpr LogicalBusConfig(void) = default;
    constexpr LogicalBusConfig(Scl scl, Sda sda, types::AllocationIntent in = {})
        : pin_scl{scl.value}, pin_sda{sda.value}, intent{in}
    {
    }
    constexpr LogicalBusConfig(Sda sda, Scl scl, types::AllocationIntent in = {})
        : pin_scl{scl.value}, pin_sda{sda.value}, intent{in}
    {
    }
};

/*!
  @namespace m5::hal::v2::i2c::caps
  @brief i2c-local backend capability bits. Bits 0-1 (HARDWARE, LOW_POWER) are
         the cross-kind reserved bits; kind-specific caps (DMA, ...) would
         start at bit 2.
 */
namespace caps {
constexpr types::backend_caps_t HARDWARE  = types::backend_caps::HARDWARE;
constexpr types::backend_caps_t LOW_POWER = types::backend_caps::LOW_POWER;
}  // namespace caps

// --- Allocation-intent helpers (shared builders, defined in bus::) -----------
// Re-export the kind-agnostic intent builders so callers write
// i2c::requireHardware() etc.; the capability masks stay internal.
using bus::automatic;
using bus::preferController;
using bus::preferHardware;
using bus::preferLowPower;
using bus::requireController;
using bus::requireHardware;
using bus::requireLowPower;
using bus::software;

/*!
  @brief Accessor-level configuration for an I2C master target.
 */
struct MasterAccessConfig : public bus::IAccessConfig {
    /*!
      @brief Bus clock frequency in Hz.
      @note On ESP targets the value is clamped to a fail-safe ceiling
      (@c M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ, see master_clock_limit.hpp) so an
      over-high request can never drive the master peripheral into abnormal
      operation; the closest achievable clock is used instead of failing.
     */
    uint32_t freq = 100000;
    /*!
      @brief On-wire I2C timeout in milliseconds.

      Bounds how long the backend tolerates a stalled wire (the software
      backend alone applies it to the whole transfer, see below):
      arduino forwards it to `Wire.setTimeOut`; espidf uses it as the
      peripheral's SCL-stretch timeout (clamped to the peripheral's
      representable ceiling) and as the stall allowance inside the
      transaction budget handed to the vendor driver (the budget itself
      additionally covers the expected wire time of the transfer, so a
      healthy transfer longer than this value is not cut short); software
      uses it as the whole-transfer deadline of the bit-bang engine,
      measured on a shared 64-bit microsecond clock (no platform-dependent
      ceiling; the check is amortized over polls, so the timeout fires
      with a small poll-stride slack). The software backend's PER-STRETCH
      timeout derived from this value still lives in fast ticks and clamps
      to the representable ceiling (native microsecond ticks: about 35
      minutes; ESP @ 240 MHz cycle ticks: about 9 seconds). Bus-lock
      acquisition is NOT a config concern — it is a per-call argument of
      `beginAccess` / `ScopedAccess` (default: wait forever).
     */
    uint32_t wire_timeout_ms = 1000;
    /*!
      @brief I2C slave address (target).

      The default `0` is also the memory-zero initial value. Most callers
      should assign this field before issuing a transfer; a deliberate
      general-call transfer may intentionally keep it at `0`.
     */
    uint16_t i2c_addr     = 0;
    bool address_is_10bit = false;
    /*!
      @brief Register-address width -- the single source of truth for
             ALL register sugar (`writeRegister` / `readRegister`).

      `0` and `1` both mean the default 1-byte register address.
      Specify `2` only for devices with 2-byte register addresses (sent
      big-endian). Other values are outside the HAL sugar contract:
      debug builds assert, release builds return `INVALID_ARGUMENT`.

      The C++ type of the register argument (a literal, a `uint8_t` or
      `uint16_t` constant) does NOT affect the wire width -- width comes
      from this field alone. So a 2-byte-address device must set this to
      `2`; the register value is range-checked against the width.
     */
    uint8_t register_address_bytes = 0;
    /*!
      @brief Whether to emit a repeated start before the next transfer.

      Defaults to `true` so existing call sites keep their behavior.
      Concrete bus implementations are free to ignore this flag when
      the underlying hardware always emits a restart.
     */
    bool use_restart = true;

    constexpr MasterAccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::I2C}
    {
    }
};

/*!
  @brief Primary short name for the master access config.

  Master is the overwhelmingly common role, so it owns the short name;
  spell out `MasterAccessConfig` only where the contrast with a slave
  configuration matters. `AccessConfig` and `MasterAccessConfig` name the
  exact same type — the README uses the short name, the headers/samples
  the long one; they are interchangeable.

  Unlike `BusConfig` (which has tag-typed ctors, `BusConfig{Scl{22},
  Sda{21}}`), the access config is configured by **field assignment or
  aggregate initialization** — there is no positional/tag ctor. Set the
  fields you need after default construction:
  @code
  m5hal::i2c::AccessConfig dev_cfg;
  dev_cfg.i2c_addr = 0x76;
  dev_cfg.freq     = 400000;
  @endcode
 */
using AccessConfig = MasterAccessConfig;

/*!
  @brief Per-call I2C transfer descriptor.

  Holds an inline prefix buffer that carries the register address (or
  any leading byte sequence). The inline buffer spares callers the
  need to build a local array and pass a pointer. A prefix longer
  than `PREFIX_CAPACITY` is out of contract — push the excess into
  the `src` Source instead.
 */
struct TransferDesc : public bus::ITransferDesc {
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

// Forward declaration — MasterAccessor takes IBus& in its ctor;
// the full IBus definition lives below.
struct IBus;

/*!
  @brief Master-side accessor for an I2C bus.

  Holds per-target configuration (slave address, frequency, timeout)
  and serializes transfers against the underlying bus via
  lock / unlock. Convenience sugars (`write`, `read`, register
  helpers, `probe`) all wrap a single `transfer` call.
 */
struct MasterAccessor : public bus::IAccessor {
    /*!
      @brief Construct an accessor bound to an I2C bus.

      The ctor takes `IBus&` directly so kind mismatch is rejected
      at compile time; the previous `bus::IBus&` signature would have
      let an SPI bus through.
     */
    MasterAccessor(IBus& bus, const MasterAccessConfig& access_config);

    /*!
      @brief Co-owning construction from an acquired shared bus.

      The canonical path: `M5_Hal.I2C.acquire(cfg)` returns a
      `shared_ptr<IBus>`; constructing the accessor from it shares
      ownership so the bus outlives the accessor even if the caller does
      not separately retain the pointer. Kind-typed (`i2c::IBus`) so a
      non-I2C bus is a compile error.
     */
    MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config);

    /*!
      @name Unbound construction + typed bind.

      The "global driver object" pattern: construct without a bus
      (optionally with the target's config) and bind in `setup()`.
      Until `bind()` succeeds, opening an access window asserts in
      debug builds and returns `INVALID_ARGUMENT` in release builds.

      NOTE: the config ctor **copies the config by value here and then
      freezes it**. Passing a config and mutating the original variable
      afterwards (e.g. filling `i2c_addr` in `setup()` after a global
      `MasterAccessor dev{cfg};` at file scope) does NOT reach the
      accessor — it keeps the values captured at construction (default
      `i2c_addr == 0`), which compiles cleanly but talks to the wrong
      address at runtime. For deferred configuration, default-construct
      and call `setConfig()` once the values are ready.
      @{
     */
    MasterAccessor(void) = default;
    explicit MasterAccessor(const MasterAccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*!
      @brief Bind (or rebind) the accessor to an I2C bus.

      Kind-typed on purpose: passing a non-I2C bus is a compile error.
      Rejected with `INVALID_ARGUMENT` while an access window is open
      (the window holds the previous bus's lock).
     */
    m5::hal::v2::result_t<void> bind(IBus& bus);
    /*! @} */

    /*! @brief Covariant override: every accessor returns its concrete config. */
    const MasterAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    /*!
      @brief Return the underlying bus as `IBus&`.

      The kind is fixed at ctor time, so the static_cast is safe. This
      replaces ad-hoc `static_cast<IBus*>(&getBus())` at call sites.
     */
    IBus& getBus(void) const;

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
    m5::hal::v2::result_t<void> setConfig(const MasterAccessConfig& cfg);

    /*!
      @brief Core transfer start API for an open transaction.

      Internally constructs `MemorySource` / `MemorySink` and dispatches
      to `bus->transfer(TransferDesc, Source*, Sink*)`. Callers write
      register addresses (or any leading bytes) directly into
      `desc.prefix`.

      `beginTransaction()` must already be active. The call reports only
      whether the transfer was accepted/started; cumulative TX/RX counts are
      returned by `endTransaction()`. Current blocking backends still complete
      the transfer before returning, but callers must not depend on that.
     */
    m5::hal::v2::result_t<void> transfer(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                         data::DataSpan dst_bytes);

    /*!
      @brief Source/Sink overload for streaming callers.

      `tx_len` and `rx_len` bound the caller data bytes for this transfer.
      `src` / `dst` are nullable; descriptor prefix bytes are metadata and
      are not counted into either length.
     */
    m5::hal::v2::result_t<void> transfer(const TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                         size_t rx_len);

    /*!
      @brief Open a bus-ownership transaction.

      This synchronous surface mirrors SPI's transaction API but, for I2C,
      currently represents only a bus access window. Wire-level START / STOP
      remain owned by each `transfer` call until the low-level transfer API is
      switched to asynchronous start semantics.
     */
    m5::hal::v2::result_t<void> beginTransaction(uint32_t timeout_ms = types::TIMEOUT_FOREVER);

    /*!
      @brief Close the transaction and return cumulative transfer totals.
     */
    m5::hal::v2::result_t<bus::TransferTotals> endTransaction(void);
    bool transferBusy(void);
    m5::hal::v2::result_t<void> waitTransfer(void);

    bool inTransaction(void) const
    {
        return _transaction_depth != 0;
    }

    /*!
      @name Source/Sink write / read — the canonical data-exchange form.

      `write` pulls up to `len` bytes from `src`; `read` fills up to
      `len` bytes into `dst`. These delegate to `transfer` (the Source/
      Sink core) with an empty `TransferDesc`. The span and raw-pointer
      overloads below are SUGAR that wrap a `MemorySource` / `MemorySink`
      and reach the same core, so `write(Source&, len)` / `read(Sink&,
      len)` carry the same shape here as on `spi` / `uart`.
      @{
     */
    m5::hal::v2::result_t<size_t> write(data::Source& src, size_t len);
    m5::hal::v2::result_t<size_t> read(data::Sink& dst, size_t len);
    /*! @} */

    /*!
      @name Span-based write / read sugars.

      Internally open a transaction, call `transfer` with an empty
      `TransferDesc`, then close the transaction and use its totals.
      @{
     */
    m5::hal::v2::result_t<size_t> write(data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> read(data::DataSpan dst_bytes);
    /*! @} */

    /*!
      @name Raw `uint8_t* + size_t` overloads.

      Save the caller a span construction when handing in a C array.
      These forward to the span overload. There is no overload-resolution
      ambiguity because `uint8_t*` and the `data::DataSpan` /
      `data::ConstDataSpan` value types are distinct.
      @{
     */
    m5::hal::v2::result_t<size_t> write(const uint8_t* src, size_t len);
    m5::hal::v2::result_t<size_t> read(uint8_t* dst, size_t len);
    /*! @} */

    /*!
      @name Register read / write sugars.

      Read or write N bytes anchored at a register address. The register
      number is just a value -- `readRegister(0x00)` and a
      `static constexpr uint8_t REG = 0xD0;` both work, and the C++ TYPE
      of the argument does NOT affect the wire width.

      Address width has a SINGLE source of truth:
      `MasterAccessConfig::register_address_bytes` (`0` / `1` = 1 byte,
      `2` = 2 bytes), set once when the device is configured -- it is a
      fixed property of the device's register map. A 2-byte width is
      transmitted MSB first (big-endian), per the I2C convention. The
      register value is range-checked against the width (a register
      `> 0xFF` on a 1-byte device returns `INVALID_ARGUMENT`); an
      unsupported width asserts in debug and returns `INVALID_ARGUMENT`
      in release.

      A 2-byte-address device (some EEPROMs / sensors) MUST therefore set
      `register_address_bytes = 2`; the majority use the 1-byte default.
      There is no type-driven or per-call width, so width can never be
      silently wrong or inconsistent between call styles.

      Value-side size and byte order are the caller's responsibility
      (compose N bytes with `data::ConstDataSpan` / `data::DataSpan`, or
      stream via `data::Source` / `data::Sink`). Big- / little-endian
      value helpers belong in M5UU's `M5UnitComponent` layer, not in HAL.
      @{
     */
    m5::hal::v2::result_t<size_t> writeRegister(int reg, data::ConstDataSpan value)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        auto r = transferSync(desc.value(), value, data::DataSpan{});
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return r->tx;
    }
    m5::hal::v2::result_t<size_t> writeRegister(int reg, uint8_t value)
    {
        return writeRegister(reg, data::ConstDataSpan{&value, 1});
    }
    m5::hal::v2::result_t<size_t> writeRegister(int reg, const uint8_t* src, size_t len)
    {
        return writeRegister(reg, data::ConstDataSpan{src, len});
    }
    m5::hal::v2::result_t<size_t> readRegister(int reg, data::DataSpan dst)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        auto totals = transferSync(desc.value(), data::ConstDataSpan{}, dst);
        if (!totals.has_value()) {
            return m5::stl::make_unexpected(totals.error());
        }
        return totals->rx;
    }
    m5::hal::v2::result_t<size_t> readRegister(int reg, uint8_t* dst, size_t len)
    {
        return readRegister(reg, data::DataSpan{dst, len});
    }
    m5::hal::v2::result_t<uint8_t> readRegister(int reg)
    {
        uint8_t v;
        auto r = readRegister(reg, data::DataSpan{&v, 1});
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return v;
    }
    /*!
      @brief Source/Sink register overloads (streaming address).

      Stream up to `len` bytes to / from a register; the address width
      comes from `register_address_bytes`, as with the other register
      overloads.
     */
    m5::hal::v2::result_t<size_t> writeRegister(int reg, data::Source& src, size_t len)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        auto r = transferSync(desc.value(), &src, len, nullptr, 0);
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return r->tx;
    }
    m5::hal::v2::result_t<size_t> readRegister(int reg, data::Sink& dst, size_t len)
    {
        auto desc = makeLiteralRegisterDesc(reg);
        if (!desc.has_value()) {
            return m5::stl::make_unexpected(desc.error());
        }
        auto totals = transferSync(desc.value(), nullptr, 0, &dst, len);
        if (!totals.has_value()) {
            return m5::stl::make_unexpected(totals.error());
        }
        return totals->rx;
    }
    /*! @} */

    /*!
      @brief Probe for the presence of the configured slave.

      Sends a zero-byte write: `OK` if the target ACKs, `I2C_NO_ACK`
      (or another bus error) otherwise. Promotes the legacy
      "empty-write" idiom to a named API.
     */
    m5::hal::v2::result_t<void> probe(void);

protected:
    m5::hal::v2::result_t<bus::TransferTotals> transferSync(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                                            data::DataSpan dst_bytes);
    m5::hal::v2::result_t<bus::TransferTotals> transferSync(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                            data::Sink* dst, size_t rx_len);
    m5::hal::v2::result_t<void> startTransfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                              data::Sink* dst, size_t rx_len);

    m5::hal::v2::result_t<TransferDesc> makeLiteralRegisterDesc(int reg) const
    {
        if (reg < 0) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        const uint8_t width = _access_config.register_address_bytes == 0 ? 1 : _access_config.register_address_bytes;
        if (width == 1) {
            if (reg > 0xFF) {
                return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
            }
            return TransferDesc{static_cast<uint8_t>(reg)};
        }
        if (width == 2) {
            if (reg > 0xFFFF) {
                return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
            }
            return TransferDesc{static_cast<uint16_t>(reg)};
        }
        M5HAL_ASSERT(false, "MasterAccessConfig::register_address_bytes supports only 0, 1, or 2");
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }

    MasterAccessConfig _access_config;
    bus::TransferTotals _transaction_totals;
    data::MemorySource _span_src;
    data::MemorySink _span_dst;
    error::error_t _transaction_error = error::error_t::OK;
    uint32_t _transaction_depth       = 0;
};

//-------------------------------------------------------------------------

/*!
  @brief Concrete I2C bus base.

  Variants derive from `IBus` and override `transfer`; the inline
  `probe` sugar uses a stack-local accessor so callers can check for
  a device without building one themselves.
 */
struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    /*!
      @brief Probe a single device without building an accessor.

      Internally constructs a stack-allocated `MasterAccessor` as a
      sentinel and uses it as the lock owner, so the probe contends for
      the bus through `Bus::lock` like any other accessor (waits up to
      `timeout_ms`, fails with `TIMEOUT_ERROR`). One probe includes
      (sentinel ctor + `beginAccess` + transfer probe path +
      `endAccess` + sentinel dtor); the ctor is inline and trivially
      cheap.

      Callers that want to customize `freq` / `timeout_ms` should
      build an `MasterAccessor` and use `setConfig` instead. The
      default `timeout_ms = 50` targets I2C scan loops (waiting one
      second per NACK is impractical) and is intentionally smaller
      than `MasterAccessConfig`'s default of 1000 ms.
     */
    m5::hal::v2::result_t<void> probe(uint16_t i2c_addr, uint32_t freq = 100000, uint32_t timeout_ms = 50);

    /*!
      @brief Core I2C transfer entry point.

      Per-call metadata travels through `TransferDesc`. Leading bytes
      (register address, ...) live in the inline `desc.prefix` /
      `desc.prefix_len` buffer, sparing callers a local array and a
      pointer hand-off.

      `src` and `dst` are nullable: `nullptr` means "no data for this
      segment" (write-only or read-only).

      Probe (empty transfer) contract: when `desc.prefix_len == 0`,
      `src == nullptr` (or `src` is already EOF), and `dst == nullptr`,
      this issues a single address+W on the wire and inspects the ACK
      bit. ACK -> `OK`; NACK -> `I2C_NO_ACK`. Every variant
      implementation MUST honor this path because `Accessor::probe`
      depends on it.

      `owner` identifies the calling accessor. It is INFORMATIONAL: the
      backend does NOT verify that `owner` currently holds the bus lock. The
      lock-owner invariant is enforced only by the `Accessor` sugar, which
      takes the lock before forwarding `this`. Calling this low-level entry
      directly therefore bypasses lock ownership and is unsafe unless the
      caller already holds the bus lock; keep an accessor alive and pass
      `&accessor`. (Documented as informational, not enforced.)

      `tx_len` / `rx_len` bound the caller data phases independently.

      The return value reports only start/completion success for the backend.
      Accessors count caller-provided Source/Sink progress and return
      cumulative totals from `endTransaction()`. `desc.prefix` bytes are
      driven on the wire but not counted. The default implementation returns
      `NOT_IMPLEMENTED`; every variant overrides it.
     */
    virtual m5::hal::v2::result_t<void> transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg,
                                                 const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                 data::Sink* dst, size_t rx_len);
    virtual m5::hal::v2::result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner,
                                                                    const MasterAccessConfig& cfg);
    virtual bool transferBusy(bus::IAccessor* owner);

protected:
    IBusConfig _config;
};

/*!
  @brief Maps a variant `BusConfig_<variant>` to its backend `Bus_<variant>`.

  Undefined primary on purpose: passing a config type without a
  specialization to `Bus::init` is an incomplete-type compile error
  (the same type-safety stance as the typed `init` it replaces). Each
  variant header specializes this next to its `Bus_<variant>`.
 */
template <class CfgT>
struct BackendFor;

/*!
  @brief Per-kind Traits for the shared facade and local backend adapter.

  Supplies the concrete I2C types, kind tag, capability, backend selector, and
  the 2-pin (SCL/SDA) projection that the kind-neutral `bus::ManagedBusFacade`
  and `bus::LocalKindAdapter` consume.
  `Bus` is forward-declared at namespace scope so `BusType` names the public
  `i2c::Bus`, not a nested type.
 */
struct Bus;  // the facade, defined just below

struct BusTraits {
    using IBus               = i2c::IBus;
    using IBusConfig         = i2c::IBusConfig;
    using LogicalBusConfig   = i2c::LogicalBusConfig;
    using MasterAccessConfig = i2c::MasterAccessConfig;
    using MasterAccessor     = i2c::MasterAccessor;
    using TransferDesc       = i2c::TransferDesc;
    using BusType            = Bus;

    static constexpr types::bus_kind_t KIND              = types::bus_kind_t::I2C;
    static constexpr types::backend_caps_t CAPS_HARDWARE = caps::HARDWARE;
    /*! @brief I2C uses the managed policy: `BusView::hardwareInUse()` forwards
               to the backend (see spec/design/bus_accessor.md §managed policy). */
    static constexpr bool MANAGED_ALLOCATION = true;

    template <class CfgT>
    using BackendFor = i2c::BackendFor<CfgT>;

    static void applyAdopt(IBusConfig& cfg, const LogicalBusConfig& logical)
    {
        cfg.pin_scl = logical.pin_scl;
        cfg.pin_sda = logical.pin_sda;
    }
    static void fillLogical(LogicalBusConfig& out, const IBusConfig& cfg)
    {
        out.pin_scl = cfg.pin_scl;
        out.pin_sda = cfg.pin_sda;
    }
    static bus::IdentityKey identityFromConfig(const IBusConfig& cfg)
    {
        return bus::IdentityKey::fromPins({cfg.pin_scl, cfg.pin_sda});
    }
    static bus::IdentityKey identityFromLogical(const LogicalBusConfig& req)
    {
        return bus::IdentityKey::fromPins({req.pin_scl, req.pin_sda});
    }
    static bool configCompatible(const IBusConfig& current, const IBusConfig& requested)
    {
        return current.pin_scl == requested.pin_scl && current.pin_sda == requested.pin_sda;
    }
};

/*!
  @brief Runtime facade for an I2C bus (the unsuffixed `i2c::Bus`).

  All machinery lives in `bus::ManagedBusFacade<BusTraits>`: the mutex +
  lock/unlock, the accessor binding, `getConfig`/`probe`, the swappable backend
  behind a `unique_ptr`, the query mirror, and the hot-swap seam.
  This thin derived type just fixes the public name `i2c::Bus` and the I2C
  Traits. `init` selects the backend from the config type via `BackendFor`, so
  `i2c::Bus bus; bus.init(cfg);` keeps working.
 */
struct Bus : public bus::ManagedBusFacade<BusTraits> {
    using bus::ManagedBusFacade<BusTraits>::transfer;

    result_t<void> transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg, const TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        return forwardBackend([&](IBus& b) { return b.transfer(owner, cfg, desc, src, tx_len, dst, rx_len); });
    }
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const MasterAccessConfig& cfg) override
    {
        return forwardBackend([&](IBus& b) { return b.waitTransfer(owner, cfg); });
    }
    bool transferBusy(bus::IAccessor* owner) override
    {
        auto* b = backend();
        return b != nullptr && b->transferBusy(owner);
    }
};

//-------------------------------------------------------------------------
// bind() is defined below the concrete IBus: at the accessor's point of
// declaration the kind IBus is still an incomplete type, so the
// derived-to-base conversion _bindBus needs is not visible yet.
inline m5::hal::v2::result_t<void> MasterAccessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    _bindBus(bus);
    return {};
}

/*!
  @brief Typed I2C view delegating registry and allocation to the HAL backend.

  Shares the acquire / logical-acquire / commit / release spine with every
  other kind through `bus::BusViewCore<BusTraits>` (see bus/bus_view.hpp);
  this derived type adds only the I2C-specific `createBusConfig()` pin
  overloads. The typed acquire path uses the backend's registry directly so
  the concrete config type still selects the local variant backend. The
  logical acquire and commit paths delegate through `bus::IHalBackend`,
  allowing the same BusView surface to point at local or future remote
  backends.
 */
class BusView : public bus::BusViewCore<BusTraits> {
public:
    using bus::BusViewCore<BusTraits>::BusViewCore;

    LogicalBusConfig createBusConfig(Scl scl, Sda sda, types::AllocationIntent intent = {}) const
    {
        return {scl, sda, intent};
    }
    LogicalBusConfig createBusConfig(Sda sda, Scl scl, types::AllocationIntent intent = {}) const
    {
        return {scl, sda, intent};
    }
};

}  // namespace m5::hal::v2::i2c

#endif
