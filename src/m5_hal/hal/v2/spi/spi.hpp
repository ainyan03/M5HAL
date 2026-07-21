// SPDX-License-Identifier: MIT
#ifndef M5_HAL_SPI_SPI_HPP_
#define M5_HAL_SPI_SPI_HPP_

#include "../bus/bus.hpp"
#include "../bus/bus_view.hpp"
#include "../bus/hal_backend.hpp"
#include "../bus/managed_bus.hpp"
#include "../bus/managed_facade.hpp"
#include "../bus/registry.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../types.hpp"

#include <atomic>
#include <memory>
#include <new>

/*!
  @namespace m5::hal::v2::spi
  @brief SPI bus, master accessor, transfer descriptor, command/address sugars.

  ## Getting a bus (entry point)

  Buses are obtained from a `Hal` facade instance — `M5_Hal` (the local
  device) or a user-constructed remote `Hal` — never by naming a concrete
  backend type yourself. Remote instances are connected via
  `Hal::connect(endpoint)`; see m5_hal.hpp.

  @code
  m5hal::spi::BusConfig cfg{m5hal::spi::Clk{18}, m5hal::spi::Mosi{23}, m5hal::spi::Miso{19}};
  auto bus = m5hal::M5_Hal.SPI.acquire(cfg);  // result_t<shared_ptr<IBus>>
  if (!bus) {
      // handle bus.error()
  }
  @endcode

  Note: `spi::BusConfig` is an alias supplied by the active build variant —
  this abstract header defines only the `IBusConfig` base and the pin tags.
  Include the v2 entry header (`<M5HAL_v2.hpp>`) and the alias resolves
  to the variant's concrete config type.

  Acquiring the same wiring twice returns the same instance (identity
  acquire; see spec/design/bus_accessor.md). The concrete backend is
  selected by the build variant through the config type. Direct
  construction (`spi::Bus bus; bus.init(cfg);`) remains the advanced path.
 */
namespace m5::hal::v2::spi {

/*!
  @brief Strong-typed CLK pin for one-line bus-config construction.

  Wraps a global `gpio_number_t` so the constructor argument carries its
  role in the type. `explicit` on purpose: a plain integer never converts
  into a tag, so an untagged positional call stays a compile error. CLK /
  MOSI / MISO share one integer type, so the strong tags reject an untagged
  positional call and a wrong-role tag at the fixed constructor positions.
  They do NOT validate that the integer inside each tag is the pin actually
  wired to that role (e.g. `Mosi{PIN_MISO}` still compiles). The constructor
  parameter order is fixed (Clk, Mosi, Miso).
 */
struct Clk {
    constexpr explicit Clk(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*! @brief Strong-typed MOSI pin. See @ref Clk. */
struct Mosi {
    constexpr explicit Mosi(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*! @brief Strong-typed MISO pin. See @ref Clk. */
struct Miso {
    constexpr explicit Miso(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*!
  @brief Bus-level configuration for SPI.

  Covers plain SPI as well as QSPI / OSPI configurations. Every pin
  is a `gpio_number_t` (`int16_t`); the default `-1` is the invalid
  sentinel. Variants resolve a non-negative value during `init()`
  via the checked creation-domain `GPIOGroup::tryGetPin(num)` lookup (the default
  domain is `M5_Hal.Gpio`; the same
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
    /*!
      @brief Tag-typed core pins (CLK / MOSI / MISO). Strong tags keep a
      swapped wiring from compiling. QSPI data lines (`pin_d2..d7`) and the
      bus-wide `pin_dc` are set by field assignment afterwards, matching the
      i2c / uart convention of tagging only the identity pins.
     */
    constexpr IBusConfig(Clk clk, Mosi mosi, Miso miso)
        : bus::IBusConfig{types::bus_kind_t::SPI}, pin_clk{clk.value}, pin_mosi{mosi.value}, pin_miso{miso.value}
    {
    }
    /*! @brief MISO-less wiring: write-only by default; single-lane half duplex may receive on MOSI. */
    constexpr IBusConfig(Clk clk, Mosi mosi)
        : bus::IBusConfig{types::bus_kind_t::SPI}, pin_clk{clk.value}, pin_mosi{mosi.value}
    {
    }
};

/*! @brief Variant-independent portable SPI bus configuration. */
using BusConfig = IBusConfig;

/*!
  @brief Pin + intent acquire request for the logical acquire path.

  `acquire<CfgT>(cfg)` pins a specific backend by config TYPE -- the
  explicit route. The logical acquire states the WIRING (CLK / MOSI / MISO, the
  3-wire core that is the bus identity) plus an `AllocationIntent` (built through
  the kind helpers below, e.g. `spi::requireHardware()`) and lets the factory
  pick hardware vs software at commit time. Identity is still the three core
  pins alone; the intent is not an identity field, so the same wiring with a
  different intent is still one bus.
 */
struct LogicalBusConfig {
    types::gpio_number_t pin_clk  = -1;
    types::gpio_number_t pin_mosi = -1;
    types::gpio_number_t pin_miso = -1;
    types::AllocationIntent intent{};  ///< Capability-based allocation request (HW/SW tier + controller).

    constexpr LogicalBusConfig(void) = default;
    constexpr LogicalBusConfig(Clk clk, Mosi mosi, Miso miso, types::AllocationIntent in = {})
        : pin_clk{clk.value}, pin_mosi{mosi.value}, pin_miso{miso.value}, intent{in}
    {
    }
    /*! @brief MISO-less wiring: the bus is identified by CLK / MOSI, with MISO
               at -1. It is write-only unless a supported half-duplex mode shares MOSI. */
    constexpr LogicalBusConfig(Clk clk, Mosi mosi, types::AllocationIntent in = {})
        : pin_clk{clk.value}, pin_mosi{mosi.value}, intent{in}
    {
    }
};

/*!
  @namespace m5::hal::v2::spi::caps
  @brief SPI backend capability bits. Bits 0-1 are reserved across kinds;
         SPI-local capabilities start at bit 2.
 */
namespace caps {
constexpr types::backend_caps_t HARDWARE = types::backend_caps::HARDWARE;
/*! @brief Backend can receive on MOSI during a MISO-less half-duplex transfer. */
constexpr types::backend_caps_t MOSI_SHARED_RX = 1u << 2;
}  // namespace caps

// --- Allocation-intent helpers (shared builders, defined in bus::) -----------
// Re-export the kind-agnostic intent builders so callers write
// spi::requireHardware() etc.; the capability masks stay internal.
using bus::automatic;
using bus::preferController;
using bus::preferHardware;
using bus::requireController;
using bus::requireHardware;
using bus::software;

/*!
  @brief Require MISO-less half-duplex RX on the MOSI pin.

  Composes with the ordinary allocation helpers. For example,
  `requireMosiSharedRx(preferHardware())` prefers a capable hardware
  controller and falls back to software, while
  `requireMosiSharedRx(requireHardware())` forbids that fallback.

  This requirement must be declared when the bus is acquired. Discovering it
  from an accessor after `commitBuses()` would be too late to choose a backend
  without replaying a transfer that may already have changed the wire.
 */
constexpr types::AllocationIntent requireMosiSharedRx(types::AllocationIntent intent = {})
{
    intent.require |= caps::MOSI_SHARED_RX;
    return intent;
}

/*!
  @brief Data-path mode of an SPI transfer.
 */
enum class SpiDataMode {
    HalfDuplex,           ///< Half duplex; with no MISO, supported backends receive on MOSI.
    FullDuplex,           ///< Full duplex.
    HalfDuplexWithDcPin,  ///< Half duplex with separate D/C; with no MISO, receive on MOSI.
    FullDuplexWithDcPin,  ///< Full duplex with a separate D/C pin.
    HalfDuplexWithDcBit,  ///< Half duplex with in-band D/C; with no MISO, receive on MOSI.
    FullDuplexWithDcBit,  ///< Full duplex with an in-band D/C bit (9-bit SPI).
    DualOutput,
    DualIo,
    QuadOutput,
    QuadIo,
    OctalOutput,
    OctalIo,
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
    spi_data_mode_t spi_data_mode = spi_data_mode_t::FullDuplex;  ///< Default full duplex.
    uint8_t spi_mode : 2;                                         ///< SPI mode 0..3, zero-initialized by the ctor.
    uint8_t spi_order : 1;                                        ///< 0 = MSB first, zero-initialized by the ctor.
    uint8_t spi_command_length    = 0;                            ///< Command-phase length in bits.
    uint8_t spi_address_length    = 0;                            ///< Address-phase length in bits.
    uint8_t spi_read_dummy_cycle  = 0;                            ///< Dummy cycles inserted before the read data phase.
    uint8_t spi_write_dummy_cycle = 0;  ///< Dummy cycles inserted before the write data phase.

    // Bitfields are zero-initialized via the mem-initializer list for C++17
    // compatibility.
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
      matching the `HalfDuplexWithDcPin` / `HalfDuplexWithDcBit` enumerators
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
        spi_data_mode      = spi_data_mode_t::HalfDuplexWithDcPin;
        spi_command_length = 8;
        return *this;
    }
    /*! @brief D/C as the 9th in-band bit (datasheet: 3-wire / 3-line
               serial). No D/C pin; the data-path mode carries the bit. */
    MasterAccessConfig& setupWithDCBit(void)
    {
        pin_dc             = -1;
        spi_data_mode      = spi_data_mode_t::HalfDuplexWithDcBit;
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
  cycles), maps `beginAccess` / `endAccess` to the physical CS scope,
  and exposes the `write` / `read` / `writeCommand*` / `read*` sugars
  that all funnel into a single `transfer` call.
 */
struct MasterAccessor : public bus::IAccessor {
    MasterAccessor(IBus& bus, const MasterAccessConfig& access_config);

    /*!
      @brief Co-owning construction from an acquired shared bus.

      `M5_Hal.SPI.acquire(cfg)` returns a `shared_ptr<IBus>`; constructing
      the accessor from it shares ownership so the bus outlives the
      accessor. Kind-typed (`spi::IBus`) so a non-SPI bus is a compile
      error.
     */
    MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config);

    /*!
      @name Unbound construction + typed bind.

      Same contract as the I2C accessor: the unbound gate sits on the
      Access opener (`beginAccess`).
      @{
     */
    MasterAccessor(void) : _context{makeOperationContext(MasterAccessConfig{})}
    {
    }
    explicit MasterAccessor(const MasterAccessConfig& access_config) : _context{makeOperationContext(access_config)}
    {
    }
    /*! @brief Bind (or rebind) to an SPI bus; rejected while a window is open. */
    [[nodiscard]] m5::hal::v2::result_t<void> bind(IBus& bus);
    /*! @} */

    const MasterAccessConfig& getConfig(void) const override
    {
        return _context.config;
    }
    /*! @brief Return the underlying bus as `IBus&` (kind fixed at ctor). */
    IBus& getBus(void) const;

    /*!
      @brief Replace the per-target configuration.

      Fails with `INVALID_STATE` while an Access
      window is open — swapping the config mid-transfer would leave
      the active Access undefined (same contract as the I2C
      accessor).
     */
    [[nodiscard]] m5::hal::v2::result_t<void> setConfig(const MasterAccessConfig& cfg);

    /*!
      @brief Transfer one I/O within an open Access and return its totals.
     */
    m5::hal::v2::result_t<bus::TransferTotals> transfer(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                                        data::DataSpan dst_bytes);
    /*!
      @brief Source/Sink overload for streaming callers.
      @param tx_len  Caller TX bytes requested from `src`.
      @param rx_len  Caller RX bytes requested into `dst`.

      `beginAccess()` must already be active. The call waits for this
      individual transfer and returns its TX/RX totals. Public sugars open a
      temporary Access when inactive and borrow an already-active Access.
     */
    m5::hal::v2::result_t<bus::TransferTotals> transfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                        data::Sink* dst, size_t rx_len);
    m5::hal::v2::result_t<bus::TransferTotals> transfer(const TransferDesc& desc, data::Source* src, data::Sink* dst,
                                                        size_t len)
    {
        return transfer(desc, src, len, dst, len);
    }
    /*!
      @name Access lifecycle (bus ownership + configuration + CS scope).
      @{
     */
    [[nodiscard]] m5::hal::v2::result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    [[nodiscard]] m5::hal::v2::result_t<void> endAccess(uint32_t timeout_ms = 1000);
    bool transferBusy(void);
    m5::hal::v2::result_t<bus::TransferStatus> getLastTransferStatus(void) const;
    /*! @} */

    /*!
      @name Plain write / read sugars (span, Source/Sink, raw pointer).
      @{
     */
    m5::hal::v2::result_t<size_t> write(data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> write(data::Source& src, size_t len);
    m5::hal::v2::result_t<size_t> read(data::DataSpan dst_bytes);
    m5::hal::v2::result_t<size_t> read(data::Sink& dst, size_t len);
    m5::hal::v2::result_t<size_t> write(const uint8_t* src, size_t len);
    m5::hal::v2::result_t<size_t> read(uint8_t* dst, size_t len);
    /*! @} */

    /*!
      @name Command / address sugars for display-like SPI peripherals.
      @{
     */
    m5::hal::v2::result_t<size_t> writeCommand(data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> writeCommand(uint32_t command);
    m5::hal::v2::result_t<size_t> writeCommandAddress(uint32_t command, uint32_t address);
    m5::hal::v2::result_t<size_t> writeCommandData(data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> writeCommandData(uint32_t command, data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> writeCommandData(uint32_t command, data::Source& src, size_t len);
    m5::hal::v2::result_t<size_t> writeCommandAddressData(uint32_t command, uint32_t address,
                                                          data::ConstDataSpan src_bytes);
    m5::hal::v2::result_t<size_t> writeCommandAddressData(uint32_t command, uint32_t address, data::Source& src,
                                                          size_t len);
    m5::hal::v2::result_t<size_t> readCommandData(uint32_t command, data::DataSpan dst_bytes);
    m5::hal::v2::result_t<size_t> readCommandData(uint32_t command, data::Sink& dst, size_t len);
    m5::hal::v2::result_t<size_t> readCommandAddressData(uint32_t command, uint32_t address, data::DataSpan dst_bytes);
    m5::hal::v2::result_t<size_t> readCommandAddressData(uint32_t command, uint32_t address, data::Sink& dst,
                                                         size_t len);
    /*! @brief Drive `count` dummy clock cycles (`count` <= 255,
               larger values fail with `INVALID_ARGUMENT`). */
    m5::hal::v2::result_t<size_t> sendDummyClock(size_t count);
    /*! @} */

protected:
    m5::hal::v2::result_t<bus::TransferTotals> transferSync(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                                            data::DataSpan dst_bytes);
    m5::hal::v2::result_t<bus::TransferTotals> transferSync(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                            data::Sink* dst, size_t rx_len);
    m5::hal::v2::result_t<bus::TransferTotals> startTransfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                             data::Sink* dst, size_t rx_len);

    bus::OperationContext<MasterAccessConfig> _context;
    bus::TransferStatus _last_transfer_status;
    data::MemorySource _span_src;
    data::MemorySink _span_dst;
    uint32_t _next_transfer_id = 0;
};

//-------------------------------------------------------------------------

/*!
  @brief Concrete SPI bus base.

  Signature is aligned with `IBus` so accessor sugars converge here. Public
  entries validate the Context and concrete variants override the protected
  `*Backend` hooks. Default hooks return `UNSUPPORTED`.
 */
struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    m5::hal::v2::result_t<void> beginOperation(bus::OperationContext<MasterAccessConfig>& context);
    m5::hal::v2::result_t<void> endOperation(bus::OperationContext<MasterAccessConfig>& context);
    /*!
      @brief Core SPI transfer entry point.

      Return-value contract:
      the bus reports whether the transfer was accepted/started. The Accessor
      waits through `waitTransfer()` and returns per-I/O totals. The command /
      address phases encoded by `desc` are NOT counted.

      Read-family accessor sugars (`read` / `readCommandData` /
      `readCommandAddressData`) surface this received count. Write-family
      sugars (`write` / `writeCommand*` / `writeCommandAddressData*`) are
      full-or-fail and surface the caller-provided transmitted data count as
      `result_t<size_t>`.

      `src` / `dst` are nullable (`nullptr` = no data for that direction).
      `tx_len` / `rx_len` bound the caller data phases independently.
      The default implementation returns `UNSUPPORTED`.
     */
    m5::hal::v2::result_t<void> transfer(bus::OperationContext<MasterAccessConfig>& context, const TransferDesc& desc,
                                         data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len);
    m5::hal::v2::result_t<bus::TransferTotals> waitTransfer(bus::OperationContext<MasterAccessConfig>& context);
    bool transferBusy(bus::OperationContext<MasterAccessConfig>& context);

protected:
    virtual m5::hal::v2::result_t<void> beginOperationBackend(bus::OperationContext<MasterAccessConfig>& context);
    virtual m5::hal::v2::result_t<void> endOperationBackend(bus::OperationContext<MasterAccessConfig>& context);
    virtual m5::hal::v2::result_t<void> transferBackend(bus::OperationContext<MasterAccessConfig>& context,
                                                        const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                        data::Sink* dst, size_t rx_len);
    virtual m5::hal::v2::result_t<bus::TransferTotals> waitTransferBackend(
        bus::OperationContext<MasterAccessConfig>& context);
    virtual bool transferBusyBackend(bus::OperationContext<MasterAccessConfig>& context);

    static result_t<void> beginOperationOn(IBus& backend, bus::OperationContext<MasterAccessConfig>& context)
    {
        return backend.beginOperationBackend(context);
    }
    static result_t<void> endOperationOn(IBus& backend, bus::OperationContext<MasterAccessConfig>& context)
    {
        return backend.endOperationBackend(context);
    }
    static result_t<void> transferOn(IBus& backend, bus::OperationContext<MasterAccessConfig>& context,
                                     const TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                     size_t rx_len)
    {
        return backend.transferBackend(context, desc, src, tx_len, dst, rx_len);
    }
    static result_t<bus::TransferTotals> waitTransferOn(IBus& backend,
                                                        bus::OperationContext<MasterAccessConfig>& context)
    {
        return backend.waitTransferBackend(context);
    }
    static bool transferBusyOn(IBus& backend, bus::OperationContext<MasterAccessConfig>& context)
    {
        return backend.transferBusyBackend(context);
    }

    IBusConfig _config;
    bus::OperationSlot _operation_slot;
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

template <class Policy>
struct NativeProvider;

/*!
  @brief Per-kind Traits for the shared facade and local backend adapter.

  Supplies the concrete SPI types, kind tag, capability, backend selector, and
  the 3-pin (CLK/MOSI/MISO) projection that the kind-neutral
  `bus::ManagedBusFacade` and `bus::LocalKindAdapter` consume.
  `Bus` is forward-declared at
  namespace scope so `BusType` names the public `spi::Bus`, not a nested type.
 */
struct Bus;  // the facade, defined just below

struct BusTraits {
    using IBus               = spi::IBus;
    using IBusConfig         = spi::IBusConfig;
    using LogicalBusConfig   = spi::LogicalBusConfig;
    using MasterAccessConfig = spi::MasterAccessConfig;
    using MasterAccessor     = spi::MasterAccessor;
    using TransferDesc       = spi::TransferDesc;
    using BusType            = Bus;

    static constexpr types::bus_kind_t KIND = types::bus_kind_t::SPI;
    // The current poolable hardware factory is ESP-IDF SPI master; every host
    // it exposes supports SPI_DEVICE_3WIRE. A future factory that differs must
    // supply per-controller caps through LocalKindAdapter::Topology.
    static constexpr types::backend_caps_t CAPS_HARDWARE = caps::HARDWARE | caps::MOSI_SHARED_RX;
    /*! @brief SPI uses the managed policy: `BusView::hardwareInUse()` forwards
               to the backend (see spec/design/bus_accessor.md §managed policy). */
    static constexpr bool MANAGED_ALLOCATION = true;

    template <class Policy>
    using NativeProvider = spi::NativeProvider<Policy>;

    static void applyAdopt(IBusConfig& cfg, const LogicalBusConfig& logical)
    {
        cfg.pin_clk  = logical.pin_clk;
        cfg.pin_mosi = logical.pin_mosi;
        cfg.pin_miso = logical.pin_miso;
    }
    static void fillLogical(LogicalBusConfig& out, const IBusConfig& cfg)
    {
        out.pin_clk  = cfg.pin_clk;
        out.pin_mosi = cfg.pin_mosi;
        out.pin_miso = cfg.pin_miso;
    }
    static bus::ResourceKey identityFromConfig(const IBusConfig& cfg)
    {
        return bus::ResourceKey::fromPins(KIND, {cfg.pin_clk, cfg.pin_mosi, cfg.pin_miso});
    }
    static bus::ResourceKey identityFromLogical(const LogicalBusConfig& req)
    {
        return bus::ResourceKey::fromPins(KIND, {req.pin_clk, req.pin_mosi, req.pin_miso});
    }
    static bool configCompatible(const IBusConfig& current, const IBusConfig& requested)
    {
        return current.pin_clk == requested.pin_clk && current.pin_dc == requested.pin_dc &&
               current.pin_mosi == requested.pin_mosi && current.pin_miso == requested.pin_miso &&
               current.pin_d2 == requested.pin_d2 && current.pin_d3 == requested.pin_d3 &&
               current.pin_d4 == requested.pin_d4 && current.pin_d5 == requested.pin_d5 &&
               current.pin_d6 == requested.pin_d6 && current.pin_d7 == requested.pin_d7;
    }
};

/*!
  @brief Runtime facade for an SPI bus (the unsuffixed `spi::Bus`).

  Shares all of `bus::ManagedBusFacade<BusTraits>` (lock + accessor binding,
  swappable backend, query mirror, the hot-swap seam). The only
  SPI-specific addition is the physical operation scope: `beginOperation` /
  `endOperation` are forwarded to the live backend, where they apply the
  device configuration and assert/deassert CS.
 */
struct Bus : public bus::ManagedBusFacade<BusTraits> {
    [[nodiscard]] result_t<void> init(const IBusConfig& config);

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<MasterAccessConfig>& context) override
    {
        return this->forwardBackend([&](IBus& backend) { return beginOperationOn(backend, context); });
    }

    result_t<void> endOperationBackend(bus::OperationContext<MasterAccessConfig>& context) override
    {
        return this->forwardBackend([&](IBus& backend) { return endOperationOn(backend, context); });
    }

    result_t<void> transferBackend(bus::OperationContext<MasterAccessConfig>& context, const TransferDesc& desc,
                                   data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        return this->forwardBackend(
            [&](IBus& backend) { return transferOn(backend, context, desc, src, tx_len, dst, rx_len); });
    }

    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<MasterAccessConfig>& context) override
    {
        return this->forwardBackend([&](IBus& backend) { return waitTransferOn(backend, context); });
    }

    bool transferBusyBackend(bus::OperationContext<MasterAccessConfig>& context) override
    {
        auto* backend = this->backend();
        return backend != nullptr && transferBusyOn(*backend, context);
    }
};

/*!
  @brief SPI view delegating registry and allocation to the HAL backend.

  Shares the acquire / logical-acquire / commit / close spine with every
  other kind through `bus::BusViewCore<BusTraits>` (see bus/bus_view.hpp);
  this derived type adds only the SPI-specific `createBusConfig()` pin
  overloads. Portable and logical acquire plus commit delegate through
  `bus::IHalBackend`,
  allowing the same BusView surface to point at local or future remote
  backends.
 */
class BusView : public bus::BusViewCore<BusTraits> {
public:
    using bus::BusViewCore<BusTraits>::BusViewCore;

    LogicalBusConfig createBusConfig(Clk clk, Mosi mosi, Miso miso, types::AllocationIntent intent = {}) const
    {
        return {clk, mosi, miso, intent};
    }
    LogicalBusConfig createBusConfig(Clk clk, Mosi mosi, types::AllocationIntent intent = {}) const
    {
        return {clk, mosi, intent};
    }
};

/*!
  @brief Non-owning SPI bus group; retained for test_bus_group and kinds that
         still use the slot-based publish/lookup table.
 */
using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v2::spi

#endif
