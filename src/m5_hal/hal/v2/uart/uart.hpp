// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_UART_HPP_
#define M5_HAL_UART_UART_HPP_

#include "../bus/bus.hpp"
#include "../bus/bus_view.hpp"
#include "../bus/hal_backend.hpp"
#include "../bus/managed_facade.hpp"
#include "../bus/registry.hpp"
#include "../data.hpp"
#include "../data/stream.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>
#include <atomic>
#include <memory>
#include <new>

/*!
  @namespace m5::hal::v2::uart
  @brief UART bus, TX/RX accessors, and stream-style byte I/O.

  ## Getting a bus (entry point)

  Buses are obtained from a `Hal` facade instance — `M5_Hal` (the local
  device) or a user-constructed remote `Hal` — never by naming a concrete
  backend type yourself. Remote instances are connected via
  `Hal::connect(endpoint)`; see m5_hal.hpp.

  @code
  m5hal::uart::BusConfig cfg{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}};
  auto bus = m5hal::M5_Hal.UART.acquire(cfg);  // result_t<shared_ptr<IBus>>
  if (!bus) {
      // handle bus.error()
  }
  @endcode

  Note: `uart::BusConfig` is an alias supplied by the active build
  variant — this abstract header defines only the `IBusConfig` base and
  the pin tags. Include the v2 entry header (`<M5HAL_v2.hpp>`) and the
  alias resolves to the variant's concrete config type. The
  pin-tag construction above is for MCU variants; the POSIX variant
  identifies a bus by `device_path` instead and deliberately has no pin
  constructor (see the IBusConfig doc below). Acquiring the same wiring
  twice returns the same instance (identity acquire; see
  spec/design/bus_accessor.md). Direct construction
  (`uart::Bus bus; bus.init(cfg);`) remains the advanced path.
 */
namespace m5::hal::v2::uart {

enum class Parity : uint8_t {
    None = 0,
    Even = 1,
    Odd  = 2,
};
using parity_t = Parity;

enum class Channel : uint8_t {
    None = 0,
    Tx   = 1u << 0,
    Rx   = 1u << 1,
    TxRx = (1u << 0) | (1u << 1),
};
using channel_t = Channel;

constexpr Channel operator|(Channel lhs, Channel rhs)
{
    return static_cast<Channel>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

constexpr Channel operator&(Channel lhs, Channel rhs)
{
    return static_cast<Channel>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

constexpr bool hasChannel(Channel value, Channel bit)
{
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) == static_cast<uint8_t>(bit);
}

/*!
  @brief Strong-typed TX pin for one-line bus-config construction.

  Wraps a global `gpio_number_t` so the constructor argument carries
  its role in the type. The constructor is `explicit` on purpose: a
  plain integer never converts into a tag, so an untagged positional
  call like `BusConfig{17, 16}` stays a compile error.
 */
struct Tx {
    constexpr explicit Tx(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*! @brief Strong-typed RX pin. See @ref Tx. */
struct Rx {
    constexpr explicit Rx(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*!
  @brief Bus-level UART configuration.

  Pin fields take global `gpio_number_t` numbers; -1 = the line is not
  used (RTS/CTS default to unused). Buffer sizes are in bytes and are
  handed to the backend driver; 0 keeps the backend's default behavior
  (e.g. ESP-IDF installs no TX ring buffer, so writes block until the
  bytes are queued).

  One-line construction passes the strong-typed pin tags — either order
  lands on the right field:
  @code
  uart::BusConfig cfg{uart::Tx{17}, uart::Rx{16}};
  @endcode
  RTS / CTS and the buffer sizes stay at their defaults and are set by
  field assignment when needed.

  This tag one-liner is for the MCU variants (`BusConfig_espidf` /
  `BusConfig_arduino`, which inherit it). The POSIX host variant
  (`BusConfig_posix`) deliberately omits the pin ctor — its endpoint selector
  is a `device_path` (which serial device to open), not pins, so it is built
  by field assignment (`cfg.device_path = "/dev/ttyUSB0";`) to avoid a pin
  one-liner that looks complete while leaving `device_path` unset. Note that
  `device_path` is a backend endpoint selector, NOT the shared-registry bus
  identity (which is the TX/RX pin pair — see `identityFromConfig` below);
  path-keyed acquire through the shared registry is not supported. `uart::BusConfig` resolves to
  the active build's variant, so the pin one-liner above compiles on an MCU
  build but not on a native/POSIX build (see spec/design/uart.md §pin).
 */
struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_tx  = -1;
    types::gpio_number_t pin_rx  = -1;
    types::gpio_number_t pin_rts = -1;
    types::gpio_number_t pin_cts = -1;
    size_t rx_buffer_size        = 256;
    size_t tx_buffer_size        = 0;

    constexpr IBusConfig(void) : bus::IBusConfig{types::bus_kind_t::UART}
    {
    }

    /*! @brief One-line pin construction (tag order is free). */
    constexpr IBusConfig(Tx tx, Rx rx) : bus::IBusConfig{types::bus_kind_t::UART}, pin_tx{tx.value}, pin_rx{rx.value}
    {
    }
    constexpr IBusConfig(Rx rx, Tx tx) : IBusConfig{tx, rx}
    {
    }
};

/*!
  @brief Pin + intent acquire request for the unified BusView surface.

  UART currently uses static backend selection through `acquire(BusConfig)`;
  the logical acquire overload exists on `BusView` for API parity with I2C/SPI
  and reports `NOT_IMPLEMENTED` until UART grows a controller-allocation policy.
 */
struct LogicalBusConfig {
    types::gpio_number_t pin_tx = -1;
    types::gpio_number_t pin_rx = -1;
    types::AllocationIntent intent{};

    constexpr LogicalBusConfig(void) = default;
    constexpr LogicalBusConfig(Tx tx, Rx rx, types::AllocationIntent in = {})
        : pin_tx{tx.value}, pin_rx{rx.value}, intent{in}
    {
    }
    constexpr LogicalBusConfig(Rx rx, Tx tx, types::AllocationIntent in = {}) : LogicalBusConfig{tx, rx, in}
    {
    }
};

/*!
  @brief Accessor-level UART configuration. All timeouts are in
         milliseconds.

  Reads wait `first_byte_timeout_ms` for the first byte and
  `inter_byte_timeout_ms` between subsequent bytes; expiry is a normal
  short read, not an error. `write_timeout_ms` bounds the write/drain
  wait (the backends differ in what "drained" means — the contract
  table is in spec/design/uart.md). Channel-lock acquisition is NOT a
  config concern: it is a per-call argument of `beginAccess` on the TX
  or RX channel accessor (default: wait forever).
 */
struct AccessConfig : public bus::IAccessConfig {
    uint32_t baud_rate             = 115200;
    uint32_t first_byte_timeout_ms = 100;
    uint32_t inter_byte_timeout_ms = 20;
    uint32_t write_timeout_ms      = 1000;
    uint8_t data_bits              = 8;
    uint8_t stop_bits              = 1;
    parity_t parity                = parity_t::None;
    bool invert                    = false;

    constexpr AccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::UART}
    {
    }
};

struct IBus;

/*!
  @brief TX-side accessor; locks only the TX channel.

  TX and RX are independent channel locks, so one owner can write
  while another reads. `beginAccess` / `endAccess` (TX channel) nest
  through a depth counter (like `Accessor::beginAccess`), and the write
  sugars open the window themselves when needed.

  A UART transaction is a TX channel exclusion scope plus byte-count
  aggregation; it has no physical CS or bus-occupancy side effect. Accessors
  must not be shared between threads. The transaction depth is only for
  same-owner reentry, while sharing the Bus through separate accessors is
  supported. `setConfig` fails with `INVALID_STATE` while an access window is
  open.
 */
struct TxAccessor : public bus::IAccessor, public data::StreamWriter {
    TxAccessor(IBus& bus, const AccessConfig& access_config);
    /*! @brief Co-owning construction from an acquired shared bus (`M5_Hal.UART.acquire(cfg)`). */
    TxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (gate: `beginAccess` on TX channel). @{ */
    TxAccessor(void) = default;
    explicit TxAccessor(const AccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to a UART bus; rejected while the TX window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

    // Non-copyable: an accessor carries lock/depth state and (as one half of
    // a combined `Accessor`) a lock-peer identity; a copy would alias both.
    TxAccessor(const TxAccessor&)            = delete;
    TxAccessor& operator=(const TxAccessor&) = delete;

    const AccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    IBus& getBus(void) const;

    result_t<void> setConfig(const AccessConfig& cfg);
    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(void);
    bool inAccess(void) const
    {
        return _tx_access_depth > 0;
    }

    result_t<void> beginTransaction(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<bus::TransferTotals> endTransaction(void);
    bool inTransaction(void) const
    {
        return _tx_txn_depth > 0;
    }

    result_t<size_t> write(data::ConstDataSpan src_bytes) override;
    result_t<size_t> write(data::Source& src, size_t len);
    result_t<size_t> write(const uint8_t* src, size_t len);

protected:
    AccessConfig _access_config;

private:
    uint32_t _tx_access_depth = 0;
    uint32_t _tx_txn_depth    = 0;
    bus::TransferTotals _tx_txn_totals;
};

/*!
  @brief RX-side accessor; locks only the RX channel.

  The mirror of `TxAccessor`: independent channel lock, depth
  counter via `beginAccess` / `endAccess` (RX channel), and `setConfig`
  fails with `INVALID_STATE` while an access window is open.

  A UART transaction is an RX channel exclusion scope plus byte-count
  aggregation; it has no physical CS or bus-occupancy side effect. Accessors
  must not be shared between threads. The transaction depth is only for
  same-owner reentry, while sharing the Bus through separate accessors is
  supported.
 */
struct RxAccessor : public bus::IAccessor, public data::StreamReader {
    RxAccessor(IBus& bus, const AccessConfig& access_config);
    /*! @brief Co-owning construction from an acquired shared bus (`M5_Hal.UART.acquire(cfg)`). */
    RxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (gate: `beginAccess` on RX channel). @{ */
    RxAccessor(void) = default;
    explicit RxAccessor(const AccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to a UART bus; rejected while the RX window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

    // Non-copyable: see TxAccessor.
    RxAccessor(const RxAccessor&)            = delete;
    RxAccessor& operator=(const RxAccessor&) = delete;

    const AccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    IBus& getBus(void) const;

    result_t<void> setConfig(const AccessConfig& cfg);
    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(void);
    bool inAccess(void) const
    {
        return _rx_access_depth > 0;
    }

    result_t<void> beginTransaction(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<bus::TransferTotals> endTransaction(void);
    bool inTransaction(void) const
    {
        return _rx_txn_depth > 0;
    }

    result_t<size_t> read(data::DataSpan dst_bytes) override;
    result_t<size_t> read(data::Sink& dst, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    /*!
      @brief Read until `delim` is stored (line-oriented sugar).

      `data::readUntil` run inside ONE RX channel-lock window, so a
      line costs one lock acquisition instead of one per byte. The
      return value is the byte count with **the delimiter included**;
      `n > 0 && dst[n - 1] == delim` decides completion, a short
      count is a timeout-bounded partial read (not an error). See
      `data::readUntil` for the full contract.
     */
    result_t<size_t> readUntil(uint8_t delim, uint8_t* dst, size_t max_len);

    result_t<size_t> readableBytes(void) override;

protected:
    AccessConfig _access_config;

private:
    uint32_t _rx_access_depth = 0;
    uint32_t _rx_txn_depth    = 0;
    bus::TransferTotals _rx_txn_totals;
};

/*!
  @brief Convenience facade bundling one TX and one RX accessor.

  `beginAccess` opens both channels in TX -> RX order, spending the
  remaining timeout budget on the second lock (an infinite budget stays
  infinite); if the RX lock fails, the already-acquired TX lock is
  rolled back. The split accessors are the primary API — the facade
  keeps simple command-response code short.
 */
struct Accessor {
    Accessor(IBus& bus, const AccessConfig& access_config);
    /*!
      @brief Co-owning construction from an acquired shared bus. Both channels share
      ownership of the bus (`M5_Hal.UART.acquire(cfg)`).
     */
    Accessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (delegates to both channels). @{ */
    Accessor(void)
    {
        wireLockPeers();
    }
    explicit Accessor(const AccessConfig& access_config) : _tx{access_config}, _rx{access_config}
    {
        wireLockPeers();
    }
    /*! @brief Bind (or rebind) both channel accessors; rejected while either window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

    // Non-copyable: the TX/RX children point at EACH OTHER through
    // `bus::IAccessor::lockPeer`; a copy's children would keep pointing at
    // the ORIGINAL's children, letting the reconfiguration gate falsely
    // treat the original's channel holds as the copy's own.
    Accessor(const Accessor&)            = delete;
    Accessor& operator=(const Accessor&) = delete;

    const AccessConfig& getConfig(void) const
    {
        return _tx.getConfig();
    }
    IBus& getBus(void) const;

    TxAccessor& tx(void)
    {
        return _tx;
    }
    const TxAccessor& tx(void) const
    {
        return _tx;
    }
    RxAccessor& rx(void)
    {
        return _rx;
    }
    const RxAccessor& rx(void) const
    {
        return _rx;
    }

    result_t<void> setConfig(const AccessConfig& cfg);
    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(void);
    bool inAccess(void) const
    {
        return _tx.inAccess() || _rx.inAccess();
    }

    result_t<size_t> write(data::ConstDataSpan src_bytes);
    result_t<size_t> write(data::Source& src, size_t len);
    result_t<size_t> write(const uint8_t* src, size_t len);

    result_t<size_t> read(data::DataSpan dst_bytes);
    result_t<size_t> read(data::Sink& dst, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    result_t<bus::TransferTotals> transfer(data::Source& src, size_t tx_len, data::Sink& dst, size_t rx_len);
    result_t<bus::TransferTotals> transfer(data::ConstDataSpan src_bytes, data::DataSpan dst_bytes);

    /*! @brief Line-oriented sugar; forwards to the RX accessor's `readUntil`. */
    result_t<size_t> readUntil(uint8_t delim, uint8_t* dst, size_t max_len);

    result_t<size_t> readableBytes(void);

private:
    /*!
      @brief Cross-wire `_tx` / `_rx` as each other's lock peer
             (bus::IAccessor::lockPeer) so the reconfiguration quiescence
             gate recognizes the two channel accessors as one logical
             holder. Called from every ctor (the bus-ref / shared_ptr
             ctors below call it from their .inl bodies).
     */
    void wireLockPeers(void)
    {
        _tx.setLockPeer(&_rx);
        _rx.setLockPeer(&_tx);
    }

protected:
    TxAccessor _tx;
    RxAccessor _rx;
};

/*!
  @brief Result of `IBus::tryAcquireOppositeChannel` (the reconfiguration
         quiescence gate a variant backend consults before re-applying a
         changed `AccessConfig` — see spec/design/uart.md).
 */
struct QuiescenceGrant {
    bool granted     = false;  //!< both channels are quiescent for the owner
    bool must_unlock = false;  //!< true = pass back to releaseOppositeChannel
    Channel opposite = Channel::Tx;
};

struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual result_t<size_t> write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src, size_t len);
    virtual result_t<size_t> read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* dst, size_t len);
    /*!
      @brief Transfer bytes in both independent UART directions.

      The contract is direction-independent concurrent progress: TX consumes
      up to `tx_len` bytes while RX produces up to `rx_len` bytes. The default
      implementation approximates this for local backends by running `write`
      first and then `read`; a backend with true full-duplex DMA may override
      it. If either step fails, the error is returned immediately, and bytes
      from the earlier step may already have moved.
     */
    virtual result_t<bus::TransferTotals> transfer(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src,
                                                   size_t tx_len, data::Sink* dst, size_t rx_len);
    virtual result_t<size_t> readableBytes(bus::IAccessor* owner, const AccessConfig& cfg);

    result_t<void> lock(bus::IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER) override;
    result_t<void> unlock(bus::IAccessor* owner) override;
    virtual result_t<void> lockChannel(bus::IAccessor* owner, Channel ch, uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    virtual result_t<void> unlockChannel(bus::IAccessor* owner, Channel ch);

    /*!
      @brief Reconfiguration quiescence gate: try to prove BOTH channels are
             idle for `owner` before a variant backend re-applies a changed
             `AccessConfig` over the one already in effect.

      Call this only for a RE-configuration (the first `AccessConfig` applied
      on a fresh bus needs no gate — see spec/design/uart.md). `entered` is
      the single channel `owner` already holds (`Channel::Tx` or
      `Channel::Rx`); `Channel::TxRx` / `Channel::None` is never granted, and
      neither is a `nullptr` owner.

      Granted means: the opposite channel is unheld and this call took it
      through a non-blocking try-lock, OR the opposite channel's holder is
      `owner` itself, or `owner`'s lock peer (`bus::IAccessor::lockPeer` --
      the combined-accessor plumbing letting `uart::Accessor`'s TX/RX
      children recognize each other as one logical holder). An opposite
      channel held by some OTHER, unrelated accessor running on the SAME
      task as the caller is treated as busy (not granted) -- see the
      same-task step below for why it is never attempted.

      Implementation note (not caller-visible, but load-bearing): the checks
      run in this order:
        1. Self/peer hold: the opposite channel's lock owner equals `owner`
           or `owner->lockPeer()` -- granted, nothing to unlock.
        2. Same-task guard: the opposite channel's lock TASK equals the
           calling task (via `runtime::currentTaskId()`), but its owner is
           neither `owner` nor `owner`'s peer (some unrelated accessor on
           this same thread holds it) -- not granted, and no try-lock is
           attempted here.
        3. Non-blocking `lockChannel(owner, opposite, 0)` -- granted with
           `must_unlock = true` on success, not granted otherwise.
      Steps 1 and 2 must run before step 3: a try-lock issued by a thread
      that already holds the mutex is undefined behavior on POSIX
      `std::timed_mutex`, and step 2 is what rules that case out once step 1
      has failed to recognize the holder as self/peer (a task id can only
      match the CALLING task's own id -- another task can never have
      written it -- so the same-task read is never stale). This gate is the
      sole sanctioned exception to the channel-lock -> state-mutex ordering
      below (a variant's internal state mutex may stay held while this
      touches a channel mutex), but only through the non-blocking attempt,
      so it can never deadlock.

      On a granted result the caller MUST call `releaseOppositeChannel`
      after the (re)configuration completes -- on every exit path, including
      error returns; `must_unlock == false` makes that call a no-op.
     */
    QuiescenceGrant tryAcquireOppositeChannel(bus::IAccessor* owner, Channel entered);
    /*!
      @brief Pair with `tryAcquireOppositeChannel`; releases the opposite
             channel lock it took, or does nothing when `grant.must_unlock`
             is false.
     */
    void releaseOppositeChannel(bus::IAccessor* owner, const QuiescenceGrant& grant);

protected:
    IBusConfig _config;
    // UART splits the bus lock into independent TX / RX channels, so it
    // carries one runtime::Mutex per channel (the composite txrx lock
    // takes both, TX first); the base Bus mutex stays unused here. Lock
    // semantics per channel match Bus::lock: wait up to timeout_ms,
    // TIMEOUT_ERROR on expiry, non-recursive, task context only.
    //
    // Variant backends (Bus_espidf / Bus_posix / Bus_arduino) additionally
    // carry their own internal state mutex -- a `runtime::Mutex` leaf NOT
    // declared here -- guarding config/coalesce state shared by both
    // channels (see spec/design/uart.md). The mandated acquisition order is
    // channel lock -> state mutex, one direction only; the sole exception is
    // `tryAcquireOppositeChannel`'s non-blocking probe of the opposite
    // channel while a state mutex is held (see its doc comment above).
    runtime::Mutex _tx_mutex;
    runtime::Mutex _rx_mutex;
    // Bookkeeping only (which accessor currently holds each channel);
    // atomic so `tryAcquireOppositeChannel`'s self-hold check never tears a
    // read against the store this same class does right after taking the
    // real lock. relaxed is enough: the mutex acquire/release already
    // supplies the ordering that matters for the guarded data itself.
    std::atomic<bus::IAccessor*> _tx_lock_owner{nullptr};
    std::atomic<bus::IAccessor*> _rx_lock_owner{nullptr};
    // Task identity of the current channel holder (nullptr when unlocked).
    // Same bookkeeping-only / relaxed rationale as the owner slots above;
    // lets `tryAcquireOppositeChannel` recognize "some OTHER, unrelated
    // accessor on THIS SAME task holds the opposite channel" and skip
    // rather than issue a same-thread try_lock (UB on POSIX
    // std::timed_mutex -- see that method's doc comment).
    std::atomic<void*> _tx_lock_task{nullptr};
    std::atomic<void*> _rx_lock_task{nullptr};
};

//-------------------------------------------------------------------------
// bind() is defined below the concrete IBus: at the accessors' point of
// declaration the kind IBus is still an incomplete type, so the
// derived-to-base conversion _bindBus needs is not visible yet.
inline result_t<void> TxAccessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _bindBus(bus);
    return {};
}
inline result_t<void> RxAccessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _bindBus(bus);
    return {};
}
inline result_t<void> Accessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    (void)_tx.bind(bus);
    (void)_rx.bind(bus);
    return {};
}

/*!
  @brief Maps a variant BusConfig_<variant> to its backend Bus_<variant>.

  Undefined primary on purpose: passing a config type without a
  specialization to Bus::init is a compile error. Each variant header
  specializes this next to its Bus_<variant>.
 */
template <class CfgT>
struct BackendFor;

struct Bus;  // the facade, defined just below

/*!
  @brief Per-kind Traits for the shared `bus::FacadeCore` and BusView.

  UART's `Bus` shares the base spine (backend ownership + `init` +
  `release` + the query mirror). Its plain `BusView` uses the same traits for
  typed acquire identity: concrete kind `IBus`, the bus-level config base, the
  variant selector (`FacadeCore`), the public `BusType`, kind tag, and the 2-pin
  (TX/RX) identity projection. UART has no intent / hot-swap surface,
  so the master-only Traits members are intentionally absent.
  `Bus` is forward-declared at namespace scope so `BusType` names the public
  `uart::Bus`, not a nested type.
 */
struct BusTraits {
    using IBus             = uart::IBus;
    using IBusConfig       = uart::IBusConfig;
    using LogicalBusConfig = uart::LogicalBusConfig;
    using BusType          = Bus;
    template <class CfgT>
    using BackendFor = uart::BackendFor<CfgT>;

    static constexpr types::bus_kind_t KIND = types::bus_kind_t::UART;
    /*! @brief UART uses the static-backend policy: `BusView::hardwareInUse()`
               is always 0 (see spec/design/bus_accessor.md §static-backend
               policy). */
    static constexpr bool MANAGED_ALLOCATION = false;

    // UART bus identity is the TX/RX pin pair ONLY. Backend-specific
    // selectors -- ESP-IDF `port_num`, POSIX `device_path` -- are NOT part of the
    // identity, so two acquires with the same pins share one bus even if they
    // name a different controller/device (first config wins). This is by design:
    // UART is a pins-keyed static view that does not (yet) participate in the
    // managed controller allocator. Selecting a specific UART controller, or a
    // POSIX device by path, is therefore out of scope for this shared pins-only
    // acquire path -- use distinct pins, or a future dedicated API. (Validating
    // backend-specific fields on a registry hit is a possible future safety
    // net.)
    static bus::IdentityKey identityFromConfig(const IBusConfig& cfg)
    {
        return bus::IdentityKey::fromPins({cfg.pin_tx, cfg.pin_rx});
    }
    static bus::IdentityKey identityFromLogical(const LogicalBusConfig& req)
    {
        return bus::IdentityKey::fromPins({req.pin_tx, req.pin_rx});
    }
    static bool configCompatible(const IBusConfig& current, const IBusConfig& requested)
    {
        return current.pin_tx == requested.pin_tx && current.pin_rx == requested.pin_rx &&
               current.pin_rts == requested.pin_rts && current.pin_cts == requested.pin_cts &&
               current.rx_buffer_size == requested.rx_buffer_size && current.tx_buffer_size == requested.tx_buffer_size;
    }
};

/*!
  @brief Runtime facade for a UART bus (the unsuffixed uart::Bus).

  All of the backend spine -- the `unique_ptr`-held backend, `init`, `release`,
  and the lock-free backend-query mirror -- lives in
  `bus::FacadeCore<BusTraits>`. This derived type adds only the UART data-path
  forwards (write / read / readableBytes). The TX / RX channel locks
  (lock / unlock / lockChannel / unlockChannel and the two channel mutexes) are
  INHERITED from uart::IBus and stay on THIS facade -- the accessors contend on
  the facade's channel mutexes, the backend's stay dormant. So the 2-mutex
  channel model needs no special handling here: only the data path is delegated.
  Mirrors the i2c::Bus design without the intent / hot-swap
  extensions.
 */
struct Bus : public bus::FacadeCore<BusTraits> {
    result_t<size_t> write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src, size_t len) override
    {
        return forwardBackend([&](IBus& b) { return b.write(owner, cfg, src, len); });
    }

    result_t<size_t> read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* dst, size_t len) override
    {
        return forwardBackend([&](IBus& b) { return b.read(owner, cfg, dst, len); });
    }

    result_t<bus::TransferTotals> transfer(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src,
                                           size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        return forwardBackend([&](IBus& b) { return b.transfer(owner, cfg, src, tx_len, dst, rx_len); });
    }

    result_t<size_t> readableBytes(bus::IAccessor* owner, const AccessConfig& cfg) override
    {
        return forwardBackend([&](IBus& b) { return b.readableBytes(owner, cfg); });
    }
};

/*!
  @brief Typed UART view delegating registry access to the HAL backend.

  Shares the acquire / logical-acquire / commit / release spine with every
  other kind through `bus::BusViewCore<BusTraits>` (see bus/bus_view.hpp);
  this derived type adds only the UART-specific `createBusConfig()` pin
  overloads. Parity with i2c::BusView: UART buses are interned by the HAL
  backend's registry keyed by physical wiring (the TX / RX pins).
  acquire(cfg) returns the bus for those pins, creating it (the facade + the
  backend selected by cfg's type) on the first call and sharing it on later
  calls for the same wiring. Identity is (TX, RX); both are required (a
  TX-only or RX-only port is not registry-acquirable -- construct it
  directly). The public view uses the static-backend policy: `commitBuses()`
  exists and is a no-op, while logical acquire reports `NOT_IMPLEMENTED` until
  UART grows a controller-allocation policy.
 */
class BusView : public bus::BusViewCore<BusTraits> {
public:
    using bus::BusViewCore<BusTraits>::BusViewCore;

    LogicalBusConfig createBusConfig(Tx tx, Rx rx, types::AllocationIntent intent = {}) const
    {
        return {tx, rx, intent};
    }
    LogicalBusConfig createBusConfig(Rx rx, Tx tx, types::AllocationIntent intent = {}) const
    {
        return {tx, rx, intent};
    }
};

/*!
  @brief Non-owning UART bus group; retained for the slot-based publish/lookup
         table and the bus::BusGroup tests.
 */
using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v2::uart

#endif
