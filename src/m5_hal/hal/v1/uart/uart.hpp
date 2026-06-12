// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_UART_HPP_
#define M5_HAL_UART_UART_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::uart {

enum class Parity : uint8_t {
    none = 0,
    even = 1,
    odd  = 2,
};
using parity_t = Parity;

enum class Channel : uint8_t {
    none = 0,
    tx   = 1u << 0,
    rx   = 1u << 1,
    txrx = (1u << 0) | (1u << 1),
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
  @brief Bus-level UART configuration.

  Pin fields take global `gpio_number_t` numbers; -1 = the line is not
  used (RTS/CTS default to unused). Buffer sizes are in bytes and are
  handed to the backend driver; 0 keeps the backend's default behavior
  (e.g. ESP-IDF installs no TX ring buffer, so writes block until the
  bytes are queued).
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
};

/*!
  @brief Accessor-level UART configuration. All timeouts are in
         milliseconds.

  Reads wait `first_byte_timeout_ms` for the first byte and
  `inter_byte_timeout_ms` between subsequent bytes; expiry is a normal
  short read, not an error. `write_timeout_ms` bounds the write/drain
  wait (the backends differ in what "drained" means — the contract
  table is in spec/design/uart.md). Channel-lock acquisition is NOT a
  config concern: it is a per-call argument of `beginTxAccess` /
  `beginRxAccess` (default: wait forever).
 */
struct AccessConfig : public bus::IAccessConfig {
    uint32_t baud_rate             = 115200;
    uint32_t first_byte_timeout_ms = 100;
    uint32_t inter_byte_timeout_ms = 20;
    uint32_t write_timeout_ms      = 1000;
    uint8_t data_bits              = 8;
    uint8_t stop_bits              = 1;
    parity_t parity                = parity_t::none;
    bool invert                    = false;

    constexpr AccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::UART}
    {
    }
};

struct IBus;

/*!
  @brief TX-side accessor; locks only the TX channel.

  TX and RX are independent channel locks, so one owner can write
  while another reads. `beginTxAccess` / `endTxAccess` nest through a
  depth counter (like `Accessor::beginAccess`), and the write sugars
  open the window themselves when needed. `setConfig` fails with
  `INVALID_ARGUMENT` while an access window is open.
 */
struct TxAccessor : public bus::IAccessor, public data::StreamWriter {
    TxAccessor(IBus& bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (gate: `beginTxAccess`). @{ */
    TxAccessor(void) = default;
    explicit TxAccessor(const AccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to a UART bus; rejected while the TX window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

    const AccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    IBus& getBus(void) const;

    result_t<void> setConfig(const AccessConfig& cfg);
    result_t<void> beginTxAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endTxAccess(void);
    bool inTxAccess(void) const
    {
        return _tx_access_depth > 0;
    }

    result_t<size_t> write(data::ConstDataSpan tx_bytes) override;
    result_t<size_t> write(data::Source& tx, size_t len);
    result_t<size_t> write(const uint8_t* tx, size_t len);

protected:
    AccessConfig _access_config;

private:
    uint32_t _tx_access_depth = 0;
    using bus::IAccessor::beginAccess;
    using bus::IAccessor::endAccess;
    using bus::IAccessor::inAccess;
};

/*!
  @brief RX-side accessor; locks only the RX channel.

  The mirror of `TxAccessor`: independent channel lock, depth
  counter via `beginRxAccess` / `endRxAccess`, and `setConfig` fails
  with `INVALID_ARGUMENT` while an access window is open.
 */
struct RxAccessor : public bus::IAccessor, public data::StreamReader {
    RxAccessor(IBus& bus, const AccessConfig& access_config);

    /*! @name Unbound construction + typed bind (gate: `beginRxAccess`). @{ */
    RxAccessor(void) = default;
    explicit RxAccessor(const AccessConfig& access_config) : _access_config{access_config}
    {
    }
    /*! @brief Bind (or rebind) to a UART bus; rejected while the RX window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

    const AccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    IBus& getBus(void) const;

    result_t<void> setConfig(const AccessConfig& cfg);
    result_t<void> beginRxAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endRxAccess(void);
    bool inRxAccess(void) const
    {
        return _rx_access_depth > 0;
    }

    result_t<size_t> read(data::DataSpan rx_bytes) override;
    result_t<size_t> read(data::Sink& rx, size_t len);
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
    using bus::IAccessor::beginAccess;
    using bus::IAccessor::endAccess;
    using bus::IAccessor::inAccess;
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

    /*! @name Unbound construction + typed bind (delegates to both channels). @{ */
    Accessor(void) = default;
    explicit Accessor(const AccessConfig& access_config) : _tx{access_config}, _rx{access_config}
    {
    }
    /*! @brief Bind (or rebind) both channel accessors; rejected while either window is open. */
    result_t<void> bind(IBus& bus);
    /*! @} */

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
        return _tx.inTxAccess() || _rx.inRxAccess();
    }

    result_t<size_t> write(data::ConstDataSpan tx_bytes);
    result_t<size_t> write(data::Source& tx, size_t len);
    result_t<size_t> write(const uint8_t* tx, size_t len);

    result_t<size_t> read(data::DataSpan rx_bytes);
    result_t<size_t> read(data::Sink& rx, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    /*! @brief Line-oriented sugar; forwards to the RX accessor's `readUntil`. */
    result_t<size_t> readUntil(uint8_t delim, uint8_t* dst, size_t max_len);

    result_t<size_t> readableBytes(void);

protected:
    TxAccessor _tx;
    RxAccessor _rx;
};

struct IBus : public bus::IBus {
    const IBusConfig& getConfig(void) const override
    {
        return _config;
    }

    virtual result_t<size_t> write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* tx, size_t len);
    virtual result_t<size_t> read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* rx, size_t len);
    virtual result_t<size_t> readableBytes(bus::IAccessor* owner, const AccessConfig& cfg);

    result_t<void> lock(bus::IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER) override;
    result_t<void> unlock(bus::IAccessor* owner) override;
    virtual result_t<void> lockChannel(bus::IAccessor* owner, Channel ch, uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    virtual result_t<void> unlockChannel(bus::IAccessor* owner, Channel ch);

protected:
    IBusConfig _config;
    // UART splits the bus lock into independent TX / RX channels, so it
    // carries one runtime::Mutex per channel (the composite txrx lock
    // takes both, TX first); the base Bus mutex stays unused here. Lock
    // semantics per channel match Bus::lock: wait up to timeout_ms,
    // TIMEOUT_ERROR on expiry, non-recursive, task context only.
    runtime::Mutex _tx_mutex;
    runtime::Mutex _rx_mutex;
    bus::IAccessor* _tx_lock_owner = nullptr;
    bus::IAccessor* _rx_lock_owner = nullptr;
};

//-------------------------------------------------------------------------
// bind() is defined below the concrete IBus: at the accessors' point of
// declaration the kind IBus is still an incomplete type, so the
// derived-to-base conversion _bindBus needs is not visible yet.
inline result_t<void> TxAccessor::bind(IBus& bus)
{
    if (inTxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _bindBus(bus);
    return {};
}
inline result_t<void> RxAccessor::bind(IBus& bus)
{
    if (inRxAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _bindBus(bus);
    return {};
}
inline result_t<void> Accessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    (void)_tx.bind(bus);
    (void)_rx.bind(bus);
    return {};
}

/*!
  @brief Non-owning UART bus registry (`M5_Hal.UART`); see `bus::BusGroup`.
 */
using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v1::uart

#endif
