// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_HPP_
#define M5_HAL_REMOTE_SERVER_HPP_

#include "../data/mux.hpp"
#include "./remote.hpp"

namespace m5::hal::v2::remote {

// ---- device side ------------------------------------------------------------

/*!
  @brief Passive remote endpoint: executes request scripts, builds replies.

  The Server owns a BytecodeRunner; `registerI2C` / `registerSPI` /
  `registerUART` / `setGPIOGroup` forward to it and record the entry for
  the `hello` capability list. Registration doubles as the allowlist —
  nothing else is reachable from the wire. To expose GPIO selectively,
  register a dedicated GPIOGroup holding only the pins to publish (see
  spec/design/remote.md §safety boundary).

  Execution policy (spec §server execution model): scripts are executed
  only from complete, CHECK16-verified frames; before running, the script
  is scanned once and rejected with `ReportError(INVALID_ARGUMENT)` when
  its total `DelayMs` exceeds `Config::max_delay_ms` or a
  `BusConfigure` carries a timeout above `Config::max_bus_timeout_ms` —
  the worst-case blocking time of one `service()` call stays bounded by
  those two knobs.

  `response_scratch` (>= kMaxScriptSize) is caller-provided and holds
  one outgoing response script.
 */
class Server {
public:
    struct Config {
        uint32_t max_delay_ms       = 100;   ///< Total DelayMs budget per script.
        uint32_t max_bus_timeout_ms = 1000;  ///< Upper bound for timeouts carried by BusConfigure.
        /*!
          @brief Upper bound for a single `BusTransfer`'s wire-requested rx_len.

          The runner allocates rx_len bytes up front, and the LenVar field
          can spell a full u32 — without a cap one hostile/buggy message
          could exhaust the device's RAM. A response frame carries
          at most `kMaxTransferRx` bytes back; the default leaves headroom
          for larger discard reads while staying allocation-safe.
         */
        uint32_t max_transfer_rx = 4096;
    };

    explicit Server(data::DataSpan response_scratch) : _runner{memory::defaultAllocator()}, _scratch{response_scratch}
    {
        initRunnerHooks();
        checkScratch();
    }
    Server(data::DataSpan response_scratch, const Config& config, memory::Allocator& alloc = memory::defaultAllocator())
        : _runner{alloc}, _scratch{response_scratch}, _config{config}
    {
        initRunnerHooks();
        checkScratch();
    }

    m5::hal::v2::result_t<void> registerI2C(uint8_t bus_id, i2c::MasterAccessor& acc);
    m5::hal::v2::result_t<void> registerSPI(uint8_t bus_id, spi::MasterAccessor& acc);
    m5::hal::v2::result_t<void> registerUART(uint8_t bus_id, uart::Accessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::Accessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::TxAccessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::RxAccessor& acc);
    void setGPIOGroup(gpio::GPIOGroup& group)
    {
        _runner.setGPIOGroup(group);
        _gpio_group = &group;
        _has_gpio   = true;
    }

    /*! @brief The GPIOGroup registered via `setGPIOGroup`, or nullptr if none is. */
    gpio::GPIOGroup* gpioGroup() const
    {
        return _has_gpio ? _gpio_group : nullptr;
    }

    /*! @brief Number of statically registered bus capabilities (for HelloResp). */
    size_t capabilityCount() const
    {
        return _cap_count;
    }

    /*! @brief The i-th registered bus capability (0 <= i < capabilityCount()). */
    const Capabilities::BusEntry& capabilityAt(size_t i) const
    {
        return _caps[i];
    }

    using bus_create_app_fn_t = m5::hal::v2::result_t<void> (*)(void* ctx, bool create, types::bus_kind_t kind,
                                                                uint8_t bus_id, data::ConstDataSpan pin_config);

    void setBusCreateHandler(bus_create_app_fn_t fn, void* ctx);

    /*! @brief Direct access to the underlying runner (delay fn injection, ...). */
    bytecode::BytecodeRunner& runner()
    {
        return _runner;
    }

    /*!
      @brief Level 1: execute one script and write the response script into `out`.

      Returns the status the response reports (`OK` or the failing
      instruction's error). Sink errors while writing the response come
      back through the expected error path.
     */
    m5::hal::v2::result_t<m5::hal::v2::error::error_t> processScript(data::ConstDataSpan script, data::Sink& dst);
    void beginFrameRequest(uint8_t seq, data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec);
    void endFrameRequest();
    bool responseDeferred() const
    {
        return _defer_current_response;
    }
    void abortPendingStream();
    void setPendingStreamTimeout(uint32_t ms)
    {
        _pending_stream_timeout_ms = ms;
    }
    m5::hal::v2::result_t<void> poll(data::MuxFrameEncoder& enc, uint32_t now_ms);

private:
    struct PendingStreamTransfer {
        bool active                        = false;
        uint8_t seq                        = 0;
        types::bus_kind_t kind             = types::bus_kind_t::Unknown;
        uint8_t bus_id                     = 0;
        uint8_t stream_id                  = 0;
        uint32_t tx_len                    = 0;
        uint32_t rx_len                    = 0;
        uint8_t meta[255]                  = {};
        size_t meta_size                   = 0;
        data::Source* rx_src               = nullptr;
        data::MuxFrameDecoder* dec         = nullptr;
        size_t tx_consumed                 = 0;
        size_t rx_produced                 = 0;
        uint32_t last_progress_ms          = 0;
        bool has_progress_time             = false;
        m5::hal::v2::error::error_t status = m5::hal::v2::error::error_t::OK;
    };

    struct ExecOutcome {
        m5::hal::v2::error::error_t status = m5::hal::v2::error::error_t::OK;
        size_t offset                      = 0;      // offending offset when prescan rejected
        bool ran                           = false;  // runner.run() was invoked (slots/offset are its own)
    };

    void initRunnerHooks();
    static m5::hal::v2::result_t<void> busCreateThunk(void* ctx, bool create, types::bus_kind_t kind, uint8_t bus_id,
                                                      data::ConstDataSpan pin_config);
    m5::hal::v2::result_t<void> handleBusCreate(bool create, types::bus_kind_t kind, uint8_t bus_id,
                                                data::ConstDataSpan pin_config);

    static m5::hal::v2::result_t<void> gpioAllowlistThunk(void* ctx, const uint8_t* pins, size_t count);
    m5::hal::v2::result_t<void> handleGpioAllowlist(const uint8_t* pins, size_t count);
    static m5::hal::v2::result_t<void> streamTransferThunk(void* ctx,
                                                           const bytecode::BytecodeRunner::StreamTransferDesc& desc);
    m5::hal::v2::result_t<void> handleStreamTransfer(const bytecode::BytecodeRunner::StreamTransferDesc& desc);
    m5::hal::v2::result_t<void> completePendingStream(data::MuxFrameEncoder& enc, m5::hal::v2::error::error_t status);
    m5::hal::v2::result_t<void> writeDeferredResponse(data::MuxFrameEncoder& enc, uint8_t seq,
                                                      m5::hal::v2::error::error_t status);
    data::ConstDataSpan pendingMeta() const
    {
        return data::ConstDataSpan{_pending_stream.meta, _pending_stream.meta_size};
    }

    ExecOutcome execute(data::ConstDataSpan script);
    m5::hal::v2::error::error_t prescan(data::ConstDataSpan script, size_t& offset) const;
    m5::hal::v2::result_t<void> recordCapability(types::bus_kind_t kind, uint8_t bus_id);
    void removeCapability(types::bus_kind_t kind, uint8_t bus_id);

    // Enforce the `response_scratch` contract (>= kMaxScriptSize bytes).
    // Crash in debug; in release degrade to an empty span, which every
    // message-building path refuses — the server goes inert instead of
    // writing out of bounds.
    void checkScratch();

    bytecode::BytecodeRunner _runner;
    data::DataSpan _scratch;
    Config _config;

    Capabilities::BusEntry _caps[Capabilities::kMaxEntries];
    size_t _cap_count = 0;
    bool _has_gpio    = false;

    bus_create_app_fn_t _bus_create_app_fn = nullptr;
    void* _bus_create_app_ctx              = nullptr;
    gpio::GPIOGroup* _gpio_group           = nullptr;
    PendingStreamTransfer _pending_stream;
    uint32_t _pending_stream_timeout_ms = 5000;
    data::MuxFrameEncoder* _current_enc = nullptr;
    data::MuxFrameDecoder* _current_dec = nullptr;
    uint8_t _current_seq                = 0;
    bool _in_frame_request              = false;
    bool _defer_current_response        = false;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_HPP_
