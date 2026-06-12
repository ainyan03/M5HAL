// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_REMOTE_HPP_
#define M5_HAL_REMOTE_REMOTE_HPP_

#include "../assert.hpp"
#include "../bytecode/bytecode.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
#include "../frame/frame.hpp"
#include "../gpio/gpio.hpp"
#include "../i2c/i2c.hpp"
#include "../i2s/i2s.hpp"
#include "../service/service.hpp"
#include "../spi/spi.hpp"
#include "../uart/uart.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// Remote bus mechanism. The authoritative spec lives in spec/design/remote.md.
//
// Messages ride frame v1 `data` frames: the stream_id selects the remote
// channel and the payload starts with a 2-byte header.
//
//   data frame payload : [TYPE:1][SEQ:1][BODY:0..238]
//
// TYPE low nibble is the message type; bit7 = NORESP (fire-and-forget
// request), bit6 = MORE (chunk continuation, reserved, always 0 in this
// version). Unknown types and set reserved bits are silently dropped by
// the receiver.
//
// The host side (RemoteSession + proxy buses such as RemoteI2CBus) sends
// bytecode scripts as requests; the device side (Server, usually polled
// through RemoteServerService) executes them on a BytecodeRunner and
// returns the response script. What is registered on the server's runner
// is exactly what a remote peer can reach (registration = allowlist).
// =============================================================================

/*!
  @namespace m5::hal::v1::remote
  @brief Remote bus mechanism: message layer, server endpoint, proxy buses.
 */
namespace m5::hal::v1::remote {

constexpr uint8_t kProtocolVersion = 1;     ///< hello/hello_resp proto_ver.
constexpr uint8_t kDefaultStreamId = 0x00;  ///< Remote message channel (spec §stream_id registry).

constexpr size_t kHeaderSize     = 2;                                     ///< TYPE + SEQ.
constexpr size_t kMaxMessageSize = frame::kMaxDataPayload;                ///< 240.
constexpr size_t kMaxBodySize    = frame::kMaxDataPayload - kHeaderSize;  ///< 238.
/*! @brief Guaranteed per-transfer receive-data limit (response script overhead subtracted).
 *
 *  Derivation (checked by the static_assert below):
 *    A response script carrying one store_data + report_complete + terminator uses:
 *      store_data     = LenVar(1+1+N) + opcode(1) + store_id(1) + data(N)
 *                     = 1 + 2 + N      (LenVar is 1 byte when 2+N <= 0xFC, i.e. N <= 250)
 *      report_complete= LenVar(2) + opcode(1) + status(1) = 3
 *      terminator     = 1
 *    Total overhead   = 3 + 3 + 1 = 7  (8 with the 1-byte safety margin chosen here)
 *    => kMaxTransferRx = kMaxBodySize - 8 = 238 - 8 = 230
 */
constexpr size_t kMaxTransferRx = 230;
static_assert(kMaxTransferRx + 8 <= kMaxBodySize,
              "kMaxTransferRx leaves insufficient room for response script overhead "
              "(store_data header + report_complete + terminator = 7 bytes; 8 chosen for margin)");
/*!
  @brief Default store slot for response data.

  Used as the `store_id` argument wherever a proxy bus encodes a single
  receive buffer into the response script. Distinguished from
  `bytecode::kDiscardStoreId` (0xFF), which signals "do not store".
  Every `request()` call resets the runner's store slots before decoding
  the response, so slot 0 is always fresh after each round trip.
 */
constexpr uint8_t kDefaultStoreId = 0;
/*! @brief Round-trip margin added on top of remote-side UART timeouts (spec §UART proxy). */
constexpr uint32_t kRemoteUartTimeoutMarginMs = 250;

enum class MessageType : uint8_t {
    hello      = 0x0,  ///< host -> device, empty body.
    hello_resp = 0x1,  ///< device -> host, capability list.
    request    = 0x2,  ///< host -> device, body = bytecode script.
    response   = 0x3,  ///< device -> host, body = response script.
    error      = 0x4,  ///< device -> host, body = [error:i8][detail...].
    ping       = 0x5,  ///< host -> device, empty body.
    pong       = 0x6,  ///< device -> host, empty body.
    event      = 0x7,  ///< device -> host, body = event script (push notifications).
};

constexpr uint8_t kTypeNorespBit    = 0x80;  ///< Fire-and-forget request flag.
constexpr uint8_t kTypeMoreBit      = 0x40;  ///< Chunk continuation (reserved, must be 0).
constexpr uint8_t kTypeReservedBits = 0x30;  ///< Must be 0 in this version.
constexpr uint8_t kTypeKindMask     = 0x0F;

/*!
  @brief Map a remote-reported i8 error code into the local error_t.

  Codes this build does not know fold into `REMOTE_FAULT`, so a newer
  server never breaks an older host (spec §error mapping).
 */
constexpr m5::hal::v1::error::error_t mapRemoteError(int8_t code)
{
    return (code >= static_cast<int8_t>(m5::hal::v1::error::error_t::UNSUPPORTED) &&
            code <= static_cast<int8_t>(m5::hal::v1::error::error_t::ASYNC_RUNNING))
               ? static_cast<m5::hal::v1::error::error_t>(code)
               : m5::hal::v1::error::error_t::REMOTE_FAULT;
}

/*! @brief Capability summary carried by `hello_resp`. */
struct Capabilities {
    struct BusEntry {
        types::bus_kind_t kind = types::bus_kind_t::UNKNOWN;
        uint8_t bus_id         = 0;
    };
    static constexpr size_t kMaxEntries = 4 * bytecode::kMaxBusBindings;

    uint8_t proto_ver = 0;
    bool has_gpio     = false;
    size_t bus_count  = 0;
    BusEntry buses[kMaxEntries];
};

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
  is scanned once and rejected with `report_error(INVALID_ARGUMENT)` when
  its total `delay_ms` exceeds `Config::max_delay_ms` or a
  `bus_configure` carries a timeout above `Config::max_bus_timeout_ms` —
  the worst-case blocking time of one `service()` call stays bounded by
  those two knobs.

  `response_scratch` (>= kMaxMessageSize) is caller-provided and holds
  one outgoing message while it is framed.
 */
class Server {
public:
    struct Config {
        uint32_t max_delay_ms       = 100;   ///< Total delay_ms budget per script.
        uint32_t max_bus_timeout_ms = 1000;  ///< Upper bound for timeouts carried by bus_configure.
        /*! @brief Bytes a stream binding must drain before an `evt_stream_credit` is emitted. */
        uint32_t stream_credit_threshold = 2048;
        /*!
          @brief Upper bound for a single `bus_transfer`'s wire-requested rx_len.

          The runner allocates rx_len bytes up front, and the LenVar field
          can spell a full u32 — without a cap one hostile/buggy message
          could exhaust the device's RAM (S16 D8). A response frame carries
          at most `kMaxTransferRx` bytes back; the default leaves headroom
          for larger discard reads while staying allocation-safe.
         */
        uint32_t max_transfer_rx = 4096;
    };

    explicit Server(data::DataSpan response_scratch) : _runner{memory::defaultAllocator()}, _scratch{response_scratch}
    {
        _runner.setGpioSubscribeHandler(&gpioSubscribeThunk, this);
        checkScratch();
    }
    Server(data::DataSpan response_scratch, const Config& config, memory::Allocator& alloc = memory::defaultAllocator())
        : _runner{alloc}, _scratch{response_scratch}, _config{config}
    {
        _runner.setGpioSubscribeHandler(&gpioSubscribeThunk, this);
        checkScratch();
    }

    m5::hal::v1::result_t<void> registerI2C(uint8_t bus_id, i2c::I2CMasterAccessor& acc);
    m5::hal::v1::result_t<void> registerSPI(uint8_t bus_id, spi::SPIMasterAccessor& acc);
    m5::hal::v1::result_t<void> registerUART(uint8_t bus_id, uart::UARTAccessor& acc);
    /*! @brief Publish an I2S TX accessor as a stream bus (spec §stream credit). */
    m5::hal::v1::result_t<void> registerI2S(uint8_t bus_id, i2s::I2STxAccessor& acc);
    void setGPIOGroup(gpio::GPIOGroup& group)
    {
        _runner.setGPIOGroup(group);
        _gpio_group = &group;
        _has_gpio   = true;
    }

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
    m5::hal::v1::result_t<m5::hal::v1::error::error_t> processScript(data::ConstDataSpan script, data::Sink& out);

    /*!
      @brief Level 2: handle one already-de-framed remote message.

      Replies (response / error / hello_resp / pong) are framed into
      `out` as `data` frames on `stream_id`. Unknown message types and
      reserved bits are dropped (`droppedCount()`); a NORESP request
      stores its failure as the pending error, delivered just before the
      next synchronous reply. Errors returned by this call are transport
      (Sink) failures only.
     */
    m5::hal::v1::result_t<void> processMessage(uint8_t type, uint8_t seq, data::ConstDataSpan body,
                                               frame::FrameWriter& out, uint8_t stream_id);

    /*! @brief Messages dropped for forward compatibility (unknown type / reserved bits). */
    size_t droppedCount() const
    {
        return _dropped;
    }
    bool hasPendingError() const
    {
        return _pending_valid;
    }

    /*!
      @name GPIO change subscriptions (spec §push イベント).

      The server owns the subscription table (`kMaxSubscriptions`); the
      runner only routes `gpio_subscribe` / `gpio_unsubscribe` here.
      Subscribing records the pin's current level and notifies from the
      NEXT change on. `pollSubscriptions` reads every subscribed pin
      and, when changes are found, emits ONE `event` message whose body
      is an `evt_gpio_state` script batching them (best-effort: no ack,
      no retransmission). Subscriptions are per-connection: `hello`
      clears the table.
      @{
     */
    static constexpr size_t kMaxSubscriptions = 8;

    void clearSubscriptions();
    size_t subscriptionCount() const
    {
        return _sub_count;
    }
    /*! @brief Poll subscribed pins; returns true when an event was emitted. */
    m5::hal::v1::result_t<bool> pollSubscriptions(frame::FrameWriter& out, uint8_t stream_id);
    /*! @} */

    /*!
      @name Stream credit flow control (spec §stream credit).

      Each registered I2S binding tracks the (free, submitted) snapshot of
      the last emitted event. `pollStreamCredit` queries the runner's
      `i2sStreamStatus` per binding and accumulates the bytes the device
      drained since that snapshot — `consumed_delta = (submitted_now -
      submitted_prev) + (free_now - free_prev)` (each term a u32-wrap diff,
      the sum is non-negative). When the accumulation reaches
      `Config::stream_credit_threshold` it emits ONE `evt_stream_credit`
      event and resets the snapshot + accumulation (best-effort, lossy like
      GPIO events). `hello` resets the management state (per-connection).
      @{
     */
    void clearStreamCredit();
    /*! @brief Poll stream bindings; returns true when an event was emitted. */
    m5::hal::v1::result_t<bool> pollStreamCredit(frame::FrameWriter& out, uint8_t stream_id);
    /*! @} */

private:
    struct ExecOutcome {
        m5::hal::v1::error::error_t status = m5::hal::v1::error::error_t::OK;
        size_t offset                      = 0;      // offending offset when prescan rejected
        bool ran                           = false;  // runner.run() was invoked (slots/offset are its own)
    };

    struct Subscription {
        types::gpio_number_t pin = -1;
        bool last_level          = false;
        bool active              = false;
    };

    // Runner-side hook: routes gpio_subscribe / gpio_unsubscribe here.
    static m5::hal::v1::result_t<void> gpioSubscribeThunk(void* ctx, bool subscribe, const types::gpio_number_t* pins,
                                                          size_t count);
    m5::hal::v1::result_t<void> handleSubscribe(bool subscribe, const types::gpio_number_t* pins, size_t count);

    ExecOutcome execute(data::ConstDataSpan script);
    m5::hal::v1::error::error_t prescan(data::ConstDataSpan script, size_t& offset) const;
    m5::hal::v1::result_t<void> sendMessage(frame::FrameWriter& out, uint8_t stream_id, uint8_t type, uint8_t seq,
                                            data::ConstDataSpan body);
    m5::hal::v1::result_t<void> flushPendingError(frame::FrameWriter& out, uint8_t stream_id, uint8_t seq);
    m5::hal::v1::result_t<void> recordCapability(types::bus_kind_t kind, uint8_t bus_id);

    // Enforce the `response_scratch` contract (>= kMaxMessageSize bytes).
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

    bool _pending_valid                        = false;
    m5::hal::v1::error::error_t _pending_error = m5::hal::v1::error::error_t::OK;
    size_t _dropped                            = 0;

    gpio::GPIOGroup* _gpio_group = nullptr;  // set by setGPIOGroup (subscription reads)
    Subscription _subs[kMaxSubscriptions];
    size_t _sub_count  = 0;
    uint8_t _event_seq = 0;

    // Per-binding stream-credit tracking (one slot per I2S bus_id).
    struct StreamCredit {
        bool active        = false;  // an I2S binding is registered at this bus_id
        bool primed        = false;  // snapshot has been seeded with a first reading
        uint32_t free_prev = 0;
        uint32_t sub_prev  = 0;
        uint32_t accum     = 0;  // consumed bytes since the last emitted event
    };
    StreamCredit _stream[bytecode::kMaxBusBindings];
};

/*!
  @brief Cooperative poll adapter: feeds a Server from a Source/Sink pair.

  Register on a `service::ServiceRunner`; each `service()` call drains the
  frames that are already available and returns. The rx accessor under
  `rx` SHOULD be configured with a zero (or tiny) first-byte timeout —
  a `StreamSource::peek` blocks up to that timeout when idle, which would
  stall every poll (spec §server execution model).

  Frames that are not remote messages (other kinds, other stream_ids,
  bodies shorter than the header) are ignored; resync events from the
  FrameReader are counted (`resyncCount()`).
 */
class RemoteServerService : public service::IService {
public:
    RemoteServerService(Server& server, data::Source& rx, data::Sink& tx, uint8_t stream_id = kDefaultStreamId)
        : _server{&server}, _reader{rx}, _writer{tx}, _stream_id{stream_id}
    {
    }

    service::ServiceResult service(const service::ServiceContext& ctx) override;

    size_t resyncCount() const
    {
        return _resync;
    }

private:
    static constexpr size_t kMaxFramesPerPoll = 4;

    Server* _server = nullptr;
    frame::FrameReader _reader;
    frame::FrameWriter _writer;
    uint8_t _stream_id = kDefaultStreamId;
    size_t _resync     = 0;
};

// ---- host side --------------------------------------------------------------

/*!
  @brief Host endpoint: sends requests, awaits and decodes the replies.

  Construct over an rx Source / tx Sink pair (typically StreamSource /
  StreamSink over UART accessors; the rx scratch must lend
  `frame::kMaxWireSize` bytes). One session is one point-to-point link.

  `request()` sends a script and blocks until the matching-SEQ response
  arrives or `Config::response_timeout_ms` passes. The response script
  runs on the session's own runner: received data is in
  `runner().storedData(id)`, and a remote-reported failure is folded
  into the returned error (after `mapRemoteError`). On timeout the next
  outgoing message is preceded by a delimiter so the server's FrameReader
  resynchronizes (spec §timeout / resync); the session never retransmits
  on its own.

  `error` messages (a NORESP failure delivered before a synchronous
  reply, or a protocol-level rejection) are recorded and readable via
  `lastRemoteError()` until `clearRemoteError()`. The record is
  seq-agnostic — `lastRemoteError()` means "the most recently observed
  remote error", whether it arrived inside `request()` or in an idle
  `poll()` — because the server clears its pending slot on send, so an
  error frame crossing a host-side timeout is the only copy there is.

  `event` messages are reserved: they are passed to the handler installed
  with `setEventHandler` (dropped when none), so a future push-capable
  server stays compatible with this host loop.
 */
class RemoteSession {
public:
    struct Config {
        uint8_t stream_id            = kDefaultStreamId;
        uint32_t response_timeout_ms = 1000;
    };

    using event_handler_t = void (*)(void* ctx, uint8_t seq, data::ConstDataSpan body);

    RemoteSession(data::Source& rx, data::Sink& tx) : _reader{rx}, _writer{tx}, _runner{memory::defaultAllocator()}
    {
        // Scripts arriving here come FROM the peer: restrict the runner
        // to receive-side opcodes so a buggy/hostile server cannot drive
        // this side's buses, pins, or clock (S16 D8).
        _runner.setReceiveOnly(true);
    }
    RemoteSession(data::Source& rx, data::Sink& tx, const Config& config,
                  memory::Allocator& alloc = memory::defaultAllocator())
        : _reader{rx}, _writer{tx}, _runner{alloc}, _config{config}
    {
        _runner.setReceiveOnly(true);  // see the delegating ctor's note (S16 D8)
    }

    /*! @brief Exchange hello / hello_resp and cache the capabilities. */
    m5::hal::v1::result_t<Capabilities> hello();
    /*! @brief Liveness probe (ping / pong). */
    m5::hal::v1::result_t<void> ping();

    /*!
      @brief Send a request script and await its response.

      The response script is executed on `runner()`; a remote-reported
      error becomes this call's error. `script.size` must be
      <= kMaxBodySize.
     */
    m5::hal::v1::result_t<void> request(data::ConstDataSpan script);

    /*! @brief Fire-and-forget request (NORESP). Failures surface as a later remote error. */
    m5::hal::v1::result_t<void> requestNoResponse(data::ConstDataSpan script);

    /*!
      @brief Drain frames that are already available, dispatching events.

      Call this from the host's idle loop when subscriptions are active
      (spec §push イベント): `event` messages are executed on `runner()`
      — install the per-pin callback with
      `runner().setGpioEventHandler(...)`. Events that arrive while a
      request is awaiting its response are dispatched there as well, so
      poll() is only needed for idle periods. Returns the number of
      event messages dispatched; blocks at most for the rx stream's own
      timeouts per frame.

      `max_frames` bounds the frames consumed in one call. NOTE: each
      frame read may block up to the rx first-byte timeout, so a caller
      that has something better to do the moment one event lands (e.g. a
      sender waiting on stream credit) must pass 1 — with a steady event
      cadence shorter than the rx timeout, a larger bound turns poll()
      into a frame-paced capture loop that starves the caller.
     */
    m5::hal::v1::result_t<size_t> poll(size_t max_frames = 8);

    /*! @brief Runner that executed the most recent response script. */
    bytecode::BytecodeRunner& runner()
    {
        return _runner;
    }

    /*!
      @brief Most recently observed `error` message payload (OK when none
             arrived since the last `clearRemoteError()`).

      Seq-agnostic by design: an error frame whose delivery crosses a
      host-side timeout still lands here (from `request()` or `poll()`).
      A NORESP user pattern is: clearRemoteError() -> fire-and-forget ->
      one synchronous exchange (e.g. ping()) -> check lastRemoteError().
     */
    m5::hal::v1::error::error_t lastRemoteError() const
    {
        return _last_remote_error;
    }
    void clearRemoteError()
    {
        _last_remote_error = m5::hal::v1::error::error_t::OK;
    }

    /*! @brief Capabilities from the most recent successful `hello()`. */
    const Capabilities& capabilities() const
    {
        return _caps;
    }

    void setEventHandler(event_handler_t handler, void* ctx)
    {
        _event_handler = handler;
        _event_ctx     = ctx;
    }

    /*!
      @brief True once the transport layer has signalled a clean disconnect.

      Set when `poll()` or the internal `awaitReply()` receives
      `END_OF_STREAM` / `CLOSED` from the frame reader; cleared by
      `reset()`. Lets the caller detect a dropped link without waiting
      for the next `request()` to fail — useful to drain pending events
      and then gate reconnect logic on this flag instead of absorbing a
      `DISCONNECTED` error on every subsequent call.
     */
    bool disconnected() const
    {
        return _disconnected;
    }

    /*!
      @brief Start a fresh connection on the same objects.

      Clears the sequence counter, the pending-resync and disconnected
      flags, the recorded remote error, and the cached capabilities.
      Call after the transport reconnects (or while probing connection
      candidates) — handles are per-connection (spec §timeout / resync),
      and reset() is what begins the next connection.
     */
    void reset()
    {
        _next_seq          = 0;
        _resync_pending    = false;
        _disconnected      = false;
        _last_remote_error = m5::hal::v1::error::error_t::OK;
        _reply_len         = 0;
        _caps              = Capabilities{};
    }

    /*!
      @name Response-timeout accessors.

      Connection utilities probe with a short timeout and restore the
      configured value once established.
      @{
     */
    uint32_t responseTimeoutMs() const
    {
        return _config.response_timeout_ms;
    }
    void setResponseTimeout(uint32_t ms)
    {
        _config.response_timeout_ms = ms;
    }
    /*! @} */

private:
    enum class AwaitKind : uint8_t { response, hello_resp, pong };

    m5::hal::v1::result_t<void> sendMessage(uint8_t type, uint8_t seq, data::ConstDataSpan body);
    m5::hal::v1::result_t<void> awaitReply(AwaitKind kind, uint8_t seq);

    frame::FrameReader _reader;
    frame::FrameWriter _writer;
    bytecode::BytecodeRunner _runner;
    Config _config;

    uint8_t _next_seq    = 0;
    bool _resync_pending = false;
    bool _disconnected   = false;

    m5::hal::v1::error::error_t _last_remote_error = m5::hal::v1::error::error_t::OK;
    Capabilities _caps;

    event_handler_t _event_handler = nullptr;
    void* _event_ctx               = nullptr;

    // hello_resp body is copied here so it survives the next frame read.
    uint8_t _reply_copy[kMaxBodySize] = {};
    size_t _reply_len                 = 0;
};

/*!
  @brief Wiring bundle: a host endpoint built directly on a transport's
         `StreamReader` / `StreamWriter` pair.

  The Stream interfaces (spec/design/data_io.md §Stream アダプタ) are the
  transport seam of the remote stack: anything that can read and write
  bytes plugs in here, UART accessors today, a TCP stream tomorrow.
  RemoteLink owns the Stream adapters, their scratch buffers, and the
  `RemoteSession`, replacing the per-call boilerplate.

  This is a convenience (sugar) tier: unlike the core classes it OWNS
  its scratch (2 x `frame::kMaxWireSize`). Callers that want full buffer
  control keep composing `StreamSource` / `StreamSink` / `RemoteSession`
  by hand.

  Establishing the connection (which transport candidate actually hosts
  a remote server) is the next layer up: see the per-transport connect
  utilities, e.g. `connectRemoteSerial` in the posix UART variant.
 */
class RemoteLink {
public:
    RemoteLink(data::StreamReader& rx, data::StreamWriter& tx,
               const RemoteSession::Config& config = RemoteSession::Config{})
        : _src{rx, data::DataSpan{_rx_scratch, sizeof(_rx_scratch)}},
          _snk{tx, data::DataSpan{_tx_scratch, sizeof(_tx_scratch)}},
          _session{_src, _snk, config}
    {
    }

    RemoteSession& session()
    {
        return _session;
    }

    /*!
      @brief Start a fresh connection: drop stale buffered input and
             reset the session.

      Combines `StreamSource::discardBuffered` (a peer's boot noise or
      a previous candidate's leftovers must not desynchronize the frame
      reader) with `RemoteSession::reset()`. Bytes still queued in the
      underlying transport are not touched — flush those at the
      transport level when needed (the posix connect utility does).
     */
    void reset()
    {
        _src.discardBuffered();
        _session.reset();
    }

private:
    // Declaration order is initialization order: scratch first.
    uint8_t _rx_scratch[frame::kMaxWireSize] = {};
    uint8_t _tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource _src;
    data::StreamSink _snk;
    RemoteSession _session;
};

/*!
  @brief I2C bus proxy: a local `i2c::I2CBus` whose transfers run remotely.

  Callers use it exactly like a local bus — build an
  `I2CMasterAccessor` on top and every sugar (write / read /
  writeRegister / probe) works, because the single `transfer` override
  marshals one self-contained script (`bus_configure` + `bus_transfer`)
  per call (spec §host side). There is no remote lock: atomicity across
  multiple transfers is out of scope for this version.

  Size limits (spec §size limits): the receive length is capped at
  `kMaxTransferRx`; a tx payload that does not fit the request script is
  rejected. Both yield `INVALID_ARGUMENT` before anything is sent.

  `remote_bus_id` selects the server-side registered bus (see the
  `hello` capability list).
 */
struct RemoteI2CBus : public i2c::I2CBus {
    RemoteI2CBus(RemoteSession& session, uint8_t remote_bus_id) : _session{&session}, _remote_bus_id{remote_bus_id}
    {
    }

    /*! @brief Local bookkeeping only — the physical bus is configured server-side.
        Takes the abstract kind config: the proxy adds no fields (S17 E1). */
    m5::hal::v1::result_t<void> init(const i2c::I2CBusConfig& config);
    m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    m5::hal::v1::result_t<size_t> transfer(bus::Accessor* owner, const i2c::I2CMasterAccessConfig& cfg,
                                           const i2c::TransferDesc& desc, data::Source* tx, data::Sink* rx) override;

private:
    RemoteSession* _session = nullptr;
    uint8_t _remote_bus_id  = 0;
};

/*!
  @brief SPI bus proxy: a local `spi::SPIBus` whose transfers run remotely.

  Same shape as `RemoteI2CBus`: one self-contained script
  (`bus_configure` + `bus_transfer`) per transfer, size caps checked
  before sending. The CS / transaction window is realized server-side by
  the registered accessor for each transfer; the local
  `beginTransaction` / `endTransaction` stay no-ops (the base defaults),
  so a CS window spanning multiple transfers is not supported — compose
  command / address / data through one `TransferDesc` instead.
 */
struct RemoteSPIBus : public spi::SPIBus {
    RemoteSPIBus(RemoteSession& session, uint8_t remote_bus_id) : _session{&session}, _remote_bus_id{remote_bus_id}
    {
    }

    /*! @brief Local bookkeeping only — the physical bus is configured server-side.
        Takes the abstract kind config: the proxy adds no fields (S17 E1). */
    m5::hal::v1::result_t<void> init(const spi::SPIBusConfig& config);
    m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    m5::hal::v1::result_t<size_t> transfer(bus::Accessor* owner, const spi::SPIMasterAccessConfig& cfg,
                                           const spi::TransferDesc& desc, data::Source* tx, data::Sink* rx) override;

private:
    RemoteSession* _session = nullptr;
    uint8_t _remote_bus_id  = 0;
};

/*!
  @brief UART bus proxy: a local `uart::UARTBus` whose I/O runs remotely.

  `write` / `read` each marshal one self-contained script. Semantics
  that differ from a local UART (spec §UART proxy):
  - `read` requests at most `kMaxTransferRx` bytes per call (a larger
    `len` is clamped — UART reads may return short anyway); the remote
    short-read count is preserved through the response slot.
  - `write` returns the requested length on success; bytecode v1 carries
    no written-count back, so a remote short write is not distinguishable
    (the failing case still reports through `report_error`).
  - `readableBytes` is UNSUPPORTED (no opcode); poll with `read` and a
    small remote `first_byte_timeout_ms` instead.
  - The session response timeout is raised per call to cover the remote
    read/write timeouts (`first_byte + len * inter_byte + margin` for
    reads, `write_timeout + margin` for writes) and restored after.
 */
struct RemoteUARTBus : public uart::UARTBus {
    RemoteUARTBus(RemoteSession& session, uint8_t remote_bus_id) : _session{&session}, _remote_bus_id{remote_bus_id}
    {
    }

    /*! @brief Local bookkeeping only — the physical bus is configured server-side.
        Takes the abstract kind config: the proxy adds no fields (S17 E1). */
    m5::hal::v1::result_t<void> init(const uart::UARTBusConfig& config);
    m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    m5::hal::v1::result_t<size_t> write(bus::Accessor* owner, const uart::UARTAccessConfig& cfg, data::Source* tx,
                                        size_t len) override;
    m5::hal::v1::result_t<size_t> read(bus::Accessor* owner, const uart::UARTAccessConfig& cfg, data::Sink* rx,
                                       size_t len) override;
    m5::hal::v1::result_t<size_t> readableBytes(bus::Accessor* owner, const uart::UARTAccessConfig& cfg) override;

private:
    m5::hal::v1::result_t<size_t> runScript(data::ConstDataSpan script, uint32_t required_timeout_ms);

    RemoteSession* _session = nullptr;
    uint8_t _remote_bus_id  = 0;
};

/*!
  @brief I2S bus proxy: a local `i2s::I2SBus` whose playback runs remotely.

  `write` follows §stream credit: NORESP `bus_write_stream` bursts paced by
  a host-side credit estimate, plus a credit wait. It returns the bytes
  sent within `cfg.write_timeout_ms` (a short write is normal, same as a
  local I2S). `writableBytes` returns the host-side credit estimate without
  an RPC. An AccessConfig change is applied before the next write with a
  synchronous `bus_configure` + `bus_stream_status` script that re-syncs
  the credit baseline; the device-side `write_timeout_ms` is forced to 0
  (non-blocking) so credit — not the server poll — does the waiting.

  Credit math (spec §stream credit): the host keeps `_sent` (mod 2^32) and,
  given the device's (free, submitted) report, computes
  `credit = free - (_sent - submitted)` with u32-modular subtraction, so a
  lost event self-heals on the next report.

  The session's runner stream-credit handler is registered on construction
  with `ctx = this`; only reports matching this proxy's kind/bus_id update
  the estimate. NOTE: a single session backing multiple RemoteI2SBus
  instances is not supported in this version (one handler slot per runner).
 */
struct RemoteI2SBus : public i2s::I2SBus {
    RemoteI2SBus(RemoteSession& session, uint8_t remote_bus_id);
    /*! @brief Unregisters the credit handler when this instance still owns the slot. */
    ~RemoteI2SBus() override;

    // The session's runner holds `this` as its stream-credit handler
    // context; copying or moving would leave that pointer dangling.
    RemoteI2SBus(const RemoteI2SBus&)            = delete;
    RemoteI2SBus& operator=(const RemoteI2SBus&) = delete;
    RemoteI2SBus(RemoteI2SBus&&)                 = delete;
    RemoteI2SBus& operator=(RemoteI2SBus&&)      = delete;

    /*! @brief Local bookkeeping only — the physical bus is configured server-side.
        Takes the abstract kind config: the proxy adds no fields (S17 E1). */
    m5::hal::v1::result_t<void> init(const i2s::I2SBusConfig& config);
    m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    m5::hal::v1::result_t<size_t> write(bus::Accessor* owner, const i2s::I2SAccessConfig& cfg, data::Source* tx,
                                        size_t len) override;
    m5::hal::v1::result_t<size_t> writableBytes(bus::Accessor* owner, const i2s::I2SAccessConfig& cfg) override;

    /*!
      @brief Cap on un-acknowledged bytes in flight (the backpressure window).

      Applied on top of the credit: even with credit available, at most this
      many bytes are outstanding toward the device. The window bounds how hard
      a burst can hit the device's UART rx ring, and (together with the
      acknowledgement latency) sets the sustainable rate:
      `rate <= window / feedback_latency`. Sizing rule (measured on a CH9102
      USB bridge @3 Mbaud, macOS host): the floor is
      `2 x stream_credit_threshold + rate x p99 event latency (~100 ms)` —
      e.g. 24 kHz/16-bit mono (48 KB/s) holds its nominal rate down to a
      4 KB window. The default covers every rate the link itself sustains
      (~110 KB/s) with margin. Must be > 0; also keep it under the device's
      UART rx ring size (a full-window burst must fit).
     */
    void setStreamWindow(uint32_t bytes)
    {
        _stream_window = bytes;
    }
    uint32_t streamWindow() const
    {
        return _stream_window;
    }

    static constexpr uint32_t kDefaultStreamWindow = 16384;

private:
    // Runner stream-credit handler: only our kind/bus_id updates the estimate.
    static void streamCreditThunk(void* ctx, types::bus_kind_t kind, uint8_t bus_id, uint32_t free, uint32_t submitted);
    void onCredit(uint32_t free, uint32_t submitted);

    // (Re)apply the AccessConfig and re-sync the credit baseline with a
    // synchronous bus_configure + bus_stream_status script.
    m5::hal::v1::result_t<void> syncConfig(const i2s::I2SAccessConfig& cfg);
    // Ask the device for a fresh (free, submitted) snapshot synchronously.
    m5::hal::v1::result_t<void> syncStatus();

    // Current host-side credit estimate (bytes the device can accept now).
    uint32_t credit() const
    {
        // free - (sent - submitted), all u32-modular (spec §stream credit).
        const uint32_t in_flight = _sent - _submitted;
        return _free - in_flight;
    }

    RemoteSession* _session = nullptr;
    uint8_t _remote_bus_id  = 0;

    bool _configured = false;
    i2s::I2SAccessConfig _applied_cfg{};

    // Credit state (all mod 2^32 / wrap-safe).
    uint32_t _sent      = 0;  // cumulative bytes the host has sent
    uint32_t _submitted = 0;  // device's last-reported cumulative accepted bytes
    uint32_t _free      = 0;  // device's last-reported writable bytes

    uint32_t _stream_window = kDefaultStreamWindow;  // see setStreamWindow
};

class RemoteGPIO;

/*!
  @brief IPort implementation backing `RemoteGPIO` (one flat bank).

  The encoded pin number is the local pin index itself; all hooks
  forward to the owning RemoteGPIO's wire operations.
 */
class RemotePort : public gpio::IPort {
public:
    explicit RemotePort(RemoteGPIO& owner) : _owner{&owner}
    {
    }

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override;
    bool _readPinEncoded(uint32_t encoded_num) override;
    void _setPinModeEncoded(uint32_t encoded_num, ::m5::hal::v1::types::gpio_mode_t mode) override;
    ::m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<::m5::hal::v1::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return pin_index;
    }

private:
    RemoteGPIO* _owner = nullptr;
};

/*!
  @brief GPIO proxy: an `IGPIO` whose pins live on the remote device.

  Register it on a host-side `GPIOGroup` slot exactly like an I/O
  expander; the resulting `Pin` handles work unchanged, and their
  identity rides on the `IPort*` (no per-pin HAL context needed).

  Wire numbering: a local pin index `n` maps to the REMOTE device's
  `makeGpioNumber(remote_slot, n)` — host slot and remote slot are
  independent number spaces, and this proxy is where they convert
  (spec §GPIO proxy).

  Error path: the `IPort` hooks return void/bool, so
  - `write` / `setMode` go out as NORESP scripts; a server-side failure
    parks as the pending error and surfaces via
    `session.lastRemoteError()` after the next synchronous exchange,
  - `read` is a synchronous RPC; on failure it returns `false` and the
    error is observable on the session likewise.

  Handles never go stale: a transport drop makes operations fail (and
  reads return false) until the session is re-established; the Pin /
  registry stay valid (spec §timeout / resync).
 */
class RemoteGPIO : public gpio::IGPIO {
public:
    RemoteGPIO(RemoteSession& session, uint16_t pin_count, ::m5::hal::v1::types::gpio_slot_t remote_slot = 0)
        : _port{*this}, _session{&session}, _pin_count{pin_count}, _remote_slot{remote_slot}
    {
    }

    /*!
      @name Change subscription sugar (spec §push イベント).

      Builds the gpio_subscribe / gpio_unsubscribe script (with the
      host-to-remote pin-number conversion) and sends it as a normal
      request, so subscription failures (`OUT_OF_RESOURCE`,
      `UNSUPPORTED`, ...) come back through the expected error path.
      Events arrive via the session: install a handler with
      `session.runner().setGpioEventHandler(...)` (it receives the
      REMOTE gpio_number space) and pump `session.poll()` while idle.
      @{
     */
    m5::hal::v1::result_t<void> subscribe(::m5::hal::v1::types::gpio_local_pin_t local_pin);
    m5::hal::v1::result_t<void> unsubscribe(::m5::hal::v1::types::gpio_local_pin_t local_pin);
    /*! @} */

    gpio::IPort* portForPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return isValid(pin_index) ? &_port : nullptr;
    }
    gpio::IPort* getPort(uint8_t portNumber) const override
    {
        return portNumber == 0 ? &_port : nullptr;
    }
    uint16_t getPinCount() const override
    {
        return _pin_count;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

private:
    void wireWrite(uint32_t local_pin, bool v);
    bool wireRead(uint32_t local_pin);
    void wireSetMode(uint32_t local_pin, ::m5::hal::v1::types::gpio_mode_t mode);
    /*!
      @brief Encode and send one single-pin op as a self-contained script.

      `op(enc, &pin)` emits the one instruction; `end()` and the
      host-to-remote pin-number conversion are shared here. With `await`
      the script goes out as a normal request (reply checked); without,
      as NORESP — a server-side failure then parks as the session's
      pending error (the IPort hooks cannot return one).
     */
    template <typename Op>
    m5::hal::v1::result_t<void> pinScript(uint32_t local_pin, bool await, Op&& op);
    ::m5::hal::v1::types::gpio_number_t remoteNumber(uint32_t local_pin) const
    {
        return ::m5::hal::v1::types::makeGpioNumber(_remote_slot,
                                                    static_cast<::m5::hal::v1::types::gpio_local_pin_t>(local_pin));
    }

    mutable RemotePort _port;
    RemoteSession* _session                        = nullptr;
    uint16_t _pin_count                            = 0;
    ::m5::hal::v1::types::gpio_slot_t _remote_slot = 0;

    friend class RemotePort;
};

}  // namespace m5::hal::v1::remote

#endif
