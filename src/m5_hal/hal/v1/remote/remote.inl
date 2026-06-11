#ifndef M5_HAL_REMOTE_REMOTE_INL_
#define M5_HAL_REMOTE_REMOTE_INL_

#include "./remote.hpp"

#include <string.h>

namespace m5::hal::v1::remote {

using error_t = m5::hal::v1::error::error_t;

namespace detail {

inline uint32_t getU32LE(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// Read a u32 config field at `off` inside `cfg` when present; tolerant
// configs may legitimately be shorter (bytecode.md, tolerant decode).
inline bool u32FieldExceeds(data::ConstDataSpan cfg, size_t off, uint32_t limit)
{
    return (cfg.size >= off + 4) && (getU32LE(cfg.data + off) > limit);
}

}  // namespace detail

// ---- Server -----------------------------------------------------------------

m5::stl::expected<void, error_t> Server::recordCapability(types::bus_kind_t kind, uint8_t bus_id)
{
    if (_cap_count >= Capabilities::kMaxEntries) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    _caps[_cap_count++] = Capabilities::BusEntry{kind, bus_id};
    return {};
}

m5::stl::expected<void, error_t> Server::registerI2C(uint8_t bus_id, i2c::I2CMasterAccessor& acc)
{
    auto r = _runner.registerI2C(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::I2C, bus_id);
}

m5::stl::expected<void, error_t> Server::registerSPI(uint8_t bus_id, spi::SPIMasterAccessor& acc)
{
    auto r = _runner.registerSPI(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::SPI, bus_id);
}

m5::stl::expected<void, error_t> Server::registerUART(uint8_t bus_id, uart::UARTAccessor& acc)
{
    auto r = _runner.registerUART(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    return recordCapability(types::bus_kind_t::UART, bus_id);
}

m5::stl::expected<void, error_t> Server::registerI2S(uint8_t bus_id, i2s::I2STxAccessor& acc)
{
    auto r = _runner.registerI2S(bus_id, acc);
    if (!r.has_value()) {
        return r;
    }
    if (bus_id < bytecode::kMaxBusBindings) {
        _stream[bus_id]        = StreamCredit{};
        _stream[bus_id].active = true;
    }
    return recordCapability(types::bus_kind_t::I2S, bus_id);
}

error_t Server::prescan(data::ConstDataSpan script, size_t& offset) const
{
    // One pass over the length-prefixed instructions. Only the execution-
    // policy checks live here (delay budget, bus timeouts); malformed
    // scripts fall through so the runner reports them with its own,
    // more precise error and offset.
    size_t at            = 0;
    uint64_t delay_total = 0;
    while (at < script.size) {
        const auto lv = bytecode::decodeLenVar(data::ConstDataSpan{script.data + at, script.size - at});
        if (lv.consumed == 0 || !lv.valid || lv.value == 0) {
            break;  // terminator or malformed -> the runner decides
        }
        const size_t instr_at = at;
        at += lv.consumed;
        if (lv.value > script.size - at) {
            break;  // truncated -> the runner reports it
        }
        const uint8_t opcode = script.data[at];
        const data::ConstDataSpan payload{script.data + at + 1, lv.value - 1};
        at += lv.value;

        if (opcode == static_cast<uint8_t>(bytecode::OpCode::delay_ms)) {
            if (payload.size >= 4) {
                delay_total += detail::getU32LE(payload.data);
                if (delay_total > _config.max_delay_ms) {
                    offset = instr_at;
                    return error_t::INVALID_ARGUMENT;
                }
            }
        } else if (opcode == static_cast<uint8_t>(bytecode::OpCode::bus_configure)) {
            if (payload.size >= 2) {
                const data::ConstDataSpan cfg{payload.data + 2, payload.size - 2};
                bool exceeded = false;
                switch (static_cast<types::bus_kind_t>(payload.data[0])) {
                    case types::bus_kind_t::I2C:
                        exceeded = detail::u32FieldExceeds(cfg, 4, _config.max_bus_timeout_ms);
                        break;
                    case types::bus_kind_t::SPI:
                        exceeded = detail::u32FieldExceeds(cfg, 6, _config.max_bus_timeout_ms);
                        break;
                    case types::bus_kind_t::UART:
                        // timeout_ms / first_byte / inter_byte / write timeouts
                        exceeded = detail::u32FieldExceeds(cfg, 4, _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, 8, _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, 12, _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, 16, _config.max_bus_timeout_ms);
                        break;
                    case types::bus_kind_t::I2S:
                        // sample_rate(0) / timeout_ms(4) / write_timeout(8)
                        exceeded = detail::u32FieldExceeds(cfg, 4, _config.max_bus_timeout_ms) ||
                                   detail::u32FieldExceeds(cfg, 8, _config.max_bus_timeout_ms);
                        break;
                    default:
                        break;
                }
                if (exceeded) {
                    offset = instr_at;
                    return error_t::INVALID_ARGUMENT;
                }
            }
        }
    }
    return error_t::OK;
}

Server::ExecOutcome Server::execute(data::ConstDataSpan script)
{
    ExecOutcome out;
    out.status = prescan(script, out.offset);
    if (error::isError(out.status)) {
        return out;
    }
    out.ran    = true;
    auto r     = _runner.run(script);
    out.status = r.has_value() ? error_t::OK : r.error();
    return out;
}

m5::stl::expected<error_t, error_t> Server::processScript(data::ConstDataSpan script, data::Sink& out)
{
    const ExecOutcome outcome = execute(script);
    if (outcome.ran) {
        auto w = _runner.writeResponse(out, outcome.status);
        if (!w.has_value()) {
            return m5::stl::make_unexpected(w.error());
        }
    } else {
        // Rejected before execution: a minimal response script carries
        // the policy error and the offending instruction's offset.
        bytecode::BytecodeEncoder enc{out};
        auto e = enc.reportError(outcome.status, outcome.offset);
        if (e.has_value()) {
            e = enc.end();
        }
        if (!e.has_value()) {
            return m5::stl::make_unexpected(e.error());
        }
    }
    return outcome.status;
}

m5::stl::expected<void, error_t> Server::sendMessage(frame::FrameWriter& out, uint8_t stream_id, uint8_t type,
                                                     uint8_t seq, data::ConstDataSpan body)
{
    if (body.size > kMaxBodySize || _scratch.size < kHeaderSize + body.size) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    _scratch.data[0] = type;
    _scratch.data[1] = seq;
    if (body.size != 0) {
        ::memcpy(_scratch.data + kHeaderSize, body.data, body.size);
    }
    auto w = out.writeData(stream_id, data::ConstDataSpan{_scratch.data, kHeaderSize + body.size});
    if (!w.has_value()) {
        return m5::stl::make_unexpected(w.error());
    }
    return {};
}

m5::stl::expected<void, error_t> Server::flushPendingError(frame::FrameWriter& out, uint8_t stream_id, uint8_t seq)
{
    if (!_pending_valid) {
        return {};
    }
    const uint8_t code = static_cast<uint8_t>(static_cast<int8_t>(_pending_error));
    auto r = sendMessage(out, stream_id, static_cast<uint8_t>(MessageType::error), seq, data::ConstDataSpan{&code, 1});
    if (r.has_value()) {
        _pending_valid = false;
        _pending_error = error_t::OK;
    }
    return r;
}

m5::stl::expected<void, error_t> Server::processMessage(uint8_t type, uint8_t seq, data::ConstDataSpan body,
                                                        frame::FrameWriter& out, uint8_t stream_id)
{
    const uint8_t kind4 = type & kTypeKindMask;
    const bool noresp   = (type & kTypeNorespBit) != 0;

    // Reserved bits / flag combinations this version does not define are
    // dropped so a newer peer never derails an older endpoint.
    if ((type & (kTypeMoreBit | kTypeReservedBits)) != 0 ||
        (noresp && kind4 != static_cast<uint8_t>(MessageType::request))) {
        ++_dropped;
        return {};
    }

    switch (static_cast<MessageType>(kind4)) {
        case MessageType::hello: {
            clearSubscriptions();
            clearStreamCredit();
            auto f = flushPendingError(out, stream_id, seq);
            if (!f.has_value()) {
                return f;
            }
            uint8_t caps[3 + 2 * Capabilities::kMaxEntries];
            caps[0]   = kProtocolVersion;
            caps[1]   = _has_gpio ? 0x01 : 0x00;
            caps[2]   = static_cast<uint8_t>(_cap_count);
            size_t at = 3;
            for (size_t i = 0; i < _cap_count; ++i) {
                caps[at++] = static_cast<uint8_t>(_caps[i].kind);
                caps[at++] = _caps[i].bus_id;
            }
            return sendMessage(out, stream_id, static_cast<uint8_t>(MessageType::hello_resp), seq,
                               data::ConstDataSpan{caps, at});
        }
        case MessageType::request: {
            if (noresp) {
                const ExecOutcome outcome = execute(body);
                if (error::isError(outcome.status)) {
                    _pending_valid = true;
                    _pending_error = outcome.status;
                }
                return {};
            }
            auto f = flushPendingError(out, stream_id, seq);
            if (!f.has_value()) {
                return f;
            }
            if (_scratch.size < kHeaderSize) {
                return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
            }
            const ExecOutcome outcome = execute(body);
            // Build the response message in place: header + response script.
            _scratch.data[0] = static_cast<uint8_t>(MessageType::response);
            _scratch.data[1] = seq;
            data::MemorySink script_sink{data::DataSpan{_scratch.data + kHeaderSize, _scratch.size - kHeaderSize}};
            bool built = false;
            if (outcome.ran) {
                built = _runner.writeResponse(script_sink, outcome.status).has_value();
            }
            if (!built) {
                // Either a pre-execution rejection or a response that did
                // not fit the scratch: degrade to a report-only script.
                const error_t code =
                    outcome.ran ? error_t::BUFFER_OVERFLOW : outcome.status;  // spec §server execution model
                script_sink =
                    data::MemorySink{data::DataSpan{_scratch.data + kHeaderSize, _scratch.size - kHeaderSize}};
                bytecode::BytecodeEncoder enc{script_sink};
                auto e = enc.reportError(code, outcome.ran ? 0 : outcome.offset);
                if (e.has_value()) {
                    e = enc.end();
                }
                if (!e.has_value()) {
                    return m5::stl::make_unexpected(e.error());
                }
            }
            auto w = out.writeData(stream_id, data::ConstDataSpan{_scratch.data, kHeaderSize + script_sink.written()});
            if (!w.has_value()) {
                return m5::stl::make_unexpected(w.error());
            }
            return {};
        }
        case MessageType::ping: {
            auto f = flushPendingError(out, stream_id, seq);
            if (!f.has_value()) {
                return f;
            }
            return sendMessage(out, stream_id, static_cast<uint8_t>(MessageType::pong), seq, data::ConstDataSpan{});
        }
        default:
            ++_dropped;
            return {};
    }
}

// ---- Server subscription management -----------------------------------------

void Server::clearSubscriptions()
{
    for (size_t i = 0; i < kMaxSubscriptions; ++i) {
        _subs[i] = Subscription{};
    }
    _sub_count = 0;
}

m5::stl::expected<void, error_t> Server::gpioSubscribeThunk(void* ctx, bool subscribe, const types::gpio_number_t* pins,
                                                            size_t count)
{
    return static_cast<Server*>(ctx)->handleSubscribe(subscribe, pins, count);
}

m5::stl::expected<void, error_t> Server::handleSubscribe(bool subscribe, const types::gpio_number_t* pins, size_t count)
{
    if (_gpio_group == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }

    if (!subscribe) {
        // Unsubscribe
        if (count == 0) {
            // Unsubscribe all
            clearSubscriptions();
            return {};
        }
        // Remove matching entries (compact the array)
        for (size_t p = 0; p < count; ++p) {
            for (size_t i = 0; i < _sub_count; ++i) {
                if (_subs[i].pin == pins[p]) {
                    // Shift remaining entries down
                    for (size_t j = i; j + 1 < _sub_count; ++j) {
                        _subs[j] = _subs[j + 1];
                    }
                    _subs[--_sub_count] = Subscription{};
                    break;  // each pin appears at most once
                }
            }
            // Non-existent pin unsubscribe is idempotent success
        }
        return {};
    }

    // Subscribe: validate and add each pin
    for (size_t p = 0; p < count; ++p) {
        // Validate the pin exists
        auto pin_handle = _gpio_group->tryGetPin(pins[p]);
        if (!pin_handle.has_value()) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        // Check for duplicate (idempotent)
        bool found = false;
        for (size_t i = 0; i < _sub_count; ++i) {
            if (_subs[i].pin == pins[p]) {
                found = true;
                break;
            }
        }
        if (found) {
            continue;  // idempotent success
        }
        if (_sub_count >= kMaxSubscriptions) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        // Record current level as baseline (notify from next change on)
        Subscription& sub = _subs[_sub_count++];
        sub.pin           = pins[p];
        sub.last_level    = pin_handle.value().read();
        sub.active        = true;
    }
    return {};
}

m5::stl::expected<bool, error_t> Server::pollSubscriptions(frame::FrameWriter& out, uint8_t stream_id)
{
    if (_sub_count == 0) {
        return false;
    }

    // Collect changed pins
    types::gpio_number_t changed_pins[kMaxSubscriptions];
    bool changed_levels[kMaxSubscriptions];
    size_t changed_count = 0;

    for (size_t i = 0; i < _sub_count; ++i) {
        auto pin_handle = _gpio_group->tryGetPin(_subs[i].pin);
        if (!pin_handle.has_value()) {
            continue;  // pin became invalid; skip gracefully
        }
        const bool current = pin_handle.value().read();
        if (current != _subs[i].last_level) {
            changed_pins[changed_count]   = _subs[i].pin;
            changed_levels[changed_count] = current;
            ++changed_count;
        }
    }

    if (changed_count == 0) {
        return false;
    }

    // Update last_level before sending (best-effort: no rollback on send failure)
    for (size_t c = 0; c < changed_count; ++c) {
        for (size_t i = 0; i < _sub_count; ++i) {
            if (_subs[i].pin == changed_pins[c]) {
                _subs[i].last_level = changed_levels[c];
                break;
            }
        }
    }

    // Build event message: _scratch[0] = TYPE event, _scratch[1] = _event_seq++
    _scratch.data[0] = static_cast<uint8_t>(MessageType::event);
    _scratch.data[1] = _event_seq++;
    data::MemorySink script_sink{data::DataSpan{_scratch.data + 2, _scratch.size - 2}};
    bytecode::BytecodeEncoder enc{script_sink};
    auto e = enc.evtGpioState(changed_pins, changed_levels, changed_count);
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error());
    }
    const size_t msg_size = 2 + script_sink.written();
    auto w                = out.writeData(stream_id, data::ConstDataSpan{_scratch.data, msg_size});
    if (!w.has_value()) {
        return m5::stl::make_unexpected(w.error());
    }
    return true;
}

// ---- Server stream credit ----------------------------------------------------

void Server::clearStreamCredit()
{
    for (size_t i = 0; i < bytecode::kMaxBusBindings; ++i) {
        const bool active = _stream[i].active;
        _stream[i]        = StreamCredit{};
        _stream[i].active = active;  // a binding registration survives a reconnect
    }
}

m5::stl::expected<bool, error_t> Server::pollStreamCredit(frame::FrameWriter& out, uint8_t stream_id)
{
    bool emitted = false;
    for (uint8_t bus_id = 0; bus_id < bytecode::kMaxBusBindings; ++bus_id) {
        StreamCredit& sc = _stream[bus_id];
        if (!sc.active) {
            continue;
        }
        auto status = _runner.i2sStreamStatus(bus_id);
        if (!status.has_value()) {
            continue;  // binding dropped or backend hiccup: skip gracefully
        }
        const uint32_t free_now = status.value().free;
        const uint32_t sub_now  = status.value().submitted;
        if (!sc.primed) {
            sc.free_prev = free_now;
            sc.sub_prev  = sub_now;
            sc.accum     = 0;
            sc.primed    = true;
            continue;
        }
        // Bytes the device drained since the last snapshot. Both terms are
        // u32-wrap differences; their sum is the consumed delta (>= 0):
        //   consumed = (submitted_now - submitted_prev)  [newly accepted]
        //            + (free_now - free_prev)            [freed by DMA]
        const uint32_t consumed = (sub_now - sc.sub_prev) + (free_now - sc.free_prev);
        sc.accum += consumed;
        sc.free_prev = free_now;
        sc.sub_prev  = sub_now;
        if (sc.accum < _config.stream_credit_threshold) {
            continue;
        }

        // Emit one evt_stream_credit event (best-effort, like GPIO events).
        _scratch.data[0] = static_cast<uint8_t>(MessageType::event);
        _scratch.data[1] = _event_seq++;
        data::MemorySink script_sink{data::DataSpan{_scratch.data + 2, _scratch.size - 2}};
        bytecode::BytecodeEncoder enc{script_sink};
        auto e = enc.evtStreamCredit(types::bus_kind_t::I2S, bus_id, free_now, sub_now);
        if (e.has_value()) {
            e = enc.end();
        }
        if (!e.has_value()) {
            return m5::stl::make_unexpected(e.error());
        }
        const size_t msg_size = 2 + script_sink.written();
        auto w                = out.writeData(stream_id, data::ConstDataSpan{_scratch.data, msg_size});
        if (!w.has_value()) {
            return m5::stl::make_unexpected(w.error());
        }
        sc.accum = 0;  // reset accumulation after a successful emit
        emitted  = true;
    }
    return emitted;
}

// ---- RemoteServerService -----------------------------------------------------

service::ServiceResult RemoteServerService::service(const service::ServiceContext&)
{
    size_t processed = 0;
    for (size_t i = 0; i < kMaxFramesPerPoll; ++i) {
        frame::View view;
        auto r = _reader.next(view);
        if (!r.has_value()) {
            if (r.error() == error_t::TIMEOUT_ERROR || r.error() == error_t::END_OF_STREAM) {
                break;  // nothing (more) to read this poll
            }
            return service::ServiceResult::Error;
        }
        if (r.value().status != frame::DecodeStatus::ok) {
            if (r.value().status != frame::DecodeStatus::need_more) {
                ++_resync;
                continue;
            }
            break;
        }
        if (view.kind != frame::Kind::data || view.kind_body.size < 1 + kHeaderSize ||
            view.kind_body.data[0] != _stream_id) {
            continue;  // not a remote message on our channel
        }
        const uint8_t* msg    = view.kind_body.data + 1;
        const size_t msg_size = view.kind_body.size - 1;
        auto p =
            _server->processMessage(msg[0], msg[1], data::ConstDataSpan{msg + 2, msg_size - 2}, _writer, _stream_id);
        if (!p.has_value()) {
            return service::ServiceResult::Error;
        }
        ++processed;
    }

    // Poll GPIO subscriptions and emit event if any pin changed.
    auto poll_r = _server->pollSubscriptions(_writer, _stream_id);
    if (!poll_r.has_value()) {
        return service::ServiceResult::Error;
    }
    if (poll_r.value()) {
        ++processed;
    }

    // Poll stream-credit bindings and emit a credit event when a binding
    // has drained past the threshold.
    auto credit_r = _server->pollStreamCredit(_writer, _stream_id);
    if (!credit_r.has_value()) {
        return service::ServiceResult::Error;
    }
    if (credit_r.value()) {
        ++processed;
    }

    return processed != 0 ? service::ServiceResult::Progress : service::ServiceResult::Idle;
}

// ---- RemoteSession ------------------------------------------------------------

m5::stl::expected<void, error_t> RemoteSession::sendMessage(uint8_t type, uint8_t seq, data::ConstDataSpan body)
{
    if (body.size > kMaxBodySize) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    if (_resync_pending) {
        // A delimiter gives the server's FrameReader a hard resync point
        // after our previous exchange timed out mid-frame (spec §resync).
        auto d = _writer.writeDelimiter();
        if (!d.has_value()) {
            return m5::stl::make_unexpected(d.error());
        }
        _resync_pending = false;
    }
    uint8_t msg[kMaxMessageSize];
    msg[0] = type;
    msg[1] = seq;
    if (body.size != 0) {
        ::memcpy(msg + kHeaderSize, body.data, body.size);
    }
    auto w = _writer.writeData(_config.stream_id, data::ConstDataSpan{msg, kHeaderSize + body.size});
    if (!w.has_value()) {
        return m5::stl::make_unexpected(w.error() == error_t::CLOSED ? error_t::DISCONNECTED : w.error());
    }
    return {};
}

m5::stl::expected<void, error_t> RemoteSession::awaitReply(AwaitKind kind, uint8_t seq)
{
    const uint8_t want   = kind == AwaitKind::response     ? static_cast<uint8_t>(MessageType::response)
                           : kind == AwaitKind::hello_resp ? static_cast<uint8_t>(MessageType::hello_resp)
                                                           : static_cast<uint8_t>(MessageType::pong);
    const uint32_t start = static_cast<uint32_t>(m5::utility::millis());
    for (;;) {
        if (static_cast<uint32_t>(m5::utility::millis()) - start > _config.response_timeout_ms) {
            _resync_pending = true;
            return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
        }
        frame::View view;
        auto r = _reader.next(view);
        if (!r.has_value()) {
            if (r.error() == error_t::TIMEOUT_ERROR) {
                continue;  // deadline check at the top of the loop
            }
            if (r.error() == error_t::END_OF_STREAM || r.error() == error_t::CLOSED) {
                _disconnected = true;
                return m5::stl::make_unexpected(error_t::DISCONNECTED);
            }
            return m5::stl::make_unexpected(r.error());
        }
        if (r.value().status != frame::DecodeStatus::ok) {
            continue;  // resync event or padding; keep waiting
        }
        if (view.kind != frame::Kind::data || view.kind_body.size < 1 + kHeaderSize ||
            view.kind_body.data[0] != _config.stream_id) {
            continue;
        }
        const uint8_t* msg    = view.kind_body.data + 1;
        const size_t msg_size = view.kind_body.size - 1;
        const uint8_t type    = msg[0];
        if ((type & (kTypeNorespBit | kTypeMoreBit | kTypeReservedBits)) != 0) {
            continue;  // reserved encodings: drop (forward compatibility)
        }
        const uint8_t kind4 = type & kTypeKindMask;
        const data::ConstDataSpan body{msg + 2, msg_size - 2};
        if (kind4 == static_cast<uint8_t>(MessageType::error)) {
            if (msg[1] == seq && body.size >= 1) {
                _last_remote_error = mapRemoteError(static_cast<int8_t>(body.data[0]));
            }
            continue;  // the synchronous reply still follows
        }
        if (kind4 == static_cast<uint8_t>(MessageType::event)) {
            if (_event_handler != nullptr) {
                _event_handler(_event_ctx, msg[1], body);
            }
            // Run event script through runner so evt_gpio_state reaches
            // the registered gpio event handler (lossy: ignore run errors).
            (void)_runner.run(body);
            continue;
        }
        if (kind4 == want && msg[1] == seq) {
            // Copy out: the borrow ends at the next FrameReader::next().
            _reply_len = body.size;
            if (body.size != 0) {
                ::memcpy(_reply_copy, body.data, body.size);
            }
            return {};
        }
        // Stale reply (older seq) or a message type a host does not
        // consume: drop and keep waiting.
    }
}

m5::stl::expected<void, error_t> RemoteSession::request(data::ConstDataSpan script)
{
    if (_disconnected) {
        return m5::stl::make_unexpected(error_t::DISCONNECTED);
    }
    if (script.size > kMaxBodySize) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const uint8_t seq = _next_seq++;
    auto s            = sendMessage(static_cast<uint8_t>(MessageType::request), seq, script);
    if (!s.has_value()) {
        return s;
    }
    auto a = awaitReply(AwaitKind::response, seq);
    if (!a.has_value()) {
        return a;
    }
    auto run = _runner.run(data::ConstDataSpan{_reply_copy, _reply_len});
    if (!run.has_value()) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (!_runner.statusReported()) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const error_t status = mapRemoteError(static_cast<int8_t>(_runner.reportedStatus()));
    if (error::isError(status)) {
        return m5::stl::make_unexpected(status);
    }
    return {};
}

m5::stl::expected<void, error_t> RemoteSession::requestNoResponse(data::ConstDataSpan script)
{
    if (_disconnected) {
        return m5::stl::make_unexpected(error_t::DISCONNECTED);
    }
    if (script.size > kMaxBodySize) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    return sendMessage(static_cast<uint8_t>(MessageType::request) | kTypeNorespBit, _next_seq++, script);
}

m5::stl::expected<Capabilities, error_t> RemoteSession::hello()
{
    if (_disconnected) {
        return m5::stl::make_unexpected(error_t::DISCONNECTED);
    }
    const uint8_t seq = _next_seq++;
    auto s            = sendMessage(static_cast<uint8_t>(MessageType::hello), seq, data::ConstDataSpan{});
    if (!s.has_value()) {
        return m5::stl::make_unexpected(s.error());
    }
    auto a = awaitReply(AwaitKind::hello_resp, seq);
    if (!a.has_value()) {
        return m5::stl::make_unexpected(a.error());
    }
    if (_reply_len < 3) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    Capabilities caps;
    caps.proto_ver = _reply_copy[0];
    caps.has_gpio  = (_reply_copy[1] & 0x01) != 0;
    const size_t n = _reply_copy[2];
    // Known prefix only; trailing bytes are a forward-compatible extension.
    if (_reply_len < 3 + 2 * n) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    caps.bus_count = n <= Capabilities::kMaxEntries ? n : Capabilities::kMaxEntries;
    for (size_t i = 0; i < caps.bus_count; ++i) {
        caps.buses[i].kind   = static_cast<types::bus_kind_t>(_reply_copy[3 + 2 * i]);
        caps.buses[i].bus_id = _reply_copy[4 + 2 * i];
    }
    if (caps.proto_ver != kProtocolVersion) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    _caps = caps;
    return caps;
}

m5::stl::expected<void, error_t> RemoteSession::ping()
{
    if (_disconnected) {
        return m5::stl::make_unexpected(error_t::DISCONNECTED);
    }
    const uint8_t seq = _next_seq++;
    auto s            = sendMessage(static_cast<uint8_t>(MessageType::ping), seq, data::ConstDataSpan{});
    if (!s.has_value()) {
        return s;
    }
    return awaitReply(AwaitKind::pong, seq);
}

m5::stl::expected<size_t, error_t> RemoteSession::poll(size_t max_frames)
{
    size_t events = 0;

    for (size_t i = 0; i < max_frames; ++i) {
        frame::View view;
        auto r = _reader.next(view);
        if (!r.has_value()) {
            if (r.error() == error_t::TIMEOUT_ERROR) {
                break;  // nothing (more) to read this poll
            }
            if (r.error() == error_t::END_OF_STREAM || r.error() == error_t::CLOSED) {
                _disconnected = true;
                break;
            }
            break;  // other error: stop draining
        }
        if (r.value().status != frame::DecodeStatus::ok) {
            if (r.value().status == frame::DecodeStatus::need_more) {
                break;
            }
            continue;  // resync event; keep draining
        }
        if (view.kind != frame::Kind::data || view.kind_body.size < 1 + kHeaderSize ||
            view.kind_body.data[0] != _config.stream_id) {
            continue;  // not a remote message on our channel
        }
        const uint8_t* msg    = view.kind_body.data + 1;
        const size_t msg_size = view.kind_body.size - 1;
        const uint8_t type    = msg[0];
        if ((type & (kTypeNorespBit | kTypeMoreBit | kTypeReservedBits)) != 0) {
            continue;  // drop reserved encodings
        }
        const uint8_t kind4            = type & kTypeKindMask;
        const data::ConstDataSpan body = {msg + 2, msg_size - 2};
        if (kind4 == static_cast<uint8_t>(MessageType::event)) {
            if (_event_handler != nullptr) {
                _event_handler(_event_ctx, msg[1], body);
            }
            (void)_runner.run(body);
            ++events;
        }
        // Other message types during idle poll are discarded
    }
    return events;
}

// ---- RemoteI2CBus -------------------------------------------------------------

m5::stl::expected<void, error_t> RemoteI2CBus::init(const bus::BusConfig& config)
{
    if (config.getBusKind() != types::bus_kind_t::I2C) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    _config = static_cast<const i2c::I2CBusConfig&>(config);
    return {};
}

m5::stl::expected<size_t, error_t> RemoteI2CBus::transfer(bus::Accessor* owner, const i2c::I2CMasterAccessConfig& cfg,
                                                          const i2c::TransferDesc& desc, data::Source* tx,
                                                          data::Sink* rx)
{
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

    // Drain tx into a staging buffer; anything beyond what one request
    // script can carry is a size-limit violation (spec §size limits).
    uint8_t tx_buf[kMaxBodySize];
    size_t tx_len = 0;
    if (tx != nullptr) {
        while (!tx->eof()) {
            const size_t space = sizeof(tx_buf) - tx_len;
            auto p             = tx->peek(space != 0 ? space : 1);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;  // EOF
            }
            if (space == 0) {
                return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
            }
            ::memcpy(tx_buf + tx_len, p.value().data, p.value().size);
            tx_len += p.value().size;
            auto adv = tx->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
        }
    }

    // The sink's reservable size is the requested receive length
    // (same convention as the local variants).
    data::DataSpan rx_span{};
    size_t rx_len = 0;
    if (rx != nullptr) {
        auto rsv = rx->reserve(SIZE_MAX);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        rx_span = rsv.value();
        rx_len  = rx_span.size;
        if (rx_len > kMaxTransferRx) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
    }

    // One self-contained script per transfer: configure + transfer.
    uint8_t script_buf[kMaxBodySize];
    data::MemorySink script_sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{script_sink};
    const uint8_t store_id = (rx != nullptr) ? uint8_t{0} : bytecode::kDiscardStoreId;
    auto e                 = enc.configure(_remote_bus_id, cfg);
    if (e.has_value()) {
        e = enc.transfer(_remote_bus_id, desc, data::ConstDataSpan{tx_buf, tx_len}, rx_len, store_id);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        // An over-long script shows up as the encoder failing to reserve.
        return m5::stl::make_unexpected(e.error() == error_t::BUFFER_OVERFLOW || e.error() == error_t::CLOSED
                                            ? error_t::INVALID_ARGUMENT
                                            : e.error());
    }

    auto rq = _session->request(data::ConstDataSpan{script_buf, script_sink.written()});
    if (!rq.has_value()) {
        return m5::stl::make_unexpected(rq.error());
    }

    size_t rx_got = 0;
    if (rx != nullptr) {
        const auto stored = _session->runner().storedData(store_id);
        if (stored.size > rx_span.size) {
            return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
        }
        if (stored.size != 0) {
            ::memcpy(rx_span.data, stored.data, stored.size);
        }
        auto c = rx->commit(stored.size);
        if (!c.has_value()) {
            return m5::stl::make_unexpected(c.error());
        }
        rx_got = stored.size;
    }
    return desc.prefix_len + tx_len + rx_got;
}

// ---- RemoteSPIBus -------------------------------------------------------------

m5::stl::expected<void, error_t> RemoteSPIBus::init(const bus::BusConfig& config)
{
    if (config.getBusKind() != types::bus_kind_t::SPI) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    _config = static_cast<const spi::SPIBusConfig&>(config);
    return {};
}

m5::stl::expected<size_t, error_t> RemoteSPIBus::transfer(bus::Accessor* owner, const spi::SPIMasterAccessConfig& cfg,
                                                          const spi::TransferDesc& desc, data::Source* tx,
                                                          data::Sink* rx)
{
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

    uint8_t tx_buf[kMaxBodySize];
    size_t tx_len = 0;
    if (tx != nullptr) {
        while (!tx->eof()) {
            const size_t space = sizeof(tx_buf) - tx_len;
            auto p             = tx->peek(space != 0 ? space : 1);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            if (space == 0) {
                return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
            }
            ::memcpy(tx_buf + tx_len, p.value().data, p.value().size);
            tx_len += p.value().size;
            auto adv = tx->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
        }
    }

    data::DataSpan rx_span{};
    size_t rx_len = 0;
    if (rx != nullptr) {
        auto rsv = rx->reserve(SIZE_MAX);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        rx_span = rsv.value();
        rx_len  = rx_span.size;
        if (rx_len > kMaxTransferRx) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
    }

    uint8_t script_buf[kMaxBodySize];
    data::MemorySink script_sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{script_sink};
    const uint8_t store_id = (rx != nullptr) ? uint8_t{0} : bytecode::kDiscardStoreId;
    auto e                 = enc.configure(_remote_bus_id, cfg);
    if (e.has_value()) {
        e = enc.transfer(_remote_bus_id, desc, data::ConstDataSpan{tx_buf, tx_len}, rx_len, store_id);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error() == error_t::BUFFER_OVERFLOW || e.error() == error_t::CLOSED
                                            ? error_t::INVALID_ARGUMENT
                                            : e.error());
    }

    auto rq = _session->request(data::ConstDataSpan{script_buf, script_sink.written()});
    if (!rq.has_value()) {
        return m5::stl::make_unexpected(rq.error());
    }

    size_t rx_got = 0;
    if (rx != nullptr) {
        const auto stored = _session->runner().storedData(store_id);
        if (stored.size > rx_span.size) {
            return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
        }
        if (stored.size != 0) {
            ::memcpy(rx_span.data, stored.data, stored.size);
        }
        auto c = rx->commit(stored.size);
        if (!c.has_value()) {
            return m5::stl::make_unexpected(c.error());
        }
        rx_got = stored.size;
    }
    return tx_len + rx_got;
}

// ---- RemoteUARTBus ------------------------------------------------------------

m5::stl::expected<void, error_t> RemoteUARTBus::init(const bus::BusConfig& config)
{
    if (config.getBusKind() != types::bus_kind_t::UART) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    _config = static_cast<const uart::UARTBusConfig&>(config);
    return {};
}

// Send one script with the session timeout temporarily raised to cover
// the remote-side blocking time (spec §UART proxy, timeout composition).
m5::stl::expected<size_t, error_t> RemoteUARTBus::runScript(data::ConstDataSpan script, uint32_t required_timeout_ms)
{
    auto& session        = *_session;
    const uint32_t saved = session.responseTimeoutMs();
    if (required_timeout_ms > saved) {
        session.setResponseTimeout(required_timeout_ms);
    }
    auto rq = session.request(script);
    session.setResponseTimeout(saved);
    if (!rq.has_value()) {
        return m5::stl::make_unexpected(rq.error());
    }
    return size_t{0};
}

m5::stl::expected<size_t, error_t> RemoteUARTBus::write(bus::Accessor* owner, const uart::UARTAccessConfig& cfg,
                                                        data::Source* tx, size_t len)
{
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

    uint8_t tx_buf[kMaxBodySize];
    size_t tx_len = 0;
    while (tx != nullptr && tx_len < len && !tx->eof()) {
        if (tx_len >= sizeof(tx_buf)) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);  // > one script's worth
        }
        const size_t cap  = sizeof(tx_buf) - tx_len;
        const size_t want = (len - tx_len) < cap ? (len - tx_len) : cap;
        auto p            = tx->peek(want);
        if (!p.has_value()) {
            return m5::stl::make_unexpected(p.error());
        }
        if (p.value().size == 0) {
            break;
        }
        ::memcpy(tx_buf + tx_len, p.value().data, p.value().size);
        tx_len += p.value().size;
        auto adv = tx->advance(p.value().size);
        if (!adv.has_value()) {
            return m5::stl::make_unexpected(adv.error());
        }
    }

    uint8_t script_buf[kMaxBodySize];
    data::MemorySink script_sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{script_sink};
    auto e = enc.configure(_remote_bus_id, cfg);
    if (e.has_value()) {
        e = enc.uartTransfer(_remote_bus_id, data::ConstDataSpan{tx_buf, tx_len}, 0, bytecode::kDiscardStoreId);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error() == error_t::BUFFER_OVERFLOW || e.error() == error_t::CLOSED
                                            ? error_t::INVALID_ARGUMENT
                                            : e.error());
    }

    auto rq = runScript(data::ConstDataSpan{script_buf, script_sink.written()},
                        cfg.write_timeout_ms + kRemoteUartTimeoutMarginMs);
    if (!rq.has_value()) {
        return m5::stl::make_unexpected(rq.error());
    }
    // bytecode v1 carries no written-count back; a remote short write is
    // indistinguishable from a full one (a failure still report_errors).
    return tx_len;
}

m5::stl::expected<size_t, error_t> RemoteUARTBus::read(bus::Accessor* owner, const uart::UARTAccessConfig& cfg,
                                                       data::Sink* rx, size_t len)
{
    (void)owner;
    if (_session == nullptr || rx == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

    auto rsv = rx->reserve(SIZE_MAX);
    if (!rsv.has_value()) {
        return m5::stl::make_unexpected(rsv.error());
    }
    data::DataSpan rx_span = rsv.value();
    // UART reads may return short anyway, so clamp instead of rejecting.
    size_t rx_len = len < rx_span.size ? len : rx_span.size;
    if (rx_len > kMaxTransferRx) {
        rx_len = kMaxTransferRx;
    }

    uint8_t script_buf[kMaxBodySize];
    data::MemorySink script_sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{script_sink};
    auto e = enc.configure(_remote_bus_id, cfg);
    if (e.has_value()) {
        e = enc.uartTransfer(_remote_bus_id, data::ConstDataSpan{}, rx_len, 0);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error());
    }

    // Worst-case remote blocking: waiting for the first byte plus one
    // inter-byte gap per remaining byte.
    const uint32_t required = cfg.first_byte_timeout_ms +
                              (rx_len != 0 ? static_cast<uint32_t>(rx_len - 1) * cfg.inter_byte_timeout_ms : 0) +
                              kRemoteUartTimeoutMarginMs;
    auto rq = runScript(data::ConstDataSpan{script_buf, script_sink.written()}, required);
    if (!rq.has_value()) {
        return m5::stl::make_unexpected(rq.error());
    }

    const auto stored = _session->runner().storedData(0);
    if (stored.size > rx_span.size) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (stored.size != 0) {
        ::memcpy(rx_span.data, stored.data, stored.size);
    }
    auto c = rx->commit(stored.size);
    if (!c.has_value()) {
        return m5::stl::make_unexpected(c.error());
    }
    return stored.size;  // remote short reads come through as-is
}

m5::stl::expected<size_t, error_t> RemoteUARTBus::readableBytes(bus::Accessor* owner, const uart::UARTAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    // No opcode carries "readable byte count"; poll with read() and a
    // small remote first_byte_timeout_ms instead (spec §UART proxy).
    return m5::stl::make_unexpected(error_t::UNSUPPORTED);
}

// ---- RemoteI2SBus -------------------------------------------------------------

namespace detail {
// Max data bytes one bus_write_stream message body can carry. The body is
// [TYPE:1][SEQ:1] (frame message header, kMaxBodySize already nets that out)
// then the instruction [LenVar(size)][opcode:1][kind:1][bus_id:1][data...].
// For the data sizes in play the LenVar is its 1-byte form, so the
// per-message overhead is lenVar(1) + opcode(1) + kind(1) + bus_id(1) = 4.
constexpr size_t kStreamInstrOverhead = 4;
constexpr size_t kMaxStreamChunk      = kMaxBodySize - kStreamInstrOverhead;
// How long a credit stall persists before a synchronous status re-sync
// (also the point where any pending NORESP error flows back to the session).
constexpr uint32_t kCreditResyncMs = 50;
// The in-flight window cap lives on the proxy (RemoteI2SBus::_stream_window,
// default kDefaultStreamWindow, runtime-tunable via setStreamWindow). Why a
// cap exists at all: credit alone allows bursting a full DMA capacity at
// once, which can drive the device's UART rx ring/FIFO to the brink and trip
// the IDF driver's rx-overflow handling (observed as a one-way rx stall on
// hardware). The window also sets the throughput ceiling together with the
// acknowledgement feedback latency: sustained rate <= window / latency. The
// default is sized for the measured worst case on a USB CDC bridge; the
// device example sizes its DMA buffer and rx ring above this default.
}  // namespace detail

RemoteI2SBus::RemoteI2SBus(RemoteSession& session, uint8_t remote_bus_id)
    : _session{&session}, _remote_bus_id{remote_bus_id}
{
    // Register the runner credit handler now (ctx = this). Only reports
    // matching our kind/bus_id update the estimate; one handler slot per
    // runner means one RemoteI2SBus per session (see the class comment).
    _session->runner().setStreamCreditHandler(&streamCreditThunk, this);
}

m5::stl::expected<void, error_t> RemoteI2SBus::init(const bus::BusConfig& config)
{
    if (config.getBusKind() != types::bus_kind_t::I2S) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    _config = static_cast<const i2s::I2SBusConfig&>(config);
    return {};
}

void RemoteI2SBus::streamCreditThunk(void* ctx, types::bus_kind_t kind, uint8_t bus_id, uint32_t free,
                                     uint32_t submitted)
{
    auto* self = static_cast<RemoteI2SBus*>(ctx);
    if (kind == types::bus_kind_t::I2S && bus_id == self->_remote_bus_id) {
        self->onCredit(free, submitted);
    }
}

void RemoteI2SBus::onCredit(uint32_t free, uint32_t submitted)
{
    // Absolute-value pair: just adopt the latest report (self-healing).
    _free      = free;
    _submitted = submitted;
#ifdef M5HAL_REMOTE_I2S_DEBUG
    ::printf("[ev %u] free=%u sub=%u sent=%u\n", static_cast<unsigned>(m5::utility::millis() & 0xFFFFF),
             static_cast<unsigned>(free), static_cast<unsigned>(submitted), static_cast<unsigned>(_sent));
#endif
}

m5::stl::expected<void, error_t> RemoteI2SBus::syncStatus()
{
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    auto e = enc.busStreamStatus(types::bus_kind_t::I2S, _remote_bus_id, 0);
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return e;
    }
    auto rq = _session->request(data::ConstDataSpan{script_buf, sink.written()});
    if (!rq.has_value()) {
        return rq;
    }
    const auto stored = _session->runner().storedData(0);
    if (stored.size < 8) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    _free      = detail::getU32LE(stored.data);
    _submitted = detail::getU32LE(stored.data + 4);
    // Re-baseline: a synchronous status response proves every earlier write
    // was either processed or lost (in-order link), so nothing of ours is in
    // flight. Without this, lost NORESP writes (CRC drop / rx overflow) would
    // permanently shrink the credit (spec §stream credit).
    _sent = _submitted;
    return {};
}

m5::stl::expected<void, error_t> RemoteI2SBus::syncConfig(const i2s::I2SAccessConfig& cfg)
{
    // Send the device a non-blocking write timeout: credit, not the server
    // poll, does the waiting (spec §I2S proxy).
    i2s::I2SAccessConfig wire_cfg = cfg;
    wire_cfg.write_timeout_ms     = 0;

    uint8_t script_buf[64];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    auto e = enc.i2sConfig(_remote_bus_id, wire_cfg);
    if (e.has_value()) {
        e = enc.busStreamStatus(types::bus_kind_t::I2S, _remote_bus_id, 0);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return e;
    }
    auto rq = _session->request(data::ConstDataSpan{script_buf, sink.written()});
    if (!rq.has_value()) {
        return rq;
    }
    const auto stored = _session->runner().storedData(0);
    if (stored.size < 8) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    _free        = detail::getU32LE(stored.data);
    _submitted   = detail::getU32LE(stored.data + 4);
    _sent        = _submitted;  // re-baseline: nothing in flight after a re-sync
    _applied_cfg = cfg;
    _configured  = true;
    return {};
}

m5::stl::expected<size_t, error_t> RemoteI2SBus::write(bus::Accessor* owner, const i2s::I2SAccessConfig& cfg,
                                                       data::Source* tx, size_t len)
{
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }

    // Apply a changed (or first) AccessConfig and re-sync the credit baseline.
    const bool cfg_changed = !_configured || cfg.sample_rate_hz != _applied_cfg.sample_rate_hz ||
                             cfg.bits_per_sample != _applied_cfg.bits_per_sample ||
                             cfg.channels != _applied_cfg.channels || cfg.timeout_ms != _applied_cfg.timeout_ms ||
                             cfg.write_timeout_ms != _applied_cfg.write_timeout_ms;
    if (cfg_changed) {
        auto s = syncConfig(cfg);
        if (!s.has_value()) {
            return m5::stl::make_unexpected(s.error());
        }
    }

    const uint32_t start   = static_cast<uint32_t>(m5::utility::millis());
    uint32_t last_progress = start;
    size_t done            = 0;
#ifdef M5HAL_REMOTE_I2S_DEBUG
    ::printf("[wr+ %u] len=%u in_flight=%u credit=%u\n", static_cast<unsigned>(start & 0xFFFFF),
             static_cast<unsigned>(len), static_cast<unsigned>(_sent - _submitted), static_cast<unsigned>(credit()));
#endif

    while (done < len && (tx == nullptr || !tx->eof())) {
        uint32_t cr = credit();
        {
            // In-flight window cap (see setStreamWindow / the detail note above).
            const uint32_t in_flight = _sent - _submitted;
            const uint32_t window    = (in_flight < _stream_window) ? (_stream_window - in_flight) : 0;
            if (cr > window) {
                cr = window;
            }
        }
        if (cr == 0) {
            // No credit: wait for the device's drain events. A persistent
            // stall triggers a synchronous status re-sync (which also lets
            // any pending NORESP error flow back to the session). The re-sync
            // check must run BEFORE the write-timeout break: session.poll()
            // can block for the full rx timeout, so checking the deadline
            // first would starve the re-sync forever.
            const uint32_t now = static_cast<uint32_t>(m5::utility::millis());
            if ((now - last_progress) >= detail::kCreditResyncMs) {
#ifdef M5HAL_REMOTE_I2S_DEBUG
                ::printf("[rs %u] resync (in_flight=%u)\n", static_cast<unsigned>(now & 0xFFFFF),
                         static_cast<unsigned>(_sent - _submitted));
#endif
                auto s = syncStatus();
                if (!s.has_value()) {
                    return m5::stl::make_unexpected(s.error());
                }
                last_progress = static_cast<uint32_t>(m5::utility::millis());
                cr            = credit();
            }
        }
        if (cr == 0) {
            // Host-side write timeout: short writes are normal for I2S.
            const uint32_t now = static_cast<uint32_t>(m5::utility::millis());
            if (cfg.write_timeout_ms != 0 && (now - start) >= cfg.write_timeout_ms) {
                break;
            }
            // Wait for ONE frame (≈ the next credit event) then re-check.
            // poll(1), not poll(): with events arriving faster than the rx
            // first-byte timeout, an unbounded poll captures the sender for
            // max_frames × event-interval (~340 ms measured) per stall, and
            // the device drains dry while the host sits here — the
            // stop-and-wait collapse that made small windows lose rate.
            (void)_session->poll(1);
            continue;
        }

        // Stage one chunk: min(credit, max message data, remaining).
        uint32_t chunk = cr;
        if (chunk > detail::kMaxStreamChunk) {
            chunk = static_cast<uint32_t>(detail::kMaxStreamChunk);
        }
        const size_t remaining = len - done;
        if (chunk > remaining) {
            chunk = static_cast<uint32_t>(remaining);
        }

        uint8_t data_buf[detail::kMaxStreamChunk];
        size_t staged = 0;
        while (staged < chunk && (tx == nullptr || !tx->eof())) {
            auto p = tx->peek(chunk - staged);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;  // EOF
            }
            const size_t take = p.value().size < (chunk - staged) ? p.value().size : (chunk - staged);
            ::memcpy(data_buf + staged, p.value().data, take);
            staged += take;
            auto adv = tx->advance(take);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
        }
        if (staged == 0) {
            break;  // source drained early
        }

        // Encode and fire one NORESP bus_write_stream.
        uint8_t script_buf[kMaxBodySize];
        data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
        bytecode::BytecodeEncoder enc{sink};
        auto e = enc.busWriteStream(types::bus_kind_t::I2S, _remote_bus_id, data_buf, staged);
        if (!e.has_value()) {
            return m5::stl::make_unexpected(e.error());
        }
        auto rq = _session->requestNoResponse(data::ConstDataSpan{script_buf, sink.written()});
        if (!rq.has_value()) {
            return m5::stl::make_unexpected(rq.error());
        }
        _sent += static_cast<uint32_t>(staged);  // u32-wrap safe
        done += staged;
        last_progress = static_cast<uint32_t>(m5::utility::millis());
    }

#ifdef M5HAL_REMOTE_I2S_DEBUG
    ::printf("[wr- %u] done=%u took=%ums\n", static_cast<unsigned>(m5::utility::millis() & 0xFFFFF),
             static_cast<unsigned>(done), static_cast<unsigned>(static_cast<uint32_t>(m5::utility::millis()) - start));
#endif
    return done;
}

m5::stl::expected<size_t, error_t> RemoteI2SBus::writableBytes(bus::Accessor* owner, const i2s::I2SAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    // Host-side credit estimate, no RPC (spec §I2S proxy).
    return static_cast<size_t>(credit());
}

// ---- RemoteGPIO / RemotePort ---------------------------------------------------

void RemotePort::_writePinEncoded(uint32_t encoded_num, bool v)
{
    _owner->wireWrite(encoded_num, v);
}

bool RemotePort::_readPinEncoded(uint32_t encoded_num)
{
    return _owner->wireRead(encoded_num);
}

void RemotePort::_setPinModeEncoded(uint32_t encoded_num, ::m5::hal::v1::types::gpio_mode_t mode)
{
    _owner->wireSetMode(encoded_num, mode);
}

void RemoteGPIO::wireWrite(uint32_t local_pin, bool v)
{
    // NORESP: the IPort hook cannot return an error; a server-side
    // failure parks as the pending error (observable on the session).
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pin = remoteNumber(local_pin);
    auto e                         = v ? enc.gpioWriteHigh(&pin, 1) : enc.gpioWriteLow(&pin, 1);
    if (e.has_value()) {
        e = enc.end();
    }
    if (e.has_value()) {
        (void)_session->requestNoResponse(data::ConstDataSpan{script_buf, sink.written()});
    }
}

bool RemoteGPIO::wireRead(uint32_t local_pin)
{
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pin = remoteNumber(local_pin);
    auto e                         = enc.gpioRead(0, &pin, 1);
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return false;
    }
    auto rq = _session->request(data::ConstDataSpan{script_buf, sink.written()});
    if (!rq.has_value()) {
        return false;  // the error stays observable on the session
    }
    const auto stored = _session->runner().storedData(0);
    return stored.size >= 1 && (stored.data[0] & 0x01) != 0;
}

void RemoteGPIO::wireSetMode(uint32_t local_pin, ::m5::hal::v1::types::gpio_mode_t mode)
{
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pin = remoteNumber(local_pin);
    auto e                         = enc.gpioSetMode(mode, &pin, 1);
    if (e.has_value()) {
        e = enc.end();
    }
    if (e.has_value()) {
        (void)_session->requestNoResponse(data::ConstDataSpan{script_buf, sink.written()});
    }
}

m5::stl::expected<void, error_t> RemoteGPIO::subscribe(::m5::hal::v1::types::gpio_local_pin_t local_pin)
{
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pin = remoteNumber(local_pin);
    auto e                         = enc.gpioSubscribe(&pin, 1);
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return e;
    }
    return _session->request(data::ConstDataSpan{script_buf, sink.written()});
}

m5::stl::expected<void, error_t> RemoteGPIO::unsubscribe(::m5::hal::v1::types::gpio_local_pin_t local_pin)
{
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pin = remoteNumber(local_pin);
    auto e                         = enc.gpioUnsubscribe(&pin, 1);
    if (e.has_value()) {
        e = enc.end();
    }
    if (!e.has_value()) {
        return e;
    }
    return _session->request(data::ConstDataSpan{script_buf, sink.written()});
}

}  // namespace m5::hal::v1::remote

#endif
