// SPDX-License-Identifier: MIT
// Native gtest for RemoteSession / RemoteServerAdapter (new frame-based API).

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/remote/backend.hpp>
#include <m5_hal/variants/frameworks/remote/detail_helpers.hpp>
#include <m5_hal/hal/v2/remote/server_handler.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

namespace {

using namespace m5::hal::v2;
namespace mem = memory;

size_t pumpValue(data::MuxFrameEncoder& enc)
{
    auto r = enc.pump();
    if (!r.has_value()) {
        ADD_FAILURE() << "MuxFrameEncoder::pump failed: " << error::toString(r.error());
        return 0;
    }
    return r.value();
}

size_t pumpValue(data::MuxFrameDecoder& dec, data::Source& src)
{
    auto r = dec.pump(src);
    if (!r.has_value()) {
        ADD_FAILURE() << "MuxFrameDecoder::pump failed: " << error::toString(r.error());
        return 0;
    }
    return r.value();
}

struct SessionPair {
    mem::Allocator& alloc = mem::defaultAllocator();
    uint8_t wire_ab_buf[4096], wire_ba_buf[4096];
    data::RingFIFO wire_ab, wire_ba;
    data::MuxFrameEncoder enc_a{alloc}, enc_b{alloc};
    data::MuxFrameDecoder dec_a{alloc}, dec_b{alloc};

    SessionPair()
    {
        wire_ab.setBuf(wire_ab_buf, sizeof(wire_ab_buf));
        wire_ba.setBuf(wire_ba_buf, sizeof(wire_ba_buf));
    }

    void pump()
    {
        pumpValue(enc_a);
        transfer(enc_a.output(), wire_ab.sink());
        pumpValue(dec_b, wire_ab.source());

        pumpValue(enc_b);
        transfer(enc_b.output(), wire_ba.sink());
        pumpValue(dec_a, wire_ba.source());
    }

    static void transfer(data::Source& src, data::Sink& dst)
    {
        while (!src.eof()) {
            auto p = src.peek(4096);
            if (!p.has_value() || p.value().size == 0) {
                break;
            }
            auto rsv = dst.reserve(p.value().size);
            if (!rsv.has_value() || rsv.value().size == 0) {
                break;
            }
            size_t n = rsv.value().size < p.value().size ? rsv.value().size : p.value().size;
            ::memcpy(rsv.value().data, p.value().data, n);
            (void)dst.commit(n);
            (void)src.advance(n);
        }
    }
};

class PortMaskRecordingPort : public gpio::IPort {
public:
    uint32_t value         = 0;
    size_t read_pin_calls  = 0;
    size_t read_port_calls = 0;

protected:
    void _writePinEncoded(uint32_t pin_mask, bool v) override
    {
        if (v) {
            value |= pin_mask;
        } else {
            value &= ~pin_mask;
        }
    }
    bool _readPinEncoded(uint32_t pin_mask) override
    {
        ++read_pin_calls;
        return (value & pin_mask) != 0;
    }
    void _setPinModeEncoded(uint32_t, types::gpio_mode_t) override
    {
    }
    uint32_t _readPortAll() override
    {
        ++read_port_calls;
        return value;
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t pin_mask) const override
    {
        for (uint8_t bit = 0; bit < 32; ++bit) {
            if ((pin_mask & (1u << bit)) != 0) {
                return bit;
            }
        }
        return 0;
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin) const override
    {
        return 1u << (pin & 31);
    }
};

struct PortMaskGPIO : public gpio::IGPIO {
    gpio::IPort* portForPin(types::gpio_local_pin_t) const override
    {
        return &port;
    }
    gpio::IPort* getPort(uint8_t) const override
    {
        return &port;
    }
    uint16_t getPinCount() const override
    {
        return 8;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }
    mutable PortMaskRecordingPort port;
};

struct TwoPortMaskGPIO : public gpio::IGPIO {
    gpio::IPort* portForPin(types::gpio_local_pin_t pin) const override
    {
        return &ports[pin >> 5];
    }
    gpio::IPort* getPort(uint8_t port) const override
    {
        return &ports[port];
    }
    uint16_t getPinCount() const override
    {
        return 40;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }
    mutable PortMaskRecordingPort ports[2];
};

struct ReorderedPortMaskGPIO : public gpio::IGPIO {
    gpio::IPort* portForPin(types::gpio_local_pin_t pin) const override
    {
        return &ports[pin == 0 ? 1 : 0];
    }
    gpio::IPort* getPort(uint8_t port) const override
    {
        return &ports[port];
    }
    uint16_t getPinCount() const override
    {
        return 2;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }
    mutable PortMaskRecordingPort ports[2];
};

struct GpioEventCapture {
    struct Event {
        types::gpio_number_t pin = -1;
        bool level               = false;
    };

    std::vector<Event> events;

    static void onEvent(void* ctx, types::gpio_number_t pin, bool level)
    {
        static_cast<GpioEventCapture*>(ctx)->events.push_back(Event{pin, level});
    }
};

result_t<void> pollGpioEventBody(remote::RemoteServerHandler& handler, std::vector<uint8_t>& event_body)
{
    event_body.clear();

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    uint8_t frame_buf[1024];
    data::RingFIFO frames;
    frames.setBuf(frame_buf, sizeof(frame_buf));

    dec.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            if (view.kind != frame::Kind::Event) {
                return;
            }
            auto* body = static_cast<std::vector<uint8_t>*>(ctx);
            body->assign(view.payload.data, view.payload.data + view.payload.size);
        },
        &event_body);

    auto poll = remote::RemoteServerHandler::poll(&handler, enc);
    if (!poll.has_value()) {
        return m5::stl::make_unexpected(poll.error());
    }
    pumpValue(enc);
    SessionPair::transfer(enc.output(), frames.sink());
    pumpValue(dec, frames.source());
    return {};
}

result_t<void> decodeGpioEventBody(const std::vector<uint8_t>& event_body, GpioEventCapture& capture)
{
    bytecode::BytecodeRunner runner{mem::defaultAllocator()};
    runner.setReceiveOnly(true);
    runner.setGpioEventHandler(&GpioEventCapture::onEvent, &capture);
    auto run = runner.runEvent({event_body.data(), event_body.size()});
    if (!run.has_value()) {
        return m5::stl::make_unexpected(run.error());
    }
    return {};
}

class RemoteTestHal : public Hal {
public:
    explicit RemoteTestHal(bus::IHalBackend* backend) : Hal{backend}
    {
    }
};

class RecordingI2SBus : public i2s::IBus {
public:
    explicit RecordingI2SBus(size_t max_write) : max_write_per_call{max_write}
    {
    }

    result_t<size_t> write(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Source* src, size_t len) override
    {
        (void)owner;
        last_cfg = cfg;
        ++write_calls;
        const size_t limit = len < max_write_per_call ? len : max_write_per_call;
        size_t done        = 0;
        while (done < limit && src != nullptr && !src->eof()) {
            auto p = src->peek(limit - done);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            bytes.insert(bytes.end(), p.value().data, p.value().data + p.value().size);
            auto adv = src->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
            done += p.value().size;
        }
        return done;
    }

    size_t max_write_per_call = 0;
    size_t write_calls        = 0;
    i2s::AccessConfig last_cfg{};
    std::vector<uint8_t> bytes;
};

class EchoUARTBus : public uart::IBus {
public:
    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src, size_t len) override
    {
        (void)owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (done < len && src != nullptr && !src->eof()) {
            auto p = src->peek(len - done);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            rx_queue.insert(rx_queue.end(), p.value().data, p.value().data + p.value().size);
            auto adv = src->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
            done += p.value().size;
        }
        return done;
    }

    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override
    {
        (void)owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (done < len && dst != nullptr && !dst->closed() && !rx_queue.empty()) {
            auto rsv = dst->reserve(len - done);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            if (rsv.value().size == 0) {
                break;
            }
            size_t n = rsv.value().size < rx_queue.size() ? rsv.value().size : rx_queue.size();
            ::memcpy(rsv.value().data, rx_queue.data(), n);
            rx_queue.erase(rx_queue.begin(), rx_queue.begin() + static_cast<std::vector<uint8_t>::difference_type>(n));
            auto commit = dst->commit(n);
            if (!commit.has_value()) {
                return m5::stl::make_unexpected(commit.error());
            }
            done += n;
        }
        return done;
    }

    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override
    {
        (void)owner;
        last_cfg = cfg;
        return rx_queue.size();
    }

    uart::AccessConfig last_cfg{};
    std::vector<uint8_t> rx_queue;
};

class SplitRxSPIBus : public spi::IBus {
public:
    explicit SplitRxSPIBus(size_t max_rx) : max_rx_per_call{max_rx}
    {
    }

    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        (void)owner;
        (void)cfg;
        (void)desc;
        ++transfer_calls;
        bus::TransferTotals totals{};
        size_t consumed = 0;
        while (consumed < tx_len && src != nullptr && !src->eof()) {
            auto p = src->peek(tx_len - consumed);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            tx_bytes.insert(tx_bytes.end(), p.value().data, p.value().data + p.value().size);
            auto adv = src->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
            consumed += p.value().size;
        }
        totals.tx = consumed;

        const size_t rx_step = rx_len < max_rx_per_call ? rx_len : max_rx_per_call;
        if (rx_step != 0 && dst != nullptr) {
            auto rsv = dst->reserve(rx_step);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            const size_t n = rsv.value().size < rx_step ? rsv.value().size : rx_step;
            for (size_t i = 0; i < n; ++i) {
                rsv.value().data[i] = static_cast<uint8_t>(0xA0u + ((rx_cursor + i) & 0x0Fu));
            }
            auto commit = dst->commit(n);
            if (!commit.has_value()) {
                return m5::stl::make_unexpected(commit.error());
            }
            rx_cursor += n;
            totals.rx = n;
        }
        last_totals = totals;
        return {};
    }

    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        auto out    = last_totals;
        last_totals = bus::TransferTotals{};
        return out;
    }

    size_t max_rx_per_call = 0;
    size_t transfer_calls  = 0;
    size_t rx_cursor       = 0;
    std::vector<uint8_t> tx_bytes;
    bus::TransferTotals last_totals;
};

class PatternI2CBus : public i2c::IBus {
public:
    PatternI2CBus(uint16_t ack_address, size_t max_rx) : ack_addr{ack_address}, max_rx_per_call{max_rx}
    {
    }

    result_t<void> transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        (void)owner;
        ++transfer_calls;
        ready_totals.clear();
        last_addr = cfg.i2c_addr;

        if (cfg.i2c_addr != ack_addr) {
            return m5::stl::make_unexpected(error::error_t::I2C_NO_ACK);
        }
        if (desc.prefix_len == 0 && tx_len == 0 && rx_len == 0) {
            ++probe_calls;
            return {};
        }

        size_t consumed = 0;
        while (consumed < tx_len && src != nullptr && !src->eof()) {
            auto p = src->peek(tx_len - consumed);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            tx_bytes.insert(tx_bytes.end(), p.value().data, p.value().data + p.value().size);
            auto adv = src->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
            consumed += p.value().size;
        }

        size_t produced      = 0;
        const size_t rx_step = rx_len < max_rx_per_call ? rx_len : max_rx_per_call;
        while (produced < rx_step && dst != nullptr && !dst->closed()) {
            auto rsv = dst->reserve(rx_step - produced);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            if (rsv.value().size == 0) {
                break;
            }
            size_t n = rsv.value().size;
            if (n > rx_step - produced) {
                n = rx_step - produced;
            }
            for (size_t i = 0; i < n; ++i) {
                rsv.value().data[i] = static_cast<uint8_t>((rx_cursor + i) & 0xFFu);
            }
            auto commit = dst->commit(n);
            if (!commit.has_value()) {
                return m5::stl::make_unexpected(commit.error());
            }
            produced += n;
        }
        rx_cursor += produced;
        ready_totals = bus::TransferTotals{consumed, produced};
        return {};
    }

    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        auto out = ready_totals;
        ready_totals.clear();
        return out;
    }

    uint16_t ack_addr      = 0x42;
    size_t max_rx_per_call = 0;
    size_t transfer_calls  = 0;
    size_t probe_calls     = 0;
    size_t rx_cursor       = 0;
    uint16_t last_addr     = 0;
    std::vector<uint8_t> tx_bytes;
    bus::TransferTotals ready_totals;
};

struct CapturedMuxFrame {
    frame::Kind kind = frame::Kind::Padding;
    uint8_t seq      = 0;
    std::vector<uint8_t> payload;
};

struct MuxFrameCapture {
    std::vector<CapturedMuxFrame> frames;

    static void onFrame(void* ctx, const frame::View& view)
    {
        auto* cap = static_cast<MuxFrameCapture*>(ctx);
        CapturedMuxFrame out;
        out.kind = view.kind;
        out.seq  = view.b3;
        if (view.payload.size != 0) {
            out.payload.assign(view.payload.data, view.payload.data + view.payload.size);
        }
        cap->frames.push_back(out);
    }
};

static void pumpEncoderToDecoder(data::MuxFrameEncoder& enc, data::RingFIFO& wire, data::MuxFrameDecoder& dec)
{
    pumpValue(enc);
    SessionPair::transfer(enc.output(), wire.sink());
    pumpValue(dec, wire.source());
}

class DynamicBusCreatePeer {
public:
    DynamicBusCreatePeer(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec) : _enc{&enc}
    {
        _runner.setBusCreateHandler(&DynamicBusCreatePeer::onBusCreate, this);
        dec.setFrameHandler(
            [](void* ctx, const frame::View& view) {
                auto* peer = static_cast<DynamicBusCreatePeer*>(ctx);
                if (view.kind == frame::Kind::Request) {
                    peer->handleRequest(view.b3, view.payload);
                }
            },
            this);
    }

    size_t create_count  = 0;
    size_t release_count = 0;
    bool fail_release    = false;
    std::function<void()> release_hook;
    types::bus_kind_t last_kind{types::bus_kind_t::I2C};
    uint8_t last_bus_id = 0xFF;
    std::vector<uint8_t> last_pin_config;

private:
    static result_t<void> onBusCreate(void* ctx, bool create, types::bus_kind_t kind, uint8_t bus_id,
                                      data::ConstDataSpan pin_config)
    {
        auto* peer        = static_cast<DynamicBusCreatePeer*>(ctx);
        peer->last_kind   = kind;
        peer->last_bus_id = bus_id;
        if (!create && peer->fail_release) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        if (create) {
            ++peer->create_count;
            peer->last_pin_config.assign(pin_config.data, pin_config.data + pin_config.size);
        } else {
            if (peer->release_hook) {
                peer->release_hook();
            }
            ++peer->release_count;
            peer->last_pin_config.clear();
        }
        return {};
    }

    void handleRequest(uint8_t seq, data::ConstDataSpan payload)
    {
        auto run    = _runner.run(payload);
        auto status = run.has_value() ? error::error_t::OK : run.error();

        uint8_t resp_buf[frame::kMaxPayload];
        data::MemorySink resp_sink{resp_buf, sizeof(resp_buf)};
        auto written = _runner.writeResponse(resp_sink, status);
        if (written.has_value()) {
            _enc->writeFrame(frame::Kind::Response, seq, {resp_buf, resp_sink.written()});
        }
    }

    data::MuxFrameEncoder* _enc;
    bytecode::BytecodeRunner _runner{mem::defaultAllocator()};
};

struct RemoteConfigCompatHarness {
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    DynamicBusCreatePeer peer{pair.enc_b, pair.dec_b};
    remote::RemoteBackend backend{session};
    RemoteTestHal hal{&backend};

    RemoteConfigCompatHarness()
    {
        session.setPeerPoll(
            [](void* ctx) {
                auto* p = static_cast<SessionPair*>(ctx);
                p->pump();
            },
            &pair);
    }
};

// Mimics the POSIX UART transport's write coalescing (`Bus_posix`, sized
// for I2S remote-audio throughput): small writes sit buffered
// until either the buffer would overflow, or a read is attempted on this
// same connection ("flush-before-read", real UART reads and writes share
// one `Bus_posix` instance). Couples one outgoing wire (buffered) with one
// incoming wire (whose peek triggers the flush) to reproduce that pairing.
class CoalescingWire {
public:
    CoalescingWire(data::Sink& real_out, data::Source& real_in) : _out{&real_out}, _in{&real_in}
    {
    }

    void flush()
    {
        if (_used == 0) {
            return;
        }
        auto rsv = _out->reserve(_used);
        if (rsv.has_value() && rsv.value().size > 0) {
            size_t n = rsv.value().size < _used ? rsv.value().size : _used;
            ::memcpy(rsv.value().data, _buf, n);
            (void)_out->commit(n);
        }
        _used = 0;
    }

    result_t<data::DataSpan> reserveWrite(size_t max_len)
    {
        if (_used >= sizeof(_buf)) {
            flush();
        }
        size_t avail = sizeof(_buf) - _used;
        size_t n     = max_len < avail ? max_len : avail;
        return data::DataSpan{_buf + _used, n};
    }
    result_t<void> commitWrite(size_t n)
    {
        _used += n;
        if (_used >= kCoalesceCap) {
            flush();
        }
        return {};
    }

    result_t<data::ConstDataSpan> peekRead(size_t max_len)
    {
        flush();  // flush-before-read, mirroring Bus_posix::read()
        return _in->peek(max_len);
    }
    result_t<void> advanceRead(size_t n)
    {
        return _in->advance(n);
    }
    bool eofRead() const
    {
        return _in->eof();
    }

private:
    static constexpr size_t kCoalesceCap = 4096;
    data::Sink* _out;
    data::Source* _in;
    uint8_t _buf[4096];
    size_t _used = 0;
};

class CoalescingSink : public data::Sink {
public:
    explicit CoalescingSink(CoalescingWire& w) : _w{&w}
    {
    }
    result_t<data::DataSpan> reserve(size_t max_len) override
    {
        return _w->reserveWrite(max_len);
    }
    result_t<void> commit(size_t n) override
    {
        return _w->commitWrite(n);
    }
    bool closed() const override
    {
        return false;
    }

private:
    CoalescingWire* _w;
};

class CoalescingSource : public data::Source {
public:
    explicit CoalescingSource(CoalescingWire& w) : _w{&w}
    {
    }
    result_t<data::ConstDataSpan> peek(size_t max_len) override
    {
        return _w->peekRead(max_len);
    }
    result_t<void> advance(size_t n) override
    {
        return _w->advanceRead(n);
    }
    bool eof() const override
    {
        return _w->eofRead();
    }

private:
    CoalescingWire* _w;
};

struct ScriptCapturePeer {
    data::MuxFrameEncoder* enc = nullptr;
    std::vector<std::vector<uint8_t>> requests;
    bool fail_next_request = false;

    explicit ScriptCapturePeer(data::MuxFrameEncoder& e) : enc{&e}
    {
    }

    void handle(const frame::View& view)
    {
        if (view.kind == frame::Kind::HelloReq) {
            uint8_t caps[] = {remote::kProtocolVersion, 0};
            enc->writeFrame(frame::Kind::HelloResp, view.b3, {caps, sizeof(caps)});
            return;
        }
        if (view.kind != frame::Kind::Request) {
            return;
        }
        requests.emplace_back(view.payload.data, view.payload.data + view.payload.size);

        uint8_t resp_buf[frame::kMaxPayload];
        data::MemorySink resp_sink{resp_buf, sizeof(resp_buf)};
        bytecode::BytecodeEncoder resp{resp_sink};
        const error::error_t status = fail_next_request ? error::error_t::IO_ERROR : error::error_t::OK;
        fail_next_request           = false;
        auto r                      = resp.reportComplete(status);
        if (r.has_value()) {
            r = resp.end();
        }
        if (r.has_value()) {
            enc->writeFrame(frame::Kind::Response, view.b3, {resp_buf, resp_sink.written()});
        }
    }

    static void onFrame(void* ctx, const frame::View& view)
    {
        static_cast<ScriptCapturePeer*>(ctx)->handle(view);
    }
};

static void attachScriptCapturePeer(SessionPair& pair, ScriptCapturePeer& peer, remote::RemoteSession& session)
{
    pair.dec_b.setFrameHandler(&ScriptCapturePeer::onFrame, &peer);
    session.setPeerPoll(
        [](void* ctx) {
            auto* p = static_cast<SessionPair*>(ctx);
            p->pump();
        },
        &pair);
}

static void attachRealServer(remote::RemoteSession& session, remote::RemoteServerAdapter& adapter,
                             remote::RemoteServerHandler& handler, remote::Server& server)
{
    handler.server = &server;
    adapter.setHandler(&remote::RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&remote::RemoteServerHandler::poll, &handler);
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);
}

static std::vector<uint8_t> writtenBytes(const uint8_t* buf, size_t len)
{
    return std::vector<uint8_t>{buf, buf + len};
}

static std::vector<uint8_t> expectedI2cStreamScript(uint8_t bus_id, uint8_t stream_id,
                                                    const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                                                    size_t tx_len, size_t rx_len)
{
    uint8_t buf[remote::kMaxScriptSize];
    data::MemorySink sink{buf, sizeof(buf)};
    bytecode::BytecodeEncoder enc{sink};
    auto r = enc.configure(bus_id, cfg);
    if (r.has_value()) {
        uint8_t meta_buf[1 + i2c::TransferDesc::PREFIX_CAPACITY];
        meta_buf[0] = desc.prefix_len;
        if (desc.prefix_len != 0) {
            ::memcpy(meta_buf + 1, desc.prefix, desc.prefix_len);
        }
        const size_t meta_len = 1u + static_cast<size_t>(desc.prefix_len);
        r = enc.streamTransfer(types::bus_kind_t::I2C, bus_id, stream_id, static_cast<uint32_t>(tx_len),
                               static_cast<uint32_t>(rx_len), {meta_buf, meta_len});
    }
    if (r.has_value()) {
        r = enc.end();
    }
    EXPECT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
    return writtenBytes(buf, sink.written());
}

static std::vector<uint8_t> expectedSpiStreamScript(uint8_t bus_id, uint8_t stream_id,
                                                    const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                                                    size_t tx_len, size_t rx_len)
{
    uint8_t buf[remote::kMaxScriptSize];
    data::MemorySink sink{buf, sizeof(buf)};
    bytecode::BytecodeEncoder enc{sink};
    uint8_t meta_buf[15];
    meta_buf[0]  = static_cast<uint8_t>((desc.dc_level_valid ? 0x01 : 0x00) | (desc.dc_level ? 0x02 : 0x00));
    meta_buf[1]  = static_cast<uint8_t>(desc.command_dc_level);
    meta_buf[2]  = static_cast<uint8_t>(desc.address_dc_level);
    meta_buf[3]  = static_cast<uint8_t>(desc.data_dc_level);
    meta_buf[4]  = static_cast<uint8_t>(desc.command & 0xFFu);
    meta_buf[5]  = static_cast<uint8_t>((desc.command >> 8) & 0xFFu);
    meta_buf[6]  = static_cast<uint8_t>((desc.command >> 16) & 0xFFu);
    meta_buf[7]  = static_cast<uint8_t>((desc.command >> 24) & 0xFFu);
    meta_buf[8]  = static_cast<uint8_t>(desc.address & 0xFFu);
    meta_buf[9]  = static_cast<uint8_t>((desc.address >> 8) & 0xFFu);
    meta_buf[10] = static_cast<uint8_t>((desc.address >> 16) & 0xFFu);
    meta_buf[11] = static_cast<uint8_t>((desc.address >> 24) & 0xFFu);
    meta_buf[12] = desc.command_bytes;
    meta_buf[13] = desc.address_bytes;
    meta_buf[14] = desc.dummy_cycles;
    // First SPI transfer now carries configure just like the other bus kinds;
    // later matching transfers omit it through the host-side config cache.
    auto r = enc.configure(bus_id, cfg);
    if (r.has_value()) {
        r = enc.streamTransfer(types::bus_kind_t::SPI, bus_id, stream_id, static_cast<uint32_t>(tx_len),
                               static_cast<uint32_t>(rx_len), {meta_buf, sizeof(meta_buf)});
    }
    if (r.has_value()) {
        r = enc.end();
    }
    EXPECT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
    return writtenBytes(buf, sink.written());
}

static std::vector<uint8_t> expectedUartStreamScript(uint8_t bus_id, uint8_t stream_id, const uart::AccessConfig& cfg,
                                                     size_t tx_len, size_t rx_len)
{
    uint8_t buf[remote::kMaxScriptSize];
    data::MemorySink sink{buf, sizeof(buf)};
    bytecode::BytecodeEncoder enc{sink};
    auto r = enc.configure(bus_id, cfg);
    if (r.has_value()) {
        r = enc.streamTransfer(types::bus_kind_t::UART, bus_id, stream_id, static_cast<uint32_t>(tx_len),
                               static_cast<uint32_t>(rx_len), {});
    }
    if (r.has_value()) {
        r = enc.end();
    }
    EXPECT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
    return writtenBytes(buf, sink.written());
}

static std::vector<uint8_t> expectedI2sStreamScript(uint8_t bus_id, uint8_t stream_id, const i2s::AccessConfig& cfg,
                                                    size_t tx_len, size_t rx_len)
{
    uint8_t buf[remote::kMaxScriptSize];
    data::MemorySink sink{buf, sizeof(buf)};
    bytecode::BytecodeEncoder enc{sink};
    auto r = enc.i2sConfig(bus_id, cfg);
    if (r.has_value()) {
        r = enc.streamTransfer(types::bus_kind_t::I2S, bus_id, stream_id, static_cast<uint32_t>(tx_len),
                               static_cast<uint32_t>(rx_len), {});
    }
    if (r.has_value()) {
        r = enc.end();
    }
    EXPECT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
    return writtenBytes(buf, sink.written());
}

static bool scriptContainsOpcode(const std::vector<uint8_t>& script, bytecode::OpCode opcode)
{
    size_t pos = 0;
    while (pos < script.size()) {
        size_t len          = 0;
        size_t len_consumed = 0;
        const uint8_t head  = script[pos];
        if (head <= 0xFC) {
            len          = head;
            len_consumed = 1;
        } else if (head == 0xFD && pos + 2 < script.size()) {
            len          = static_cast<size_t>(script[pos + 1]) | (static_cast<size_t>(script[pos + 2]) << 8);
            len_consumed = 3;
        } else if (head == 0xFE && pos + 4 < script.size()) {
            len = static_cast<size_t>(script[pos + 1]) | (static_cast<size_t>(script[pos + 2]) << 8) |
                  (static_cast<size_t>(script[pos + 3]) << 16) | (static_cast<size_t>(script[pos + 4]) << 24);
            len_consumed = 5;
        } else {
            return false;
        }
        if (len == 0) {
            return false;
        }
        if (pos + len_consumed >= script.size() || pos + len_consumed + len > script.size()) {
            return false;
        }
        if (script[pos + len_consumed] == static_cast<uint8_t>(opcode)) {
            return true;
        }
        pos += len_consumed + len;
    }
    return false;
}

TEST(MuxRemoteSession, RequestResponseRoundtrip)
{
    SessionPair pair;

    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    session.setPeerPoll(
        [](void* ctx) {
            auto* p = static_cast<SessionPair*>(ctx);
            p->pump();
        },
        &pair);

    struct ServerCtx {
        data::MuxFrameEncoder* enc = nullptr;
    } server_ctx;
    server_ctx.enc = &pair.enc_b;

    pair.dec_b.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            auto* s = static_cast<ServerCtx*>(ctx);
            if (view.kind == frame::Kind::Request) {
                uint8_t resp[] = {0xDE, 0xAD};
                s->enc->writeFrame(frame::Kind::Response, view.b3, {resp, sizeof(resp)});
            }
        },
        &server_ctx);

    const uint8_t script[] = {0x00};
    auto r                 = session.request({script, sizeof(script)});
    ASSERT_TRUE(r.has_value());

    auto resp = session.lastResponse();
    ASSERT_EQ(resp.size, 2u);
    EXPECT_EQ(resp.data[0], 0xDE);
    EXPECT_EQ(resp.data[1], 0xAD);
}

TEST(MuxRemoteSession, RequestNoResponseFlushesCoalescingTransport)
{
    SessionPair pair;

    // Wrap A's outgoing wire (wire_ab) and incoming wire (wire_ba) with a
    // coalescing transport so this test reproduces the real POSIX UART
    // pairing: reading flushes pending writes on the same connection.
    CoalescingWire wire{pair.wire_ab.sink(), pair.wire_ba.source()};
    CoalescingSink coalescing_sink{wire};
    CoalescingSource coalescing_source{wire};

    remote::RemoteSession session{pair.enc_a, pair.dec_a, coalescing_source, coalescing_sink};

    const uint8_t script[] = {0x00};
    auto r                 = session.requestNoResponse({script, sizeof(script)});
    ASSERT_TRUE(r.has_value());

    // The frame must reach the peer's wire on its own. A regression here
    // (a bare flushTx() with no read-side pump) leaves fire-and-forget
    // writes stranded in the coalescing buffer until some unrelated later
    // read happens to flush them — this is what made plain `gpio wr`
    // silently never reach the device on a generic-server Core2
    // (HIL finding; `gpio mode`/`gpio rawrd` use request(),
    // which always follows with a blocking read and so never hit this).
    auto peeked = pair.wire_ab.source().peek(64);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_GT(peeked.value().size, 0u);
}

TEST(MuxRemoteSession, ResetFlushesCoalescingTransport)
{
    SessionPair pair;

    CoalescingWire wire{pair.wire_ab.sink(), pair.wire_ba.source()};
    CoalescingSink coalescing_sink{wire};
    CoalescingSource coalescing_source{wire};

    remote::RemoteSession session{pair.enc_a, pair.dec_a, coalescing_source, coalescing_sink};

    auto r = session.reset();
    ASSERT_TRUE(r.has_value());

    // Reset is also fire-and-forget. It must not be stranded in the same
    // coalescing buffer path that previously broke requestNoResponse().
    auto peeked = pair.wire_ab.source().peek(64);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_GT(peeked.value().size, 0u);
}

TEST(MuxRemoteSession, HelloRoundtrip)
{
    SessionPair pair;

    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    session.setPeerPoll(
        [](void* ctx) {
            auto* p = static_cast<SessionPair*>(ctx);
            p->pump();
        },
        &pair);

    struct ServerCtx {
        data::MuxFrameEncoder* enc = nullptr;
    } server_ctx;
    server_ctx.enc = &pair.enc_b;

    pair.dec_b.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            auto* s = static_cast<ServerCtx*>(ctx);
            if (view.kind == frame::Kind::HelloReq) {
                uint8_t caps[] = {0x01, 0x03};
                s->enc->writeFrame(frame::Kind::HelloResp, view.b3, {caps, sizeof(caps)});
            }
        },
        &server_ctx);

    auto r = session.hello();
    ASSERT_TRUE(r.has_value());

    auto resp = session.lastResponse();
    ASSERT_EQ(resp.size, 2u);
    EXPECT_EQ(resp.data[0], 0x01);
}

TEST(MuxRemoteSession, PingRoundtrip)
{
    SessionPair pair;

    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    session.setPeerPoll(
        [](void* ctx) {
            auto* p = static_cast<SessionPair*>(ctx);
            p->pump();
        },
        &pair);

    struct ServerCtx {
        data::MuxFrameEncoder* enc = nullptr;
    } server_ctx;
    server_ctx.enc = &pair.enc_b;

    pair.dec_b.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            auto* s = static_cast<ServerCtx*>(ctx);
            if (view.kind == frame::Kind::Ping) {
                s->enc->writeFrame(frame::Kind::Pong, view.b3, {});
            }
        },
        &server_ctx);

    auto r = session.ping();
    ASSERT_TRUE(r.has_value());
}

TEST(RemoteTransferWire, BusRemoteScriptsMatchLegacyOpcodeBytes)
{
    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        i2c::Bus_remote bus{session, 0, i2c::IBusConfig{}};
        i2c::MasterAccessConfig cfg;
        cfg.freq                   = 400000;
        cfg.wire_timeout_ms        = 33;
        cfg.i2c_addr               = 0x52;
        cfg.use_restart            = true;
        cfg.register_address_bytes = 1;
        i2c::TransferDesc desc;
        desc.prefix_len = 2;
        desc.prefix[0]  = 0x10;
        desc.prefix[1]  = 0x20;
        uint8_t tx[]    = {0xA1, 0xA2, 0xA3};
        uint8_t rx[4]   = {};
        data::MemorySource src{tx, sizeof(tx)};
        data::MemorySink dst{rx, sizeof(rx)};

        auto r = bus.transfer(nullptr, cfg, desc, &src, sizeof(tx), &dst, sizeof(rx));
        ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
        ASSERT_EQ(peer.requests.size(), 1u);
        EXPECT_EQ(peer.requests[0], expectedI2cStreamScript(0, 0, cfg, desc, sizeof(tx), sizeof(rx)));
    }

    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        spi::Bus_remote bus{session, 1};
        spi::MasterAccessConfig cfg;
        spi::TransferDesc desc;
        desc.dc_level_valid = true;
        desc.dc_level       = false;
        desc.command        = 0x11223344;
        desc.address        = 0x55667788;
        desc.command_bytes  = 2;
        desc.address_bytes  = 3;
        desc.dummy_cycles   = 5;
        uint8_t tx[]        = {0x01, 0x02};
        uint8_t rx[2]       = {};
        data::MemorySource src{tx, sizeof(tx)};
        data::MemorySink dst{rx, sizeof(rx)};

        auto r = bus.transfer(nullptr, cfg, desc, &src, sizeof(tx), &dst, sizeof(rx));
        ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
        ASSERT_EQ(peer.requests.size(), 1u);
        EXPECT_EQ(peer.requests[0], expectedSpiStreamScript(1, 0, cfg, desc, sizeof(tx), sizeof(rx)));
    }

    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        i2s::Bus_remote bus{session, 2};
        i2s::AccessConfig cfg;
        cfg.sample_rate_hz   = 44100;
        cfg.write_timeout_ms = 91;
        cfg.read_timeout_ms  = 92;
        cfg.bits_per_sample  = 24;
        cfg.channels         = 2;
        uint8_t tx[]         = {0x31, 0x32, 0x33, 0x34};
        uint8_t rx[3]        = {};
        data::MemorySource src{tx, sizeof(tx)};
        data::MemorySink dst{rx, sizeof(rx)};

        auto r = bus.transfer(nullptr, cfg, &src, sizeof(tx), &dst, sizeof(rx));
        ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
        EXPECT_EQ(r->tx, sizeof(tx));
        EXPECT_EQ(r->rx, sizeof(rx));
        ASSERT_EQ(peer.requests.size(), 1u);
        EXPECT_EQ(peer.requests[0], expectedI2sStreamScript(2, 0, cfg, sizeof(tx), sizeof(rx)));
    }

    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        uart::Bus_remote bus{session, 3, uart::IBusConfig{}};
        uart::AccessConfig cfg;
        cfg.baud_rate             = 115200;
        cfg.first_byte_timeout_ms = 21;
        cfg.inter_byte_timeout_ms = 22;
        cfg.write_timeout_ms      = 23;
        cfg.data_bits             = 7;
        cfg.stop_bits             = 2;
        cfg.parity                = uart::parity_t::Even;
        cfg.invert                = true;
        uint8_t tx[]              = {0x51, 0x52, 0x53};
        uint8_t rx[6]             = {};
        data::MemorySource src{tx, sizeof(tx)};
        data::MemorySink dst{rx, sizeof(rx)};

        auto r = bus.transfer(nullptr, cfg, &src, sizeof(tx), &dst, sizeof(rx));
        ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
        EXPECT_EQ(r->tx, sizeof(tx));
        EXPECT_EQ(r->rx, sizeof(rx));
        ASSERT_EQ(peer.requests.size(), 1u);
        EXPECT_EQ(peer.requests[0], expectedUartStreamScript(3, 0, cfg, sizeof(tx), sizeof(rx)));
    }
}

TEST(RemoteTransferWire, UartI2sProxiesRejectUnboundSessionAndNullArguments)
{
    uart::AccessConfig ucfg;
    i2s::AccessConfig icfg;
    uint8_t buf[4] = {};
    data::MemorySource src{buf, sizeof(buf)};
    data::MemorySink dst{buf, sizeof(buf)};

    {
        // A proxy without a session must fail loudly, not report success.
        uart::Bus_remote uart_bus;
        i2s::Bus_remote i2s_bus;
        auto uw = uart_bus.write(nullptr, ucfg, &src, sizeof(buf));
        ASSERT_FALSE(uw.has_value());
        EXPECT_EQ(uw.error(), error::error_t::INVALID_STATE);
        auto ur = uart_bus.read(nullptr, ucfg, &dst, sizeof(buf));
        ASSERT_FALSE(ur.has_value());
        EXPECT_EQ(ur.error(), error::error_t::INVALID_STATE);
        auto iw = i2s_bus.write(nullptr, icfg, &src, sizeof(buf));
        ASSERT_FALSE(iw.has_value());
        EXPECT_EQ(iw.error(), error::error_t::INVALID_STATE);
        auto ir = i2s_bus.read(nullptr, icfg, &dst, sizeof(buf));
        ASSERT_FALSE(ir.has_value());
        EXPECT_EQ(ir.error(), error::error_t::INVALID_STATE);
    }

    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        uart::Bus_remote uart_bus{session, 0, uart::IBusConfig{}};
        i2s::Bus_remote i2s_bus{session, 1};

        // A nonzero length with a null Source/Sink is an API contract
        // violation, not an empty transfer.
        auto uw = uart_bus.write(nullptr, ucfg, nullptr, sizeof(buf));
        ASSERT_FALSE(uw.has_value());
        EXPECT_EQ(uw.error(), error::error_t::INVALID_ARGUMENT);
        auto ur = uart_bus.read(nullptr, ucfg, nullptr, sizeof(buf));
        ASSERT_FALSE(ur.has_value());
        EXPECT_EQ(ur.error(), error::error_t::INVALID_ARGUMENT);
        auto iw = i2s_bus.write(nullptr, icfg, nullptr, sizeof(buf));
        ASSERT_FALSE(iw.has_value());
        EXPECT_EQ(iw.error(), error::error_t::INVALID_ARGUMENT);
        auto ir = i2s_bus.read(nullptr, icfg, nullptr, sizeof(buf));
        ASSERT_FALSE(ir.has_value());
        EXPECT_EQ(ir.error(), error::error_t::INVALID_ARGUMENT);

        // Zero length stays a no-op success and puts nothing on the wire.
        auto uz = uart_bus.write(nullptr, ucfg, &src, 0);
        ASSERT_TRUE(uz.has_value());
        EXPECT_EQ(uz.value(), 0u);
        auto iz = i2s_bus.read(nullptr, icfg, &dst, 0);
        ASSERT_TRUE(iz.has_value());
        EXPECT_EQ(iz.value(), 0u);
        EXPECT_TRUE(peer.requests.empty());
    }
}

TEST(RemoteTransferWire, ClosedSessionHandleMakesExistingProxyReturnClosed)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    auto handle = std::make_shared<remote::RemoteSessionHandle>();
    handle->bind(session);
    uart::Bus_remote bus{handle, 0, uart::IBusConfig{}};

    handle->close();
    uart::AccessConfig cfg;
    uint8_t byte = 0x5A;
    data::MemorySource src{&byte, 1};
    auto written = bus.write(nullptr, cfg, &src, 1);
    ASSERT_FALSE(written.has_value());
    EXPECT_EQ(written.error(), error::error_t::CLOSED);
}

TEST(RemoteTransferWire, HeterogeneousProxiesSerializeOneInflightSession)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    ScriptCapturePeer peer{pair.enc_b};
    attachScriptCapturePeer(pair, peer, session);
    auto handle = std::make_shared<remote::RemoteSessionHandle>();
    handle->bind(session);
    uart::Bus_remote uart_bus{handle, 0, uart::IBusConfig{}};
    i2s::Bus_remote i2s_bus{handle, 1, i2s::IBusConfig{}};
    uart::AccessConfig uart_cfg;
    i2s::AccessConfig i2s_cfg;
    std::atomic<bool> start{false};
    std::atomic<bool> uart_ok{false};
    std::atomic<bool> i2s_ok{false};

    std::thread uart_thread{[&] {
        uint8_t payload[] = {0x11, 0x12};
        data::MemorySource src{payload, sizeof(payload)};
        while (!start.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        uart_ok.store(uart_bus.write(nullptr, uart_cfg, &src, sizeof(payload)).has_value(), std::memory_order_release);
    }};
    std::thread i2s_thread{[&] {
        uint8_t payload[] = {0x21, 0x22};
        data::MemorySource src{payload, sizeof(payload)};
        while (!start.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        i2s_ok.store(i2s_bus.write(nullptr, i2s_cfg, &src, sizeof(payload)).has_value(), std::memory_order_release);
    }};

    start.store(true, std::memory_order_release);
    uart_thread.join();
    i2s_thread.join();
    EXPECT_TRUE(uart_ok.load(std::memory_order_acquire));
    EXPECT_TRUE(i2s_ok.load(std::memory_order_acquire));
    ASSERT_EQ(peer.requests.size(), 2u);
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusStreamTransfer));
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusStreamTransfer));
}

TEST(RemoteI2cLock, UsesCommonTimedMutexContract)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    i2c::Bus_remote bus{session, 0, i2c::IBusConfig{}};
    i2c::MasterAccessConfig cfg;
    i2c::MasterAccessor first{bus, cfg};
    i2c::MasterAccessor second{bus, cfg};
    std::atomic<bool> first_locked{false};
    std::atomic<bool> release_first{false};
    std::atomic<bool> holder_ok{false};

    std::thread holder{[&] {
        auto locked = first.beginAccess(0);
        holder_ok.store(locked.has_value(), std::memory_order_release);
        first_locked.store(true, std::memory_order_release);
        while (!release_first.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        if (locked.has_value()) {
            holder_ok.store(first.endAccess().has_value(), std::memory_order_release);
        }
    }};

    while (!first_locked.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    EXPECT_TRUE(holder_ok.load(std::memory_order_acquire));
    auto blocked                 = second.beginAccess(10);
    const bool timed_out         = !blocked.has_value();
    error::error_t blocked_error = timed_out ? blocked.error() : error::error_t::OK;
    if (blocked.has_value()) {
        (void)second.endAccess();
    }
    release_first.store(true, std::memory_order_release);
    holder.join();
    EXPECT_TRUE(timed_out);
    EXPECT_EQ(blocked_error, error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(holder_ok.load(std::memory_order_acquire));

    auto acquired = second.beginAccess(50);
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());
    auto unlocked = second.endAccess();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << error::toString(unlocked.error());
}

TEST(RemoteBackend, ProxyLastOwnerBestEffortReleasesPeerBus)
{
    RemoteConfigCompatHarness h;
    auto acquired = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());
    ASSERT_EQ(h.peer.create_count, 1u);
    ASSERT_EQ(h.peer.release_count, 0u);

    acquired.value().reset();
    EXPECT_EQ(h.peer.release_count, 1u);
}

TEST(RemoteBackend, SpiFeatureIntentIsRejectedBeforeBusCreate)
{
    RemoteConfigCompatHarness h;
    spi::LogicalBusConfig cfg{spi::Clk{18}, spi::Mosi{23}, spi::requireMosiSharedRx()};

    auto acquired = h.hal.SPI.acquire(cfg);
    ASSERT_FALSE(acquired.has_value());
    EXPECT_EQ(acquired.error(), error::error_t::NOT_IMPLEMENTED);
    EXPECT_EQ(h.peer.create_count, 0u);
}

TEST(RemoteBackend, FailedDestructorReleaseQuarantinesBusId)
{
    RemoteConfigCompatHarness h;
    h.peer.fail_release = true;
    auto first          = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
    ASSERT_EQ(h.peer.last_bus_id, 0u);

    first.value().reset();
    EXPECT_EQ(h.peer.release_count, 0u);
    EXPECT_EQ(h.backend.busRegistry().liveCount(), 1u);

    h.peer.fail_release = false;
    auto blocked        = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), error::error_t::BUSY);

    auto second = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{32}, i2c::Sda{33}});
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    EXPECT_EQ(h.peer.last_bus_id, 1u);
}

TEST(RemoteBackend, ExplicitReleaseClosesProxyRevivedFromWeakPointer)
{
    RemoteConfigCompatHarness h;
    auto acquired = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());

    std::weak_ptr<bus::IBus> weak = acquired.value();
    std::shared_ptr<bus::IBus> revived;
    h.peer.release_hook = [&]() { revived = weak.lock(); };
    auto released       = h.hal.I2C.release(acquired.value());
    ASSERT_TRUE(released.has_value()) << "err=" << error::toString(released.error());
    ASSERT_TRUE(revived);

    auto stale = std::static_pointer_cast<i2c::IBus>(revived);
    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;
    auto operation = stale->transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(operation.has_value());
    EXPECT_EQ(operation.error(), error::error_t::CLOSED);
}

TEST(RemoteBackend, ExplicitReleaseFailureRollsBackAndCanRetry)
{
    RemoteConfigCompatHarness h;
    auto acquired = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());

    h.peer.fail_release = true;
    auto failed         = h.hal.I2C.release(acquired.value());
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), error::error_t::IO_ERROR);
    ASSERT_TRUE(acquired.value());

    h.peer.fail_release = false;
    auto retried        = h.hal.I2C.release(acquired.value());
    ASSERT_TRUE(retried.has_value()) << "err=" << error::toString(retried.error());
    EXPECT_FALSE(acquired.value());
    EXPECT_EQ(h.peer.release_count, 1u);
}

TEST(RemoteBackend, NaturalReleaseInsideSessionLeaseDoesNotDeadlockAndQuarantinesId)
{
    RemoteConfigCompatHarness h;
    const i2c::LogicalBusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};
    auto acquired = h.hal.I2C.acquire(cfg);
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());
    ASSERT_EQ(h.peer.last_bus_id, 0u);

    auto handle = h.session.sharedHandle();
    {
        remote::RemoteSessionHandle::Lease session_lease{*handle};
        ASSERT_TRUE(session_lease);
        acquired.value().reset();
    }
    EXPECT_EQ(h.peer.release_count, 0u);

    auto same_identity = h.hal.I2C.acquire(cfg);
    ASSERT_FALSE(same_identity.has_value());
    EXPECT_EQ(same_identity.error(), error::error_t::BUSY);
    EXPECT_EQ(h.backend.busRegistry().liveCount(), 1u);

    auto reacquired = h.hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{32}, i2c::Sda{33}});
    ASSERT_TRUE(reacquired.has_value()) << "err=" << error::toString(reacquired.error());
    EXPECT_EQ(h.peer.last_bus_id, 1u);
}

TEST(RemoteBackend, NaturalReleaseKeepsIdentityTombstonedThroughPeerCallback)
{
    RemoteConfigCompatHarness h;
    const i2c::LogicalBusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};
    auto acquired = h.hal.I2C.acquire(cfg);
    ASSERT_TRUE(acquired.has_value()) << "err=" << error::toString(acquired.error());

    error::error_t callback_error = error::error_t::OK;
    h.peer.release_hook           = [&]() {
        auto during_release = h.hal.I2C.acquire(cfg);
        ASSERT_FALSE(during_release.has_value());
        callback_error = during_release.error();
    };
    acquired.value().reset();
    EXPECT_EQ(callback_error, error::error_t::BUSY);
    EXPECT_EQ(h.peer.release_count, 1u);

    h.peer.release_hook = {};
    auto reacquired     = h.hal.I2C.acquire(cfg);
    ASSERT_TRUE(reacquired.has_value()) << "err=" << error::toString(reacquired.error());
    EXPECT_EQ(h.peer.last_bus_id, 0u);
}

TEST(RemoteSessionHandle, BorrowedHandlesShareCanonicalGate)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    auto first  = remote::makeBorrowedSessionHandle(session);
    auto second = remote::makeBorrowedSessionHandle(session);
    ASSERT_TRUE(first);
    EXPECT_EQ(first.get(), second.get());
}

TEST(RemoteTransferWire, RepeatedUartWriteOmitsUnchangedConfigure)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    ScriptCapturePeer peer{pair.enc_b};
    attachScriptCapturePeer(pair, peer, session);

    uart::Bus_remote bus{session, 0, uart::IBusConfig{}};
    uart::AccessConfig cfg;
    cfg.baud_rate            = 115200;
    uint8_t first_payload[]  = {0x11, 0x12};
    uint8_t second_payload[] = {0x21, 0x22};
    data::MemorySource first_src{first_payload, sizeof(first_payload)};
    data::MemorySource second_src{second_payload, sizeof(second_payload)};

    auto first = bus.write(nullptr, cfg, &first_src, sizeof(first_payload));
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
    auto second = bus.write(nullptr, cfg, &second_src, sizeof(second_payload));
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());

    ASSERT_EQ(peer.requests.size(), 2u);
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusConfigure));
    EXPECT_FALSE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusConfigure));
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusStreamTransfer));
}

TEST(RemoteTransferWire, ConfigChangeResendsConfigure)
{
    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        uart::Bus_remote bus{session, 0, uart::IBusConfig{}};
        uart::AccessConfig cfg;
        cfg.baud_rate     = 115200;
        uint8_t payload[] = {0x31};
        data::MemorySource first_src{payload, sizeof(payload)};
        data::MemorySource second_src{payload, sizeof(payload)};

        auto first = bus.write(nullptr, cfg, &first_src, sizeof(payload));
        ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
        cfg.baud_rate = 230400;
        auto second   = bus.write(nullptr, cfg, &second_src, sizeof(payload));
        ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());

        ASSERT_EQ(peer.requests.size(), 2u);
        EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusConfigure));
        EXPECT_TRUE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusConfigure));
    }

    {
        SessionPair pair;
        remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
        ScriptCapturePeer peer{pair.enc_b};
        attachScriptCapturePeer(pair, peer, session);

        i2c::Bus_remote bus{session, 0, i2c::IBusConfig{}};
        i2c::MasterAccessConfig cfg;
        cfg.i2c_addr = 0x52;
        i2c::TransferDesc desc;

        auto first = bus.transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);
        ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
        auto same = bus.transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);
        ASSERT_TRUE(same.has_value()) << "err=" << error::toString(same.error());
        cfg.i2c_addr = 0x53;
        auto changed = bus.transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);
        ASSERT_TRUE(changed.has_value()) << "err=" << error::toString(changed.error());

        ASSERT_EQ(peer.requests.size(), 3u);
        EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusConfigure));
        EXPECT_FALSE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusConfigure));
        EXPECT_TRUE(scriptContainsOpcode(peer.requests[2], bytecode::OpCode::BusConfigure));
    }
}

TEST(RemoteTransferWire, HelloInvalidatesConfigCache)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    ScriptCapturePeer peer{pair.enc_b};
    attachScriptCapturePeer(pair, peer, session);

    uart::Bus_remote bus{session, 0, uart::IBusConfig{}};
    uart::AccessConfig cfg;
    cfg.baud_rate     = 115200;
    uint8_t payload[] = {0x41};
    data::MemorySource first_src{payload, sizeof(payload)};
    data::MemorySource second_src{payload, sizeof(payload)};
    data::MemorySource third_src{payload, sizeof(payload)};

    auto first = bus.write(nullptr, cfg, &first_src, sizeof(payload));
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
    auto second = bus.write(nullptr, cfg, &second_src, sizeof(payload));
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    auto hello = session.hello();
    ASSERT_TRUE(hello.has_value()) << "err=" << error::toString(hello.error());
    auto third = bus.write(nullptr, cfg, &third_src, sizeof(payload));
    ASSERT_TRUE(third.has_value()) << "err=" << error::toString(third.error());

    ASSERT_EQ(peer.requests.size(), 3u);
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusConfigure));
    EXPECT_FALSE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusConfigure));
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[2], bytecode::OpCode::BusConfigure));
}

TEST(RemoteTransferWire, ErrorResponseDoesNotUpdateConfigCache)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    ScriptCapturePeer peer{pair.enc_b};
    attachScriptCapturePeer(pair, peer, session);

    uart::Bus_remote bus{session, 0, uart::IBusConfig{}};
    uart::AccessConfig cfg;
    cfg.baud_rate            = 115200;
    uint8_t first_payload[]  = {0x51};
    uint8_t second_payload[] = {0x52};
    data::MemorySource first_src{first_payload, sizeof(first_payload)};
    data::MemorySource second_src{second_payload, sizeof(second_payload)};

    peer.fail_next_request = true;
    auto first             = bus.write(nullptr, cfg, &first_src, sizeof(first_payload));
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error::error_t::IO_ERROR);
    auto second = bus.write(nullptr, cfg, &second_src, sizeof(second_payload));
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());

    ASSERT_EQ(peer.requests.size(), 2u);
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[0], bytecode::OpCode::BusConfigure));
    EXPECT_TRUE(scriptContainsOpcode(peer.requests[1], bytecode::OpCode::BusConfigure));
}

TEST(RemoteConfigCompat, LogicalSpiThenTypedIgnoresWireOmittedDc)
{
    RemoteConfigCompatHarness h;

    auto logical = h.hal.SPI.acquire(spi::LogicalBusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}});
    ASSERT_TRUE(logical.has_value()) << "err=" << error::toString(logical.error());

    spi::BusConfig_remote typed;
    typed.pin_clk  = 18;
    typed.pin_mosi = 23;
    typed.pin_miso = 19;
    typed.pin_dc   = 5;

    auto reacquired = h.hal.SPI.acquire(typed);
    ASSERT_TRUE(reacquired.has_value()) << "err=" << error::toString(reacquired.error());
    EXPECT_EQ(logical.value().get(), reacquired.value().get());
    EXPECT_EQ(h.peer.create_count, 1u);
}

TEST(RemoteConfigCompat, LogicalUartThenTypedIgnoresWireOmittedFlowControl)
{
    RemoteConfigCompatHarness h;

    auto logical = h.hal.UART.acquire(uart::LogicalBusConfig{uart::Tx{17}, uart::Rx{16}});
    ASSERT_TRUE(logical.has_value()) << "err=" << error::toString(logical.error());

    uart::BusConfig_remote typed;
    typed.pin_tx  = 17;
    typed.pin_rx  = 16;
    typed.pin_rts = 4;

    auto reacquired = h.hal.UART.acquire(typed);
    ASSERT_TRUE(reacquired.has_value()) << "err=" << error::toString(reacquired.error());
    EXPECT_EQ(logical.value().get(), reacquired.value().get());
    EXPECT_EQ(h.peer.create_count, 1u);
}

TEST(RemoteConfigCompat, LogicalI2sThenTypedIgnoresMclkAndNormalizesBuffers)
{
    RemoteConfigCompatHarness h;

    auto logical = h.hal.I2S.acquire(i2s::LogicalBusConfig{i2s::Bclk{12}, i2s::Ws{0}, i2s::Dout{2}, i2s::Din{34}});
    ASSERT_TRUE(logical.has_value()) << "err=" << error::toString(logical.error());

    i2s::BusConfig_remote typed;
    typed.pin_bclk       = 12;
    typed.pin_ws         = 0;
    typed.pin_dout       = 2;
    typed.pin_din        = 34;
    typed.pin_mclk       = 3;
    typed.tx_buffer_size = 8000;
    typed.rx_buffer_size = 8191;

    auto reacquired = h.hal.I2S.acquire(typed);
    ASSERT_TRUE(reacquired.has_value()) << "err=" << error::toString(reacquired.error());
    EXPECT_EQ(logical.value().get(), reacquired.value().get());
    EXPECT_EQ(h.peer.create_count, 1u);
}

TEST(RemoteConfigCompat, TypedI2sRejectsIncompleteStandardWiringBeforeBusCreate)
{
    RemoteConfigCompatHarness h;
    const auto expect_invalid = [&h](i2s::BusConfig_remote cfg) {
        auto acquired = h.hal.I2S.acquire(cfg);
        ASSERT_FALSE(acquired.has_value());
        EXPECT_EQ(acquired.error(), error::error_t::INVALID_ARGUMENT) << "err=" << error::toString(acquired.error());
        EXPECT_EQ(h.peer.create_count, 0u);
    };

    i2s::BusConfig_remote missing_bclk;
    missing_bclk.pin_ws   = 0;
    missing_bclk.pin_dout = 2;
    expect_invalid(missing_bclk);

    i2s::BusConfig_remote missing_ws;
    missing_ws.pin_bclk = 12;
    missing_ws.pin_din  = 34;
    expect_invalid(missing_ws);

    i2s::BusConfig_remote missing_data;
    missing_data.pin_bclk = 12;
    missing_data.pin_ws   = 0;
    expect_invalid(missing_data);
}

TEST(RemoteConfigCompat, TypedUartRejectsDifferentWireBufferUnit)
{
    RemoteConfigCompatHarness h;

    uart::BusConfig_remote cfg_a;
    cfg_a.pin_tx         = 25;
    cfg_a.pin_rx         = 26;
    cfg_a.rx_buffer_size = 512;

    auto first = h.hal.UART.acquire(cfg_a);
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());

    uart::BusConfig_remote cfg_b = cfg_a;
    cfg_b.rx_buffer_size         = 1024;

    auto second = h.hal.UART.acquire(cfg_b);
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), error::error_t::INVALID_STATE) << "err=" << error::toString(second.error());
    EXPECT_EQ(h.peer.create_count, 1u);
}

TEST(MuxRemoteServerAdapter, DispatchesRequestToHandler)
{
    SessionPair pair;

    struct HandlerCtx {
        int calls             = 0;
        frame::Kind last_kind = frame::Kind::Padding;
        uint8_t last_seq      = 0;
        size_t last_payload   = 0;
    } handler_ctx;

    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ab.sink()};
    adapter.setExternalPoll(true);
    adapter.setHandler(
        [](void* ctx, frame::Kind kind, uint8_t seq, data::ConstDataSpan payload, data::MuxFrameEncoder& enc,
           data::MuxFrameDecoder& dec) -> result_t<void> {
            (void)dec;
            auto* h = static_cast<HandlerCtx*>(ctx);
            h->calls++;
            h->last_kind    = kind;
            h->last_seq     = seq;
            h->last_payload = payload.size;
            if (kind == frame::Kind::Request) {
                enc.writeFrame(frame::Kind::Response, seq, payload);
            }
            return {};
        },
        &handler_ctx);

    const uint8_t script[] = {0x42, 0x43};
    pair.enc_a.writeFrame(frame::Kind::Request, 0x07, {script, sizeof(script)});
    pair.pump();

    adapter.pumpWire();
    auto r = adapter.service();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 1u);
    EXPECT_EQ(handler_ctx.calls, 1);
    EXPECT_EQ(handler_ctx.last_kind, frame::Kind::Request);
    EXPECT_EQ(handler_ctx.last_seq, 0x07);
    EXPECT_EQ(handler_ctx.last_payload, sizeof(script));
}

TEST(RemoteServerHandler, GpioPollReadsSubscribedPortOnceAndEmitsPinEvents)
{
    PortMaskGPIO device;
    gpio::GPIOGroup group{&device};
    remote::RemoteServerHandler handler;
    handler.gpio_group = &group;

    const types::gpio_number_t pins[] = {types::makeGpioNumber(0, 1), types::makeGpioNumber(0, 3)};
    auto sub                          = remote::RemoteServerHandler::gpioSubscribe(&handler, true, pins, 2);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());
    for (const auto pin : pins) {
        auto mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
        ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());
    }

    device.port.read_pin_calls  = 0;
    device.port.read_port_calls = 0;
    device.port.value           = (1u << 1) | (1u << 3);

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    uint8_t frame_buf[1024];
    data::RingFIFO frames;
    frames.setBuf(frame_buf, sizeof(frame_buf));

    std::vector<uint8_t> event_body;
    dec.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            if (view.kind != frame::Kind::Event) {
                return;
            }
            auto* body = static_cast<std::vector<uint8_t>*>(ctx);
            body->assign(view.payload.data, view.payload.data + view.payload.size);
        },
        &event_body);

    auto poll = remote::RemoteServerHandler::poll(&handler, enc);
    ASSERT_TRUE(poll.has_value()) << "err=" << error::toString(poll.error());
    EXPECT_EQ(device.port.read_port_calls, 1u);
    EXPECT_EQ(device.port.read_pin_calls, 0u);

    pumpValue(enc);
    SessionPair::transfer(enc.output(), frames.sink());
    pumpValue(dec, frames.source());
    ASSERT_FALSE(event_body.empty());

    bytecode::BytecodeRunner runner{alloc};
    runner.setReceiveOnly(true);
    GpioEventCapture capture;
    runner.setGpioEventHandler(&GpioEventCapture::onEvent, &capture);
    auto run = runner.runEvent({event_body.data(), event_body.size()});
    ASSERT_TRUE(run.has_value()) << "err=" << error::toString(run.error());

    ASSERT_EQ(capture.events.size(), 2u);
    EXPECT_EQ(capture.events[0].pin, pins[0]);
    EXPECT_TRUE(capture.events[0].level);
    EXPECT_EQ(capture.events[1].pin, pins[1]);
    EXPECT_TRUE(capture.events[1].level);
}

TEST(RemoteServerHandler, GpioSubscriptionUsesIGPIOPortOrdinal)
{
    ReorderedPortMaskGPIO device;
    gpio::GPIOGroup group{&device};
    remote::RemoteServerHandler handler;
    handler.gpio_group = &group;

    const auto pin = types::makeGpioNumber(0, 0);  // local 0 belongs to port ordinal 1
    auto sub       = remote::RemoteServerHandler::gpioSubscribe(&handler, true, &pin, 1);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());
    auto mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
    ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());

    device.ports[0].read_port_calls = 0;
    device.ports[1].read_port_calls = 0;
    device.ports[1].value           = 1u;

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    uint8_t frame_buf[1024];
    data::RingFIFO frames;
    frames.setBuf(frame_buf, sizeof(frame_buf));
    std::vector<uint8_t> event_body;
    dec.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            if (view.kind == frame::Kind::Event) {
                auto* body = static_cast<std::vector<uint8_t>*>(ctx);
                body->assign(view.payload.data, view.payload.data + view.payload.size);
            }
        },
        &event_body);

    auto poll = remote::RemoteServerHandler::poll(&handler, enc);
    ASSERT_TRUE(poll.has_value()) << "err=" << error::toString(poll.error());
    EXPECT_EQ(device.ports[0].read_port_calls, 0u);
    EXPECT_EQ(device.ports[1].read_port_calls, 1u);

    pumpValue(enc);
    SessionPair::transfer(enc.output(), frames.sink());
    pumpValue(dec, frames.source());
    ASSERT_FALSE(event_body.empty());

    bytecode::BytecodeRunner runner{alloc};
    runner.setReceiveOnly(true);
    GpioEventCapture capture;
    runner.setGpioEventHandler(&GpioEventCapture::onEvent, &capture);
    auto run = runner.runEvent({event_body.data(), event_body.size()});
    ASSERT_TRUE(run.has_value()) << "err=" << error::toString(run.error());
    ASSERT_EQ(capture.events.size(), 1u);
    EXPECT_EQ(capture.events[0].pin, pin);
    EXPECT_TRUE(capture.events[0].level);
}

TEST(RemoteServerHandler, GpioPollHandlesSubscribedPortOnePins)
{
    TwoPortMaskGPIO device;
    gpio::GPIOGroup group{&device};
    remote::RemoteServerHandler handler;
    handler.gpio_group = &group;

    const types::gpio_number_t pins[] = {types::makeGpioNumber(0, 32), types::makeGpioNumber(0, 33)};
    auto sub                          = remote::RemoteServerHandler::gpioSubscribe(&handler, true, pins, 2);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());
    for (const auto pin : pins) {
        auto mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
        ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());
    }

    device.ports[0].read_port_calls = 0;
    device.ports[1].read_port_calls = 0;
    device.ports[1].read_pin_calls  = 0;
    device.ports[1].value           = (1u << 0) | (1u << 1);

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    uint8_t frame_buf[1024];
    data::RingFIFO frames;
    frames.setBuf(frame_buf, sizeof(frame_buf));

    std::vector<uint8_t> event_body;
    dec.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            if (view.kind != frame::Kind::Event) {
                return;
            }
            auto* body = static_cast<std::vector<uint8_t>*>(ctx);
            body->assign(view.payload.data, view.payload.data + view.payload.size);
        },
        &event_body);

    auto poll = remote::RemoteServerHandler::poll(&handler, enc);
    ASSERT_TRUE(poll.has_value()) << "err=" << error::toString(poll.error());
    EXPECT_EQ(device.ports[0].read_port_calls, 0u);
    EXPECT_EQ(device.ports[1].read_port_calls, 1u);
    EXPECT_EQ(device.ports[1].read_pin_calls, 0u);

    pumpValue(enc);
    SessionPair::transfer(enc.output(), frames.sink());
    pumpValue(dec, frames.source());
    ASSERT_FALSE(event_body.empty());

    bytecode::BytecodeRunner runner{alloc};
    runner.setReceiveOnly(true);
    GpioEventCapture capture;
    runner.setGpioEventHandler(&GpioEventCapture::onEvent, &capture);
    auto run = runner.runEvent({event_body.data(), event_body.size()});
    ASSERT_TRUE(run.has_value()) << "err=" << error::toString(run.error());

    ASSERT_EQ(capture.events.size(), 2u);
    EXPECT_EQ(capture.events[0].pin, pins[0]);
    EXPECT_TRUE(capture.events[0].level);
    EXPECT_EQ(capture.events[1].pin, pins[1]);
    EXPECT_TRUE(capture.events[1].level);
}

TEST(RemoteServerHandler, GpioChangeEventsRequireModeOptInAndHelloResetsMask)
{
    PortMaskGPIO device;
    gpio::GPIOGroup group{&device};
    remote::RemoteServerHandler handler;
    handler.gpio_group = &group;

    const auto pin = types::makeGpioNumber(0, 2);
    auto sub       = remote::RemoteServerHandler::gpioSubscribe(&handler, true, &pin, 1);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());

    std::vector<uint8_t> event_body;
    device.port.value = 1u << 2;
    auto masked_poll  = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(masked_poll.has_value()) << "err=" << error::toString(masked_poll.error());
    EXPECT_TRUE(event_body.empty());

    auto mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
    ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());
    device.port.value = 0;
    auto opt_in_poll  = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(opt_in_poll.has_value()) << "err=" << error::toString(opt_in_poll.error());
    ASSERT_FALSE(event_body.empty());

    GpioEventCapture capture;
    auto decoded = decodeGpioEventBody(event_body, capture);
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    ASSERT_EQ(capture.events.size(), 1u);
    EXPECT_EQ(capture.events[0].pin, pin);
    EXPECT_FALSE(capture.events[0].level);

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    auto hello = remote::RemoteServerHandler::handler(&handler, frame::Kind::HelloReq, 1, {}, enc, dec);
    ASSERT_TRUE(hello.has_value()) << "err=" << error::toString(hello.error());

    sub = remote::RemoteServerHandler::gpioSubscribe(&handler, true, &pin, 1);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());
    device.port.value = 1u << 2;
    auto after_hello  = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(after_hello.has_value()) << "err=" << error::toString(after_hello.error());
    EXPECT_TRUE(event_body.empty());
}

TEST(RemoteServerHandler, GpioPinsClaimedClearsModeOptInUntilModeIsSetAgain)
{
    PortMaskGPIO device;
    gpio::GPIOGroup group{&device};
    remote::RemoteServerHandler handler;
    handler.gpio_group = &group;

    remote::ServerBusPool pool;
    pool.pins_claimed_fn  = &remote::RemoteServerHandler::gpioPinsClaimed;
    pool.pins_claimed_ctx = &handler;

    const auto pin = types::makeGpioNumber(0, 2);
    auto sub       = remote::RemoteServerHandler::gpioSubscribe(&handler, true, &pin, 1);
    ASSERT_TRUE(sub.has_value()) << "err=" << error::toString(sub.error());

    auto mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
    ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());
    device.port.value = 1u << 2;

    std::vector<uint8_t> event_body;
    auto opt_in_poll = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(opt_in_poll.has_value()) << "err=" << error::toString(opt_in_poll.error());
    ASSERT_FALSE(event_body.empty());

    GpioEventCapture capture;
    auto decoded = decodeGpioEventBody(event_body, capture);
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    ASSERT_EQ(capture.events.size(), 1u);
    EXPECT_EQ(capture.events[0].pin, pin);
    EXPECT_TRUE(capture.events[0].level);

    ASSERT_NE(pool.pins_claimed_fn, nullptr);
    const types::gpio_number_t claimed[] = {pin};
    pool.pins_claimed_fn(pool.pins_claimed_ctx, claimed, 1);

    device.port.value = 0;
    auto claimed_poll = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(claimed_poll.has_value()) << "err=" << error::toString(claimed_poll.error());
    EXPECT_TRUE(event_body.empty());

    mode = remote::RemoteServerHandler::gpioModeSet(&handler, pin, types::gpio_mode_t::Input);
    ASSERT_TRUE(mode.has_value()) << "err=" << error::toString(mode.error());
    device.port.value  = 1u << 2;
    auto reopt_in_poll = pollGpioEventBody(handler, event_body);
    ASSERT_TRUE(reopt_in_poll.has_value()) << "err=" << error::toString(reopt_in_poll.error());
    ASSERT_FALSE(event_body.empty());

    GpioEventCapture reopt_capture;
    decoded = decodeGpioEventBody(event_body, reopt_capture);
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    ASSERT_EQ(reopt_capture.events.size(), 1u);
    EXPECT_EQ(reopt_capture.events[0].pin, pin);
    EXPECT_TRUE(reopt_capture.events[0].level);
}

TEST(RemoteServerHandler, GpioSubscribeRequestEmitsInitialSnapshotEvent)
{
    TwoPortMaskGPIO device;
    device.ports[1].value = 1u << 0;
    gpio::GPIOGroup group{&device};

    uint8_t server_scratch[remote::kMaxScriptSize];
    remote::Server srv{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    srv.setGPIOGroup(group);

    remote::RemoteServerHandler handler;
    handler.server     = &srv;
    handler.gpio_group = &group;
    srv.runner().setGpioSubscribeHandler(&remote::RemoteServerHandler::gpioSubscribe, &handler);
    srv.runner().setGpioModeHandler(&remote::RemoteServerHandler::gpioModeSet, &handler);

    const types::gpio_number_t pins[] = {types::makeGpioNumber(0, 32), types::makeGpioNumber(0, 33)};
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder script_enc{script};
    auto enc_sub = script_enc.gpioSubscribe(pins, 2);
    ASSERT_TRUE(enc_sub.has_value());
    auto enc_end = script_enc.end();
    ASSERT_TRUE(enc_end.has_value());

    mem::Allocator& alloc = mem::defaultAllocator();
    data::MuxFrameEncoder enc{alloc};
    data::MuxFrameDecoder dec{alloc};
    uint8_t frame_buf[1024];
    data::RingFIFO frames;
    frames.setBuf(frame_buf, sizeof(frame_buf));

    struct Capture {
        std::vector<frame::Kind> kinds;
        std::vector<uint8_t> event_body;
        size_t response_count = 0;
    } capture_frames;

    dec.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            auto* cap = static_cast<Capture*>(ctx);
            cap->kinds.push_back(view.kind);
            if (view.kind == frame::Kind::Event) {
                cap->event_body.assign(view.payload.data, view.payload.data + view.payload.size);
            } else if (view.kind == frame::Kind::Response) {
                ++cap->response_count;
            }
        },
        &capture_frames);

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 7,
                                                        {script_buf, script.written()}, enc, dec);
    ASSERT_TRUE(handled.has_value());

    pumpValue(enc);
    SessionPair::transfer(enc.output(), frames.sink());
    pumpValue(dec, frames.source());

    ASSERT_GE(capture_frames.kinds.size(), 2u);
    EXPECT_EQ(capture_frames.kinds[0], frame::Kind::Event);
    EXPECT_EQ(capture_frames.kinds[1], frame::Kind::Response);
    EXPECT_EQ(capture_frames.response_count, 1u);
    ASSERT_FALSE(capture_frames.event_body.empty());

    bytecode::BytecodeRunner runner{alloc};
    runner.setReceiveOnly(true);
    GpioEventCapture capture;
    runner.setGpioEventHandler(&GpioEventCapture::onEvent, &capture);
    auto run = runner.runEvent({capture_frames.event_body.data(), capture_frames.event_body.size()});
    ASSERT_TRUE(run.has_value());

    ASSERT_EQ(capture.events.size(), 2u);
    EXPECT_EQ(capture.events[0].pin, pins[0]);
    EXPECT_TRUE(capture.events[0].level);
    EXPECT_EQ(capture.events[1].pin, pins[1]);
    EXPECT_FALSE(capture.events[1].level);
}

TEST(RemoteServerHandler, GpioSnapshotYieldsLastEncoderSlotToResponse)
{
    TwoPortMaskGPIO device;
    device.ports[1].value = 1u << 0;
    gpio::GPIOGroup group{&device};

    uint8_t server_scratch[remote::kMaxScriptSize];
    remote::Server srv{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    srv.setGPIOGroup(group);

    remote::RemoteServerHandler handler;
    handler.server     = &srv;
    handler.gpio_group = &group;
    srv.runner().setGpioSubscribeHandler(&remote::RemoteServerHandler::gpioSubscribe, &handler);

    const types::gpio_number_t pin = types::makeGpioNumber(0, 32);
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder script_enc{script};
    auto encoded = script_enc.gpioSubscribe(&pin, 1);
    if (encoded.has_value()) {
        encoded = script_enc.end();
    }
    ASSERT_TRUE(encoded.has_value()) << "err=" << error::toString(encoded.error());

    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    for (size_t i = 0; i + 1 < data::BlockSource::kMaxBlocks; ++i) {
        ASSERT_TRUE(pair.enc_b.writeFrame(frame::Kind::Ping, static_cast<uint8_t>(i), {}));
    }
    ASSERT_EQ(pair.enc_b.output().blockCount(), data::BlockSource::kMaxBlocks - 1);

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 7,
                                                        {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(handled.has_value()) << "err=" << error::toString(handled.error());
    EXPECT_EQ(pair.enc_b.output().blockCount(), data::BlockSource::kMaxBlocks);

    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    size_t event_count    = 0;
    size_t response_count = 0;
    for (const auto& captured : capture.frames) {
        event_count += captured.kind == frame::Kind::Event ? 1u : 0u;
        response_count += captured.kind == frame::Kind::Response && captured.seq == 7 ? 1u : 0u;
    }
    EXPECT_EQ(event_count, 0u);
    EXPECT_EQ(response_count, 1u);
}

TEST(RemoteServerStreamTransfer, I2SWriteReachesRegisteredBusAndCompletes)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 16000;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 1;
    cfg.write_timeout_ms = 100;
    RecordingI2SBus bus{37};
    i2s::TxAccessor tx{bus, cfg};
    auto reg = server.registerI2S(0, tx);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;
    adapter.setHandler(&remote::RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&remote::RemoteServerHandler::poll, &handler);
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    std::array<uint8_t, 513> payload{};
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>((i * 13u + 7u) & 0xFFu);
    }
    data::MemorySource src{payload.data(), payload.size()};
    const uint8_t stream_id = session.attachStream(&src, nullptr);
    ASSERT_NE(stream_id, 0xFF);

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e = enc.i2sConfig(0, cfg);
    if (e.has_value()) {
        e = enc.streamTransfer(types::bus_kind_t::I2S, 0, stream_id, static_cast<uint32_t>(payload.size()), 0, {});
    }
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto req = session.request({script_buf, script.written()});
    session.detachStream(stream_id);
    ASSERT_TRUE(req.has_value()) << "err=" << error::toString(req.error());
    auto chk = session.checkResponse();
    ASSERT_TRUE(chk.has_value()) << "err=" << error::toString(chk.error());

    EXPECT_GT(bus.write_calls, 1u);
    ASSERT_EQ(bus.bytes.size(), payload.size());
    EXPECT_EQ(0, ::memcmp(bus.bytes.data(), payload.data(), payload.size()));
    EXPECT_TRUE(src.eof());
}

TEST(RemoteTransferWire, OverlongSourceIsCappedAtRequestedTxLen)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 16000;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 1;
    cfg.write_timeout_ms = 100;
    RecordingI2SBus device_bus{64};
    i2s::TxAccessor tx{device_bus, cfg};
    auto reg = server.registerI2S(0, tx);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;
    adapter.setHandler(&remote::RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&remote::RemoteServerHandler::poll, &handler);
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    std::array<uint8_t, 20> payload{};
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i + 1);
    }
    data::MemorySource src{payload.data(), payload.size()};
    i2s::Bus_remote host_bus{session, 0};

    auto written = host_bus.write(nullptr, cfg, &src, 8);
    ASSERT_TRUE(written.has_value()) << "err=" << error::toString(written.error());
    EXPECT_EQ(written.value(), 8u);

    ASSERT_EQ(device_bus.bytes.size(), 8u);
    EXPECT_EQ(0, ::memcmp(device_bus.bytes.data(), payload.data(), 8));
    EXPECT_FALSE(src.eof());
    auto rest = src.peek(payload.size());
    ASSERT_TRUE(rest.has_value()) << "err=" << error::toString(rest.error());
    ASSERT_EQ(rest.value().size, payload.size() - 8);
    EXPECT_EQ(rest.value().data[0], payload[8]);
}

TEST(RemoteTransferWire, UARTHostTransferRoundtripThroughServer)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    uart::AccessConfig cfg;
    cfg.baud_rate             = 921600;
    cfg.first_byte_timeout_ms = 100;
    cfg.inter_byte_timeout_ms = 20;
    cfg.write_timeout_ms      = 100;
    EchoUARTBus device_bus;
    uart::Accessor acc{device_bus, cfg};
    auto reg = server.registerUART(0, acc);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;
    adapter.setHandler(&remote::RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&remote::RemoteServerHandler::poll, &handler);
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    uint8_t tx[]           = {0xE1, 0xE2, 0xE3, 0xE4, 0xE5};
    uint8_t rx[sizeof(tx)] = {};
    data::MemorySource src{tx, sizeof(tx)};
    data::MemorySink dst{rx, sizeof(rx)};
    uart::Bus_remote host_bus{session, 0, uart::IBusConfig{}};

    auto result = host_bus.transfer(nullptr, cfg, &src, sizeof(tx), &dst, sizeof(rx));
    ASSERT_TRUE(result.has_value()) << "err=" << error::toString(result.error());
    EXPECT_EQ(result->tx, sizeof(tx));
    EXPECT_EQ(result->rx, sizeof(rx));
    EXPECT_TRUE(src.eof());
    EXPECT_EQ(0, ::memcmp(rx, tx, sizeof(tx)));
    EXPECT_EQ(device_bus.last_cfg.baud_rate, 921600u);
}

TEST(RemoteServerStreamTransfer, ReadOnlyLargeI2CTransferCompletesThroughPendingPoll)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    PatternI2CBus device_bus{0x42, frame::kMaxDataPayload};
    i2c::MasterAccessConfig device_cfg;
    i2c::MasterAccessor device_acc{device_bus, device_cfg};
    auto reg = server.registerI2C(0, device_acc);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());
    attachRealServer(session, adapter, handler, server);

    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote host_bus{session, 0, bus_cfg};
    i2c::MasterAccessConfig cfg;
    cfg.i2c_addr        = 0x42;
    cfg.wire_timeout_ms = 100;
    i2c::MasterAccessor host_acc{host_bus, cfg};

    static constexpr size_t kLargeRxLen = 8192;
    std::array<uint8_t, kLargeRxLen> rx{};
    data::MemorySink dst{rx.data(), rx.size()};

    auto read = host_acc.read(dst, rx.size());
    ASSERT_TRUE(read.has_value()) << "err=" << error::toString(read.error());
    EXPECT_EQ(read.value(), rx.size());
    ASSERT_EQ(dst.written(), rx.size());
    EXPECT_GT(device_bus.transfer_calls, data::BlockSource::kMaxBlocks);
    EXPECT_EQ(device_bus.rx_cursor, rx.size());
    for (size_t i = 0; i < rx.size(); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(i & 0xFFu)) << "i=" << i;
    }
}

TEST(RemoteServerStreamTransfer, ReadOnlyI2CBackpressureDoesNotDropPolledData)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    PatternI2CBus device_bus{0x42, frame::kMaxDataPayload};
    i2c::MasterAccessConfig device_cfg;
    i2c::MasterAccessor device_acc{device_bus, device_cfg};
    auto reg = server.registerI2C(0, device_acc);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());
    handler.server = &server;

    static constexpr uint8_t kStreamId = 1;
    static constexpr size_t kRxLen     = frame::kMaxDataPayload * (data::BlockSource::kMaxBlocks + 2);
    std::array<uint8_t, kRxLen> rx{};
    data::MemorySink dst{rx.data(), rx.size()};
    ASSERT_TRUE(pair.dec_a.setSink(kStreamId, dst));

    i2c::MasterAccessConfig cfg;
    cfg.i2c_addr        = 0x42;
    cfg.wire_timeout_ms = 100;
    uint8_t meta[]      = {0};
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e = enc.configure(0, cfg);
    if (e.has_value()) {
        e = enc.streamTransfer(types::bus_kind_t::I2C, 0, kStreamId, 0, static_cast<uint32_t>(kRxLen),
                               {meta, sizeof(meta)});
    }
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 11,
                                                        {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(handled.has_value()) << "err=" << error::toString(handled.error());
    ASSERT_TRUE(server.responseDeferred());

    auto first_poll = server.poll(pair.enc_b, 100);
    ASSERT_TRUE(first_poll.has_value()) << "err=" << error::toString(first_poll.error());
    constexpr size_t kDataQueueLimit = data::BlockSource::kMaxBlocks - 1;
    EXPECT_EQ(pair.enc_b.output().blockCount(), kDataQueueLimit);
    EXPECT_EQ(device_bus.transfer_calls, kDataQueueLimit);
    EXPECT_EQ(device_bus.rx_cursor, frame::kMaxDataPayload * kDataQueueLimit);

    const size_t calls_at_full  = device_bus.transfer_calls;
    const size_t cursor_at_full = device_bus.rx_cursor;
    auto blocked_poll           = server.poll(pair.enc_b, 101);
    ASSERT_TRUE(blocked_poll.has_value()) << "err=" << error::toString(blocked_poll.error());
    EXPECT_EQ(pair.enc_b.output().blockCount(), kDataQueueLimit);
    EXPECT_EQ(device_bus.transfer_calls, calls_at_full);
    EXPECT_EQ(device_bus.rx_cursor, cursor_at_full);

    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_EQ(dst.written(), cursor_at_full);

    auto resumed_poll = server.poll(pair.enc_b, 102);
    ASSERT_TRUE(resumed_poll.has_value()) << "err=" << error::toString(resumed_poll.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);

    ASSERT_EQ(dst.written(), rx.size());
    EXPECT_EQ(device_bus.rx_cursor, rx.size());
    size_t response_count = 0;
    for (const auto& f : capture.frames) {
        if (f.kind == frame::Kind::Response) {
            ++response_count;
        }
    }
    EXPECT_EQ(response_count, 1u);
    for (size_t i = 0; i < rx.size(); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(i & 0xFFu)) << "i=" << i;
    }
}

TEST(RemoteTransferWire, ZeroLengthI2CProbeReachesRemoteBus)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    PatternI2CBus device_bus{0x42, frame::kMaxDataPayload};
    i2c::MasterAccessConfig device_cfg;
    i2c::MasterAccessor device_acc{device_bus, device_cfg};
    auto reg = server.registerI2C(0, device_acc);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());
    attachRealServer(session, adapter, handler, server);

    i2c::Bus_remote host_bus{session, 0, i2c::IBusConfig{}};

    auto ack = host_bus.probe(0x42, 100000, 50);
    ASSERT_TRUE(ack.has_value()) << "err=" << error::toString(ack.error());
    EXPECT_EQ(device_bus.probe_calls, 1u);
    EXPECT_EQ(device_bus.last_addr, 0x42u);

    auto nack = host_bus.probe(0x21, 100000, 50);
    ASSERT_FALSE(nack.has_value());
    EXPECT_EQ(nack.error(), error::error_t::I2C_NO_ACK) << "err=" << error::toString(nack.error());
    EXPECT_EQ(device_bus.last_addr, 0x21u);
}

TEST(RemoteServerStreamTransfer, HelloAbortsPendingStream)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 16000;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 1;
    cfg.write_timeout_ms = 100;
    RecordingI2SBus bus{37};
    i2s::TxAccessor tx{bus, cfg};
    auto reg = server.registerI2S(0, tx);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e = enc.streamTransfer(types::bus_kind_t::I2S, 0, 1, 8, 0, {});
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto first = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 3, {script_buf, script.written()},
                                                      pair.enc_b, pair.dec_b);
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
    EXPECT_TRUE(server.responseDeferred());

    auto hello = remote::RemoteServerHandler::handler(&handler, frame::Kind::HelloReq, 4, {}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(hello.has_value()) << "err=" << error::toString(hello.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    ASSERT_EQ(capture.frames.size(), 1u);
    EXPECT_EQ(capture.frames[0].kind, frame::Kind::HelloResp);
    capture.frames.clear();

    auto second = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 5,
                                                       {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    EXPECT_TRUE(server.responseDeferred());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(capture.frames.empty());

    server.abortPendingStream();
}

TEST(RemoteServerStreamTransfer, PendingStreamStallTimesOut)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};
    server.setPendingStreamTimeout(10);

    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 16000;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 1;
    cfg.write_timeout_ms = 100;
    RecordingI2SBus bus{37};
    i2s::TxAccessor tx{bus, cfg};
    auto reg = server.registerI2S(0, tx);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e = enc.streamTransfer(types::bus_kind_t::I2S, 0, 2, 8, 0, {});
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 6,
                                                        {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(handled.has_value()) << "err=" << error::toString(handled.error());
    EXPECT_TRUE(server.responseDeferred());

    auto first_poll = server.poll(pair.enc_b, 100);
    ASSERT_TRUE(first_poll.has_value()) << "err=" << error::toString(first_poll.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(capture.frames.empty());

    auto edge_poll = server.poll(pair.enc_b, 110);
    ASSERT_TRUE(edge_poll.has_value()) << "err=" << error::toString(edge_poll.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(capture.frames.empty());

    auto timed_out = server.poll(pair.enc_b, 111);
    ASSERT_TRUE(timed_out.has_value()) << "err=" << error::toString(timed_out.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    ASSERT_EQ(capture.frames.size(), 1u);
    EXPECT_EQ(capture.frames[0].kind, frame::Kind::Response);
    EXPECT_EQ(capture.frames[0].seq, 6);

    bytecode::BytecodeRunner runner{mem::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run = runner.run({capture.frames[0].payload.data(), capture.frames[0].payload.size()});
    ASSERT_TRUE(run.has_value()) << "err=" << error::toString(run.error());
    ASSERT_TRUE(runner.statusReported());
    EXPECT_EQ(runner.reportedStatus(), error::error_t::TIMEOUT_ERROR);
    capture.frames.clear();

    auto second = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 7,
                                                       {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    EXPECT_TRUE(server.responseDeferred());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(capture.frames.empty());

    server.abortPendingStream();
}

TEST(RemoteServerStreamTransfer, TxRxPendingCompletesOnlyAfterBothDirectionsFinish)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    SplitRxSPIBus bus{2};
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor acc{bus, cfg};
    auto reg = server.registerSPI(0, acc);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    uint8_t spi_meta[15] = {};
    auto e               = enc.streamTransfer(types::bus_kind_t::SPI, 0, 2, 3, 5, {spi_meta, sizeof(spi_meta)});
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 10,
                                                        {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(handled.has_value()) << "err=" << error::toString(handled.error());
    ASSERT_TRUE(server.responseDeferred());

    uint8_t tx_bytes[] = {0x11, 0x22, 0x33};
    data::MemorySource tx_src{tx_bytes, sizeof(tx_bytes)};
    ASSERT_TRUE(pair.enc_a.attach(2, tx_src));
    pumpValue(pair.enc_a);
    SessionPair::transfer(pair.enc_a.output(), pair.wire_ab.sink());
    pumpValue(pair.dec_b, pair.wire_ab.source());

    auto first_poll = server.poll(pair.enc_b, 100);
    ASSERT_TRUE(first_poll.has_value()) << "err=" << error::toString(first_poll.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(server.responseDeferred());
    EXPECT_EQ(bus.tx_bytes.size(), sizeof(tx_bytes));
    size_t response_count = 0;
    for (const auto& f : capture.frames) {
        if (f.kind == frame::Kind::Response) {
            ++response_count;
        }
    }
    EXPECT_EQ(response_count, 0u);
    EXPECT_EQ(bus.rx_cursor, 2u);

    auto second_poll = server.poll(pair.enc_b, 101);
    ASSERT_TRUE(second_poll.has_value()) << "err=" << error::toString(second_poll.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    response_count = 0;
    for (const auto& f : capture.frames) {
        if (f.kind == frame::Kind::Response) {
            ++response_count;
        }
    }
    EXPECT_EQ(response_count, 1u);
    EXPECT_EQ(bus.rx_cursor, 5u);

    pair.enc_a.detach(2);
}

TEST(RemoteServerStreamTransfer, StreamTransferMustBeTerminal)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 16000;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 1;
    cfg.write_timeout_ms = 100;
    RecordingI2SBus bus{37};
    i2s::TxAccessor tx{bus, cfg};
    auto reg = server.registerI2S(0, tx);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e                        = enc.streamTransfer(types::bus_kind_t::I2S, 0, 3, 8, 0, {});
    const size_t configure_offset = script.written();
    if (e.has_value()) {
        e = enc.i2sConfig(0, cfg);
    }
    if (e.has_value()) {
        e = enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 8,
                                                        {script_buf, script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(handled.has_value()) << "err=" << error::toString(handled.error());
    EXPECT_FALSE(server.responseDeferred());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    ASSERT_EQ(capture.frames.size(), 1u);
    EXPECT_EQ(capture.frames[0].kind, frame::Kind::Response);
    EXPECT_EQ(capture.frames[0].seq, 8);
    EXPECT_EQ(bus.write_calls, 0u);

    bytecode::BytecodeRunner runner{mem::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run = runner.run({capture.frames[0].payload.data(), capture.frames[0].payload.size()});
    ASSERT_TRUE(run.has_value()) << "err=" << error::toString(run.error());
    ASSERT_TRUE(runner.statusReported());
    EXPECT_EQ(runner.reportedStatus(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(runner.reportedOffset(), configure_offset);
    capture.frames.clear();

    // rx-only stream transfers defer the response the same way, so the
    // terminal rule applies to them too: a later instruction's stored
    // slots would be dropped from the deferred Response.
    data::MemorySink rx_script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder rx_enc{rx_script};
    e                           = rx_enc.streamTransfer(types::bus_kind_t::I2S, 0, 5, 0, 8, {});
    const size_t rx_only_offset = rx_script.written();
    if (e.has_value()) {
        e = rx_enc.i2sConfig(0, cfg);
    }
    if (e.has_value()) {
        e = rx_enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto rx_handled = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 10,
                                                           {script_buf, rx_script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(rx_handled.has_value()) << "err=" << error::toString(rx_handled.error());
    EXPECT_FALSE(server.responseDeferred());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    ASSERT_EQ(capture.frames.size(), 1u);
    EXPECT_EQ(capture.frames[0].kind, frame::Kind::Response);
    EXPECT_EQ(capture.frames[0].seq, 10);
    bytecode::BytecodeRunner rx_runner{mem::defaultAllocator()};
    rx_runner.setReceiveOnly(true);
    auto rx_run = rx_runner.run({capture.frames[0].payload.data(), capture.frames[0].payload.size()});
    ASSERT_TRUE(rx_run.has_value()) << "err=" << error::toString(rx_run.error());
    ASSERT_TRUE(rx_runner.statusReported());
    EXPECT_EQ(rx_runner.reportedStatus(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(rx_runner.reportedOffset(), rx_only_offset);
    capture.frames.clear();

    data::MemorySink second_script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder second_enc{second_script};
    e = second_enc.streamTransfer(types::bus_kind_t::I2S, 0, 4, 8, 0, {});
    if (e.has_value()) {
        e = second_enc.end();
    }
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());

    auto second = remote::RemoteServerHandler::handler(&handler, frame::Kind::Request, 9,
                                                       {script_buf, second_script.written()}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    EXPECT_TRUE(server.responseDeferred());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    EXPECT_TRUE(capture.frames.empty());

    server.abortPendingStream();
}

TEST(MuxRemoteSession, AttachStreamTxOnly)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};

    const uint8_t raw[] = {0x11, 0x22, 0x33};
    data::MemorySource src{raw, sizeof(raw)};

    uint8_t id = session.attachStream(&src, nullptr);
    EXPECT_NE(id, 0xFF);

    session.detachStream(id);
}

TEST(MuxRemoteSession, AttachStreamRxOnly)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};

    uint8_t rx_buf[64] = {};
    data::MemorySink sink{rx_buf, sizeof(rx_buf)};

    uint8_t id = session.attachStream(nullptr, &sink);
    EXPECT_NE(id, 0xFF);

    session.detachStream(id);
}

TEST(MuxRemoteSession, AttachStreamBidirectional)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};

    const uint8_t tx_data[] = {0xAA, 0xBB};
    data::MemorySource tx_src{tx_data, sizeof(tx_data)};
    uint8_t rx_buf[64] = {};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};

    uint8_t id = session.attachStream(&tx_src, &rx_sink);
    EXPECT_NE(id, 0xFF);

    EXPECT_EQ(session.encoder().stream(id), &tx_src);

    session.detachStream(id);
    EXPECT_EQ(session.encoder().stream(id), nullptr);
}

TEST(MuxRemoteSession, AttachStreamDataRoundtrip)
{
    SessionPair pair;
    remote::RemoteSession session_a{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};

    const uint8_t tx_data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    data::MemorySource tx_src{tx_data, sizeof(tx_data)};
    uint8_t rx_buf[64] = {};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};

    uint8_t id = session_a.attachStream(&tx_src, &rx_sink);

    auto* server_rx = pair.dec_b.createStream(id, 512);
    ASSERT_NE(server_rx, nullptr);

    pumpValue(pair.enc_a);
    pair.pump();

    auto peeked = server_rx->peek(256);
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked.value().size, sizeof(tx_data));
    EXPECT_EQ(::memcmp(peeked.value().data, tx_data, sizeof(tx_data)), 0);

    const uint8_t reply[] = {0xCA, 0xFE};
    pair.enc_b.writeFrame(frame::Kind::Data, id, {reply, sizeof(reply)});
    pair.pump();

    EXPECT_EQ(rx_buf[0], 0xCA);
    EXPECT_EQ(rx_buf[1], 0xFE);

    session_a.detachStream(id);
}

// ---- RemoteSession stream_id quarantine (host-timeout resync) -------------
//
// A peer that captures a Request's seq/b3 but withholds its Response,
// standing in for a device that is still mid-transfer when the host gives up
// waiting (spec/design/remote.md §timeout / resync). sendPendingResponse()
// releases it later on demand so tests can control exactly when the
// "delayed terminal frame" arrives.
struct DelayedResponsePeer {
    data::MuxFrameEncoder* enc = nullptr;
    bool captured              = false;
    uint8_t seq                = 0;

    static void onFrame(void* ctx, const frame::View& view)
    {
        auto* p = static_cast<DelayedResponsePeer*>(ctx);
        if (view.kind == frame::Kind::Request) {
            p->captured = true;
            p->seq      = view.b3;
        }
    }

    void sendPendingResponse()
    {
        const uint8_t payload[] = {0x00};
        enc->writeFrame(frame::Kind::Response, seq, {payload, sizeof(payload)});
    }
};

// Answers the current Request normally, but first (once) replays a stale
// Response for an earlier, already-quarantined seq — reproducing the
// "mismatched frame observed while awaitResponse() is waiting on a *later*
// request" release path.
struct StaleThenCurrentResponsePeer {
    data::MuxFrameEncoder* enc = nullptr;
    bool has_stale             = false;
    uint8_t stale_seq          = 0;

    void queueStale(uint8_t seq)
    {
        has_stale = true;
        stale_seq = seq;
    }

    static void onFrame(void* ctx, const frame::View& view)
    {
        auto* p = static_cast<StaleThenCurrentResponsePeer*>(ctx);
        if (view.kind != frame::Kind::Request) {
            return;
        }
        if (p->has_stale) {
            const uint8_t stale_payload[] = {0x01};
            p->enc->writeFrame(frame::Kind::Response, p->stale_seq, {stale_payload, sizeof(stale_payload)});
            p->has_stale = false;
        }
        const uint8_t resp[] = {0x02};
        p->enc->writeFrame(frame::Kind::Response, view.b3, {resp, sizeof(resp)});
    }
};

// Answers every normal Request immediately (same seq echoed back), but
// never answers a NORESP request (bit7 of B3 set) — matching the real
// protocol contract for requestNoResponse() (spec/design/remote.md
// "NORESP request").
struct NorespAwareEchoPeer {
    data::MuxFrameEncoder* enc = nullptr;

    static void onFrame(void* ctx, const frame::View& view)
    {
        auto* p = static_cast<NorespAwareEchoPeer*>(ctx);
        if (view.kind != frame::Kind::Request || (view.b3 & 0x80) != 0) {
            return;
        }
        const uint8_t resp[] = {0x00};
        p->enc->writeFrame(frame::Kind::Response, view.b3, {resp, sizeof(resp)});
    }
};

// A Source whose peek()/advance() always fail. Used to force a pumpWire()
// error strictly after a Request has already been enqueued/flushed to a
// separate, real TX sink — reproducing the "post-enqueue, non-timeout
// pump error" case (F4/F5) without needing a peer at all.
class AlwaysFailingSource : public data::Source {
public:
    result_t<data::ConstDataSpan> peek(size_t) override
    {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    result_t<void> advance(size_t) override
    {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    bool eof() const override
    {
        return false;
    }
};

static void setPairPeerPoll(SessionPair& pair, remote::RemoteSession& session)
{
    session.setPeerPoll(
        [](void* ctx) {
            auto* p = static_cast<SessionPair*>(ctx);
            p->pump();
        },
        &pair);
}

TEST(MuxRemoteSession, TimedOutStreamIsNotImmediatelyReused)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);

    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    EXPECT_EQ(timed_out.error(), error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(peer.captured);
    session.quarantineStream(id0, seq0);

    // The quarantined id must not come back out of allocateStreamId() while
    // its release condition is still unmet.
    const uint8_t next = session.attachStream(nullptr, nullptr);
    ASSERT_NE(next, 0xFF);
    EXPECT_NE(next, id0);
    session.detachStream(next);
}

TEST(MuxRemoteSession, DelayedResponseObservedViaPollReleasesQuarantineForReuse)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // Fill the rest of the pool so id0 is the only slot that a subsequent
    // allocateStreamId() could hand out once released.
    std::array<uint8_t, remote::RemoteSession::kMaxStreams> filled{};
    size_t filled_count = 0;
    for (;;) {
        const uint8_t id = session.attachStream(nullptr, nullptr);
        if (id == 0xFF) {
            break;
        }
        filled[filled_count++] = id;
    }
    EXPECT_EQ(session.attachStream(nullptr, nullptr), 0xFF);

    // The delayed terminal Response arrives and is observed purely through
    // poll() — no awaitResponse() is pending at this point.
    peer.sendPendingResponse();
    auto polled = session.poll();
    ASSERT_TRUE(polled.has_value());

    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);

    for (size_t i = 0; i < filled_count; ++i) {
        session.detachStream(filled[i]);
    }
}

TEST(MuxRemoteSession, MismatchedFrameDuringAwaitResponseReleasesQuarantine)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer stall_peer;
    stall_peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &stall_peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // The next request's peer replays the stale Response for the timed-out
    // seq before answering the new one, so awaitResponse() must observe the
    // mismatch (b3 != its own awaiting seq) and still release id0.
    StaleThenCurrentResponsePeer peer2;
    peer2.enc = &pair.enc_b;
    peer2.queueStale(stall_peer.seq);
    pair.dec_b.setFrameHandler(&StaleThenCurrentResponsePeer::onFrame, &peer2);

    cfg.response_timeout_ms = 2000;
    session.setConfig(cfg);
    const uint8_t script2[] = {0x01};
    auto ok                 = session.request({script2, sizeof(script2)});
    ASSERT_TRUE(ok.has_value()) << "err=" << error::toString(ok.error());

    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);
}

TEST(MuxRemoteSession, ShortQuarantineTimerExpiresWithoutAnyResponse)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                 = session.getConfig();
    cfg.response_timeout_ms  = 5;
    cfg.stream_quarantine_ms = 5;  // shortened insurance timer
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // No Response ever arrives for this seq — only the insurance timer can
    // release the entry.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);
}

TEST(MuxRemoteSession, AllStreamsQuarantinedExhaustsAttachPool)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t script[] = {0x00};
    for (uint8_t i = 0; i < remote::RemoteSession::kMaxStreams; ++i) {
        const uint8_t id = session.attachStream(nullptr, nullptr);
        ASSERT_EQ(id, i);
        uint8_t seq    = 0xFF;
        auto timed_out = session.request({script, sizeof(script)}, &seq);
        ASSERT_FALSE(timed_out.has_value());
        session.quarantineStream(id, seq);
    }

    EXPECT_EQ(session.attachStream(nullptr, nullptr), 0xFF);
}

TEST(MuxRemoteSession, QuarantinedStreamDataFrameDoesNotLeakIntoNewSink)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    uint8_t rx_buf[64] = {};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};
    const uint8_t id1 = session.attachStream(nullptr, &rx_sink);
    ASSERT_EQ(id1, 1u);

    // Stray Data for the still-quarantined id must not surface anywhere —
    // it targets a stream whose Sink was cleared by quarantineStream(), so
    // the decoder has to drop it (mux.inl deliverData() null-Sink path).
    const uint8_t stray[] = {0xEE, 0xEE};
    pair.enc_b.writeFrame(frame::Kind::Data, id0, {stray, sizeof(stray)});

    // Legitimate Data for the new transfer's id, in the same pump batch.
    const uint8_t good[] = {0x01, 0x02};
    pair.enc_b.writeFrame(frame::Kind::Data, id1, {good, sizeof(good)});

    pair.pump();

    EXPECT_EQ(rx_buf[0], 0x01);
    EXPECT_EQ(rx_buf[1], 0x02);
    EXPECT_EQ(rx_buf[2], 0x00);  // untouched — the stray Data never reached this sink
    session.detachStream(id1);
}

TEST(MuxRemoteSession, HelloClearsAllQuarantine)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // Switch the peer to answer HelloReq so hello() can succeed.
    pair.dec_b.setFrameHandler(
        [](void* ctx, const frame::View& view) {
            auto* enc = static_cast<data::MuxFrameEncoder*>(ctx);
            if (view.kind == frame::Kind::HelloReq) {
                uint8_t caps[] = {remote::kProtocolVersion, 0};
                enc->writeFrame(frame::Kind::HelloResp, view.b3, {caps, sizeof(caps)});
            }
        },
        &pair.enc_b);

    cfg.response_timeout_ms = 2000;
    session.setConfig(cfg);
    auto hr = session.hello();
    ASSERT_TRUE(hr.has_value()) << "err=" << error::toString(hr.error());

    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);
}

TEST(MuxRemoteSession, ResetDoesNotClearQuarantine)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // reset() is fire-and-forget: the host never waits for the device's
    // reset Response, so a successful send proves nothing about whether
    // the device has stopped the old transfer (F2 / spec/design/remote.md
    // §timeout / resync, item 3). Only a terminal-frame match, the
    // insurance timer, or hello() may release the entry.
    cfg.response_timeout_ms = 2000;
    session.setConfig(cfg);
    auto reset_result = session.reset();
    ASSERT_TRUE(reset_result.has_value()) << "err=" << error::toString(reset_result.error());

    const uint8_t next = session.attachStream(nullptr, nullptr);
    ASSERT_NE(next, 0xFF);
    EXPECT_NE(next, id0);
    session.detachStream(next);
}

TEST(MuxRemoteSession, StaleDataRefreshesQuarantineInactivityTimer)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    // Margins are deliberately wide (50ms refresh vs 250ms window): a loaded
    // CI host can stall this thread for tens of ms, which must not read as
    // "the producer went quiet".
    cfg.stream_quarantine_ms = 250;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    // Keep delivering stray Data for the quarantined id across more wall
    // time than one insurance-timer window (8 * 50ms > 250ms). Each arrival
    // must reset the "last activity seen" clock via the stale-Data observer
    // (F1) — mirroring the server's own inactivity semantics
    // (Config::stream_quarantine_ms) instead of expiring on a fixed
    // deadline.
    const uint8_t stray[] = {0xEE};
    for (int i = 0; i < 8; ++i) {
        pair.enc_b.writeFrame(frame::Kind::Data, id0, {stray, sizeof(stray)});
        pair.pump();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const uint8_t still_quarantined = session.attachStream(nullptr, nullptr);
    ASSERT_NE(still_quarantined, 0xFF);
    EXPECT_NE(still_quarantined, id0);
    session.detachStream(still_quarantined);

    // Once the stray Data stops, the insurance timer runs its course from
    // its last refresh.
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);
}

TEST(MuxRemoteSession, StaleDataRefreshesQuarantineForRxBearingStream)
{
    // Same as above but the quarantined stream HAD an rx Sink (the common
    // shape for the contamination hazard): after quarantineStream() the
    // decoder stream stays `active` with its direct Sink cleared, which is
    // a different deliverData() branch than the never-had-a-Sink case — it
    // must feed the inactivity timer all the same.
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    // Same wide margins as the tx-only variant above (load tolerance).
    cfg.stream_quarantine_ms = 250;
    session.setConfig(cfg);

    uint8_t rx_buf[32];
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};
    const uint8_t id0 = session.attachStream(nullptr, &rx_sink);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    session.quarantineStream(id0, seq0);

    const uint8_t stray[] = {0xEE};
    for (int i = 0; i < 8; ++i) {
        pair.enc_b.writeFrame(frame::Kind::Data, id0, {stray, sizeof(stray)});
        pair.pump();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const uint8_t still_quarantined = session.attachStream(nullptr, nullptr);
    ASSERT_NE(still_quarantined, 0xFF);
    EXPECT_NE(still_quarantined, id0);
    session.detachStream(still_quarantined);

    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const uint8_t reused = session.attachStream(nullptr, nullptr);
    EXPECT_EQ(reused, id0);
    session.detachStream(reused);
}

// Calls remoteTransferWire() directly (the actual production integration
// branch every Bus_remote::transfer()/write()/read() runs through) instead
// of manually invoking quarantineStream() the way the RemoteSession-level
// quarantine tests do. Reverting remote_transfer.inl's quarantine-vs-detach
// decision to an unconditional detach would leave those tests green, but
// must fail this one (F8).
TEST(RemoteTransferWire, TimedOutRequestAutoQuarantinesStreamIdThroughProductionPath)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    // Captures the Request but never answers it — the peer's device-side
    // processing is still "in flight" from the host's point of view.
    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    uint8_t tx[]  = {0x01, 0x02, 0x03};
    uint8_t rx[4] = {};
    data::MemorySource src{tx, sizeof(tx)};
    data::MemorySink dst{rx, sizeof(rx)};

    auto r = remote::remoteTransferWire(&session, types::bus_kind_t::UART, 0, {}, {}, &src, sizeof(tx), &dst,
                                        sizeof(rx), cfg.response_timeout_ms);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(peer.captured);

    // remoteTransferWire() attached stream_id 0 (fresh session) and must
    // have quarantined it on this timeout, not freed it for immediate
    // reuse.
    const uint8_t next = session.attachStream(nullptr, nullptr);
    ASSERT_NE(next, 0xFF);
    EXPECT_NE(next, 0u);
    session.detachStream(next);
}

TEST(MuxRemoteSession, SeqWrapDuringQuarantineDoesNotMisreleaseIt)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    setPairPeerPoll(pair, session);

    DelayedResponsePeer peer;
    peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&DelayedResponsePeer::onFrame, &peer);

    auto cfg                = session.getConfig();
    cfg.response_timeout_ms = 5;
    session.setConfig(cfg);

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);
    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto timed_out         = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(timed_out.has_value());
    ASSERT_EQ(seq0, 0u);  // first seq issued by a fresh session
    session.quarantineStream(id0, seq0);

    // Switch to a peer that answers normal requests immediately (echoing
    // their seq) but stays silent for NORESP ones, matching the protocol.
    NorespAwareEchoPeer echo_peer;
    echo_peer.enc = &pair.enc_b;
    pair.dec_b.setFrameHandler(&NorespAwareEchoPeer::onFrame, &echo_peer);
    cfg.response_timeout_ms = 2000;
    session.setConfig(cfg);

    // Cycle the seq counter across the full 7 bit space (128 values) via
    // NORESP requests. Without nextSeq() skipping the quarantined seq
    // (F3), the 128th call here would land back on seq 0 — id0's own
    // quarantined seq — and a later unrelated Response for that reused seq
    // would incorrectly release id0's quarantine.
    const uint8_t norresp_script[] = {0x01};
    for (int i = 0; i < 127; ++i) {
        auto r = session.requestNoResponse({norresp_script, sizeof(norresp_script)});
        ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error()) << " i=" << i;
    }

    // This normal request must NOT have been assigned seq 0 (skipped
    // because it is still quarantined), and its Response must not release
    // id0's entry.
    const uint8_t script2[] = {0x02};
    uint8_t seq1            = 0xFF;
    auto ok                 = session.request({script2, sizeof(script2)}, &seq1);
    ASSERT_TRUE(ok.has_value()) << "err=" << error::toString(ok.error());
    EXPECT_NE(seq1, 0u);

    const uint8_t next = session.attachStream(nullptr, nullptr);
    ASSERT_NE(next, 0xFF);
    EXPECT_NE(next, id0);
    session.detachStream(next);
}

TEST(MuxRemoteSession, NonTimeoutPumpErrorAfterEnqueueAlsoQuarantines)
{
    SessionPair pair;
    AlwaysFailingSource failing_rx;
    // wire_tx is a real, working sink (pair.wire_ab.sink()) so writeFrame()
    // + flushTx() genuinely enqueue and drain the Request before the
    // always-failing RX source is ever touched — reproducing "the Request
    // left the host, but the failure that follows has nothing to do with a
    // response timeout" (F4/F5), the simplest peer-free form of this case.
    remote::RemoteSession session{pair.enc_a, pair.dec_a, failing_rx, pair.wire_ab.sink()};

    const uint8_t id0 = session.attachStream(nullptr, nullptr);
    ASSERT_EQ(id0, 0u);

    const uint8_t script[] = {0x00};
    uint8_t seq0           = 0xFF;
    auto req               = session.request({script, sizeof(script)}, &seq0);
    ASSERT_FALSE(req.has_value());
    EXPECT_NE(req.error(), error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(session.lastRequestEnqueued());

    session.quarantineStream(id0, seq0);
    const uint8_t next = session.attachStream(nullptr, nullptr);
    ASSERT_NE(next, 0xFF);
    EXPECT_NE(next, id0);
    session.detachStream(next);
}

struct CreditFlowPair {
    mem::Allocator alloc_a;
    mem::Allocator alloc_b;
    uint8_t wire_ab_buf[8192], wire_ba_buf[8192];
    data::RingFIFO wire_ab, wire_ba;
    data::MuxFrameEncoder enc_a{alloc_a}, enc_b{alloc_b};
    data::MuxFrameDecoder dec_a{alloc_a}, dec_b{alloc_b};

    CreditFlowPair()
    {
        wire_ab.setBuf(wire_ab_buf, sizeof(wire_ab_buf));
        wire_ba.setBuf(wire_ba_buf, sizeof(wire_ba_buf));
    }
};

TEST(MuxRemoteCreditFlow, SenderStopsAtCreditAndResumesAfterBlockConsumption)
{
    CreditFlowPair pair;
    constexpr size_t target_credit = 3;
    std::array<void*, mem::Allocator::tempBlockCount()> held{};
    for (size_t i = 0; i < mem::Allocator::tempBlockCount() - target_credit; ++i) {
        held[i] = pair.alloc_b.allocate(frame::kMaxFrameSize, mem::usage_t::Temp);
        ASSERT_NE(held[i], nullptr);
    }

    std::array<uint8_t, frame::kMaxDataPayload * 5> raw{};
    for (size_t i = 0; i < raw.size(); ++i) {
        raw[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource tx_src{raw.data(), raw.size()};
    auto* rx_src = pair.dec_b.createBlockStream(0);
    ASSERT_NE(rx_src, nullptr);
    auto* rx_blocks = static_cast<data::BlockSource*>(rx_src);

    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    auto initial_poll = session.poll();
    ASSERT_TRUE(initial_poll.has_value());
    EXPECT_TRUE(pair.enc_a.creditGated());
    EXPECT_EQ(pair.enc_a.remoteCredit(), target_credit);

    ASSERT_TRUE(pair.enc_a.attach(0, tx_src));
    for (int i = 0; i < 8; ++i) {
        auto r = session.poll();
        ASSERT_TRUE(r.has_value());
    }
    EXPECT_EQ(pair.enc_a.remoteCredit(), 0u);
    EXPECT_EQ(rx_blocks->blockCount(), target_credit);
    EXPECT_FALSE(tx_src.eof());

    auto first = rx_src->peek(frame::kMaxDataPayload);
    ASSERT_TRUE(first.has_value());
    ASSERT_EQ(first.value().size, frame::kMaxDataPayload);
    ASSERT_TRUE(rx_src->advance(first.value().size).has_value());
    EXPECT_EQ(rx_blocks->blockCount(), target_credit - 1);

    for (int i = 0; i < 8; ++i) {
        auto r = session.poll();
        ASSERT_TRUE(r.has_value());
    }
    EXPECT_EQ(rx_blocks->blockCount(), target_credit);
    EXPECT_EQ(pair.enc_a.remoteCredit(), 0u);
    EXPECT_FALSE(tx_src.eof());

    pair.enc_a.releaseAll();
    pair.dec_b.releaseAll();
    for (void* p : held) {
        if (p != nullptr) {
            pair.alloc_b.deallocate(p);
        }
    }
}

TEST(MuxRemoteCreditFlow, CreditResentAfterTempReleaseEvenWhenAbsoluteValueIsUnchanged)
{
    CreditFlowPair pair;
    remote::detail::CreditNotifier credit_b;

    struct CreditHandler {
        static void onFrame(void* ctx, const frame::View& view)
        {
            if (view.kind == frame::Kind::Credit) {
                static_cast<data::MuxFrameEncoder*>(ctx)->updateRemoteCredit(view.b3);
            }
        }
    };
    pair.dec_a.setFrameHandler(&CreditHandler::onFrame, &pair.enc_a);

    auto* rx_src = pair.dec_b.createBlockStream(0);
    ASSERT_NE(rx_src, nullptr);
    auto* rx_blocks = static_cast<data::BlockSource*>(rx_src);

    credit_b.pump(pair.enc_b, pair.dec_b);
    SessionPair::transfer(pair.enc_b.output(), pair.wire_ba.sink());
    EXPECT_EQ(pumpValue(pair.dec_a, pair.wire_ba.source()), 1u);
    ASSERT_TRUE(pair.enc_a.creditGated());
    const uint8_t advertised = pair.enc_a.remoteCredit();
    ASSERT_GT(advertised, 0u);

    constexpr size_t kCycles = 48;
    std::array<uint8_t, frame::kMaxDataPayload * kCycles> raw{};
    for (size_t i = 0; i < raw.size(); ++i) {
        raw[i] = static_cast<uint8_t>(i & 0xFF);
    }

    size_t consumed            = 0;
    size_t last_release_count  = pair.alloc_b.tempReleaseCount();
    size_t restored_credit_sum = 0;
    for (size_t cycle = 0; cycle < kCycles; ++cycle) {
        // Attach exactly one frame's worth per cycle: pump() batches until
        // its source runs dry, and this test needs the one-frame-per-cycle
        // cadence to sample the notifier when the absolute value recycles.
        data::MemorySource tx_src{raw.data() + cycle * frame::kMaxDataPayload, frame::kMaxDataPayload};
        ASSERT_TRUE(pair.enc_a.attach(0, tx_src));
        ASSERT_EQ(pair.enc_a.remoteCredit(), advertised) << "cycle=" << cycle;
        EXPECT_EQ(pumpValue(pair.enc_a), 1u) << "cycle=" << cycle;
        SessionPair::transfer(pair.enc_a.output(), pair.wire_ab.sink());
        EXPECT_EQ(pumpValue(pair.dec_b, pair.wire_ab.source()), 1u) << "cycle=" << cycle;
        ASSERT_EQ(rx_blocks->blockCount(), 1u) << "cycle=" << cycle;

        auto p = rx_src->peek(frame::kMaxDataPayload);
        ASSERT_TRUE(p.has_value()) << "cycle=" << cycle;
        ASSERT_GT(p.value().size, 0u) << "cycle=" << cycle;
        consumed += p.value().size;
        ASSERT_TRUE(rx_src->advance(p.value().size).has_value()) << "cycle=" << cycle;
        ASSERT_EQ(rx_blocks->blockCount(), 0u) << "cycle=" << cycle;
        ASSERT_GT(pair.alloc_b.tempReleaseCount(), last_release_count) << "cycle=" << cycle;
        last_release_count = pair.alloc_b.tempReleaseCount();

        credit_b.pump(pair.enc_b, pair.dec_b);
        SessionPair::transfer(pair.enc_b.output(), pair.wire_ba.sink());
        EXPECT_EQ(pumpValue(pair.dec_a, pair.wire_ba.source()), 1u) << "cycle=" << cycle;
        EXPECT_EQ(pair.enc_a.remoteCredit(), advertised) << "cycle=" << cycle;
        restored_credit_sum += pair.enc_a.remoteCredit();
    }

    EXPECT_EQ(consumed, raw.size());
    EXPECT_EQ(restored_credit_sum, static_cast<size_t>(advertised) * kCycles);

    pair.enc_a.releaseAll();
    pair.enc_b.releaseAll();
    pair.dec_b.releaseAll();
}

// Regression: the advertised credit must not exceed the block-stream
// queue capacity (BlockSource::kMaxBlocks) even when the temp pool has
// more free blocks (default pool = 32 blocks > kMaxBlocks = 16).
TEST(MuxRemoteCreditFlow, AdvertisedCreditCappedByBlockStreamSlots)
{
    CreditFlowPair pair;
    ASSERT_GT(mem::Allocator::tempBlockCount(), data::BlockSource::kMaxBlocks);

    std::array<uint8_t, frame::kMaxDataPayload*(data::BlockSource::kMaxBlocks + 8)> raw{};
    for (size_t i = 0; i < raw.size(); ++i) {
        raw[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource tx_src{raw.data(), raw.size()};
    auto* rx_src = pair.dec_b.createBlockStream(0);
    ASSERT_NE(rx_src, nullptr);
    auto* rx_blocks = static_cast<data::BlockSource*>(rx_src);

    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    auto initial_poll = session.poll();
    ASSERT_TRUE(initial_poll.has_value());
    EXPECT_TRUE(pair.enc_a.creditGated());
    EXPECT_EQ(pair.enc_a.remoteCredit(), data::BlockSource::kMaxBlocks);

    // Without consumption the receiver queues at most kMaxBlocks frames.
    ASSERT_TRUE(pair.enc_a.attach(0, tx_src));
    for (int i = 0; i < 24; ++i) {
        auto r = session.poll();
        ASSERT_TRUE(r.has_value());
    }
    EXPECT_EQ(rx_blocks->blockCount(), data::BlockSource::kMaxBlocks);
    EXPECT_EQ(pair.enc_a.remoteCredit(), 0u);
    EXPECT_FALSE(tx_src.eof());

    // Consuming everything recovers credit and lets the transfer finish.
    size_t consumed = 0;
    for (int i = 0; i < 200 && (consumed < raw.size() || !tx_src.eof()); ++i) {
        auto p = rx_src->peek(frame::kMaxDataPayload);
        if (p.has_value() && p.value().size > 0) {
            consumed += p.value().size;
            ASSERT_TRUE(rx_src->advance(p.value().size).has_value());
        }
        auto r = session.poll();
        ASSERT_TRUE(r.has_value());
    }
    EXPECT_EQ(consumed, raw.size());
    EXPECT_TRUE(tx_src.eof());

    pair.enc_a.releaseAll();
    pair.dec_b.releaseAll();
}

// ============================================================================
// E2E: Bus_remote stream transfer over virtual wire
//
// Server side uses a simple frame handler that processes Request frames
// by running the bytecode and pumping data streams manually. This avoids
// the complexity of RemoteServerAdapter and focuses on testing the
// attachStream + BusStreamTransfer data path.
// ============================================================================

struct E2EServer;

static E2EServer* g_active_e2e_server = nullptr;

struct E2EServer {
    data::MuxFrameEncoder* enc = nullptr;
    data::MuxFrameDecoder* dec = nullptr;
    i2c::IBus i2c_bus;
    i2c::MasterAccessor i2c_acc{i2c_bus, i2c::MasterAccessConfig{}};
    spi::IBus spi_bus;
    spi::MasterAccessor spi_acc{spi_bus, spi::MasterAccessConfig{}};
    bytecode::BytecodeRunner runner{mem::defaultAllocator()};

    struct PendingTransfer {
        bool active           = false;
        uint8_t seq           = 0;
        error::error_t status = error::error_t::OK;
        bytecode::BytecodeRunner::StreamTransferDesc desc{};
        data::Source* rx_src = nullptr;
        size_t tx_consumed   = 0;
    };

    PendingTransfer pending;

    E2EServer(data::MuxFrameEncoder& e, data::MuxFrameDecoder& d) : enc{&e}, dec{&d}
    {
        g_active_e2e_server = this;
        (void)runner.registerI2C(0, i2c_acc);
        (void)runner.registerSPI(0, spi_acc);
        runner.setStreamTransferHandler(onStreamTransfer, this);

        dec->setFrameHandler(
            [](void* ctx, const frame::View& view) {
                auto* s = static_cast<E2EServer*>(ctx);
                if (view.kind == frame::Kind::Request) {
                    s->handleRequest(view.b3, view.payload);
                }
            },
            this);
    }

    ~E2EServer()
    {
        if (g_active_e2e_server == this) {
            g_active_e2e_server = nullptr;
        }
    }

    void handleRequest(uint8_t seq, data::ConstDataSpan payload)
    {
        auto status = runner.run(payload);

        auto err = status.has_value() ? error::error_t::OK : status.error();
        if (pending.active) {
            pending.seq    = seq;
            pending.status = err;
            if (error::isError(err)) {
                completePending();
            }
            return;
        }
        writeResponse(seq, err);
    }

    void processPending()
    {
        if (!pending.active || error::isError(pending.status)) {
            return;
        }

        while (pending.tx_consumed < pending.desc.tx_len && pending.rx_src != nullptr) {
            const size_t remaining = static_cast<size_t>(pending.desc.tx_len) - pending.tx_consumed;
            auto p                 = pending.rx_src->peek(remaining);
            if (!p.has_value() || p.value().size == 0) {
                return;
            }

            const size_t n = p.value().size;
            writeEcho(p.value(), n);
            (void)pending.rx_src->advance(n);
            pending.tx_consumed += n;
        }

        if (pending.tx_consumed >= pending.desc.tx_len) {
            completePending();
        }
    }

    void writeResponse(uint8_t seq, error::error_t err)
    {
        // Mirror RemoteServerHandler: bit 7 of seq suppresses the Response.
        if ((seq & 0x80) != 0) {
            return;
        }
        uint8_t resp_buf[frame::kMaxPayload];
        data::MemorySink resp_sink(resp_buf, sizeof(resp_buf));
        (void)runner.writeResponse(resp_sink, err);
        enc->writeFrame(frame::Kind::Response, seq, {resp_buf, resp_sink.written()});
    }

    void writeEcho(data::ConstDataSpan bytes, size_t n)
    {
        if (pending.desc.rx_len == 0) {
            return;
        }

        uint8_t echo[252];
        size_t echo_remaining =
            pending.desc.rx_len > pending.tx_consumed ? pending.desc.rx_len - pending.tx_consumed : 0;
        size_t echo_total  = n < echo_remaining ? n : echo_remaining;
        size_t echo_offset = 0;
        while (echo_offset < echo_total) {
            size_t echo_n = (echo_total - echo_offset) < sizeof(echo) ? (echo_total - echo_offset) : sizeof(echo);
            ::memcpy(echo, bytes.data + echo_offset, echo_n);
            for (size_t i = 0; i < echo_n; ++i) {
                echo[i] ^= 0xFF;
            }
            enc->writeFrame(frame::Kind::Data, pending.desc.stream_id, {echo, echo_n});
            echo_offset += echo_n;
        }
    }

    void completePending()
    {
        if (!pending.active) {
            return;
        }
        dec->destroyStream(pending.desc.stream_id);
        writeResponse(pending.seq, pending.status);
        pending = PendingTransfer{};
    }

    static result_t<void> onStreamTransfer(void* ctx, const bytecode::BytecodeRunner::StreamTransferDesc& desc)
    {
        auto* s = static_cast<E2EServer*>(ctx);

        if (desc.tx_len > 0) {
            if (s->pending.active) {
                return m5::stl::make_unexpected(error::error_t::BUSY);
            }
            auto* rx_src = s->dec->createStream(desc.stream_id, 4096);
            if (rx_src == nullptr) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            s->pending.active = true;
            s->pending.desc   = desc;
            s->pending.rx_src = rx_src;
            return {};
        }

        if (desc.rx_len > 0 && desc.tx_len == 0) {
            uint8_t fill[252];
            size_t remaining = desc.rx_len;
            uint8_t val      = 0x42;
            while (remaining > 0) {
                size_t chunk = remaining < sizeof(fill) ? remaining : sizeof(fill);
                ::memset(fill, val++, chunk);
                s->enc->writeFrame(frame::Kind::Data, desc.stream_id, {fill, chunk});
                remaining -= chunk;
            }
        }

        return {};
    }
};

// E2E with two threads: client and server each pump their own encoder/decoder.

struct TwoThreadWire {
    mem::Allocator& alloc = mem::defaultAllocator();
    uint8_t wire_ab_buf[8192], wire_ba_buf[8192];
    data::RingFIFO wire_ab, wire_ba;
    data::MuxFrameEncoder enc_a{alloc}, enc_b{alloc};
    data::MuxFrameDecoder dec_a{alloc}, dec_b{alloc};
    std::atomic<bool> server_running{true};

    TwoThreadWire()
    {
        wire_ab.setBuf(wire_ab_buf, sizeof(wire_ab_buf));
        wire_ba.setBuf(wire_ba_buf, sizeof(wire_ba_buf));
    }

    void clientPump()
    {
        pumpValue(enc_a);
        SessionPair::transfer(enc_a.output(), wire_ab.sink());
        pumpValue(dec_a, wire_ba.source());
    }

    void serverPump()
    {
        pumpValue(enc_b);
        SessionPair::transfer(enc_b.output(), wire_ba.sink());
        pumpValue(dec_b, wire_ab.source());
        if (g_active_e2e_server != nullptr) {
            g_active_e2e_server->processPending();
        }
    }
};

static void runServerLoop(TwoThreadWire* w)
{
    while (w->server_running.load(std::memory_order_acquire)) {
        w->serverPump();
        std::this_thread::yield();
    }
}

TEST(E2EStreamTransfer, I2CWriteOnly)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    const uint8_t tx_data[] = {0x10, 0x20, 0x30, 0x40, 0x50};
    data::MemorySource tx_src{tx_data, sizeof(tx_data)};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, &tx_src, sizeof(tx_data), nullptr, 0);
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    EXPECT_TRUE(tx_src.eof());
}

TEST(E2EStreamTransfer, LargeTransferExceedsOldLimit)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    std::array<uint8_t, 1000> tx_data;
    for (size_t i = 0; i < tx_data.size(); ++i) {
        tx_data[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource tx_src{tx_data.data(), tx_data.size()};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, &tx_src, tx_data.size(), nullptr, 0);
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    EXPECT_TRUE(tx_src.eof());
}

// Regression: repeated small stream transfers on one session must not exhaust
// a peer resource — a leak per transfer cycle surfaces as a late TIMEOUT after
// ~100 cycles (observed on hardware at ~230 KB cumulative with 2 KiB writes).
TEST(E2EStreamTransfer, RepeatedTransfersSoakDoesNotWedge)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    std::array<uint8_t, 2048> tx_data;
    for (size_t i = 0; i < tx_data.size(); ++i) {
        tx_data[i] = static_cast<uint8_t>(i & 0xFF);
    }
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};
    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    for (int cycle = 0; cycle < 300; ++cycle) {
        data::MemorySource tx_src{tx_data.data(), tx_data.size()};
        auto r = bus.transfer(nullptr, cfg, desc, &tx_src, tx_data.size(), nullptr, 0);
        if (!r.has_value()) {
            w.server_running.store(false, std::memory_order_release);
            srv.join();
            FAIL() << "transfer failed at cycle " << cycle << ": " << error::toString(r.error())
                   << " src_eof=" << tx_src.eof() << " credit_gated=" << w.enc_a.creditGated()
                   << " remote_credit=" << static_cast<int>(w.enc_a.remoteCredit());
        }
        EXPECT_TRUE(tx_src.eof()) << "cycle " << cycle;
    }
    w.server_running.store(false, std::memory_order_release);
    srv.join();
}

TEST(E2EStreamTransfer, I2CWriteReadEchoMatches)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    const uint8_t tx_data[] = {0x10, 0x20, 0x30, 0x40, 0x50};
    uint8_t rx_buf[sizeof(tx_data)]{};
    data::MemorySource tx_src{tx_data, sizeof(tx_data)};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, &tx_src, sizeof(tx_data), &rx_sink, sizeof(rx_buf));
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    EXPECT_TRUE(tx_src.eof());
    ASSERT_EQ(rx_sink.written(), sizeof(rx_buf));
    for (size_t i = 0; i < sizeof(tx_data); ++i) {
        EXPECT_EQ(rx_buf[i], static_cast<uint8_t>(tx_data[i] ^ 0xFF)) << "i=" << i;
    }
}

TEST(E2EStreamTransfer, I2CReadOnlyFillPattern)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    uint8_t rx_buf[600]{};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, nullptr, 0, &rx_sink, sizeof(rx_buf));
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    ASSERT_EQ(rx_sink.written(), sizeof(rx_buf));
    for (size_t i = 0; i < sizeof(rx_buf); ++i) {
        const uint8_t expected = static_cast<uint8_t>(0x42 + (i / 252));
        EXPECT_EQ(rx_buf[i], expected) << "i=" << i;
    }
}

TEST(E2EStreamTransfer, LargeBidirectionalEchoMatches)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    std::array<uint8_t, 1000> tx_data;
    std::array<uint8_t, 1000> rx_buf{};
    for (size_t i = 0; i < tx_data.size(); ++i) {
        tx_data[i] = static_cast<uint8_t>(i & 0xFF);
    }
    data::MemorySource tx_src{tx_data.data(), tx_data.size()};
    data::MemorySink rx_sink{rx_buf.data(), rx_buf.size()};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, &tx_src, tx_data.size(), &rx_sink, rx_buf.size());
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    EXPECT_TRUE(tx_src.eof());
    ASSERT_EQ(rx_sink.written(), rx_buf.size());
    for (size_t i = 0; i < rx_buf.size(); ++i) {
        EXPECT_EQ(rx_buf[i], static_cast<uint8_t>((i & 0xFF) ^ 0xFF)) << "i=" << i;
    }
}

TEST(E2EStreamTransfer, SPIFullDuplexEchoMatches)
{
    TwoThreadWire w;
    E2EServer server{w.enc_b, w.dec_b};
    std::thread srv{runServerLoop, &w};

    remote::RemoteSession session{w.enc_a, w.dec_a, w.wire_ba.source(), w.wire_ab.sink()};
    session.setPeerPoll([](void* ctx) { static_cast<TwoThreadWire*>(ctx)->clientPump(); }, &w);

    const uint8_t tx_data[] = {0xA5, 0x5A, 0x00, 0xFF, 0x11, 0x22, 0x33, 0x44,
                               0x55, 0x66, 0x77, 0x88, 0x99, 0xAB, 0xCD, 0xEF};
    uint8_t rx_buf[sizeof(tx_data)]{};
    data::MemorySource tx_src{tx_data, sizeof(tx_data)};
    data::MemorySink rx_sink{rx_buf, sizeof(rx_buf)};
    spi::Bus_remote bus{session, 0};

    spi::MasterAccessConfig cfg;
    spi::TransferDesc desc;

    auto r = bus.transfer(nullptr, cfg, desc, &tx_src, sizeof(tx_data), &rx_sink, sizeof(rx_buf));
    w.server_running.store(false, std::memory_order_release);
    srv.join();

    ASSERT_TRUE(r.has_value()) << "transfer failed: " << error::toString(r.error());
    EXPECT_TRUE(tx_src.eof());
    ASSERT_EQ(rx_sink.written(), sizeof(rx_buf));
    for (size_t i = 0; i < sizeof(tx_data); ++i) {
        EXPECT_EQ(rx_buf[i], static_cast<uint8_t>(tx_data[i] ^ 0xFF)) << "i=" << i;
    }
}

// A peer that answers every Request with a Response carrying no terminal
// Report (or an empty payload) — exercises RemoteSession::checkResponse's
// protocol contract that a report-less response is a protocol violation.
struct NoReportResponsePeer {
    data::MuxFrameEncoder* enc = nullptr;
    bool empty_response        = false;

    void handle(const frame::View& view)
    {
        if (view.kind != frame::Kind::Request) {
            return;
        }
        if (empty_response) {
            enc->writeFrame(frame::Kind::Response, view.b3, {});
            return;
        }
        uint8_t resp_buf[frame::kMaxPayload];
        data::MemorySink resp_sink{resp_buf, sizeof(resp_buf)};
        bytecode::BytecodeEncoder resp{resp_sink};
        // Terminator only: a well-formed script that reports nothing.
        auto r = resp.end();
        if (r.has_value()) {
            enc->writeFrame(frame::Kind::Response, view.b3, {resp_buf, resp_sink.written()});
        }
    }

    static void onFrame(void* ctx, const frame::View& view)
    {
        static_cast<NoReportResponsePeer*>(ctx)->handle(view);
    }
};

static void runNoReportCase(bool empty_response)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    NoReportResponsePeer peer;
    peer.enc            = &pair.enc_b;
    peer.empty_response = empty_response;
    pair.dec_b.setFrameHandler(&NoReportResponsePeer::onFrame, &peer);
    session.setPeerPoll([](void* ctx) { static_cast<SessionPair*>(ctx)->pump(); }, &pair);

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto e = enc.end();
    ASSERT_TRUE(e.has_value()) << "err=" << error::toString(e.error());
    auto req = session.request({script_buf, script.written()});
    ASSERT_TRUE(req.has_value()) << "err=" << error::toString(req.error());

    auto chk = session.checkResponse();
    ASSERT_FALSE(chk.has_value()) << "expected PROTOCOL_ERROR for a report-less response";
    EXPECT_EQ(chk.error(), error::error_t::PROTOCOL_ERROR);
}

TEST(RemoteSessionCheckResponse, MissingReportIsProtocolError)
{
    runNoReportCase(/*empty_response=*/false);
}

TEST(RemoteSessionCheckResponse, EmptyResponseIsProtocolError)
{
    runNoReportCase(/*empty_response=*/true);
}

// The remote I2C proxy encodes the transfer prefix into a fixed
// meta_buf[1 + PREFIX_CAPACITY]; a prefix_len beyond capacity must be rejected
// up front instead of overflowing the stack buffer in encodeI2cMeta.
TEST(RemoteI2cProxy, TransferRejectsOverlongPrefix)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    i2c::IBusConfig bus_cfg;
    i2c::Bus_remote host_bus{session, 0, bus_cfg};

    i2c::MasterAccessConfig cfg;
    i2c::TransferDesc desc;
    desc.prefix_len = static_cast<uint8_t>(i2c::TransferDesc::PREFIX_CAPACITY + 1);

    auto r = host_bus.transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(r.has_value()) << "expected INVALID_ARGUMENT for an over-length I2C prefix";
    EXPECT_EQ(r.error(), error::error_t::INVALID_ARGUMENT);
}

// HelloResp must serialize the server's statically registered bus capabilities
// as [proto_ver][flags][n]([bus_kind][bus_id])*n, and the host decoder must see
// them (spec/design/remote.md §hello).
TEST(RemoteServerHandler, HelloRespReportsRegisteredCapabilities)
{
    SessionPair pair;
    MuxFrameCapture capture;
    pair.dec_a.setFrameHandler(&MuxFrameCapture::onFrame, &capture);
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    PatternI2CBus i2c_bus{0x42, 16};
    i2c::MasterAccessConfig i2c_cfg;
    i2c::MasterAccessor i2c_acc{i2c_bus, i2c_cfg};
    ASSERT_TRUE(server.registerI2C(0, i2c_acc).has_value());

    uart::AccessConfig uart_cfg;
    EchoUARTBus uart_bus;
    uart::Accessor uart_acc{uart_bus, uart_cfg};
    ASSERT_TRUE(server.registerUART(2, uart_acc).has_value());

    handler.server = &server;

    auto hello = remote::RemoteServerHandler::handler(&handler, frame::Kind::HelloReq, 7, {}, pair.enc_b, pair.dec_b);
    ASSERT_TRUE(hello.has_value()) << "err=" << error::toString(hello.error());
    pumpEncoderToDecoder(pair.enc_b, pair.wire_ba, pair.dec_a);
    ASSERT_EQ(capture.frames.size(), 1u);
    ASSERT_EQ(capture.frames[0].kind, frame::Kind::HelloResp);

    const auto& body = capture.frames[0].payload;
    auto caps        = remote::detail::decodeHelloCaps(data::ConstDataSpan{body.data(), body.size()});
    ASSERT_TRUE(caps.has_value()) << "err=" << error::toString(caps.error());
    EXPECT_EQ(caps->proto_ver, remote::kProtocolVersion);
    ASSERT_EQ(caps->bus_count, 2u);

    bool saw_i2c = false, saw_uart = false;
    for (size_t i = 0; i < caps->bus_count; ++i) {
        if (caps->buses[i].kind == types::bus_kind_t::I2C && caps->buses[i].bus_id == 0) {
            saw_i2c = true;
        }
        if (caps->buses[i].kind == types::bus_kind_t::UART && caps->buses[i].bus_id == 2) {
            saw_uart = true;
        }
    }
    EXPECT_TRUE(saw_i2c) << "I2C(0) capability missing from HelloResp";
    EXPECT_TRUE(saw_uart) << "UART(2) capability missing from HelloResp";
    EXPECT_FALSE(caps->has_gpio);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
