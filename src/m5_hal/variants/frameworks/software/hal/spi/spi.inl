// SPDX-License-Identifier: MIT
// Software (bit-bang) SPI implementation. Included by M5HAL_v2.cpp via the
// software variant's hal.inl hub.

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_INL

#include "spi.hpp"

#include <M5Utility.hpp>

#include <cstddef>
#include <new>

namespace m5::hal::v2::spi {

namespace {
namespace impl_software {

constexpr uint32_t kNsecPerSec = 1000000000u;

struct BusDcLevelCache {
    const void* bus = nullptr;
    bool high       = false;
};

BusDcLevelCache g_bus_dc_levels[8];

bool busDcLevelHigh(const void* bus)
{
    for (const auto& entry : g_bus_dc_levels) {
        if (entry.bus == bus) {
            return entry.high;
        }
    }
    return false;
}

void noteBusDcLevel(const void* bus, bool high)
{
    BusDcLevelCache* empty = nullptr;
    for (auto& entry : g_bus_dc_levels) {
        if (entry.bus == bus) {
            entry.high = high;
            return;
        }
        if (empty == nullptr && entry.bus == nullptr) {
            empty = &entry;
        }
    }
    if (empty != nullptr) {
        empty->bus  = bus;
        empty->high = high;
    }
}

result_t<service::fast_tick_t> halfPeriodTick(const spi::MasterAccessConfig& cfg)
{
    if (cfg.freq == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const uint64_t denom = static_cast<uint64_t>(cfg.freq) * 2ull;
    uint64_t half_nsec   = (static_cast<uint64_t>(kNsecPerSec) + (denom / 2ull)) / denom;
    if (half_nsec == 0) {
        half_nsec = 1;
    }
    return service::nsecToFastTickCeil(static_cast<service::tick_nsec_t>(half_nsec), service::fastTickFrequencyHz());
}

void waitUntil(service::fast_tick_t due_tick)
{
    while (!service::hasReached(service::fastTick(), due_tick)) {
    }
}

void stepClock(gpio::Pin& clk, bool level, service::fast_tick_t half_tick, service::fast_tick_t& due_tick)
{
    due_tick += half_tick;
    waitUntil(due_tick);
    clk.write(level);
}

struct TransferPlan {
    bool cpol         = false;
    bool cpha         = false;
    bool first_level  = false;
    bool second_level = true;
    bool has_mosi     = false;
    bool has_miso     = false;
    uint8_t masks[8]{};
};

TransferPlan makePlan(const spi::MasterAccessConfig& cfg, bool has_mosi, bool has_miso)
{
    TransferPlan plan;
    plan.cpol     = (cfg.spi_mode & 0x02) != 0;
    plan.cpha     = (cfg.spi_mode & 0x01) != 0;
    plan.has_mosi = has_mosi;
    plan.has_miso = has_miso;
    // A bit is driven as MOSI update -> first clock level -> second clock
    // level (MOSI ordering rationale = writeFirstEdge). This keeps the
    // MOSI/MISO work in one half-cycle instead of spreading it across both.
    plan.first_level  = plan.cpol ^ plan.cpha;
    plan.second_level = !plan.first_level;

    uint8_t mask = (cfg.spi_order == 0) ? uint8_t{0x80} : uint8_t{0x01};
    for (uint8_t i = 0; i < 8; ++i) {
        plan.masks[i] = mask;
        mask          = (cfg.spi_order == 0) ? static_cast<uint8_t>(mask >> 1) : static_cast<uint8_t>(mask << 1);
    }
    return plan;
}

bool isSingleLaneHalfDuplexMode(spi::spi_data_mode_t mode)
{
    using spi::spi_data_mode_t;
    return mode == spi_data_mode_t::HalfDuplex || mode == spi_data_mode_t::HalfDuplexWithDcPin ||
           mode == spi_data_mode_t::HalfDuplexWithDcBit;
}

result_t<void> resolveOptionalPin(types::gpio_number_t gpio_num, gpio::Pin& out)
{
    out = {};
    if (gpio_num < 0) {
        return {};
    }
    auto pin = M5_Hal.Gpio.tryGetPin(gpio_num);
    if (!pin.has_value()) {
        return m5::stl::make_unexpected(pin.error());
    }
    out = pin.value();
    return {};
}

class ByteTransferState {
public:
    void begin(uint8_t tx_byte)
    {
        _tx_byte             = tx_byte;
        _rx_byte             = 0;
        _bit_index           = 0;
        _active              = true;
        _done                = false;
        _pending_second_edge = false;
    }

    bool active() const
    {
        return _active;
    }

    bool done() const
    {
        return _done;
    }

    uint8_t rxByte() const
    {
        return _rx_byte;
    }

    void clearDone()
    {
        _done = false;
    }

    uint_fast8_t pollEdges(uint_fast8_t edge_budget, gpio::Pin& clk, gpio::Pin& mosi, gpio::Pin& miso,
                           const TransferPlan& plan)
    {
        if (!plan.has_miso) {
            return pollWriteOnlyEdges(edge_budget, clk, mosi, plan);
        }
        if (edge_budget == 0 || !_active) {
            return edge_budget;
        }

        if (_pending_second_edge) {
            writeSecondEdge(clk, miso, plan);
            --edge_budget;
            if (!_active || edge_budget == 0) {
                return edge_budget;
            }
        }

        while (_bit_index < 8) {
            writeFirstEdge(clk, mosi, plan);
            if (edge_budget < 2) {
                return 0;
            }
            writeSecondEdge(clk, miso, plan);
            edge_budget = static_cast<uint_fast8_t>(edge_budget - 2);
            if (!_active || edge_budget == 0) {
                return edge_budget;
            }
        }
        return edge_budget;
    }

private:
    void finishSecondEdge()
    {
        ++_bit_index;
        _pending_second_edge = false;
        if (_bit_index >= 8) {
            _active = false;
            _done   = true;
        }
    }

    void writeFirstEdge(gpio::Pin& clk, gpio::Pin& mosi, const TransferPlan& plan)
    {
        // MOSI must already be valid when the launch edge appears on the
        // wire: some slaves sample MOSI near the launch edge instead of the
        // nominal sample edge (classic ESP32 slave in CPHA=1 — a hardware
        // master's output delay is a few ns, but a GPIO write gap after the
        // edge makes such a slave read the PREVIOUS bit, shifting the whole
        // stream 1 bit late). Writing MOSI first also removes the window
        // where preemption between the two writes stretches that gap.
        // Canonical doc = spec/design/spi.md wire-timing invariants.
        if (plan.has_mosi) {
            mosi.write((_tx_byte & plan.masks[_bit_index]) != 0);
        }
        clk.write(plan.first_level);
        _pending_second_edge = true;
    }

    void writeSecondEdge(gpio::Pin& clk, gpio::Pin& miso, const TransferPlan& plan)
    {
        clk.write(plan.second_level);
        if (miso.read()) {
            _rx_byte |= plan.masks[_bit_index];
        }
        finishSecondEdge();
    }

    uint_fast8_t pollWriteOnlyEdges(uint_fast8_t edge_budget, gpio::Pin& clk, gpio::Pin& mosi, const TransferPlan& plan)
    {
        if (edge_budget == 0 || !_active) {
            return edge_budget;
        }

        if (_pending_second_edge) {
            writeSecondEdgeNoRead(clk, plan);
            --edge_budget;
            if (!_active || edge_budget == 0) {
                return edge_budget;
            }
        }

        while (_bit_index < 8) {
            writeFirstEdge(clk, mosi, plan);
            if (edge_budget < 2) {
                return 0;
            }
            writeSecondEdgeNoRead(clk, plan);
            edge_budget = static_cast<uint_fast8_t>(edge_budget - 2);
            if (!_active || edge_budget == 0) {
                return edge_budget;
            }
        }
        return edge_budget;
    }

    void writeSecondEdgeNoRead(gpio::Pin& clk, const TransferPlan& plan)
    {
        clk.write(plan.second_level);
        finishSecondEdge();
    }

    uint8_t _tx_byte          = 0;
    uint8_t _rx_byte          = 0;
    uint8_t _bit_index        = 0;
    bool _active              = false;
    bool _done                = false;
    bool _pending_second_edge = false;
};

class TransferService {
    enum class Phase : uint8_t {
        command,
        address,
        dummy,
        data,
        done,
    };

public:
    TransferService(gpio::Pin& clk, gpio::Pin& mosi, gpio::Pin& miso, gpio::Pin& dc)
        : _clk{clk}, _mosi{mosi}, _miso{miso}, _dc{dc}
    {
    }

    void begin(const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc, data::Source* src, size_t tx_len,
               data::Sink* dst, size_t rx_len, service::fast_tick_t half_tick, bool has_mosi, bool has_miso,
               bool half_duplex, bool shared_data_pin)
    {
        _plan            = makePlan(cfg, has_mosi, half_duplex ? false : has_miso);
        _tx              = src;
        _rx              = dst;
        _tx_remaining    = (src != nullptr) ? tx_len : 0;
        _rx_remaining    = (dst != nullptr) ? rx_len : 0;
        _half_tick       = half_tick;
        _half_duplex     = half_duplex;
        _shared_data_pin = shared_data_pin;
        _input_available = has_miso;
        _rx_phase        = false;
        if (_shared_data_pin) {
            _mosi.setMode(types::gpio_mode_t::Output);
        }
        // Intra-call spin anchor: begin() itself paces real edges through
        // setDC (enterPhase below), so it reads the live counter once. This
        // absolute tick never leaves the call — polls re-derive their own
        // real-axis anchor from ctx (see service()).
        _due_tick               = service::fastTick();
        _command                = desc.command;
        _address                = desc.address;
        _command_bytes          = desc.command_bytes;
        _address_bytes          = desc.address_bytes;
        _command_dc             = desc.command_dc_level;
        _address_dc             = desc.address_dc_level;
        _data_dc                = desc.data_dc_level;
        const bool has_phase_dc = _command_dc >= 0 || _address_dc >= 0 || _data_dc >= 0;
        if (!has_phase_dc && desc.dc_level_valid) {
            _data_dc = desc.dc_level ? 1 : 0;
        }
        _dummy_remaining = desc.dummy_cycles;
        _totals.clear();
        _tx_span     = {};
        _rx_span     = {};
        _chunk_len   = 0;
        _chunk_index = 0;
        _clk.write(_plan.cpol);
        enterPhase(firstPhase());
        // Virtual-timeline origin: begin() exits right at its last real edge
        // (setDC spins to _due_tick before writing), so "last edge position"
        // maps to virtual now. Sub-half-period mapping error only delays the
        // first polled edge — the safe direction.
        _svc_now = 0;
        _due_v   = 0;
    }

    bool active() const
    {
        return _phase != Phase::done;
    }

    bus::TransferTotals totals() const
    {
        return _totals;
    }

    error::error_t error() const
    {
        return _error;
    }

    service::ServicePoll service(const service::ServiceContext& ctx)
    {
        if (!active()) {
            return service::ServiceResult::Done;
        }

        // Two-axis bookkeeping: scheduling decisions live on the private
        // virtual timeline (_svc_now/_due_v, advanced only by caller-vouched
        // elapsed), while the edge pacing below spins on the real counter.
        // Map _due_v into THIS call's real axis through the fixed point
        // (local_tick <-> _svc_now); after the mapping, every comparison in
        // the body is same-axis/same-call, so the pre-existing real-tick
        // machinery runs unchanged. The mapping runs for EVERY poll —
        // including the dummy phase, which bypasses the gate below but still
        // paces edges from _due_tick (a carried-over real tick here was the
        // exact cross-core comparison this contract removes).
        _svc_now += ctx.elapsed;
        _due_tick = static_cast<service::fast_tick_t>(ctx.local_tick - (_svc_now - _due_v));

        uint_fast8_t edge_budget = 0;
        if (_phase != Phase::dummy) {
            const auto next_due = nextEdgeDue();
            if (!service::hasReached(ctx.local_tick, next_due)) {
                // Not due yet: _due_tick was not advanced, no write-back
                // needed. next_due is ahead of local_tick here, so the
                // difference is the positive relative hint.
                return {service::ServiceResult::Idle, static_cast<service::fast_tick_t>(next_due - ctx.local_tick)};
            }
            edge_budget = availableEdgeBudget(ctx.local_tick, next_due);
        }

        auto polled = poll(edge_budget);
        // Write the advanced edge position back onto the virtual axis via
        // the same fixed point, whatever poll() did to _due_tick (edge
        // steps, DC waits, late re-anchor).
        _due_v = static_cast<service::fast_tick_t>(_svc_now + (_due_tick - ctx.local_tick));
        if (!polled.has_value()) {
            _error = polled.error();
            enterDone();  // park the clock at idle even on the error path
            _due_v = static_cast<service::fast_tick_t>(_svc_now + (_due_tick - ctx.local_tick));
            return service::ServiceResult::Error;
        }
        return polled.value();
    }

private:
    result_t<service::ServiceResult> poll(uint_fast8_t edge_budget)
    {
        switch (_phase) {
            case Phase::command:
            case Phase::address: {
                const Phase current_phase = _phase;
                auto result               = pollMetaByte(edge_budget);
                if (result == service::ServiceResult::Done) {
                    if (_meta_remaining == 0) {
                        enterPhase(nextPhase(current_phase));
                    }
                    return service::ServiceResult::Progress;
                }
                return result;
            }

            case Phase::dummy:
                pollDummyClock();
                if (_dummy_remaining == 0) {
                    enterPhase(Phase::data);
                }
                return service::ServiceResult::Progress;

            case Phase::data:
                if (_chunk_index >= _chunk_len) {
                    auto chunk = acquireChunk();
                    if (!chunk.has_value()) {
                        return m5::stl::make_unexpected(chunk.error());
                    }
                    if (_phase == Phase::done) {
                        return service::ServiceResult::Done;
                    }
                }
                {
                    auto result = pollDataByte(edge_budget);
                    if (result == service::ServiceResult::Done) {
                        if (_chunk_index >= _chunk_len) {
                            auto finished = finishChunk();
                            if (!finished.has_value()) {
                                return m5::stl::make_unexpected(finished.error());
                            }
                        }
                        return service::ServiceResult::Progress;
                    }
                    return result;
                }

            case Phase::done:
            default:
                return service::ServiceResult::Done;
        }
    }

    Phase firstPhase() const
    {
        if (_command_bytes != 0) {
            return Phase::command;
        }
        if (_address_bytes != 0) {
            return Phase::address;
        }
        if (_dummy_remaining != 0) {
            return Phase::dummy;
        }
        return Phase::data;
    }

    Phase nextPhase(Phase phase) const
    {
        switch (phase) {
            case Phase::command:
                if (_address_bytes != 0) {
                    return Phase::address;
                }
                if (_dummy_remaining != 0) {
                    return Phase::dummy;
                }
                return Phase::data;

            case Phase::address:
                return (_dummy_remaining != 0) ? Phase::dummy : Phase::data;

            case Phase::dummy:
                return Phase::data;

            case Phase::data:
            case Phase::done:
            default:
                return Phase::done;
        }
    }

    void enterPhase(Phase phase)
    {
        _phase = phase;
        switch (phase) {
            case Phase::command:
                setDC(_command_dc);
                _meta_value     = _command;
                _meta_remaining = _command_bytes;
                break;

            case Phase::address:
                setDC(_address_dc);
                _meta_value     = _address;
                _meta_remaining = _address_bytes;
                break;

            case Phase::data:
                setDC(_data_dc);
                if ((_command_dc >= 0 || _address_dc >= 0) && _data_dc < 0) {
                    setDC(1);
                }
                if (_half_duplex && !txPending()) {
                    enterRxPhase();
                }
                break;

            case Phase::dummy:
            case Phase::done:
            default:
                break;
        }
    }

    // DC changes ride the half-period grid: phase entry happens in the
    // same poll as the previous phase's final sample edge, so an
    // immediate write would change DC a few µs after that edge (razor-
    // thin hold for the device, and unresolvable for the capture-based
    // wire test). Spending one half period first puts the DC transition
    // cleanly between the phases.
    void setDC(int8_t level)
    {
        if (_dc.isValid() && level >= 0) {
            _due_tick += _half_tick;
            waitUntil(_due_tick);
            _dc.write(level != 0);
        }
    }

    // Terminal transition: give the final half period its full width,
    // park the clock at the idle level (CPOL) - previously it stayed at
    // second_level until the NEXT transfer's begin(), so CS deassert
    // happened on an active clock - and let idle settle for one more
    // half period so the deassert edge has a clean setup time. The park
    // write is a no-op for modes that already end at idle (CPHA=1).
    void enterDone()
    {
        stepClock(_clk, _plan.cpol, _half_tick, _due_tick);
        _due_tick += _half_tick;
        waitUntil(_due_tick);
        if (_shared_data_pin) {
            _mosi.setMode(types::gpio_mode_t::Output);
        }
        _phase = Phase::done;
    }

    bool txPending() const
    {
        return _tx != nullptr && _tx_remaining > 0 && !_tx->eof();
    }

    void enterRxPhase()
    {
        _rx_phase      = true;
        _plan.has_mosi = false;
        _plan.has_miso = _input_available;
        if (_shared_data_pin) {
            _mosi.setMode(types::gpio_mode_t::Input);
        }
    }

    service::fast_tick_t nextEdgeDue() const
    {
        return static_cast<service::fast_tick_t>(_due_tick + _half_tick);
    }

    uint_fast8_t availableEdgeBudget(service::fast_tick_t now_tick, service::fast_tick_t next_due)
    {
        const auto second_due = static_cast<service::fast_tick_t>(next_due + _half_tick);
        if (service::hasReached(now_tick, second_due)) {
            _due_tick = now_tick;
        } else {
            _due_tick = next_due;
        }
        return 1;
    }

    service::ServiceResult pollMetaByte(uint_fast8_t edge_budget)
    {
        if (!_byte.active() && !_byte.done()) {
            const uint8_t shift = static_cast<uint8_t>((_meta_remaining - 1u) * 8u);
            const uint8_t byte  = static_cast<uint8_t>(_meta_value >> shift);
            _byte.begin(byte);
        }
        (void)_byte.pollEdges(edge_budget, _clk, _mosi, _miso, _plan);
        if (_byte.done()) {
            --_meta_remaining;
            _byte.clearDone();
            return service::ServiceResult::Done;
        }
        return service::ServiceResult::Progress;
    }

    void pollDummyClock()
    {
        // Clock dummy cycles exactly like data bits (first -> second
        // level). The line rests at second_level after a byte, so the
        // old "!cpol then cpol" order made the first dummy cycle's
        // leading write a no-transition for CPHA=0 - the wire carried
        // one less dummy edge than configured (off-by-one against
        // devices that count dummy clocks on the sample edge). With
        // first/second ordering every cycle toggles onto its sample
        // edge regardless of the resting level.
        stepClock(_clk, _plan.first_level, _half_tick, _due_tick);
        stepClock(_clk, _plan.second_level, _half_tick, _due_tick);
        --_dummy_remaining;
    }

    result_t<void> acquireChunk()
    {
        _tx_span     = {};
        _rx_span     = {};
        _chunk_len   = 0;
        _chunk_index = 0;

        if (!_half_duplex || !_rx_phase) {
            if (_tx != nullptr && _tx_remaining > 0 && !_tx->eof()) {
                auto peeked = _tx->peek(_tx_remaining);
                if (!peeked.has_value()) {
                    return m5::stl::make_unexpected(peeked.error());
                }
                _tx_span = peeked.value();
                _tx_span = _tx_span.first(_tx_remaining);
            }
        }
        if (_half_duplex && !_rx_phase && _tx_span.size == 0) {
            enterRxPhase();
        }
        if (!_half_duplex || _rx_phase) {
            if (_rx != nullptr && _rx_remaining > 0 && !_rx->closed()) {
                auto reserved = _rx->reserve(_rx_remaining);
                if (!reserved.has_value()) {
                    return m5::stl::make_unexpected(reserved.error());
                }
                _rx_span = reserved.value();
                _rx_span = _rx_span.first(_rx_remaining);
            }
        }

        _chunk_len = (_tx_span.size > _rx_span.size) ? _tx_span.size : _rx_span.size;
        if (_chunk_len == 0) {
            enterDone();  // natural end of the transfer
        }
        return {};
    }

    service::ServiceResult pollDataByte(uint_fast8_t edge_budget)
    {
        if (!_byte.active() && !_byte.done()) {
            const uint8_t tx_byte = (_chunk_index < _tx_span.size) ? _tx_span.data[_chunk_index] : 0xFF;
            _byte.begin(tx_byte);
        }
        (void)_byte.pollEdges(edge_budget, _clk, _mosi, _miso, _plan);
        if (_byte.done()) {
            if (_chunk_index < _rx_span.size) {
                _rx_span.data[_chunk_index] = _byte.rxByte();
            }
            ++_chunk_index;
            _byte.clearDone();
            return service::ServiceResult::Done;
        }
        return service::ServiceResult::Progress;
    }

    result_t<void> finishChunk()
    {
        if (_rx_span.size > 0) {
            auto committed = _rx->commit(_rx_span.size);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
        }
        if (_tx_span.size > 0) {
            auto advanced = _tx->advance(_tx_span.size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
        }
        _tx_remaining -= (_tx_span.size < _tx_remaining) ? _tx_span.size : _tx_remaining;
        _rx_remaining -= (_rx_span.size < _rx_remaining) ? _rx_span.size : _rx_remaining;
        _totals.tx += _tx_span.size;
        _totals.rx += _rx_span.size;
        _tx_span     = {};
        _rx_span     = {};
        _chunk_len   = 0;
        _chunk_index = 0;
        return {};
    }

    gpio::Pin& _clk;
    gpio::Pin& _mosi;
    gpio::Pin& _miso;
    gpio::Pin& _dc;
    TransferPlan _plan{};
    data::Source* _tx = nullptr;
    data::Sink* _rx   = nullptr;
    data::ConstDataSpan _tx_span{};
    data::DataSpan _rx_span{};
    bus::TransferTotals _totals{};
    service::fast_tick_t _half_tick = 1;
    // Real-axis edge cursor, valid only WITHIN one call (begin or a poll):
    // re-derived from ctx at every poll entry, written back to _due_v at
    // poll exit. Never compared against a tick from another call.
    service::fast_tick_t _due_tick = 0;
    // Private virtual clock and the virtual image of _due_tick (last edge
    // position, half period NOT included).
    service::fast_tick_t _svc_now = 0;
    service::fast_tick_t _due_v   = 0;
    ByteTransferState _byte{};
    size_t _chunk_len        = 0;
    size_t _chunk_index      = 0;
    size_t _tx_remaining     = 0;
    size_t _rx_remaining     = 0;
    uint32_t _command        = 0;
    uint32_t _address        = 0;
    uint32_t _meta_value     = 0;
    uint8_t _command_bytes   = 0;
    uint8_t _address_bytes   = 0;
    uint8_t _meta_remaining  = 0;
    uint8_t _dummy_remaining = 0;
    int8_t _command_dc       = -1;
    int8_t _address_dc       = -1;
    int8_t _data_dc          = -1;
    bool _half_duplex        = false;
    bool _shared_data_pin    = false;
    bool _input_available    = false;
    bool _rx_phase           = false;
    Phase _phase             = Phase::done;
    error::error_t _error    = error::error_t::OK;
};

}  // namespace impl_software
}  // anonymous namespace

Bus_software::Bus_software()
{
}

Bus_software::~Bus_software()
{
    clearTransferService();
}

result_t<void> Bus_software::release(void)
{
    clearTransferService();
    return {};
}

service::ServicePoll Bus_software::serviceImpl(const service::ServiceContext& ctx)
{
    return serviceTransfer(ctx);
}

void Bus_software::unregisterTransferService(void)
{
    if (_transfer_registered.exchange(false, std::memory_order_relaxed)) {
        (void)M5_Hal.Services.remove(*this);
    }
}

void Bus_software::clearTransferService(void)
{
    unregisterTransferService();
    if (_transfer_service != nullptr) {
        delete static_cast<impl_software::TransferService*>(_transfer_service);
        _transfer_service = nullptr;
    }
    _transfer_owner = nullptr;
    _transfer_error = error::error_t::OK;
    _transfer_totals.clear();
    _transfer_gate.reset();
}

service::ServicePoll Bus_software::serviceTransfer(const service::ServiceContext& ctx)
{
    using GateState = service::CompletionGate::State;
    if (_transfer_gate.state() != GateState::Busy) {
        unregisterTransferService();
        return service::ServiceResult::Idle;
    }

    auto* service = static_cast<impl_software::TransferService*>(_transfer_service);
    auto result   = service->service(ctx);
    // Terminal order matters: finish() must precede unregisterTransferService().
    // Once _transfer_registered is cleared, a concurrent teardown
    // (release()/dtor/init) skips the synchronous remove() and may delete the
    // service and reset the gate -- a finish() issued after that would write
    // freed/cleared storage. Publishing first keeps every write to this object
    // inside the window the teardown's remove() still waits for.
    if (result == service::ServiceResult::Error) {
        _transfer_error  = service->error();
        _transfer_totals = service->totals();
        _transfer_gate.finish(GateState::Error);
        unregisterTransferService();
        return result;
    }
    if (result == service::ServiceResult::Done) {
        _transfer_totals = service->totals();
        _transfer_gate.finish(GateState::Done);
        unregisterTransferService();
        return result;
    }
    return result;
}

result_t<void> Bus_software::init(const BusConfig_software& config)
{
    clearTransferService();
    _config = config;

    if (_config.pin_clk < 0) {
        M5_LIB_LOGE("software::spi::Bus_software::init: CLK pin not set");
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto clk = M5_Hal.Gpio.tryGetPin(_config.pin_clk);
    if (!clk.has_value()) {
        return m5::stl::make_unexpected(clk.error());
    }
    _pin_clk = clk.value();

    auto dc = impl_software::resolveOptionalPin(_config.pin_dc, _pin_dc);
    if (!dc.has_value()) {
        return m5::stl::make_unexpected(dc.error());
    }
    auto mosi = impl_software::resolveOptionalPin(_config.pin_mosi, _pin_mosi);
    if (!mosi.has_value()) {
        return m5::stl::make_unexpected(mosi.error());
    }
    auto miso = impl_software::resolveOptionalPin(_config.pin_miso, _pin_miso);
    if (!miso.has_value()) {
        return m5::stl::make_unexpected(miso.error());
    }

    _pin_clk.setMode(types::gpio_mode_t::Output);
    _pin_clk.writeLow();
    if (_pin_dc.isValid()) {
        _pin_dc.setMode(types::gpio_mode_t::Output);
        _pin_dc.writeHigh();
        impl_software::noteBusDcLevel(this, true);
    }
    if (_pin_mosi.isValid()) {
        _pin_mosi.setMode(types::gpio_mode_t::Output);
        _pin_mosi.writeLow();
    }
    if (_pin_miso.isValid()) {
        _pin_miso.setMode(types::gpio_mode_t::Input);
    }
    return {};
}

result_t<void> Bus_software::transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                                      const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                      size_t rx_len)
{
    auto waited = waitTransfer(owner, cfg);
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }

    // This variant bit-bangs a single-lane MOSI/MISO pair. Multi-lane
    // modes (dual/quad/octal) are physically unimplemented. Single-lane
    // half duplex runs TX and RX as sequential phases; with no MISO pin,
    // the MOSI pin changes to input for the RX phase.
    bool half_duplex = false;
    {
        using spi::spi_data_mode_t;
        const auto mode       = cfg.spi_data_mode;
        const bool multi_lane = mode == spi_data_mode_t::DualOutput || mode == spi_data_mode_t::DualIo ||
                                mode == spi_data_mode_t::QuadOutput || mode == spi_data_mode_t::QuadIo ||
                                mode == spi_data_mode_t::OctalOutput || mode == spi_data_mode_t::OctalIo;
        half_duplex = impl_software::isSingleLaneHalfDuplexMode(mode);
        if (multi_lane) {
            return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
        }
    }
    if (!_pin_clk.isValid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto half_tick = impl_software::halfPeriodTick(cfg);
    if (!half_tick.has_value()) {
        return m5::stl::make_unexpected(half_tick.error());
    }
    if (desc.command_bytes > 4 || desc.address_bytes > 4) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const bool has_mosi        = _pin_mosi.isValid();
    const bool has_miso        = _pin_miso.isValid();
    const bool shared_data_pin = rx_len > 0 && !has_miso && half_duplex && has_mosi;
    if (rx_len > 0 && !has_miso && !shared_data_pin) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    // Per-device D/C override: a non-negative accessor pin_dc beats the
    // bus-level default. The override pin is resolved/configured once
    // and cached (resolution walks the GPIOGroup; too slow per call).
    if (cfg.pin_dc >= 0 && cfg.pin_dc != _acc_dc_num) {
        auto acc_dc = M5_Hal.Gpio.tryGetPin(cfg.pin_dc);
        if (!acc_dc.has_value()) {
            return m5::stl::make_unexpected(acc_dc.error());
        }
        _pin_dc_acc = acc_dc.value();
        _pin_dc_acc.setMode(types::gpio_mode_t::Output);
        _pin_dc_acc.writeHigh();
        _acc_dc_num = cfg.pin_dc;
    }
    gpio::Pin& dc_pin = cfg.pin_dc >= 0 ? _pin_dc_acc : _pin_dc;

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && dc_pin.isValid() && desc.dc_level_valid) {
        dc_pin.write(desc.dc_level);
        if (cfg.pin_dc < 0) {
            impl_software::noteBusDcLevel(this, desc.dc_level);
        }
    }
    if (cfg.pin_dc < 0 && has_phase_dc) {
        if (desc.data_dc_level >= 0) {
            impl_software::noteBusDcLevel(this, desc.data_dc_level != 0);
        } else if (desc.command_dc_level >= 0 || desc.address_dc_level >= 0) {
            impl_software::noteBusDcLevel(this, true);
        }
    }

    gpio::Pin& input_pin = shared_data_pin ? _pin_mosi : _pin_miso;

    auto* transfer_service = new (std::nothrow) impl_software::TransferService{_pin_clk, _pin_mosi, input_pin, dc_pin};
    if (transfer_service == nullptr) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _transfer_service = transfer_service;
    _transfer_owner   = owner;
    _transfer_error   = error::error_t::OK;
    _transfer_gate.arm();

    transfer_service->begin(cfg, desc, src, tx_len, dst, rx_len, half_tick.value(), has_mosi,
                            has_miso || shared_data_pin, half_duplex, shared_data_pin);
    // First poll: a fresh stream vouches for nothing yet (elapsed 0);
    // local_tick anchors the intra-call edge spins.
    auto first = serviceTransfer(service::ServiceContext{0, service::fastTick()});
    if (first == service::ServiceResult::Error) {
        const auto err = _transfer_error;
        clearTransferService();
        return m5::stl::make_unexpected(err);
    }

    if (_transfer_gate.busy()) {
        _transfer_registered.store(true, std::memory_order_relaxed);
        if (!M5_Hal.Services.add(*this)) {
            clearTransferService();
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
    }

    return {};
}

result_t<bus::TransferTotals> Bus_software::waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)cfg;
    using GateState = service::CompletionGate::State;
    if (_transfer_gate.state() != GateState::Idle && _transfer_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }

    // Yield budget sized to outlast a typical transfer: the sleep phase
    // quantizes completion latency to the FreeRTOS tick (10 ms at the
    // IDF-default 100 Hz), so it must stay the priority-inversion liveness
    // backstop, not the expected path (measured: a short yield
    // phase doubled the 256-byte exchange median on a 100 Hz-tick build).
    service::SpinBackoff backoff{50000};
    // Sole-pumper poll stream: measured per iteration so a task that
    // migrates cores mid-wait gap-drops (elapsed=0) instead of comparing
    // ticks from two different cycle counters.
    service::TickStream pump_stream;
    while (_transfer_gate.busy()) {
        if (_transfer_registered.load(std::memory_order_relaxed)) {
            // Runner-owned state: pump only through runOnce()'s try-lock so
            // this thread can never poll the same TransferService concurrently
            // with the runner task (double-pump window at auto-run start).
            // The wait must eventually BLOCK, not merely yield: taskYIELD()
            // only yields to READY tasks of the SAME priority.
            if (M5_Hal.Services.autoRunActive() || !M5_Hal.Services.runOnce()) {
                backoff.step();
            } else {
                backoff.reset();
            }
        } else {
            // Unpublished state: this thread is the sole pumper; spin at full
            // speed to honor the bit-bang half-period schedule (no backoff).
            const auto s = service::sampleTickWithDomain();
            if (serviceTransfer(service::ServiceContext{pump_stream.step(s.tick, s.domain), s.tick}) ==
                service::ServiceResult::Error) {
                break;
            }
        }
    }

    if (_transfer_gate.state() == GateState::Error) {
        const auto err = _transfer_error;
        clearTransferService();
        return m5::stl::make_unexpected(err);
    }

    const auto totals = _transfer_totals;
    clearTransferService();
    return totals;
}

bool Bus_software::transferBusy(bus::IAccessor* owner)
{
    return _transfer_gate.busy() && _transfer_owner == owner;
}

result_t<void> Bus_software::beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    if (!_pin_clk.isValid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    auto cs_result = impl_software::resolveOptionalPin(cfg.pin_cs, _transaction_cs);
    if (!cs_result.has_value()) {
        return m5::stl::make_unexpected(cs_result.error());
    }

    _pin_clk.write((cfg.spi_mode & 0x02) != 0);
    if (_transaction_cs.isValid()) {
        _transaction_cs.setMode(types::gpio_mode_t::Output);
        _transaction_cs.writeLow();
    }
    if (_pin_dc.isValid() && !impl_software::busDcLevelHigh(this)) {
        _pin_dc.writeHigh();
        impl_software::noteBusDcLevel(this, true);
    }
    return {};
}

result_t<void> Bus_software::endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    auto waited = waitTransfer(owner, cfg);
    _pin_clk.write((cfg.spi_mode & 0x02) != 0);
    if (_transaction_cs.isValid()) {
        _transaction_cs.writeHigh();
        _transaction_cs = {};
    }
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }
    return {};
}

}  // namespace m5::hal::v2::spi

#endif
