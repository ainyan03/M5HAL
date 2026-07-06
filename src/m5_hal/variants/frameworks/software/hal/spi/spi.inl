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
    // A bit is driven as first clock level -> MOSI update -> second clock
    // level. This keeps the MOSI/MISO work in one half-cycle instead of
    // spreading it across both.
    plan.first_level  = plan.cpol ^ plan.cpha;
    plan.second_level = !plan.first_level;

    uint8_t mask = (cfg.spi_order == 0) ? uint8_t{0x80} : uint8_t{0x01};
    for (uint8_t i = 0; i < 8; ++i) {
        plan.masks[i] = mask;
        mask          = (cfg.spi_order == 0) ? static_cast<uint8_t>(mask >> 1) : static_cast<uint8_t>(mask << 1);
    }
    return plan;
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
        clk.write(plan.first_level);
        if (plan.has_mosi) {
            mosi.write((_tx_byte & plan.masks[_bit_index]) != 0);
        }
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
               data::Sink* dst, size_t rx_len, service::fast_tick_t half_tick, bool has_mosi, bool has_miso)
    {
        _plan          = makePlan(cfg, has_mosi, has_miso);
        _tx            = src;
        _rx            = dst;
        _tx_remaining  = (src != nullptr) ? tx_len : 0;
        _rx_remaining  = (dst != nullptr) ? rx_len : 0;
        _half_tick     = half_tick;
        _due_tick      = service::fastTick();
        _command       = desc.command;
        _address       = desc.address;
        _command_bytes = desc.command_bytes;
        _address_bytes = desc.address_bytes;
        _command_dc    = desc.command_dc_level;
        _address_dc    = desc.address_dc_level;
        _data_dc       = desc.data_dc_level;
        if (_data_dc < 0 && desc.dc_level_valid) {
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

        uint_fast8_t edge_budget = 0;
        if (_phase != Phase::dummy) {
            const auto next_due = nextEdgeDue();
            if (!service::hasReached(ctx.now_tick, next_due)) {
                return {service::ServiceResult::Idle, next_due};
            }
            edge_budget = availableEdgeBudget(ctx.now_tick, next_due);
        }

        auto polled = poll(edge_budget);
        if (!polled.has_value()) {
            _error = polled.error();
            enterDone();  // park the clock at idle even on the error path
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
        _phase = Phase::done;
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

        if (_tx != nullptr && _tx_remaining > 0 && !_tx->eof()) {
            auto peeked = _tx->peek(_tx_remaining);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            _tx_span = peeked.value();
            _tx_span = _tx_span.first(_tx_remaining);
        }
        if (_rx != nullptr && _rx_remaining > 0 && !_rx->closed()) {
            auto reserved = _rx->reserve(_rx_remaining);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            _rx_span = reserved.value();
            _rx_span = _rx_span.first(_rx_remaining);
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
    service::fast_tick_t _due_tick  = 0;
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
    if (_transfer_registered) {
        (void)M5_Hal.Services.remove(*this);
        _transfer_registered = false;
    }
}

void Bus_software::clearTransferService(void)
{
    unregisterTransferService();
    if (_transfer_service != nullptr) {
        delete static_cast<impl_software::TransferService*>(_transfer_service);
        _transfer_service = nullptr;
    }
    _transfer_owner  = nullptr;
    _transfer_active = false;
    _transfer_done   = true;
    _transfer_error  = error::error_t::OK;
    _transfer_totals.clear();
}

service::ServicePoll Bus_software::serviceTransfer(const service::ServiceContext& ctx)
{
    if (!_transfer_active || _transfer_done) {
        unregisterTransferService();
        return service::ServiceResult::Idle;
    }

    auto* service = static_cast<impl_software::TransferService*>(_transfer_service);
    auto result   = service->service(ctx);
    if (result == service::ServiceResult::Error) {
        _transfer_error  = service->error();
        _transfer_totals = service->totals();
        unregisterTransferService();
        _transfer_done = true;
        return result;
    }
    if (result == service::ServiceResult::Done) {
        _transfer_totals = service->totals();
        unregisterTransferService();
        _transfer_done = true;
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
    // modes (dual/quad/octal) are physically unimplemented: always
    // reject. Half-duplex modes share the full-duplex wire shape as long
    // as a transfer carries data in only ONE direction (the meta phase
    // is already sent sequentially — the DC demos rely on that); what
    // cannot be honored is half-duplex with BOTH src and dst data, which
    // full-duplex clocking would corrupt.
    {
        using spi::spi_data_mode_t;
        const auto mode       = cfg.spi_data_mode;
        const bool multi_lane = mode == spi_data_mode_t::DualOutput || mode == spi_data_mode_t::DualIo ||
                                mode == spi_data_mode_t::QuadOutput || mode == spi_data_mode_t::QuadIo ||
                                mode == spi_data_mode_t::OctalOutput || mode == spi_data_mode_t::OctalIo;
        const bool half_duplex = mode == spi_data_mode_t::HalfDuplex || mode == spi_data_mode_t::HalfDuplexWithDcPin ||
                                 mode == spi_data_mode_t::HalfDuplexWithDcBit;
        if (multi_lane ||
            (half_duplex && src != nullptr && tx_len > 0 && !src->eof() && dst != nullptr && rx_len > 0)) {
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

    const bool has_mosi = _pin_mosi.isValid();
    const bool has_miso = _pin_miso.isValid();

    auto* transfer_service = new (std::nothrow) impl_software::TransferService{_pin_clk, _pin_mosi, _pin_miso, dc_pin};
    if (transfer_service == nullptr) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _transfer_service = transfer_service;
    _transfer_owner   = owner;
    _transfer_active  = true;
    _transfer_done    = false;
    _transfer_error   = error::error_t::OK;

    transfer_service->begin(cfg, desc, src, tx_len, dst, rx_len, half_tick.value(), has_mosi, has_miso);
    auto first = serviceTransfer(service::ServiceContext{service::fastTick()});
    if (first == service::ServiceResult::Error) {
        const auto err = _transfer_error;
        clearTransferService();
        return m5::stl::make_unexpected(err);
    }

    if (!_transfer_done) {
        if (M5_Hal.Services.add(*this)) {
            _transfer_registered = true;
        } else {
            clearTransferService();
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
    }

    return {};
}

result_t<bus::TransferTotals> Bus_software::waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)cfg;
    if (_transfer_active && _transfer_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }

    while (_transfer_active && !_transfer_done && _transfer_error == error::error_t::OK) {
        if (_transfer_registered && M5_Hal.Services.autoRunActive()) {
            runtime::yield();
        } else {
            auto result = serviceTransfer(service::ServiceContext{service::fastTick()});
            if (result == service::ServiceResult::Error) {
                break;
            }
        }
    }

    if (error::isError(_transfer_error)) {
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
    return _transfer_active && _transfer_owner == owner && !_transfer_done && _transfer_error == error::error_t::OK;
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
