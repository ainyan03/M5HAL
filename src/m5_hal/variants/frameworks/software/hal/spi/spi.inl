// SPDX-License-Identifier: MIT
// Software (bit-bang) SPI implementation. Included by M5HAL_v1.cpp via the
// software variant's hal.inl hub.

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_INL

#include "spi.hpp"

#include <M5Utility.hpp>

#include <cstddef>

namespace m5::hal::v1::spi {

namespace {
namespace impl_software {

constexpr uint32_t kNsecPerSec = 1000000000u;

::m5::hal::v1::result_t<::m5::hal::v1::service::fast_tick_t> halfPeriodTick(
    const ::m5::hal::v1::spi::MasterAccessConfig& cfg)
{
    if (cfg.freq == 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    const uint64_t denom = static_cast<uint64_t>(cfg.freq) * 2ull;
    uint64_t half_nsec   = (static_cast<uint64_t>(kNsecPerSec) + (denom / 2ull)) / denom;
    if (half_nsec == 0) {
        half_nsec = 1;
    }
    return ::m5::hal::v1::service::nsecToFastTickCeil(static_cast<::m5::hal::v1::service::tick_nsec_t>(half_nsec),
                                                      ::m5::hal::v1::service::fastTickFrequencyHz());
}

void waitUntil(::m5::hal::v1::service::fast_tick_t due_tick)
{
    while (!::m5::hal::v1::service::hasReached(::m5::hal::v1::service::fastTick(), due_tick)) {
    }
}

void stepClock(::m5::hal::v1::gpio::Pin& clk, bool level, ::m5::hal::v1::service::fast_tick_t half_tick,
               ::m5::hal::v1::service::fast_tick_t& due_tick)
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

TransferPlan makePlan(const ::m5::hal::v1::spi::MasterAccessConfig& cfg, bool has_mosi, bool has_miso)
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

::m5::hal::v1::result_t<void> resolveOptionalPin(::m5::hal::v1::types::gpio_number_t gpio_num,
                                                 ::m5::hal::v1::gpio::Pin& out)
{
    out = {};
    if (gpio_num < 0) {
        return {};
    }
    auto pin = ::m5::hal::v1::M5_Hal.Gpio.tryGetPin(gpio_num);
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

    uint_fast8_t pollEdges(uint_fast8_t edge_budget, ::m5::hal::v1::gpio::Pin& clk, ::m5::hal::v1::gpio::Pin& mosi,
                           ::m5::hal::v1::gpio::Pin& miso, const TransferPlan& plan)
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

    void writeFirstEdge(::m5::hal::v1::gpio::Pin& clk, ::m5::hal::v1::gpio::Pin& mosi, const TransferPlan& plan)
    {
        clk.write(plan.first_level);
        if (plan.has_mosi) {
            mosi.write((_tx_byte & plan.masks[_bit_index]) != 0);
        }
        _pending_second_edge = true;
    }

    void writeSecondEdge(::m5::hal::v1::gpio::Pin& clk, ::m5::hal::v1::gpio::Pin& miso, const TransferPlan& plan)
    {
        clk.write(plan.second_level);
        if (miso.read()) {
            _rx_byte |= plan.masks[_bit_index];
        }
        finishSecondEdge();
    }

    uint_fast8_t pollWriteOnlyEdges(uint_fast8_t edge_budget, ::m5::hal::v1::gpio::Pin& clk,
                                    ::m5::hal::v1::gpio::Pin& mosi, const TransferPlan& plan)
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

    void writeSecondEdgeNoRead(::m5::hal::v1::gpio::Pin& clk, const TransferPlan& plan)
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

class TransferService : public ::m5::hal::v1::service::IService {
    enum class Phase : uint8_t {
        command,
        address,
        dummy,
        data,
        done,
    };

public:
    TransferService(::m5::hal::v1::gpio::Pin& clk, ::m5::hal::v1::gpio::Pin& mosi, ::m5::hal::v1::gpio::Pin& miso,
                    ::m5::hal::v1::gpio::Pin& dc)
        : _clk{clk}, _mosi{mosi}, _miso{miso}, _dc{dc}
    {
    }

    void begin(const ::m5::hal::v1::spi::MasterAccessConfig& cfg, const ::m5::hal::v1::spi::TransferDesc& desc,
               ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx,
               ::m5::hal::v1::service::fast_tick_t half_tick, bool has_mosi, bool has_miso)
    {
        _plan          = makePlan(cfg, has_mosi, has_miso);
        _tx            = tx;
        _rx            = rx;
        _half_tick     = half_tick;
        _due_tick      = ::m5::hal::v1::service::fastTick();
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
        _transferred     = 0;
        _tx_span         = {};
        _rx_span         = {};
        _chunk_len       = 0;
        _chunk_index     = 0;
        _clk.write(_plan.cpol);
        enterPhase(firstPhase());
    }

    bool active() const
    {
        return _phase != Phase::done;
    }

    size_t transferred() const
    {
        return _transferred;
    }

    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        if (!active()) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }

        uint_fast8_t edge_budget = 0;
        if (_phase != Phase::dummy) {
            edge_budget = availableEdgeBudget(ctx.now_tick);
            if (edge_budget == 0) {
                return ::m5::hal::v1::service::ServiceResult::Idle;
            }
        }

        auto polled = poll(edge_budget);
        if (!polled.has_value()) {
            _error = polled.error();
            enterDone();  // park the clock at idle even on the error path
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        if (!active()) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        return polled.value();
    }

private:
    ::m5::hal::v1::result_t<::m5::hal::v1::service::ServiceResult> poll(uint_fast8_t edge_budget)
    {
        switch (_phase) {
            case Phase::command:
            case Phase::address: {
                const Phase current_phase = _phase;
                auto result               = pollMetaByte(edge_budget);
                if (result == ::m5::hal::v1::service::ServiceResult::Done && _meta_remaining == 0) {
                    enterPhase(nextPhase(current_phase));
                    return ::m5::hal::v1::service::ServiceResult::Progress;
                }
                return result;
            }

            case Phase::dummy:
                pollDummyClock();
                if (_dummy_remaining == 0) {
                    enterPhase(Phase::data);
                }
                return ::m5::hal::v1::service::ServiceResult::Progress;

            case Phase::data:
                if (_chunk_index >= _chunk_len) {
                    auto chunk = acquireChunk();
                    if (!chunk.has_value()) {
                        return m5::stl::make_unexpected(chunk.error());
                    }
                    if (_phase == Phase::done) {
                        return ::m5::hal::v1::service::ServiceResult::Done;
                    }
                }
                {
                    auto result = pollDataByte(edge_budget);
                    if (result == ::m5::hal::v1::service::ServiceResult::Done) {
                        if (_chunk_index >= _chunk_len) {
                            auto finished = finishChunk();
                            if (!finished.has_value()) {
                                return m5::stl::make_unexpected(finished.error());
                            }
                        }
                        return ::m5::hal::v1::service::ServiceResult::Progress;
                    }
                    return result;
                }

            case Phase::done:
            default:
                return ::m5::hal::v1::service::ServiceResult::Done;
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

    uint_fast8_t availableEdgeBudget(::m5::hal::v1::service::fast_tick_t now_tick)
    {
        // At most ONE edge per poll, and a delayed poll re-anchors the
        // schedule instead of catching up: granting the backlog as a
        // multi-edge budget would emit those edges back-to-back at CPU
        // speed, momentarily clocking far above the configured rate.
        // Same policy as software i2c — when the poll is late the clock
        // slips (runs late), it never runs faster than configured.
        const auto next_due = static_cast<::m5::hal::v1::service::fast_tick_t>(_due_tick + _half_tick);
        if (!::m5::hal::v1::service::hasReached(now_tick, next_due)) {
            return 0;
        }
        const auto second_due = static_cast<::m5::hal::v1::service::fast_tick_t>(next_due + _half_tick);
        if (::m5::hal::v1::service::hasReached(now_tick, second_due)) {
            _due_tick = now_tick;  // backlog dropped; next edge is a full half period away
        } else {
            _due_tick = next_due;
        }
        return 1;
    }

    ::m5::hal::v1::service::ServiceResult pollMetaByte(uint_fast8_t edge_budget)
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
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        return ::m5::hal::v1::service::ServiceResult::Progress;
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

    ::m5::hal::v1::result_t<void> acquireChunk()
    {
        _tx_span     = {};
        _rx_span     = {};
        _chunk_len   = 0;
        _chunk_index = 0;

        if (_tx != nullptr && !_tx->eof()) {
            auto peeked = _tx->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            _tx_span = peeked.value();
        }
        if (_rx != nullptr && !_rx->closed()) {
            auto reserved = _rx->reserve(SIZE_MAX);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            _rx_span = reserved.value();
        }

        _chunk_len = (_tx_span.size > _rx_span.size) ? _tx_span.size : _rx_span.size;
        if (_chunk_len == 0) {
            enterDone();  // natural end of the transfer
        }
        return {};
    }

    ::m5::hal::v1::service::ServiceResult pollDataByte(uint_fast8_t edge_budget)
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
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }

    ::m5::hal::v1::result_t<void> finishChunk()
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
        _transferred += _chunk_len;
        _tx_span     = {};
        _rx_span     = {};
        _chunk_len   = 0;
        _chunk_index = 0;
        return {};
    }

    ::m5::hal::v1::gpio::Pin& _clk;
    ::m5::hal::v1::gpio::Pin& _mosi;
    ::m5::hal::v1::gpio::Pin& _miso;
    ::m5::hal::v1::gpio::Pin& _dc;
    TransferPlan _plan{};
    ::m5::hal::v1::data::Source* _tx = nullptr;
    ::m5::hal::v1::data::Sink* _rx   = nullptr;
    ::m5::hal::v1::data::ConstDataSpan _tx_span{};
    ::m5::hal::v1::data::DataSpan _rx_span{};
    ::m5::hal::v1::service::fast_tick_t _half_tick = 1;
    ::m5::hal::v1::service::fast_tick_t _due_tick  = 0;
    ByteTransferState _byte{};
    size_t _chunk_len                    = 0;
    size_t _chunk_index                  = 0;
    size_t _transferred                  = 0;
    uint32_t _command                    = 0;
    uint32_t _address                    = 0;
    uint32_t _meta_value                 = 0;
    uint8_t _command_bytes               = 0;
    uint8_t _address_bytes               = 0;
    uint8_t _meta_remaining              = 0;
    uint8_t _dummy_remaining             = 0;
    int8_t _command_dc                   = -1;
    int8_t _address_dc                   = -1;
    int8_t _data_dc                      = -1;
    Phase _phase                         = Phase::done;
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
};

}  // namespace impl_software
}  // anonymous namespace

::m5::hal::v1::result_t<void> Bus_software::init(const BusConfig_software& config)
{
    _config = config;

    if (_config.pin_clk < 0) {
        M5_LIB_LOGE("software::spi::Bus_software::init: CLK pin not set");
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    auto clk = ::m5::hal::v1::M5_Hal.Gpio.tryGetPin(_config.pin_clk);
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

    _pin_clk.setMode(::m5::hal::v1::types::gpio_mode_t::Output);
    _pin_clk.writeLow();
    if (_pin_dc.isValid()) {
        _pin_dc.setMode(::m5::hal::v1::types::gpio_mode_t::Output);
        _pin_dc.writeHigh();
    }
    if (_pin_mosi.isValid()) {
        _pin_mosi.setMode(::m5::hal::v1::types::gpio_mode_t::Output);
        _pin_mosi.writeLow();
    }
    if (_pin_miso.isValid()) {
        _pin_miso.setMode(::m5::hal::v1::types::gpio_mode_t::Input);
    }
    return {};
}

::m5::hal::v1::result_t<size_t> Bus_software::transfer(::m5::hal::v1::bus::IAccessor* owner,
                                                       const ::m5::hal::v1::spi::MasterAccessConfig& cfg,
                                                       const ::m5::hal::v1::spi::TransferDesc& desc,
                                                       ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    (void)owner;
    // This variant bit-bangs a single-lane MOSI/MISO pair. Multi-lane
    // modes (dual/quad/octal) are physically unimplemented: always
    // reject. Half-duplex modes share the full-duplex wire shape as long
    // as a transfer carries data in only ONE direction (the meta phase
    // is already sent sequentially — the DC demos rely on that); what
    // cannot be honored is half-duplex with BOTH tx and rx data, which
    // full-duplex clocking would corrupt.
    {
        using ::m5::hal::v1::spi::spi_data_mode_t;
        const auto mode       = cfg.spi_data_mode;
        const bool multi_lane = mode == spi_data_mode_t::dual_output || mode == spi_data_mode_t::dual_io ||
                                mode == spi_data_mode_t::quad_output || mode == spi_data_mode_t::quad_io ||
                                mode == spi_data_mode_t::octal_output || mode == spi_data_mode_t::octal_io;
        const bool half_duplex = mode == spi_data_mode_t::halfduplex ||
                                 mode == spi_data_mode_t::halfduplex_with_dc_pin ||
                                 mode == spi_data_mode_t::halfduplex_with_dc_bit;
        if (multi_lane || (half_duplex && tx != nullptr && !tx->eof() && rx != nullptr)) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
        }
    }
    if (!_pin_clk.isValid()) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    auto half_tick = impl_software::halfPeriodTick(cfg);
    if (!half_tick.has_value()) {
        return m5::stl::make_unexpected(half_tick.error());
    }
    if (desc.command_bytes > 4 || desc.address_bytes > 4) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    // Per-device D/C override: a non-negative accessor pin_dc beats the
    // bus-level default. The override pin is resolved/configured once
    // and cached (resolution walks the GPIOGroup; too slow per call).
    if (cfg.pin_dc >= 0 && cfg.pin_dc != _acc_dc_num) {
        auto acc_dc = ::m5::hal::v1::M5_Hal.Gpio.tryGetPin(cfg.pin_dc);
        if (!acc_dc.has_value()) {
            return m5::stl::make_unexpected(acc_dc.error());
        }
        _pin_dc_acc = acc_dc.value();
        _pin_dc_acc.setMode(::m5::hal::v1::types::gpio_mode_t::Output);
        _pin_dc_acc.writeHigh();
        _acc_dc_num = cfg.pin_dc;
    }
    ::m5::hal::v1::gpio::Pin& dc_pin = cfg.pin_dc >= 0 ? _pin_dc_acc : _pin_dc;

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && dc_pin.isValid() && desc.dc_level_valid) {
        dc_pin.write(desc.dc_level);
    }

    const bool has_mosi = _pin_mosi.isValid();
    const bool has_miso = _pin_miso.isValid();

    impl_software::TransferService transfer_service{_pin_clk, _pin_mosi, _pin_miso, dc_pin};
    transfer_service.begin(cfg, desc, tx, rx, half_tick.value(), has_mosi, has_miso);
    while (transfer_service.active()) {
        auto result =
            transfer_service.service(::m5::hal::v1::service::ServiceContext{::m5::hal::v1::service::fastTick()});
        if (result == ::m5::hal::v1::service::ServiceResult::Error) {
            return m5::stl::make_unexpected(transfer_service.error());
        }
    }

    return transfer_service.transferred();
}

::m5::hal::v1::result_t<void> Bus_software::beginTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                             const ::m5::hal::v1::spi::MasterAccessConfig& cfg)
{
    (void)owner;
    if (!_pin_clk.isValid()) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    auto cs_result = impl_software::resolveOptionalPin(cfg.pin_cs, _transaction_cs);
    if (!cs_result.has_value()) {
        return m5::stl::make_unexpected(cs_result.error());
    }

    _pin_clk.write((cfg.spi_mode & 0x02) != 0);
    if (_transaction_cs.isValid()) {
        _transaction_cs.setMode(::m5::hal::v1::types::gpio_mode_t::Output);
        _transaction_cs.writeLow();
    }
    return {};
}

::m5::hal::v1::result_t<void> Bus_software::endTransaction(::m5::hal::v1::bus::IAccessor* owner,
                                                           const ::m5::hal::v1::spi::MasterAccessConfig& cfg)
{
    (void)owner;
    _pin_clk.write((cfg.spi_mode & 0x02) != 0);
    if (_transaction_cs.isValid()) {
        _transaction_cs.writeHigh();
        _transaction_cs = {};
    }
    return {};
}

}  // namespace m5::hal::v1::spi

#endif
