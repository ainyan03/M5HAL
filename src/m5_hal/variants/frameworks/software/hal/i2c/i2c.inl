// SPDX-License-Identifier: MIT
// Software (bit-bang) I2C implementation. Included by M5HAL_v1.cpp via
// the variant's hal.inl hub. Flat-injected as the default I2C when no
// platform variant offers a hardware implementation.

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include <M5Utility.hpp>

#include <thread>

// Sync runner pacing. YIELD_PROBE_SPINS tunes the normal (production) idle path:
// when the service is idle the runner yields the thread only after this many busy
// probes (and only if the next due time is far enough out). This yield is
// load-bearing for host cooperation — a same-thread master+slave native test
// needs the runner to yield so the peer can make progress (cf. ADR 014).
#ifndef M5HAL_CONFIG_SOFTWARE_I2C_YIELD_PROBE_SPINS
#define M5HAL_CONFIG_SOFTWARE_I2C_YIELD_PROBE_SPINS 64
#endif

// M5HAL_DEBUG_* is the diagnostic family (default off, not a supported config).
// NO_WAIT=1 compiles the whole yield path out so the runner spins continuously;
// it breaks host cooperation, so use it only to isolate timing while debugging
// the runner.
#ifndef M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT
#define M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT 0
#endif

namespace m5::variants::frameworks::software::hal::v1::i2c {

using namespace ::m5::hal::v1;  // resolve unqualified interface::/types::/bus:: refs

namespace {

class PinMasterLineDriver : public detail::MasterLineDriver {
public:
    PinMasterLineDriver(::m5::hal::v1::gpio::Pin& scl, ::m5::hal::v1::gpio::Pin& sda) : _scl(scl), _sda(sda)
    {
    }

    void writeSclHigh() override
    {
        _scl.writeHigh();
    }
    void writeSclLow() override
    {
        _scl.writeLow();
    }
    void writeSda(bool high) override
    {
        _sda.write(high);
    }
    bool readScl() const override
    {
        return _scl.read();
    }
    bool readSda() const override
    {
        return _sda.read();
    }

private:
    ::m5::hal::v1::gpio::Pin& _scl;
    ::m5::hal::v1::gpio::Pin& _sda;
};

uint32_t serviceFastTickFrequencyHz()
{
    const static uint32_t frequency_hz = ::m5::hal::v1::service::fastTickFrequencyHz();
    return frequency_hz;
}

::m5::hal::v1::service::fast_tick_t serviceNsecToTick(::m5::hal::v1::service::tick_nsec_t nsec)
{
    return ::m5::hal::v1::service::nsecToFastTickCeil(nsec, serviceFastTickFrequencyHz());
}

detail::MasterTiming serviceTimingToTicks(const detail::MasterTiming& timing)
{
    // Keep detail services generic, but run the synchronous software I2C path
    // in fastTick units so the hot loop does not pay fastTick->nsec conversion.
    detail::MasterTiming result;
    result.half_period = serviceNsecToTick(timing.half_period);
    result.timeout     = serviceNsecToTick(timing.timeout);
    return result;
}

::m5::hal::v1::service::fast_tick_t serviceNowTick()
{
    return ::m5::hal::v1::service::fastTick();
}

template <typename Service>
::m5::hal::v1::result_t<void> runErrorReportingService(Service& service,
                                                       ::m5::hal::v1::service::fast_tick_t deadline_tick)
{
#if !M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT
    uint32_t idle_spins = 0;
#endif
    for (;;) {
        const auto now_tick = serviceNowTick();
        // Whole-transfer deadline (S16 D10): `timeout_ms` bounds the
        // ENTIRE transfer, matching the espidf backend's per-transfer
        // semantics. Without this, only individual clock stretches were
        // bounded and a transfer could stall indefinitely in aggregate.
        if (::m5::hal::v1::service::hasReached(now_tick, deadline_tick)) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::TIMEOUT_ERROR);
        }
        auto result = service.service(::m5::hal::v1::service::ServiceContext{now_tick});
        if (result == ::m5::hal::v1::service::ServiceResult::Done) {
            return {};
        }
        if (result == ::m5::hal::v1::service::ServiceResult::Error) {
            return m5::stl::make_unexpected(service.error());
        }
        if (result == ::m5::hal::v1::service::ServiceResult::Idle) {
#if !M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT
            if (++idle_spins < M5HAL_CONFIG_SOFTWARE_I2C_YIELD_PROBE_SPINS) {
                continue;
            }
            idle_spins                                                        = 0;
            constexpr ::m5::hal::v1::service::tick_nsec_t kYieldThresholdNsec = 1000000u;
            const auto wait_tick = ::m5::hal::v1::service::elapsedTicks(service.dueTick(), now_tick);
            if (wait_tick >= serviceNsecToTick(kYieldThresholdNsec)) {
                std::this_thread::yield();
            }
#endif
        } else {
#if !M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT
            idle_spins = 0;
#endif
        }
    }
}

::m5::hal::v1::result_t<void> sendAddressWithServices(::m5::hal::v1::gpio::Pin& scl, ::m5::hal::v1::gpio::Pin& sda,
                                                      const detail::MasterTiming& timing, uint8_t addr_byte,
                                                      ::m5::hal::v1::service::fast_tick_t deadline_tick)
{
    PinMasterLineDriver lines{scl, sda};
    detail::MasterTransactionService transaction;

    transaction.beginAddress(lines, timing, addr_byte, serviceNowTick());
    return runErrorReportingService(transaction, deadline_tick);
}

::m5::hal::v1::result_t<size_t> writeBytesWithService(::m5::hal::v1::gpio::Pin& scl, ::m5::hal::v1::gpio::Pin& sda,
                                                      const detail::MasterTiming& timing, const uint8_t* data,
                                                      size_t len, ::m5::hal::v1::service::fast_tick_t deadline_tick)
{
    PinMasterLineDriver lines{scl, sda};
    detail::MasterTransactionService transaction;
    if (len == 0) {
        return size_t{0};
    }
    transaction.beginWriteBuffer(lines, timing, data, len, serviceNowTick());
    auto result = runErrorReportingService(transaction, deadline_tick);
    if (!result) {
        return m5::stl::make_unexpected(result.error());
    }
    return transaction.transferred();
}

::m5::hal::v1::result_t<size_t> readBytesWithService(::m5::hal::v1::gpio::Pin& scl, ::m5::hal::v1::gpio::Pin& sda,
                                                     const detail::MasterTiming& timing, uint8_t* data, size_t len,
                                                     bool last_nack, ::m5::hal::v1::service::fast_tick_t deadline_tick)
{
    PinMasterLineDriver lines{scl, sda};
    detail::MasterTransactionService transaction;
    if (len == 0) {
        return size_t{0};
    }
    transaction.beginReadBuffer(lines, timing, data, len, last_nack, serviceNowTick());
    auto result = runErrorReportingService(transaction, deadline_tick);
    if (!result) {
        return m5::stl::make_unexpected(result.error());
    }
    return transaction.transferred();
}

::m5::hal::v1::result_t<void> sendStopWithService(::m5::hal::v1::gpio::Pin& scl, ::m5::hal::v1::gpio::Pin& sda,
                                                  const detail::MasterTiming& timing)
{
    PinMasterLineDriver lines{scl, sda};
    detail::MasterTransactionService transaction;
    transaction.beginStop(lines, timing, serviceNowTick());
    // STOP is the cleanup path: give it its own small budget so a blown
    // whole-transfer deadline still lets the bus be released cleanly.
    return runErrorReportingService(transaction, serviceNowTick() + timing.timeout);
}

}  // anonymous namespace

::m5::hal::v1::result_t<void> Bus::init(const BusConfig& config)
{
    _config = config;

    // BusConfig uses the single gpio_number_t path. Resolve through
    // `M5HALCore::Gpio` (the singleton GPIOGroup) with the CHECKED
    // `tryGetPin` — the pin numbers are caller input, so a bad value
    // must come back through the expected path, not the assert/UB
    // fast path of `getPin`. Cache the resulting `Pin` into
    // `_pin_scl` / `_pin_sda` so the transfer hot path skips the lookup.
    if (_config.pin_scl < 0 || _config.pin_sda < 0) {
        M5_LIB_LOGE("software::i2c::Bus::init: pins not set");
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    auto scl_pin = ::m5::hal::v1::M5_Hal.Gpio.tryGetPin(_config.pin_scl);
    auto sda_pin = ::m5::hal::v1::M5_Hal.Gpio.tryGetPin(_config.pin_sda);
    if (!scl_pin.has_value() || !sda_pin.has_value()) {
        M5_LIB_LOGE("software::i2c::Bus::init: pin resolution failed");
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _pin_scl = scl_pin.value();
    _pin_sda = sda_pin.value();
    _pin_scl.setMode(::m5::hal::v1::types::gpio_mode_t::OutputOpenDrainPullup);
    _pin_scl.writeLow();
    _pin_sda.setMode(::m5::hal::v1::types::gpio_mode_t::OutputOpenDrainPullup);
    _pin_sda.writeLow();
    _pin_scl.writeHigh();
    m5::utility::delayMicroseconds(5);
    _pin_sda.writeHigh();
    return {};
}

::m5::hal::v1::result_t<size_t> Bus::transfer(::m5::hal::v1::bus::Accessor* owner,
                                              const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg,
                                              const ::m5::hal::v1::i2c::TransferDesc& desc,
                                              ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    // `owner` is reserved for future lock / unlock semantics
    // (the M5UU lock migration is currently on hold); ignored here.
    (void)owner;
    auto* scl = &_pin_scl;
    auto* sda = &_pin_sda;
    if (!scl->isValid() || !sda->isValid()) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    // This bit-bang master drives 7-bit addressing only: reject a 10-bit
    // request instead of silently truncating the address (which would
    // address the wrong device). Same degradation as the espidf gen4
    // backend.
    if (cfg.address_is_10bit || cfg.i2c_addr > 0x007Fu) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    auto timing = detail::MasterTiming::fromConfig(cfg);
    if (!timing.has_value()) {
        return m5::stl::make_unexpected(timing.error());
    }
    const auto service_timing = serviceTimingToTicks(*timing);
    // Whole-transfer deadline: cfg.timeout_ms bounds this entire transfer
    // (espidf-equivalent per-transfer semantics, S16 D10). The per-stretch
    // timeout inside MasterServiceTiming still applies on top.
    const auto deadline_tick = serviceNowTick() + service_timing.timeout;

    // Pass `desc.prefix` into the legacy code path (header.data /
    // header.size) as a local `ConstDataSpan`. `desc` lives as a
    // const reference parameter so its lifetime covers this entire
    // function body.
    ::m5::hal::v1::data::ConstDataSpan header{desc.prefix, desc.prefix_len};

    size_t total          = 0;
    bool write_phase_open = false;

    auto write_addr = [&](bool read_bit) -> ::m5::hal::v1::result_t<void> {
        uint8_t addr_byte = static_cast<uint8_t>((cfg.i2c_addr << 1) | (read_bit ? 1 : 0));
        return sendAddressWithServices(*scl, *sda, service_timing, addr_byte, deadline_tick);
    };

    bool have_tx = (tx != nullptr && !tx->eof());
    bool have_rx = (rx != nullptr);

    // Probe contract (see i2c.hpp): an all-empty descriptor sends
    // only address+W and inspects the ACK. Sequence: start condition
    // -> address byte (with ACK check) -> stop. `WriteByteService`
    // surfaces `I2C_NO_ACK` on a missing ACK, which we relay back to
    // the caller.
    if (!header.size && !have_tx && !have_rx) {
        auto wa = write_addr(false);
        // Always emit a stop to return the bus to idle, even if the address NACKed.
        auto s = sendStopWithService(*scl, *sda, service_timing);
        if (!wa) {
            return m5::stl::make_unexpected(wa.error());
        }
        if (!s) {
            return m5::stl::make_unexpected(s.error());
        }
        return size_t{0};
    }

    if (header.size || have_tx) {
        auto wa = write_addr(false);
        if (!wa) {
            return m5::stl::make_unexpected(wa.error());
        }
        write_phase_open = true;
        if (header.size) {
            auto w = writeBytesWithService(*scl, *sda, service_timing, header.data, header.size, deadline_tick);
            if (!w) {
                sendStopWithService(*scl, *sda, service_timing);
                return m5::stl::make_unexpected(w.error());
            }
            // Prefix bytes are NOT counted: the return value is the data
            // phase only (tx + rx), matching SPI (S16 D4).
        }
        if (tx) {
            // Drain Source via peek/advance, writing each peeked chunk.
            while (!tx->eof()) {
                auto peeked = tx->peek(SIZE_MAX);
                if (!peeked.has_value()) {
                    sendStopWithService(*scl, *sda, service_timing);
                    return m5::stl::make_unexpected(peeked.error());
                }
                auto span = peeked.value();
                if (span.size == 0) {
                    break;  // explicit end-of-stream
                }
                auto w = writeBytesWithService(*scl, *sda, service_timing, span.data, span.size, deadline_tick);
                if (!w) {
                    sendStopWithService(*scl, *sda, service_timing);
                    return m5::stl::make_unexpected(w.error());
                }
                auto adv = tx->advance(*w);
                if (!adv.has_value()) {
                    sendStopWithService(*scl, *sda, service_timing);
                    return m5::stl::make_unexpected(adv.error());
                }
                total += *w;
            }
        }
    }

    if (rx) {
        auto rsv = rx->reserve(SIZE_MAX);
        if (!rsv.has_value()) {
            if (write_phase_open) sendStopWithService(*scl, *sda, service_timing);
            return m5::stl::make_unexpected(rsv.error());
        }
        auto rx_span = rsv.value();
        if (rx_span.size > 0) {
            if (write_phase_open && !cfg.use_restart) {
                // Stop + Start (no repeated start).
                auto s = sendStopWithService(*scl, *sda, service_timing);
                if (!s) return m5::stl::make_unexpected(s.error());
                write_phase_open = false;
            }
            auto wa = write_addr(true);
            if (!wa) {
                if (write_phase_open) sendStopWithService(*scl, *sda, service_timing);
                return m5::stl::make_unexpected(wa.error());
            }
            auto rd = readBytesWithService(*scl, *sda, service_timing, rx_span.data, rx_span.size, true, deadline_tick);
            if (!rd) {
                sendStopWithService(*scl, *sda, service_timing);
                return m5::stl::make_unexpected(rd.error());
            }
            auto com = rx->commit(*rd);
            if (!com.has_value()) {
                sendStopWithService(*scl, *sda, service_timing);
                return m5::stl::make_unexpected(com.error());
            }
            total += *rd;
        }
    }

    auto s = sendStopWithService(*scl, *sda, service_timing);
    if (!s) {
        return m5::stl::make_unexpected(s.error());
    }
    return total;
}

}  // namespace m5::variants::frameworks::software::hal::v1::i2c

#endif
