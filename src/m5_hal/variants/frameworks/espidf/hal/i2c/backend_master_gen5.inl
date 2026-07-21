// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5

#include "backend_master_write_buffer.inl"

#include "../../detail/esp_err_map.hpp"

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
// ensureDevice()'s scl_wait_us clamp branches on CONFIG_IDF_TARGET_ESP32;
// FreeRTOS.h pulls sdkconfig.h in every known IDF, but make it explicit so
// the branch never silently evaluates false on an exotic include order.
#if __has_include(<sdkconfig.h>)
#include <sdkconfig.h>
#endif

#include <climits>
#include <cstdint>

#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::i2c {

namespace {
namespace impl_espidf_gen5 {

// The shared write-buffer helper stays in the variant namespace
// (backend_master_write_buffer.inl); this TU-local alias keeps the
// `detail::` spelling working in the backend implementation below.
namespace detail = ::m5::variants::frameworks::espidf::hal::v2::i2c::detail;

error::error_t mapEspErr(::esp_err_t err)
{
    switch (err) {
        case ESP_ERR_INVALID_STATE:
            // The gen5 sync driver returns this for ANY transaction that did
            // not reach DONE: on v5.x that folds NACK, SCL timeout,
            // arbitration loss and abort-recovery into one code (v6.0+ splits
            // the NACK case out as ESP_ERR_INVALID_RESPONSE below). These are
            // wire-level failures, not caller errors -- mapping them to
            // INVALID_ARGUMENT sent a real-bus diagnosis chasing argument
            // validation. I2C_BUS_ERROR is the closest truthful class the
            // return code allows.
            return error::error_t::I2C_BUS_ERROR;
        case ESP_ERR_NOT_FOUND:
            return error::error_t::I2C_NO_ACK;
        case ESP_ERR_INVALID_RESPONSE:
            // ESP-IDF v6.0+ reports a NACKed transfer with this code (v5.x
            // used ESP_ERR_INVALID_STATE, which is ambiguous with driver-state
            // errors and therefore not mapped to I2C_NO_ACK).
            return error::error_t::I2C_NO_ACK;
        default:
            return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::I2C_BUS_ERROR);
    }
}

::i2c_addr_bit_len_t addressBitLen(bool address_is_10bit)
{
#if SOC_I2C_SUPPORT_10BIT_ADDR
    return address_is_10bit ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7;
#else
    // I2C_ADDR_BIT_LEN_10 is gated behind SOC_I2C_SUPPORT_10BIT_ADDR (absent on
    // targets without 10-bit support, and on esp32 IDF patches before it was
    // backported: v5.2.4 / v5.3.3 / v5.4.1). ensureDevice() rejects a 10-bit
    // request up front when the cap is missing, so we only reach 7-bit here.
    (void)address_is_10bit;
    return I2C_ADDR_BIT_LEN_7;
#endif
}

bool isValidAddress(const i2c::MasterAccessConfig& cfg)
{
    return cfg.address_is_10bit ? (cfg.i2c_addr <= 0x03FFu) : (cfg.i2c_addr <= 0x007Fu);
}

// The IDF blocking API's xfer_timeout argument is a whole-transaction budget,
// raced against the wire by a tick-quantized FreeRTOS wait (i2c_master.c:
// pdMS_TO_TICKS + xQueueReceive). Passing wire_timeout_ms straight through
// made that budget expire MID-TRANSFER on healthy wires whenever the transfer
// straddled a tick boundary -- at a 100 Hz tick, 10 ms is one tick, and a
// one-tick wait ends at the next tick interrupt after 0..10 ms of real time;
// the driver's error path then FSM-resets the peripheral mid-byte (measured
// abort rate = wire_time / tick_period). wire_timeout_ms is specified as a
// bound on wire-level *progress* (i2c.hpp), not on total transfer duration,
// so hand IDF the expected wire time plus that progress allowance plus two
// tick periods (an N-tick wait guarantees only N-1 full tick periods of real
// time).
int transactionTimeoutMs(uint32_t wire_timeout_ms, uint64_t total_bytes, uint32_t freq_hz)
{
    if (wire_timeout_ms == types::TIMEOUT_FOREVER) {
        return -1;  // IDF blocking API: wait forever
    }
    // A Sink-driven read may legitimately request SIZE_MAX ("drain the sink");
    // saturate before the bit math below can wrap uint64.
    constexpr uint64_t kMaxCountedBytes = (UINT64_MAX / (9u * 1000u)) - 4u;
    if (total_bytes > kMaxCountedBytes) {
        return INT_MAX;
    }
    // 9 SCL cycles per byte (8 data + ACK); +4 bytes covers the address
    // byte(s) and START/RESTART/STOP framing across both phases.
    const uint64_t wire_ms = ((total_bytes + 4u) * 9u * 1000u) / (freq_hz != 0 ? freq_hz : 1u) + 1u;
    const uint64_t total   = wire_ms + wire_timeout_ms + 2u * portTICK_PERIOD_MS;
    return static_cast<int>(total > static_cast<uint64_t>(INT_MAX) ? INT_MAX : total);
}

}  // namespace impl_espidf_gen5
}  // namespace

error::error_t Bus_espidf::attachBorrowedNative(::i2c_master_bus_handle_t bus_handle)
{
    if (!initializationAllowed(false)) {
        return error::error_t::INVALID_STATE;
    }
    if (bus_handle == nullptr) {
        return error::error_t::INVALID_ARGUMENT;
    }
    if (_bus_handle != nullptr) {
        auto closed = teardownBackend();
        if (closed.disposition != bus::CloseDisposition::Success) {
            return closed.error_code;
        }
    }
    _bus_handle      = bus_handle;
    _owns_bus        = false;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)teardownBackend();
        return initialized.error();
    }
    return error::error_t::OK;
}

result_t<void> Bus_espidf::init(const IBusConfig& config)
{
    // The portable config deliberately has no controller field. Let the
    // ESP-IDF gen5 driver choose any free HP controller so two independent
    // portable buses can coexist; explicit controller assignment belongs to
    // the logical allocation path below.
    return initBackend(config, detail_espidf_i2c::kPortableControllerAuto);
}

result_t<void> Bus_espidf::initBackend(const IBusConfig& config, int8_t controller)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (config.pin_scl < 0 || config.pin_sda < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_bus_handle != nullptr) {
        auto closed = teardownBackend();
        if (closed.disposition != bus::CloseDisposition::Success) {
            return m5::stl::make_unexpected(closed.error_code);
        }
    }
    _config = config;

    _controller_port                     = controller;  // cached for controllerId()
    ::i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port                  = controller;
    bus_config.scl_io_num                = static_cast<::gpio_num_t>(_config.pin_scl);
    bus_config.sda_io_num                = static_cast<::gpio_num_t>(_config.pin_sda);
#if M5HAL_ESPIDF_I2C_LP_POOL
    // clk_source / lp_source_clk share a union (driver/i2c_master.h): an
    // LP_I2C port needs the LP source clock, and assigning the HP member
    // here would just be reinterpreting the same bits under the wrong name
    // for that port -- so branch on which member to set instead of always
    // assigning clk_source.
    if (isLowPowerControllerForI2C(controllerId())) {
        bus_config.lp_source_clk = LP_I2C_SCLK_DEFAULT;
    } else {
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    }
#else
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
#endif
    bus_config.glitch_ignore_cnt = 7;
    // Keep the gen5 master in ESP-IDF's synchronous mode. Setting
    // trans_queue_depth > 0 switches the bus to the experimental async queue
    // path; that path is not compatible with i2c_master_probe(), reports bus
    // errors only through callbacks, and constrains async use to one active
    // device per bus. M5HAL therefore keeps hardware I2C synchronous and
    // reserves ServiceRunner-driven async for software I2C and other backends
    // whose state machine is fully owned by M5HAL.
    bus_config.trans_queue_depth = 0;
    // enable_internal_pullup applies uniformly regardless of HP vs. LP port
    // (i2c_master.c: `pull_up_enable = bus_config->flags.enable_internal_pullup`
    // is the same assignment either way; ESP-IDF v5.5.3 checked) -- no LP-
    // specific branch needed here. Hardware measurement on an LP_I2C-capable
    // chip confirms LP_I2C works with this flag set as-is.
    bus_config.flags.enable_internal_pullup = 1;

    auto err    = ::i2c_new_master_bus(&bus_config, &_bus_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (error::isError(mapped)) {
        _bus_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }
    _owns_bus        = true;
    _rebuild_pending = false;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)teardownBackend();
        return initialized;
    }
    return {};
}

bus::CloseOutcome Bus_espidf::teardownBackend(void)
{
    auto removed = removeDevice();
    if (!removed.has_value()) {
        return bus::CloseOutcome::partialOrUnknown(removed.error());
    }

    if (_bus_handle != nullptr && _owns_bus) {
        // Transactional teardown: clear the handle/ownership only after
        // the ESP-IDF delete succeeds (see gen4). On error keep them set so the
        // dtor / a retry can free the bus, and a swap keeps a usable old backend.
        auto mapped = impl_espidf_gen5::mapEspErr(::i2c_del_master_bus(_bus_handle));
        if (error::isError(mapped)) {
            return bus::CloseOutcome::partialOrUnknown(mapped);
        }
        _bus_handle            = nullptr;
        _owns_bus              = false;
        _rebuild_pending       = false;
        auto released_identity = releaseNativeIdentity();
        if (!released_identity.has_value()) {
            return bus::CloseOutcome::partialOrUnknown(released_identity.error());
        }
        return bus::CloseOutcome::success();
    }
    _bus_handle            = nullptr;
    _owns_bus              = false;
    _rebuild_pending       = false;
    auto released_identity = releaseNativeIdentity();
    if (!released_identity.has_value()) {
        return bus::CloseOutcome::partialOrUnknown(released_identity.error());
    }
    return bus::CloseOutcome::success();
}

result_t<void> Bus_espidf::removeDevice(void)
{
    if (_dev_handle == nullptr) {
        return {};
    }

    auto mapped = impl_espidf_gen5::mapEspErr(::i2c_master_bus_rm_device(_dev_handle));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _dev_handle           = nullptr;
    _dev_addr             = 0;
    _dev_freq             = 0;
    _dev_scl_wait_us      = 0;
    _dev_address_is_10bit = false;
    return {};
}

// A wire-level fault (SCL-stretch timeout / bus error; NACK excluded, the
// driver drives a STOP for that case on its own) leaves residue in three
// places, each needing its own recovery step:
//  1. The wire may hold an unterminated transaction: the gen5 software-timeout
//     path only marks driver state and defers the FSM reset / clear-bus to the
//     *next* transaction, during which a listening slave keeps shifting in
//     whatever appears on the wire as continued transaction data. Reset (STOP +
//     clear pulses) immediately instead of leaving that window open.
//  2. i2c_master_bus_reset() is a hardware-FSM reset only -- the driver's
//     software state survives it. On SOC_I2C_STOP_INDEPENDENT targets (classic
//     ESP32) the gen5 driver never clears its contains_read flag once a read
//     ran, so after a fault burst every completion interrupt still re-enters
//     the driver's receive handler, which dereferences the current operation's
//     data pointer without a NULL guard -- measured on hardware as a
//     deterministic StoreProhibited (EXCVADDR=0) when a stretch-timeout burst
//     races transaction teardown. Rebuilding the bus is the only
//     application-level way to reset that state, so escalate to a full
//     rebuild when this backend owns the bus.
//  3. The slave that caused the fault is typically still stretching SCL;
//     issuing the next transaction while it holds the bus walks straight back
//     into the same fault path. Wait for both lines to release, then give the
//     slave a short settle before the caller can retry. This wait is wire
//     work, so it shares the caller's declared wire tolerance: it is capped
//     by wire_timeout_ms (a 0 / tight budget skips it) on top of the fixed
//     ceiling below, keeping probe/scan latency bounds intact.
void Bus_espidf::recoverBusAfterWireFault(error::error_t mapped, uint32_t wire_timeout_ms)
{
    if (mapped != error::error_t::TIMEOUT_ERROR && mapped != error::error_t::I2C_BUS_ERROR) {
        return;
    }
    if (_bus_handle == nullptr) {
        return;
    }
    (void)::i2c_master_bus_reset(_bus_handle);

    if (_owns_bus) {
        // Explicit teardown-then-init: avoid creating a second bus on the same
        // port when deletion of the old bus fails.
        IBusConfig config;
        config.pin_scl = _config.pin_scl;
        config.pin_sda = _config.pin_sda;
        auto closed    = teardownBackend();
        if (closed.disposition != bus::CloseDisposition::Success) {
            // Keep the old handle: it was FSM-reset above, so it stays usable
            // even though the driver-internal soft state could not be cleared.
            M5_LIB_LOGW("I2C wire-fault recovery: rebuild skipped (teardown failed); continuing on reset bus");
        } else if (!initBackend(config, static_cast<int8_t>(_controller_port)).has_value()) {
            // A transient failure (e.g. NO_MEM) must not permanently kill an
            // owned bus: transfer() retries the rebuild lazily on the next
            // call instead of reporting the null handle as caller misuse.
            _rebuild_pending = true;
            M5_LIB_LOGW("I2C wire-fault recovery could not rebuild the bus; will retry on next transfer");
            return;
        }
    }

    constexpr uint32_t kIdlePollStepUs = 50;
    // Just past the classic ESP32 SCL-timeout register ceiling (13.1 ms, see
    // ensureDevice): the longest stretch the peripheral itself would have
    // tolerated before declaring the fault we are recovering from.
    constexpr uint32_t kIdleWaitBudgetUs = 15000;
    // Settle pacing of this magnitude is what field reports on the same fault
    // class found effective (esp-idf issue 18105).
    constexpr uint32_t kSettleUs = 2000;
    // timeoutMsToUsecU32 saturates (TIMEOUT_FOREVER included), so the fixed
    // ceilings below still bound the forever case.
    const uint32_t caller_budget_us = ::m5::hal::v2::detail::timeoutMsToUsecU32(wire_timeout_ms);
    const uint32_t idle_budget_us   = caller_budget_us < kIdleWaitBudgetUs ? caller_budget_us : kIdleWaitBudgetUs;
    if (_config.pin_scl >= 0 && _config.pin_sda >= 0) {
        for (uint32_t waited = 0; waited < idle_budget_us; waited += kIdlePollStepUs) {
            if (::gpio_get_level(static_cast<::gpio_num_t>(_config.pin_scl)) != 0 &&
                ::gpio_get_level(static_cast<::gpio_num_t>(_config.pin_sda)) != 0) {
                break;
            }
            // delayUs busy-waits on ESP-IDF; yield each step so equal-priority
            // tasks are not starved while the accessor still holds the bus.
            runtime::yield();
            runtime::delayUs(kIdlePollStepUs);
        }
    }
    const uint32_t settle_us = caller_budget_us < kSettleUs ? caller_budget_us : kSettleUs;
    for (uint32_t settled = 0; settled < settle_us; settled += 100) {
        runtime::yield();
        runtime::delayUs(100);
    }
}

result_t<void> Bus_espidf::ensureDevice(const i2c::MasterAccessConfig& cfg)
{
#if !SOC_I2C_SUPPORT_10BIT_ADDR
    if (cfg.address_is_10bit) {
        // This target's I2C peripheral does not expose 10-bit addressing.
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif
    bool clamped         = false;
    const uint32_t freq  = clampMasterClockHz(cfg.freq, &clamped);
    uint32_t scl_wait_us = ::m5::hal::v2::detail::timeoutMsToUsecU32(cfg.wire_timeout_ms);
#if defined(CONFIG_IDF_TARGET_ESP32)
    // Classic ESP32: the SCL-timeout register is 20 bits of APB (80 MHz)
    // cycles and the LL conversion does not clamp -- past ~13.1 ms the value
    // silently TRUNCATES mod 2^20 (the 1000 ms default wrapped to an
    // effective ~3.9 ms). Saturate at the register ceiling instead.
    constexpr uint32_t kMaxSclWaitUs = 13107;
    if (scl_wait_us > kMaxSclWaitUs) {
        scl_wait_us = kMaxSclWaitUs;
    }
#endif
    if (_dev_handle != nullptr && _dev_addr == cfg.i2c_addr && _dev_freq == freq && _dev_scl_wait_us == scl_wait_us &&
        _dev_address_is_10bit == cfg.address_is_10bit) {
        return {};
    }
    if (clamped) {
        M5_LIB_LOGW("I2C master clock %u Hz exceeds the peripheral ceiling; clamped to %u Hz",
                    static_cast<unsigned>(cfg.freq), static_cast<unsigned>(freq));
    }

    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    ::i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length       = impl_espidf_gen5::addressBitLen(cfg.address_is_10bit);
    dev_config.device_address        = cfg.i2c_addr;
    dev_config.scl_speed_hz          = freq;
    // i2c_device_config_t::scl_wait_us was added to the bus-device API in IDF
    // v5.2.2 (backported into the v5.2 line); skip it on older patches.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 2)
    dev_config.scl_wait_us = scl_wait_us;
#endif

    auto err    = ::i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (error::isError(mapped)) {
        _dev_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }

    _dev_addr             = cfg.i2c_addr;
    _dev_freq             = freq;
    _dev_scl_wait_us      = scl_wait_us;
    _dev_address_is_10bit = cfg.address_is_10bit;

    return {};
}

result_t<void> Bus_espidf::transferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context,
                                           const i2c::TransferDesc& desc, data::Source* src, size_t tx_len,
                                           data::Sink* dst, size_t rx_len)
{
    auto* owner     = &bus::OperationSlot::contextOwner(context);
    const auto& cfg = context.config;
    (void)owner;
    if (_bus_handle == nullptr) {
        if (!_rebuild_pending) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        // Deferred wire-fault recovery (see recoverBusAfterWireFault): retry
        // the rebuild here; if it keeps failing, report the bus as faulted
        // rather than as caller misuse.
        IBusConfig config;
        config.pin_scl = _config.pin_scl;
        config.pin_sda = _config.pin_sda;
        if (!initBackend(config, static_cast<int8_t>(_controller_port)).has_value()) {
            return m5::stl::make_unexpected(error::error_t::I2C_BUS_ERROR);
        }
    }
    if (cfg.freq == 0 || !impl_espidf_gen5::isValidAddress(cfg)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    impl_espidf_gen5::detail::TempWriteBuffer write_bytes;
    auto built = write_bytes.build(data::ConstDataSpan{desc.prefix, desc.prefix_len}, src, tx_len);
    if (!built.has_value()) {
        return m5::stl::make_unexpected(built.error());
    }

    const bool have_tx = !write_bytes.empty();
    const bool have_rx = (dst != nullptr && rx_len > 0);
    const int timeout  = impl_espidf_gen5::transactionTimeoutMs(
        cfg.wire_timeout_ms, static_cast<uint64_t>(write_bytes.size()) + (have_rx ? rx_len : 0),
        clampMasterClockHz(cfg.freq));

    if (!have_tx && !have_rx) {
        if (cfg.address_is_10bit) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        auto removed = removeDevice();
        if (!removed.has_value()) {
            return m5::stl::make_unexpected(removed.error());
        }

        auto err    = ::i2c_master_probe(_bus_handle, cfg.i2c_addr, timeout);
        auto mapped = impl_espidf_gen5::mapEspErr(err);
        if (error::isError(mapped)) {
            recoverBusAfterWireFault(mapped, cfg.wire_timeout_ms);
            return m5::stl::make_unexpected(mapped);
        }
        return {};
    }

    auto ensured = ensureDevice(cfg);
    if (!ensured.has_value()) {
        return m5::stl::make_unexpected(ensured.error());
    }

    // Data phase only: the staged buffer holds prefix + src, but the
    // prefix is not counted in the return value (matches SPI).
    size_t total    = write_bytes.size() - desc.prefix_len;
    size_t received = 0;

    auto finish = [&](::esp_err_t result) -> result_t<void> {
        auto result_map = impl_espidf_gen5::mapEspErr(result);
        if (error::isError(result_map)) {
            recoverBusAfterWireFault(result_map, cfg.wire_timeout_ms);
            return m5::stl::make_unexpected(result_map);
        }
        _transfer_totals.tx += total - received;
        _transfer_totals.rx += received;
        return {};
    };

    if (!have_rx) {
        return finish(::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout));
    }

    auto rsv = dst->reserve(rx_len);
    if (!rsv.has_value()) {
        return m5::stl::make_unexpected(rsv.error());
    }
    auto rx_span = rsv.value().first(rx_len);
    if (rx_span.size == 0) {
        // A sink that reserves 0 bytes degrades this to write-only: the
        // src phase must still hit the wire — returning early here
        // reported "sent" without ever transmitting.
        if (have_tx) {
            return finish(::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout));
        }
        return finish(ESP_OK);
    }

    ::esp_err_t err = ESP_OK;
    if (have_tx && cfg.use_restart) {
        err = ::i2c_master_transmit_receive(_dev_handle, write_bytes.data(), write_bytes.size(), rx_span.data,
                                            rx_span.size, timeout);
    } else {
        if (have_tx) {
            err = ::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout);
        }
        if (err == ESP_OK) {
            err = ::i2c_master_receive(_dev_handle, rx_span.data, rx_span.size, timeout);
        }
    }
    if (err == ESP_OK) {
        auto com = dst->commit(rx_span.size);
        if (!com.has_value()) {
            return m5::stl::make_unexpected(com.error());
        }
        total += rx_span.size;
        received += rx_span.size;
    }
    return finish(err);
}

result_t<bus::TransferTotals> Bus_espidf::waitTransferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context)
{
    auto* owner     = &bus::OperationSlot::contextOwner(context);
    const auto& cfg = context.config;
    (void)owner;
    (void)cfg;
    auto totals = _transfer_totals;
    _transfer_totals.clear();
    return totals;
}

bool Bus_espidf::transferBusyBackend(bus::OperationContext<i2c::MasterAccessConfig>& context)
{
    auto* owner = &bus::OperationSlot::contextOwner(context);
    (void)owner;
    return false;
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
