// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL

#include "slave.hpp"

#if (defined(ESP_PLATFORM) || defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#include "../../detail/esp_err_map.hpp"

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_private/spi_slave_internal.h>
#include <driver/gpio.h>
#include <new>
#include <string.h>

#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::spi {

namespace {
namespace impl_espidf_slave {

error::error_t mapEspErr(::esp_err_t err)
{
    // Unlike the other backends, INVALID_STATE is kept distinct from
    // INVALID_ARGUMENT here: the spi_slave driver reports lifecycle
    // conflicts (already initialized / not initialized) with
    // ESP_ERR_INVALID_STATE, and callers of the slave lifecycle API can
    // act on that distinction (re-init vs bad config).
    return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::IO_ERROR,
                                                                       error::error_t::INVALID_STATE);
}

size_t copySpan(uint8_t* destination, size_t capacity, data::ConstDataSpan first, data::ConstDataSpan second)
{
    const size_t first_size = first.size < capacity ? first.size : capacity;
    if (first_size != 0) {
        ::memcpy(destination, first.data, first_size);
    }
    const size_t remaining   = capacity - first_size;
    const size_t second_size = second.size < remaining ? second.size : remaining;
    if (second_size != 0) {
        ::memcpy(destination + first_size, second.data, second_size);
    }
    return first_size + second_size;
}

}  // namespace impl_espidf_slave
}  // namespace

result_t<void> SpiSlaveBus_espidf::init(const spi::SlaveBusConfig& cfg)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (cfg.pin_clk < 0 || cfg.pin_mosi < 0 || cfg.pin_miso < 0 || cfg.pin_cs < 0 || cfg.spi_mode > 3) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    ::spi_host_device_t host = SPI2_HOST;
#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)
    int host_ordinal = 0;
    if (!detail_espidf_spi::slaveHostOrdinal(cfg.controller, 2, static_cast<int>(SPI2_HOST), host_ordinal)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    host = static_cast<::spi_host_device_t>(host_ordinal);
#else
    if (!detail_espidf_spi::hostForSlaveController(cfg.controller, host)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif

    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }

    // Classic ESP32 DMA may overwrite the final word for a non-word-sized
    // receive. Keep the public wire bound at 32767 while reserving its padded
    // storage extent so that documented driver behavior stays in-bounds.
    _tx_bounce = static_cast<uint8_t*>(::heap_caps_malloc(kBounceStorageCapacity, MALLOC_CAP_DMA));
    _rx_bounce = static_cast<uint8_t*>(::heap_caps_malloc(kBounceStorageCapacity, MALLOC_CAP_DMA));
    if (_tx_bounce == nullptr || _rx_bounce == nullptr) {
        freeBounceBuffers();
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    _config          = cfg;
    _host            = host;
    _configured      = true;
    _broken          = false;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)resetForInitialization();
        return initialized;
    }
    return {};
}

bus::CloseOutcome SpiSlaveBus_espidf::teardownBackend(void)
{
    const auto driver_error = abortOperation();
    if (error::isError(driver_error)) {
        // The driver may still retain the descriptor, callback target, and DMA
        // buffers. Preserve that driver-owned storage until a later teardown
        // succeeds; the stopped task itself is safe to reclaim because the
        // callback and caller gates are already closed.
        _broken = true;
        deleteStoppedWorker();
        return bus::CloseOutcome::partialOrUnknown(driver_error);
    }
    deleteStoppedWorker();
    freeBounceBuffers();
    _configured = false;
    _broken     = false;
    return bus::CloseOutcome::success();
}

result_t<void> SpiSlaveBus_espidf::resetForInitialization(void)
{
    auto outcome = teardownBackend();
    if (outcome.disposition == bus::CloseDisposition::Success) {
        return {};
    }
    if (outcome.disposition == bus::CloseDisposition::PartialOrUnknown) {
        quarantineLifecycleAfterPartialTeardown();
    }
    return m5::stl::make_unexpected(outcome.error_code);
}

result_t<void> SpiSlaveBus_espidf::beginOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context)
{
    if (!_configured || _broken || _driver_active || _worker_task != nullptr) {
        return m5::stl::make_unexpected(_broken ? error::error_t::IO_ERROR : error::error_t::INVALID_STATE);
    }
    if (context.config.transaction_bytes == 0 || context.config.transaction_bytes > kBounceCapacity) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _callback_state = new (std::nothrow) CallbackState;
    if (_callback_state == nullptr) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    // ISlaveBus invokes this exact overload only for SpiSlaveAccessor.
    _access_config = context.config;
    _generation    = context.runtime.generation;
    auto& accessor = static_cast<SpiSlaveAccessor&>(operationOwner(context));
    _accessor.store(&accessor, std::memory_order_relaxed);
    _accessor_borrows.store(0, std::memory_order_relaxed);
    _accessor_gate_open.store(true, std::memory_order_release);
    _callback_state->gate.store(0, std::memory_order_release);

    ::spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num        = _config.pin_mosi;
    buscfg.miso_io_num        = _config.pin_miso;
    buscfg.sclk_io_num        = _config.pin_clk;
    buscfg.quadwp_io_num      = -1;
    buscfg.quadhd_io_num      = -1;
    buscfg.max_transfer_sz    = static_cast<int>(context.config.transaction_bytes);

    ::spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num                   = _config.pin_cs;
    slvcfg.queue_size                     = 1;
    slvcfg.mode                           = _config.spi_mode;
    slvcfg.flags                          = _config.spi_order ? SPI_SLAVE_BIT_LSBFIRST : 0;
    slvcfg.post_trans_cb                  = &SpiSlaveBus_espidf::postTransaction;

    const esp_err_t initialize_error = ::spi_slave_initialize(_host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    auto mapped                      = impl_espidf_slave::mapEspErr(initialize_error);
    if (error::isError(mapped)) {
        ESP_LOGE("m5hal_spi_slave", "spi_slave_initialize host=%d failed: %s (0x%x)", static_cast<int>(_host),
                 ::esp_err_to_name(initialize_error), static_cast<unsigned>(initialize_error));
        closeCallbackGate();
        closeAccessorGate();
        detachAccessorAfterWorkerStopped();
        delete _callback_state;
        _callback_state = nullptr;
        return m5::stl::make_unexpected(mapped);
    }
    _driver_active  = true;
    _driver_enabled = true;

    _worker_error.store(error::error_t::ASYNC_RUNNING, std::memory_order_relaxed);
    _worker_state.store(WorkerState::Starting, std::memory_order_relaxed);
    _callback_state->transaction_completed.store(false, std::memory_order_relaxed);
    _accepting.store(true, std::memory_order_release);
    if (::xTaskCreate(&SpiSlaveBus_espidf::workerEntry, "m5hal_spi_slave", 3072, this, configMAX_PRIORITIES - 1,
                      &_worker_task) != pdPASS) {
        _accepting.store(false, std::memory_order_release);
        _worker_state.store(WorkerState::Stopped, std::memory_order_release);
        closeCallbackGate();
        closeAccessorGate();
        detachAccessorAfterWorkerStopped();
        mapped = stopDriver();
        if (error::isError(mapped)) {
            _broken = true;
            return m5::stl::make_unexpected(mapped);
        }
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _callback_state->worker.store(_worker_task, std::memory_order_release);

    for (;;) {
        const WorkerState state = _worker_state.load(std::memory_order_acquire);
        if (state == WorkerState::Running) {
            return {};
        }
        const error::error_t observed_error = _worker_error.load(std::memory_order_acquire);
        if (state == WorkerState::Failed ||
            (state == WorkerState::Stopped && observed_error != error::error_t::ASYNC_RUNNING)) {
            const error::error_t worker_error = observed_error;
            mapped                            = abortOperation();
            if (error::isError(mapped)) {
                _broken = true;
                return m5::stl::make_unexpected(mapped);
            }
            return m5::stl::make_unexpected(worker_error);
        }
        if (bus::remainingTimeout(context.runtime, runtime::millis()) == 0) {
            mapped = abortOperation();
            if (error::isError(mapped)) {
                _broken = true;
                return m5::stl::make_unexpected(mapped);
            }
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        ::taskYIELD();
    }
}

result_t<void> SpiSlaveBus_espidf::endOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context)
{
    auto& accessor = static_cast<SpiSlaveAccessor&>(operationOwner(context));
    if (&accessor != _accessor.load(std::memory_order_acquire) || context.runtime.generation != _generation ||
        !_driver_active || _worker_task == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    _accepting.store(false, std::memory_order_release);
    const uint32_t remaining = bus::remainingTimeout(context.runtime, runtime::millis());
    bool stopped             = false;
    if (::gpio_get_level(static_cast<::gpio_num_t>(_config.pin_cs)) == 0) {
        // A low CS denotes the natural SPI frame in progress. Leave the worker
        // armed for its post-transaction notification until the close budget
        // expires; it observes accepting=false and does not queue another frame.
        stopped = waitForWorkerStop(remaining);
    } else {
        ::xTaskNotifyGive(_worker_task);
        stopped = waitForWorkerStop(remaining);
    }
    if (!stopped) {
        const error::error_t aborted = abortOperation();
        if (error::isError(aborted)) {
            _broken = true;
            return m5::stl::make_unexpected(aborted);
        }
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }

    error::error_t outcome = _worker_error.load(std::memory_order_acquire);
    if (outcome == error::error_t::ASYNC_RUNNING) {
        outcome = error::error_t::OK;
    }
    closeCallbackGate();
    closeAccessorGate();
    detachAccessorAfterWorkerStopped();
    auto stopped_driver = stopDriver();
    if (error::isError(stopped_driver)) {
        // Keep the suspended callback target and all descriptor storage alive.
        // A later close() or init() is the only recovery attempt after this
        // persistent break.
        _broken = true;
        deleteStoppedWorker();
        return m5::stl::make_unexpected(stopped_driver);
    }
    deleteStoppedWorker();
    if (error::isError(outcome)) {
        return m5::stl::make_unexpected(outcome);
    }
    return {};
}

void SpiSlaveBus_espidf::workerEntry(void* arg)
{
    auto* self = static_cast<SpiSlaveBus_espidf*>(arg);
    if (self->_callback_state != nullptr) {
        self->_callback_state->worker.store(::xTaskGetCurrentTaskHandle(), std::memory_order_release);
    }
    self->workerLoop();
}

void SpiSlaveBus_espidf::postTransaction(::spi_slave_transaction_t* transaction)
{
    auto* state   = static_cast<CallbackState*>(transaction->user);
    uint32_t gate = state->gate.load(std::memory_order_acquire);
    for (;;) {
        if ((gate & kCallbackGateClosed) != 0 || (gate & kCallbackGateCount) == kCallbackGateCount) {
            return;
        }
        if (state->gate.compare_exchange_weak(gate, gate + 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
            break;
        }
    }
    state->transaction_completed.store(true, std::memory_order_release);
    BaseType_t higher_priority_woken = pdFALSE;
    const TaskHandle_t worker        = state->worker.load(std::memory_order_acquire);
    if (worker != nullptr) {
        ::vTaskNotifyGiveFromISR(worker, &higher_priority_woken);
    }
    state->gate.fetch_sub(1, std::memory_order_release);
    if (higher_priority_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void SpiSlaveBus_espidf::workerLoop()
{
    error::error_t worker_error = prepareAndQueue();
    if (error::isError(worker_error)) {
        _worker_error.store(worker_error, std::memory_order_release);
        _worker_state.store(WorkerState::Failed, std::memory_order_release);
    } else {
        _worker_state.store(WorkerState::Running, std::memory_order_release);
    }

    while (!error::isError(worker_error)) {
        (void)::ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (_callback_state->transaction_completed.exchange(false, std::memory_order_acq_rel)) {
            worker_error = completeTransaction();
            if (error::isError(worker_error)) {
                break;
            }
        }
        if (!_accepting.load(std::memory_order_acquire)) {
            break;
        }
        worker_error = prepareAndQueue();
    }

    _worker_error.store(worker_error, std::memory_order_release);
    if (error::isError(worker_error)) {
        publish(slave::SlaveEvent::BusBroken);
    }
    _worker_state.store(WorkerState::Stopped, std::memory_order_release);
    for (;;) {
        (void)::ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

error::error_t SpiSlaveBus_espidf::prepareAndQueue()
{
    const size_t capacity = _access_config.transaction_bytes;
    ::memset(_tx_bounce, _config.tx_fill_byte, capacity);
    ::memset(_rx_bounce, 0, capacity);
    _tx_payload_bytes = 0;
    _tx_frame_queued  = false;
    _tx_frame_bytes   = 0;

    SpiSlaveAccessor* accessor = borrowAccessor();
    if (accessor == nullptr) {
        return error::error_t::CLOSED;
    }
    slave::SlaveQueue& tx = accessor->backendTxQueue();
    if (_access_config.tx_mode == slave::QueueMode::Byte) {
        auto view = tx.peekBytes(capacity);
        if (view.has_value()) {
            _tx_payload_bytes = impl_espidf_slave::copySpan(_tx_bounce, capacity, view->first, view->second);
        } else if (view.error() != error::error_t::WOULD_BLOCK) {
            const auto outcome = view.error();
            returnAccessor();
            return outcome;
        }
    } else {
        auto frame = tx.peekFrame();
        if (frame.has_value()) {
            _tx_frame_bytes   = frame->first.size + frame->second.size;
            _tx_payload_bytes = impl_espidf_slave::copySpan(_tx_bounce, capacity, frame->first, frame->second);
            _tx_frame_queued  = true;
        } else if (frame.error() != error::error_t::WOULD_BLOCK) {
            const auto outcome = frame.error();
            returnAccessor();
            return outcome;
        }
    }
    auto& transaction     = _callback_state->transaction;
    transaction           = {};
    transaction.length    = capacity * 8;
    transaction.tx_buffer = _tx_bounce;
    transaction.rx_buffer = _rx_bounce;
    transaction.user      = _callback_state;
    _callback_state->transaction_completed.store(false, std::memory_order_release);
    const esp_err_t queue_error = ::spi_slave_queue_trans(_host, &transaction, 0);
    const error::error_t queued = impl_espidf_slave::mapEspErr(queue_error);
    if (error::isError(queued)) {
        ESP_LOGE("m5hal_spi_slave", "spi_slave_queue_trans host=%d failed: %s (0x%x)", static_cast<int>(_host),
                 ::esp_err_to_name(queue_error), static_cast<unsigned>(queue_error));
    }
    returnAccessor();
    return queued;
}

error::error_t SpiSlaveBus_espidf::completeTransaction()
{
    ::spi_slave_transaction_t* completed = nullptr;
    const error::error_t collected =
        impl_espidf_slave::mapEspErr(::spi_slave_get_trans_result(_host, &completed, portMAX_DELAY));
    if (error::isError(collected)) {
        return collected;
    }
    auto& transaction = _callback_state->transaction;
    if (completed != &transaction) {
        return error::error_t::IO_ERROR;
    }

    const size_t capacity = _access_config.transaction_bytes;
    size_t wire_bytes     = static_cast<size_t>(transaction.trans_len) / 8;
    if (wire_bytes > capacity) {
        wire_bytes = capacity;
    }

    SpiSlaveAccessor* accessor = borrowAccessor();
    if (accessor == nullptr) {
        return error::error_t::OK;
    }
    slave::SlaveQueue& tx    = accessor->backendTxQueue();
    slave::SlaveQueue& rx    = accessor->backendRxQueue();
    slave::SlaveEvent events = slave::SlaveEvent::TxSpace;

    const size_t consumed = wire_bytes < _tx_payload_bytes ? wire_bytes : _tx_payload_bytes;
    result_t<void> advanced;
    if (_access_config.tx_mode == slave::QueueMode::Byte) {
        advanced = tx.popBytes(consumed);
    } else if (_tx_frame_queued) {
        advanced = tx.popFrame();
    }
    if (!advanced.has_value()) {
        const auto outcome = advanced.error();
        returnAccessor();
        return outcome;
    }

    const bool underrun = wire_bytes > _tx_payload_bytes;
    const bool tx_truncated =
        _access_config.tx_mode == slave::QueueMode::Frame && _tx_frame_queued && consumed < _tx_frame_bytes;
    if (underrun) {
        tx.recordUnderrun();
        events |= slave::SlaveEvent::Underrun;
    }
    if (tx_truncated) {
        const size_t dropped = _tx_frame_bytes - consumed;
        tx.recordDroppedFrames(0, static_cast<uint32_t>(dropped > UINT32_MAX ? UINT32_MAX : dropped));
        events |= slave::SlaveEvent::Overflow;
    }

    if (_access_config.rx_mode == slave::QueueMode::Frame) {
        slave::FrameMetadata metadata;
        metadata.frame_id = ++_frame_id;
        metadata.flags    = slave::FrameFlags::Begin | slave::FrameFlags::End;
        if (underrun) {
            metadata.flags |= slave::FrameFlags::Underrun;
        }
        if (tx_truncated) {
            metadata.flags |= slave::FrameFlags::Overflow | slave::FrameFlags::Truncated;
        }
        auto written = rx.writeObservedFrame({_rx_bounce, wire_bytes}, metadata);
        if (!written.has_value() && written.error() != error::error_t::WOULD_BLOCK) {
            const auto outcome = written.error();
            returnAccessor();
            return outcome;
        }
        if (!written.has_value()) {
            events |= slave::SlaveEvent::Overflow;
        }
        events |= slave::SlaveEvent::FrameCompleted;
    } else if (wire_bytes != 0) {
        auto written = rx.write({_rx_bounce, wire_bytes});
        if (!written.has_value()) {
            if (written.error() != error::error_t::WOULD_BLOCK) {
                const auto outcome = written.error();
                returnAccessor();
                return outcome;
            }
            rx.recordDroppedFrames(0, static_cast<uint32_t>(wire_bytes));
            events |= slave::SlaveEvent::Overflow;
        } else if (written.value() < wire_bytes) {
            rx.recordDroppedFrames(0, static_cast<uint32_t>(wire_bytes - written.value()));
            events |= slave::SlaveEvent::Overflow;
        }
    }

    if (wire_bytes != 0) {
        events |= slave::SlaveEvent::RxAvailable;
    }
    accessor->backendEvents().publish(events, rx.readable(), tx.writable(), _generation);
    returnAccessor();
    return error::error_t::OK;
}

void SpiSlaveBus_espidf::publish(slave::SlaveEvent events)
{
    SpiSlaveAccessor* accessor = borrowAccessor();
    if (accessor == nullptr) {
        return;
    }
    accessor->backendEvents().publish(events, accessor->backendRxQueue().readable(),
                                      accessor->backendTxQueue().writable(), _generation);
    returnAccessor();
}

SpiSlaveAccessor* SpiSlaveBus_espidf::borrowAccessor()
{
    if (!_accessor_gate_open.load(std::memory_order_acquire)) {
        return nullptr;
    }
    _accessor_borrows.fetch_add(1, std::memory_order_acq_rel);
    if (!_accessor_gate_open.load(std::memory_order_acquire)) {
        _accessor_borrows.fetch_sub(1, std::memory_order_release);
        return nullptr;
    }
    SpiSlaveAccessor* accessor = _accessor.load(std::memory_order_acquire);
    if (accessor == nullptr) {
        _accessor_borrows.fetch_sub(1, std::memory_order_release);
    }
    return accessor;
}

void SpiSlaveBus_espidf::returnAccessor()
{
    _accessor_borrows.fetch_sub(1, std::memory_order_release);
}

void SpiSlaveBus_espidf::closeAccessorGate()
{
    _accessor_gate_open.store(false, std::memory_order_release);
}

void SpiSlaveBus_espidf::detachAccessorAfterWorkerStopped()
{
    // The worker is the sole accessor borrower. Once it has stopped (or has
    // been externally deleted), no stale lease can execute a later release.
    _accessor_borrows.store(0, std::memory_order_release);
    _accessor.store(nullptr, std::memory_order_release);
}

void SpiSlaveBus_espidf::closeCallbackGate()
{
    if (_callback_state != nullptr) {
        _callback_state->gate.fetch_or(kCallbackGateClosed, std::memory_order_acq_rel);
    }
}

bool SpiSlaveBus_espidf::waitForCallbackDrain(uint32_t timeout_ms)
{
    if (_callback_state == nullptr) {
        return true;
    }
    const uint32_t started = runtime::millis();
    while ((_callback_state->gate.load(std::memory_order_acquire) & kCallbackGateCount) != 0) {
        if (runtime::millis() - started >= timeout_ms) {
            return false;
        }
        ::taskYIELD();
    }
    return true;
}

error::error_t SpiSlaveBus_espidf::abortOperation()
{
    // First stop all callback-to-worker and worker-to-caller paths. The task
    // is then given a short grace period before it is deleted. This is the
    // bounded escape hatch for a master that never clocks an already queued
    // descriptor. spi_slave_free() subsequently removes the callback source.
    closeCallbackGate();
    _accepting.store(false, std::memory_order_release);
    closeAccessorGate();
    if (_worker_task != nullptr) {
        ::xTaskNotifyGive(_worker_task);
        if (!waitForWorkerStop(kAbortWorkerGraceMs)) {
            forceDeleteWorker();
        }
    }
    detachAccessorAfterWorkerStopped();
    const error::error_t stopped = stopDriver();
    deleteStoppedWorker();
    return stopped;
}

void SpiSlaveBus_espidf::deleteStoppedWorker()
{
    if (_worker_task != nullptr && _worker_state.load(std::memory_order_acquire) == WorkerState::Stopped) {
        if (_callback_state != nullptr) {
            _callback_state->worker.store(nullptr, std::memory_order_release);
        }
        ::vTaskDelete(_worker_task);
        _worker_task = nullptr;
    }
}

void SpiSlaveBus_espidf::forceDeleteWorker()
{
    if (_worker_task != nullptr) {
        if (_callback_state != nullptr) {
            _callback_state->worker.store(nullptr, std::memory_order_release);
        }
        ::vTaskDelete(_worker_task);
        _worker_task = nullptr;
        _worker_state.store(WorkerState::Stopped, std::memory_order_release);
    }
}

bool SpiSlaveBus_espidf::waitForWorkerStop(uint32_t timeout_ms)
{
    const uint32_t started = runtime::millis();
    while (_worker_state.load(std::memory_order_acquire) != WorkerState::Stopped) {
        if (timeout_ms != types::TIMEOUT_FOREVER && runtime::millis() - started >= timeout_ms) {
            return false;
        }
        ::taskYIELD();
    }
    return true;
}

error::error_t SpiSlaveBus_espidf::stopDriver()
{
    if (!_driver_active) {
        return error::error_t::OK;
    }
    // Fence new master transactions before observing CS. Checking CS first
    // would leave a TOCTOU window in which the master could assert it before
    // disable, making the later private queue_reset call undefined.
    if (_driver_enabled) {
        const error::error_t disabled = impl_espidf_slave::mapEspErr(::spi_slave_disable(_host));
        if (error::isError(disabled)) {
            return disabled;
        }
        _driver_enabled = false;
    }
    // A low CS may denote an in-flight DMA transaction. ESP-IDF explicitly
    // declares queue_reset undefined in that state and provides no public hard
    // abort. Preserve only backend-owned storage and report unsupported; a
    // later teardown may retry after the master returns CS high.
    if (::gpio_get_level(static_cast<::gpio_num_t>(_config.pin_cs)) == 0) {
        return error::error_t::UNSUPPORTED;
    }
    // No callback that entered before close may overlap driver teardown.
    if (!waitForCallbackDrain(kAbortWorkerGraceMs)) {
        return error::error_t::TIMEOUT_ERROR;
    }
    // queue_reset is private API, so capability/version gating is intentional
    // and documented by this backend rather than inferred from
    // spi_slave_free(). A successful free synchronously removes the interrupt
    // source; only then may CallbackState be deleted.
    const error::error_t reset = impl_espidf_slave::mapEspErr(::spi_slave_queue_reset(_host));
    if (error::isError(reset)) {
        return reset;
    }
    const error::error_t mapped = impl_espidf_slave::mapEspErr(::spi_slave_free(_host));
    if (!error::isError(mapped)) {
        _driver_active = false;
        delete _callback_state;
        _callback_state = nullptr;
    }
    return mapped;
}

void SpiSlaveBus_espidf::abandonDriverOwnedStorage()
{
    // Terminal destructor fallback. The ESP-IDF driver may still own the
    // descriptor and DMA buffers, so intentionally leak only that backend
    // storage. CallbackState is independent of `this`, closed, and has no live
    // worker/caller pointer; a late ISR can therefore return safely.
    closeCallbackGate();
    if (_callback_state != nullptr) {
        _callback_state->worker.store(nullptr, std::memory_order_release);
    }
    _callback_state = nullptr;
    _tx_bounce      = nullptr;
    _rx_bounce      = nullptr;
    _driver_active  = false;
    _driver_enabled = false;
    _configured     = false;
}

void SpiSlaveBus_espidf::freeBounceBuffers()
{
    if (_tx_bounce != nullptr) {
        ::heap_caps_free(_tx_bounce);
        _tx_bounce = nullptr;
    }
    if (_rx_bounce != nullptr) {
        ::heap_caps_free(_rx_bounce);
        _rx_bounce = nullptr;
    }
}

}  // namespace m5::hal::v2::spi

#endif  // ESP_PLATFORM || host harness

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL
