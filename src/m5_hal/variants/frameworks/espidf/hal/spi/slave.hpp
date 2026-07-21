// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/spi/slave.hpp"
#include "spi.hpp"

// SPI slave is full-duplex on every ESP32-family SoC (driver/spi_slave.h), so
// unlike the I2C slave there is no per-SoC backend split. The half-duplex
// register-map model (spi_slave_hd, HW v2 only) is intentionally out of scope.
#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS) && __has_include(<driver/spi_slave.h>) && \
    __has_include(<esp_private/spi_slave_internal.h>)
#define M5HAL_ESPIDF_SPI_HAS_SLAVE 1
#elif defined(ESP_PLATFORM) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0) && \
    __has_include(<driver/spi_slave.h>) && __has_include(<esp_private/spi_slave_internal.h>)
#define M5HAL_ESPIDF_SPI_HAS_SLAVE 1
#else
#define M5HAL_ESPIDF_SPI_HAS_SLAVE 0
#endif

#if (defined(ESP_PLATFORM) || defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#include <atomic>
#include <driver/spi_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::spi {

/*! @brief ESP-IDF full-duplex SPI slave with an Access-scoped receive worker.

  init() owns only configuration and bounce-buffer preparation. beginOperation()
  starts the peripheral and a fixed one-descriptor worker; endOperation() stops
  requeueing and disables new wire acceptance during bounded close cleanup.
  When CS is idle it resets and frees the driver queue, so an unclocked
  descriptor cannot outlive its storage. An in-flight CS-low close reports
  UNSUPPORTED and defers queue cleanup to a later close. Queue/event updates happen
  in worker task context. The ESP-IDF ISR callback only wakes that worker.
 */
class SpiSlaveBus_espidf : public spi::ISlaveBus {
public:
    // DMA-capable bounce-buffer capacity (per direction).
    //
    // 32767, not 32768: the transfer length lives in an 18-bit register field
    // that stores (bit_num - 1) on this chip family. The exact maximum boundary
    // has corrupted transfers on ESP32-S3 hardware while 32767 bytes is clean.
    static constexpr size_t kBounceCapacity = 32767;

    SpiSlaveBus_espidf() = default;
    ~SpiSlaveBus_espidf() override
    {
        if (teardownBackend().disposition != bus::CloseDisposition::Success) {
            abandonDriverOwnedStorage();
        }
    }

    result_t<void> init(const spi::SlaveBusConfig& cfg) override;
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }
    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return _config.controller;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        return bus::detail::BusCapabilitiesBuilder{bus::IBus::capabilities()}
            .enable(bus::BusFeature::SlaveByteTx)
            .enable(bus::BusFeature::SlaveByteRx)
            .enable(bus::BusFeature::SlaveFrameTx)
            .enable(bus::BusFeature::SlaveFrameRx)
            .setLimit(bus::BusLimit::MaxSlaveTransactionBytes, static_cast<uint32_t>(kBounceCapacity))
            .build();
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context) override;
    bus::CloseOutcome closeBackend(void) override
    {
        return teardownBackend();
    }

private:
    bus::CloseOutcome teardownBackend(void);
    result_t<void> resetForInitialization(void);
    static constexpr size_t kBounceStorageCapacity = (kBounceCapacity + 3u) & ~size_t{3u};
    static constexpr uint32_t kAbortWorkerGraceMs  = 100;

    enum class WorkerState : uint8_t { Stopped, Starting, Running, Failed };

    struct CallbackState {
        std::atomic<uint32_t> gate{uint32_t{1} << 31};
        std::atomic<TaskHandle_t> worker{nullptr};
        std::atomic<bool> transaction_completed{false};
        ::spi_slave_transaction_t transaction{};
    };

    static void workerEntry(void* arg);
    static void postTransaction(::spi_slave_transaction_t* transaction);
    void workerLoop();
    error::error_t prepareAndQueue();
    error::error_t completeTransaction();
    void publish(slave::SlaveEvent events);
    SpiSlaveAccessor* borrowAccessor();
    void returnAccessor();
    void closeAccessorGate();
    void detachAccessorAfterWorkerStopped();
    void closeCallbackGate();
    bool waitForCallbackDrain(uint32_t timeout_ms);
    error::error_t abortOperation();
    void deleteStoppedWorker();
    void forceDeleteWorker();
    bool waitForWorkerStop(uint32_t timeout_ms);
    error::error_t stopDriver();
    void freeBounceBuffers();
    void abandonDriverOwnedStorage();

    ::spi_host_device_t _host = SPI2_HOST;
    bool _configured          = false;
    bool _driver_active       = false;
    bool _driver_enabled      = false;
    bool _broken              = false;

    uint8_t* _tx_bounce = nullptr;
    uint8_t* _rx_bounce = nullptr;

    // Exactly one descriptor is outstanding. It and both buffers outlive the
    // driver queue even when the master never asserts CS.
    CallbackState* _callback_state = nullptr;
    bool _tx_frame_queued          = false;
    size_t _tx_payload_bytes       = 0;
    size_t _tx_frame_bytes         = 0;
    uint32_t _frame_id             = 0;

    TaskHandle_t _worker_task = nullptr;
    std::atomic<WorkerState> _worker_state{WorkerState::Stopped};
    std::atomic<bool> _accepting{false};
    static constexpr uint32_t kCallbackGateClosed = uint32_t{1} << 31;
    static constexpr uint32_t kCallbackGateCount  = ~kCallbackGateClosed;
    std::atomic<error::error_t> _worker_error{error::error_t::OK};

    // The worker owns the immutable operation data it needs. The accessor is
    // caller-owned and is only visible while the close gate is open. Closing
    // the gate and draining the finite borrow count makes detachment safe.
    spi::SlaveAccessConfig _access_config{};
    uint32_t _generation = 0;
    std::atomic<SpiSlaveAccessor*> _accessor{nullptr};
    std::atomic<bool> _accessor_gate_open{false};
    std::atomic<uint32_t> _accessor_borrows{0};
};

// The slave backend is constructed directly and is not part of the pooled
// hardware-controller facade.

}  // namespace m5::hal::v2::spi

#endif  // ESP_PLATFORM || host harness

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP
