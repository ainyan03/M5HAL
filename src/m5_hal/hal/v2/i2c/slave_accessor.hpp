// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_HPP_
#define M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_HPP_

#include "../service/service.hpp"
#include "../slave/event.hpp"
#include "../slave/queue.hpp"
#include "slave.hpp"
#include "slave_frame.hpp"

namespace m5::hal::v2::i2c {

/*!
  @brief Long-lived I2C slave endpoint backed by caller-owned SPSC queues.

  `beginAccess()` starts accepting START-to-STOP frames and `endAccess()`
  stops slave acceptance before releasing the bus. Local queue I/O remains
  available while inactive for TX preload and post-stop RX drain. Exact I2C
  segment details, when a backend can observe them, are consumed atomically
  with their common frame descriptor through `rxFrames()`.
 */
class SlaveAccessor : public bus::IAccessor {
public:
    SlaveAccessor(ISlaveBus& bus, slave::QueueStorage<> tx_storage, slave::QueueStorage<> rx_storage,
                  I2cSegmentStorage segment_storage, const SlaveAccessConfig& config = {});

    const SlaveAccessConfig& getConfig() const override;
    result_t<void> setConfig(const SlaveAccessConfig& config);
    ISlaveBus& getBus() const;

    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    bool inAccess() const;

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);
    size_t readable() const;
    size_t writable() const;

    slave::FrameSinkView txFrames();
    I2cFrameSourceView rxFrames();
    result_t<void> clearTx();
    result_t<void> clearRx();
    slave::QueueStatus txStatus() const;
    slave::QueueStatus rxStatus() const;
    slave::QueueStatus clearTxStatus();
    slave::QueueStatus clearRxStatus();

    result_t<void> setEventCallback(slave::SlaveEventCallback callback, void* user);
    result_t<void> dispatchEvents();
    result_t<void> acknowledgeEvents(slave::SlaveEvent events);
    service::IService& eventService();

    // Backend endpoints. Frame mode is available only through the combined
    // writer so common descriptors and I2C detail cannot be advanced apart.
    slave::SlaveQueue& backendTxQueue();
    result_t<size_t> backendWriteRx(data::ConstDataSpan src);
    size_t backendRxWritable() const;
    void backendRecordDroppedBytes(uint32_t dropped_bytes);
    void backendRecordDroppedFrame(uint32_t dropped_bytes);
    I2cObservedFrameWriter backendRxFrames();
    slave::SlaveEventEndpoint& backendEvents();

private:
    static slave::SlaveEventInfo probeEventLevel(void* user, uint32_t generation);

    bus::OperationContext<SlaveAccessConfig> context_;
    slave::SlaveQueue tx_queue_{};
    slave::SlaveQueue rx_queue_{};
    I2cSegmentQueue segment_queue_{};
    slave::SlaveEventEndpoint events_{};
    error::error_t queue_bind_error_ = error::error_t::OK;
};

}  // namespace m5::hal::v2::i2c

#include "slave_accessor.inl"

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_HPP_
