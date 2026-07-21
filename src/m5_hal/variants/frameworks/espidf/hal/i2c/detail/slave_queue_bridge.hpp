// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_QUEUE_BRIDGE_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_QUEUE_BRIDGE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#ifndef M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_
#define M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_       __attribute__((always_inline))
#define M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_LOCAL_ 1
#endif

namespace m5::hal::v2::i2c::detail {

enum class SlaveQueueBridgeState : uint8_t { Idle, Active, Closing, Quiesced };
enum class SlaveQueueBridgeResult : uint8_t { Accepted, NoSpace, Rejected, Broken };
enum class SlaveQueueWorkerStep : uint8_t { Empty, Ready, RxBufferTooSmall, Rejected, Broken };

enum class SlaveQueueRawEventKind : uint8_t {
    RxPayload,
    TxLoadedReal,
    TxLoadedFill,
    FifoOccupancy,
    StopBoundary,
    TxEmptyBoundary,
    AbortBoundary,
};

struct SlaveQueueRawEvent {
    SlaveQueueRawEventKind kind = SlaveQueueRawEventKind::RxPayload;
    uint32_t generation         = 0;
    uint32_t count              = 0;
    uint32_t value              = 0;
    uint32_t sequence           = 0;
};

struct SlaveQueueMutableSpan {
    uint8_t* data = nullptr;
    size_t size   = 0;
};

struct SlaveQueueRxPrepareResult {
    SlaveQueueBridgeResult status = SlaveQueueBridgeResult::Rejected;
    SlaveQueueMutableSpan first{};
    SlaveQueueMutableSpan second{};
};

struct SlaveQueueTxPrepareResult {
    SlaveQueueBridgeResult status = SlaveQueueBridgeResult::Rejected;
    size_t count                  = 0;
    size_t real                   = 0;
    size_t fill                   = 0;
};

/*!
  @brief Byte-mode-only fixed-storage ISR-to-worker I2C slave bridge.

  Address, repeated-START and Frame semantics are deliberately outside this
  prototype. There is one serialized ISR producer and one worker consumer.
  Every capacity is a power of two below 2^31, so monotonically increasing
  uint32_t indices are wrap-safe while their live distance stays bounded by a
  ring capacity. Bounded 32-bit atomic builtins provide the memory ordering
  without allocator, result_t, Accessor, or common queue in ISR code. Targets
  with native atomics use lock-free instructions; supported single-core targets
  may use ESP-IDF's interrupt-masked helpers instead.

  isrBegin() obtains a move-only lease. Its lifetime covers prepare, the actual
  hardware FIFO read/write, and commit or cancel. Closing atomically shuts the
  lease gate and quiescence waits for all earlier leases to leave.

  RX reserves event and payload capacity before exposing up to two writable
  ring spans. commitRx() publishes the bytes; cancel() leaves all indices
  unchanged. TX prepare copies real/fill bytes without advancing ownership.
  commitTxLoaded() is called only after the hardware write succeeds.

  workerPeekStep() never consumes. The worker first copies/replays the fact,
  then calls workerCommitStep(); a common RX queue WOULD_BLOCK can therefore
  retry the same staged payload. A move-only WorkerSession covers the complete
  common-queue/ledger iteration, including external Accessor work. Force close
  rejects new worker sessions but waits for the accepted iteration to leave.
  TX real bytes remain owned until ledger replay confirms them. A boundary
  rewinds unclocked real bytes for retransmission.
 */
template <size_t EventCapacity, size_t RxCapacity, size_t TxCapacity>
class SlaveQueueBridge {
public:
    static_assert(EventCapacity >= 2 && (EventCapacity & (EventCapacity - 1)) == 0,
                  "event capacity must be a power of two with a boundary reserve");
    static_assert(RxCapacity != 0 && (RxCapacity & (RxCapacity - 1)) == 0, "RX capacity must be a power of two");
    static_assert(TxCapacity != 0 && (TxCapacity & (TxCapacity - 1)) == 0, "TX capacity must be a power of two");
    static_assert(EventCapacity < (uint64_t{1} << 31), "event capacity must be below 2^31");
    static_assert(RxCapacity < (uint64_t{1} << 31), "RX capacity must be below 2^31");
    static_assert(TxCapacity < (uint64_t{1} << 31), "TX capacity must be below 2^31");
    static_assert(sizeof(uint32_t) == 4 && alignof(uint32_t) >= 4, "ISR state requires aligned 32-bit words");

    class IsrSession {
    public:
        IsrSession(const IsrSession&)            = delete;
        IsrSession& operator=(const IsrSession&) = delete;

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ IsrSession(IsrSession&& other) noexcept
        {
            moveFrom(other);
        }

        IsrSession& operator=(IsrSession&& other) noexcept
        {
            if (this != &other) {
                release();
                moveFrom(other);
            }
            return *this;
        }

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ ~IsrSession() noexcept
        {
            release();
        }

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult status() const noexcept
        {
            return status_;
        }

    private:
        friend class SlaveQueueBridge;
        enum class Pending : uint8_t { None, Rx, Tx };

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ IsrSession(SlaveQueueBridge* owner, uint32_t generation) noexcept
            : owner_(owner), generation_(generation)
        {
            status_   = owner_->acquireSession(generation_);
            acquired_ = status_ == SlaveQueueBridgeResult::Accepted;
        }

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ void moveFrom(IsrSession& other) noexcept
        {
            owner_          = other.owner_;
            generation_     = other.generation_;
            status_         = other.status_;
            acquired_       = other.acquired_;
            pending_        = other.pending_;
            start_          = other.start_;
            count_          = other.count_;
            real_           = other.real_;
            fill_           = other.fill_;
            event_need_     = other.event_need_;
            other.owner_    = nullptr;
            other.acquired_ = false;
            other.pending_  = Pending::None;
        }

        M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ void release() noexcept
        {
            if (owner_ != nullptr && acquired_) {
                if (pending_ != Pending::None) {
                    owner_->markBroken();
                }
                pending_ = Pending::None;
                storeRelease(owner_->isr_busy_, 0);
                fetchSub(owner_->isr_gate_, 1);
            }
            owner_    = nullptr;
            acquired_ = false;
        }

        SlaveQueueBridge* owner_       = nullptr;
        uint32_t generation_           = 0;
        SlaveQueueBridgeResult status_ = SlaveQueueBridgeResult::Rejected;
        bool acquired_                 = false;
        Pending pending_               = Pending::None;
        uint32_t start_                = 0;
        uint32_t count_                = 0;
        uint32_t real_                 = 0;
        uint32_t fill_                 = 0;
        uint32_t event_need_           = 0;
    };

    class WorkerSession {
    public:
        WorkerSession(const WorkerSession&)            = delete;
        WorkerSession& operator=(const WorkerSession&) = delete;
        WorkerSession(WorkerSession&& other) noexcept
        {
            moveFrom(other);
        }
        WorkerSession& operator=(WorkerSession&& other) noexcept
        {
            if (this != &other) {
                release();
                moveFrom(other);
            }
            return *this;
        }
        ~WorkerSession() noexcept
        {
            release();
        }
        SlaveQueueBridgeResult status() const noexcept
        {
            return status_;
        }

    private:
        friend class SlaveQueueBridge;
        WorkerSession(SlaveQueueBridge* owner, uint32_t generation) noexcept : owner_(owner), generation_(generation)
        {
            status_   = owner_->acquireWorkerSession(generation_, lease_token_);
            acquired_ = status_ == SlaveQueueBridgeResult::Accepted;
        }
        void moveFrom(WorkerSession& other) noexcept
        {
            owner_             = other.owner_;
            generation_        = other.generation_;
            lease_token_       = other.lease_token_;
            status_            = other.status_;
            acquired_          = other.acquired_;
            other.owner_       = nullptr;
            other.acquired_    = false;
            other.lease_token_ = 0;
        }
        void release() noexcept
        {
            if (owner_ != nullptr && acquired_) {
                owner_->releaseWorkerSession(lease_token_);
            }
            owner_    = nullptr;
            acquired_ = false;
        }
        SlaveQueueBridge* owner_       = nullptr;
        uint32_t generation_           = 0;
        uint32_t lease_token_          = 0;
        SlaveQueueBridgeResult status_ = SlaveQueueBridgeResult::Rejected;
        bool acquired_                 = false;
    };

    bool workerBegin(uint32_t generation, uint8_t fill_byte) noexcept
    {
        if (generation == 0 || workerState() != SlaveQueueBridgeState::Idle) {
            return false;
        }
        resetIndices();
        fill_byte_     = fill_byte;
        worker_peeked_ = false;
        boundary_kind_ = SlaveQueueRawEventKind::AbortBoundary;
        storeRelaxed(broken_, 0);
        storeRelaxed(isr_gate_, 0);
        storeRelaxed(isr_busy_, 0);
        storeRelaxed(worker_gate_, 0);
        storeRelaxed(worker_busy_, 0);
        storeRelease(generation_, generation);
        storeRelease(state_, encode(SlaveQueueBridgeState::Active));
        return true;
    }

    bool workerRequestClose(uint32_t generation) noexcept
    {
        if (loadAcquire(generation_) != generation || workerState() != SlaveQueueBridgeState::Active) {
            return false;
        }
        storeRelease(state_, encode(SlaveQueueBridgeState::Closing));
        fetchOr(isr_gate_, kIsrClosed);
        return true;
    }

    bool workerTryQuiesce(uint32_t generation) noexcept
    {
        if (loadAcquire(generation_) != generation || workerState() != SlaveQueueBridgeState::Closing) {
            return false;
        }
        if (loadAcquire(isr_gate_) != kIsrClosed || loadAcquire(event_read_) != loadAcquire(event_write_) ||
            loadAcquire(boundary_ready_) != 0 || loadAcquire(tx_paused_) != 0 || loadAcquire(boundary_required_) != 0) {
            return false;
        }
        const uint32_t gate = fetchOr(worker_gate_, kIsrClosed);
        if ((gate & kIsrRefMask) != 0) return false;
        storeRelease(state_, encode(SlaveQueueBridgeState::Quiesced));
        return true;
    }

    // Hard teardown after graceful replay can no longer complete. This never
    // touches caller-owned common queues. Broken remains observable until the
    // explicit workerReset() that follows endpoint detachment.
    bool workerForceQuiesce(uint32_t generation) noexcept
    {
        if (loadAcquire(generation_) != generation || workerState() != SlaveQueueBridgeState::Closing) {
            return false;
        }
        const uint32_t gate = fetchOr(worker_gate_, kIsrClosed);
        if ((gate & kIsrRefMask) != 0 || loadAcquire(isr_gate_) != kIsrClosed) return false;
        storeRelease(event_read_, loadAcquire(event_write_));
        storeRelease(rx_read_, loadAcquire(rx_write_));
        const uint32_t tail = loadAcquire(tx_tail_);
        storeRelease(tx_head_, tail);
        storeRelease(tx_load_, tail);
        storeRelease(boundary_ready_, 0);
        storeRelease(tx_paused_, 0);
        storeRelease(boundary_required_, 0);
        worker_peeked_ = false;
        markBroken();
        storeRelease(state_, encode(SlaveQueueBridgeState::Quiesced));
        return true;
    }

    // Emergency teardown after the producer task has already been stopped or
    // externally deleted.  The caller must first stop every producer capable of
    // owning a WorkerSession.  Closing must also have fenced ISR acquisition,
    // and every previously accepted ISR session must have left.  Only under
    // those preconditions may the worker lease count be discarded.
    bool workerAbandonAfterProducerStopped(uint32_t generation) noexcept
    {
        if (loadAcquire(generation_) != generation || workerState() != SlaveQueueBridgeState::Closing ||
            loadAcquire(isr_gate_) != kIsrClosed || loadAcquire(isr_busy_) != 0) {
            return false;
        }
        storeRelease(worker_gate_, kIsrClosed);
        storeRelease(worker_busy_, 0);
        storeRelease(event_read_, loadAcquire(event_write_));
        storeRelease(rx_read_, loadAcquire(rx_write_));
        const uint32_t tail = loadAcquire(tx_tail_);
        storeRelease(tx_head_, tail);
        storeRelease(tx_load_, tail);
        storeRelease(boundary_ready_, 0);
        storeRelease(tx_paused_, 0);
        storeRelease(boundary_required_, 0);
        worker_peeked_ = false;
        markBroken();
        storeRelease(state_, encode(SlaveQueueBridgeState::Quiesced));
        return true;
    }

    // Last-resort recovery after the interrupt source and producer task have
    // both been stopped externally. With no code left that can acquire a new
    // session, all leases and staged state can be invalidated independent of a
    // partially corrupted state/generation. release()/re-init uses this seam so
    // a failed operation close cannot strand the bridge in Closing forever.
    void workerRecoverAfterAllProducersStopped() noexcept
    {
        storeRelease(isr_gate_, kIsrClosed);
        storeRelease(worker_gate_, kIsrClosed);
        storeRelease(isr_busy_, 0);
        storeRelease(worker_busy_, 0);
        storeRelease(event_read_, loadAcquire(event_write_));
        storeRelease(rx_read_, loadAcquire(rx_write_));
        const uint32_t tail = loadAcquire(tx_tail_);
        storeRelease(tx_head_, tail);
        storeRelease(tx_load_, tail);
        storeRelease(boundary_ready_, 0);
        storeRelease(tx_paused_, 0);
        storeRelease(boundary_required_, 0);
        worker_peeked_ = false;
        markBroken();
        storeRelease(state_, encode(SlaveQueueBridgeState::Quiesced));
    }

    bool workerReset() noexcept
    {
        if (workerState() != SlaveQueueBridgeState::Quiesced) {
            return false;
        }
        resetIndices();
        worker_peeked_ = false;
        storeRelaxed(broken_, 0);
        storeRelaxed(isr_gate_, kIsrClosed);
        storeRelaxed(isr_busy_, 0);
        storeRelaxed(worker_gate_, kIsrClosed);
        storeRelaxed(worker_busy_, 0);
        storeRelease(generation_, 0);
        storeRelease(state_, encode(SlaveQueueBridgeState::Idle));
        return true;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeState workerState() const noexcept
    {
        return decode(loadAcquire(state_));
    }
    uint32_t generation() const noexcept
    {
        return loadAcquire(generation_);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ bool broken() const noexcept
    {
        return loadAcquire(broken_) != 0;
    }
    bool txPaused() const noexcept
    {
        return loadAcquire(tx_paused_) != 0;
    }
    bool boundaryRequired() const noexcept
    {
        return loadAcquire(boundary_required_) != 0;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ IsrSession isrBegin(uint32_t generation) noexcept
    {
        return IsrSession(this, generation);
    }

    WorkerSession workerBeginSession(uint32_t generation) noexcept
    {
        return WorkerSession(this, generation);
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueRxPrepareResult prepareRx(IsrSession& session,
                                                                                 size_t count) noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return {usable, {}, {}};
        if (session.pending_ != IsrSession::Pending::None || count == 0 || count > RxCapacity) {
            return misuseRx(session);
        }
        if (!hasNormalEventSpace(1)) return {SlaveQueueBridgeResult::NoSpace, {}, {}};
        const uint32_t read  = loadAcquire(rx_read_);
        const uint32_t write = loadRelaxed(rx_write_);
        if (count > RxCapacity - distance(write, read)) return {SlaveQueueBridgeResult::NoSpace, {}, {}};

        const size_t first =
            count < RxCapacity - (write & (RxCapacity - 1)) ? count : RxCapacity - (write & (RxCapacity - 1));
        session.pending_ = IsrSession::Pending::Rx;
        session.start_   = write;
        session.count_   = static_cast<uint32_t>(count);
        return {SlaveQueueBridgeResult::Accepted,
                {&rx_bytes_[write & (RxCapacity - 1)], first},
                {rx_bytes_.data(), count - first}};
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult commitRx(IsrSession& session) noexcept
    {
        const auto usable = validatePending(session, IsrSession::Pending::Rx);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (loadRelaxed(rx_write_) != session.start_ || !hasNormalEventSpace(1)) return internalBroken(session);
        storeRelease(rx_write_, session.start_ + session.count_);
        publishEvent({SlaveQueueRawEventKind::RxPayload, session.generation_, session.count_, 0, session.start_});
        clearPending(session);
        return SlaveQueueBridgeResult::Accepted;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueTxPrepareResult prepareTx(IsrSession& session, uint8_t* output,
                                                                                 size_t count) noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return {usable, 0, 0, 0};
        if (session.pending_ != IsrSession::Pending::None || output == nullptr || count == 0 ||
            count > std::numeric_limits<uint32_t>::max()) {
            return misuseTx(session);
        }
        if (loadAcquire(tx_paused_) != 0) return {SlaveQueueBridgeResult::Rejected, 0, 0, 0};
        const uint32_t load       = loadRelaxed(tx_load_);
        const uint32_t tail       = loadAcquire(tx_tail_);
        const size_t real         = count < distance(tail, load) ? count : distance(tail, load);
        const size_t fill         = count - real;
        const uint32_t event_need = static_cast<uint32_t>((real != 0) + (fill != 0));
        if (!hasNormalEventSpace(event_need)) return {SlaveQueueBridgeResult::NoSpace, 0, 0, 0};
        for (size_t i = 0; i < real; ++i) output[i] = tx_bytes_[(load + i) & (TxCapacity - 1)];
        for (size_t i = real; i < count; ++i) output[i] = fill_byte_;
        session.pending_    = IsrSession::Pending::Tx;
        session.start_      = load;
        session.count_      = static_cast<uint32_t>(count);
        session.real_       = static_cast<uint32_t>(real);
        session.fill_       = static_cast<uint32_t>(fill);
        session.event_need_ = event_need;
        return {SlaveQueueBridgeResult::Accepted, count, real, fill};
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult commitTxLoaded(IsrSession& session) noexcept
    {
        const auto usable = validatePending(session, IsrSession::Pending::Tx);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (loadRelaxed(tx_load_) != session.start_ || !hasNormalEventSpace(session.event_need_)) {
            return internalBroken(session);
        }
        storeRelease(tx_load_, session.start_ + session.real_);
        if (session.real_ != 0) {
            publishEvent({SlaveQueueRawEventKind::TxLoadedReal, session.generation_, session.real_, 0, 0});
        }
        if (session.fill_ != 0) {
            publishEvent({SlaveQueueRawEventKind::TxLoadedFill, session.generation_, session.fill_, 0, 0});
        }
        storeRelease(boundary_required_, 1);
        clearPending(session);
        return SlaveQueueBridgeResult::Accepted;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult cancel(IsrSession& session) noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (session.pending_ == IsrSession::Pending::None) return SlaveQueueBridgeResult::Rejected;
        clearPending(session);
        return SlaveQueueBridgeResult::Accepted;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult observeFifo(IsrSession& session,
                                                                                uint32_t occupancy) noexcept
    {
        return publishSimple(session, SlaveQueueRawEventKind::FifoOccupancy, occupancy);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult stopBoundary(IsrSession& session,
                                                                                 uint32_t occupancy) noexcept
    {
        return publishBoundary(session, SlaveQueueRawEventKind::StopBoundary, occupancy);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult txEmptyBoundary(IsrSession& session,
                                                                                    uint32_t occupancy) noexcept
    {
        return publishBoundary(session, SlaveQueueRawEventKind::TxEmptyBoundary, occupancy);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult abortBoundary(IsrSession& session,
                                                                                  uint32_t occupancy) noexcept
    {
        return publishBoundary(session, SlaveQueueRawEventKind::AbortBoundary, occupancy);
    }

    SlaveQueueBridgeResult workerStageTx(WorkerSession& session, const uint8_t* data, size_t count) noexcept
    {
        const auto usable = validateWorkerSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (workerState() != SlaveQueueBridgeState::Active || broken()) return rejectionStatus();
        if (count == 0) return SlaveQueueBridgeResult::Accepted;
        if (data == nullptr) return markAndReturnBroken();
        if (count > TxCapacity) return SlaveQueueBridgeResult::NoSpace;
        const uint32_t head = loadAcquire(tx_head_);
        const uint32_t tail = loadRelaxed(tx_tail_);
        if (count > TxCapacity - distance(tail, head)) return SlaveQueueBridgeResult::NoSpace;
        for (size_t i = 0; i < count; ++i) tx_bytes_[(tail + i) & (TxCapacity - 1)] = data[i];
        storeRelease(tx_tail_, tail + static_cast<uint32_t>(count));
        return SlaveQueueBridgeResult::Accepted;
    }

    SlaveQueueWorkerStep workerPeekStep(WorkerSession& session, SlaveQueueRawEvent& output,
                                        uint8_t* rx_output = nullptr, size_t rx_capacity = 0) noexcept
    {
        if (validateWorkerSession(session) != SlaveQueueBridgeResult::Accepted) return SlaveQueueWorkerStep::Rejected;
        const uint32_t read = loadRelaxed(event_read_);
        if (read == loadAcquire(event_write_)) return SlaveQueueWorkerStep::Empty;
        const auto event = events_[read & (EventCapacity - 1)];
        output           = event;
        if (event.kind == SlaveQueueRawEventKind::RxPayload) {
            if (rx_output == nullptr || rx_capacity < event.count) return SlaveQueueWorkerStep::RxBufferTooSmall;
            const uint32_t rx_read = loadRelaxed(rx_read_);
            if (event.sequence != rx_read || event.count > distance(loadAcquire(rx_write_), rx_read)) {
                markBroken();
                return SlaveQueueWorkerStep::Broken;
            }
            for (uint32_t i = 0; i < event.count; ++i) rx_output[i] = rx_bytes_[(rx_read + i) & (RxCapacity - 1)];
        }
        worker_peeked_    = true;
        worker_peek_read_ = read;
        return SlaveQueueWorkerStep::Ready;
    }

    SlaveQueueBridgeResult workerCommitStep(WorkerSession& session) noexcept
    {
        const auto usable = validateWorkerSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        const uint32_t read = loadRelaxed(event_read_);
        if (read == loadAcquire(event_write_) || !worker_peeked_ || worker_peek_read_ != read) {
            return SlaveQueueBridgeResult::Rejected;
        }
        const auto event = events_[read & (EventCapacity - 1)];
        if (event.generation != session.generation_) return markAndReturnBroken();
        if (event.kind == SlaveQueueRawEventKind::RxPayload) {
            const uint32_t rx_read = loadRelaxed(rx_read_);
            if (event.sequence != rx_read || event.count > distance(loadAcquire(rx_write_), rx_read)) {
                return markAndReturnBroken();
            }
            storeRelease(rx_read_, rx_read + event.count);
        }
        if (isBoundary(event.kind)) {
            boundary_kind_ = event.kind;
            storeRelease(boundary_ready_, 1);
        }
        worker_peeked_ = false;
        storeRelease(event_read_, read + 1);
        return SlaveQueueBridgeResult::Accepted;
    }

    SlaveQueueBridgeResult workerConfirmTxReal(WorkerSession& session, uint32_t count) noexcept
    {
        const auto usable = validateWorkerSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        const uint32_t head = loadRelaxed(tx_head_);
        if (count > distance(loadAcquire(tx_load_), head)) {
            markBroken();
            return SlaveQueueBridgeResult::Broken;
        }
        storeRelease(tx_head_, head + count);
        return SlaveQueueBridgeResult::Accepted;
    }

    SlaveQueueBridgeResult workerResolveTxBoundary(WorkerSession& session, uint32_t confirmed_real,
                                                   uint32_t unclocked_real) noexcept
    {
        const auto usable = validateWorkerSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (loadAcquire(tx_paused_) == 0 || loadAcquire(boundary_ready_) == 0) return SlaveQueueBridgeResult::Rejected;
        const uint32_t head = loadRelaxed(tx_head_);
        const uint32_t load = loadAcquire(tx_load_);
        if (static_cast<uint64_t>(distance(load, head)) != static_cast<uint64_t>(confirmed_real) + unclocked_real ||
            (boundary_kind_ == SlaveQueueRawEventKind::TxEmptyBoundary && unclocked_real != 0)) {
            markBroken();
            return SlaveQueueBridgeResult::Broken;
        }
        const auto confirmed = workerConfirmTxReal(session, confirmed_real);
        if (confirmed != SlaveQueueBridgeResult::Accepted) return confirmed;
        storeRelease(tx_load_, loadAcquire(tx_head_));
        clearBoundary();
        return SlaveQueueBridgeResult::Accepted;
    }

    SlaveQueueBridgeResult workerDiscardTxCache(WorkerSession& session) noexcept
    {
        const auto usable = validateWorkerSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (loadAcquire(tx_paused_) == 0 || loadAcquire(boundary_ready_) == 0 ||
            boundary_kind_ != SlaveQueueRawEventKind::AbortBoundary)
            return SlaveQueueBridgeResult::Rejected;
        const uint32_t tail = loadAcquire(tx_tail_);
        storeRelease(tx_head_, tail);
        storeRelease(tx_load_, tail);
        clearBoundary();
        return SlaveQueueBridgeResult::Accepted;
    }

    size_t workerTxRetained() const noexcept
    {
        return distance(loadAcquire(tx_tail_), loadAcquire(tx_head_));
    }

    size_t workerTxWritable() const noexcept
    {
        return TxCapacity - workerTxRetained();
    }

private:
    static constexpr uint32_t kIsrClosed  = uint32_t{1} << 31;
    static constexpr uint32_t kIsrRefMask = kIsrClosed - 1;

    static uint32_t encode(SlaveQueueBridgeState value) noexcept
    {
        return static_cast<uint32_t>(value);
    }
    static SlaveQueueBridgeState decode(uint32_t value) noexcept
    {
        return static_cast<SlaveQueueBridgeState>(value);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static uint32_t distance(uint32_t newer, uint32_t older) noexcept
    {
        return newer - older;
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static uint32_t loadRelaxed(const uint32_t& value) noexcept
    {
        return __atomic_load_n(&value, __ATOMIC_RELAXED);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static uint32_t loadAcquire(const uint32_t& value) noexcept
    {
        return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static void storeRelaxed(uint32_t& target, uint32_t value) noexcept
    {
        __atomic_store_n(&target, value, __ATOMIC_RELAXED);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static void storeRelease(uint32_t& target, uint32_t value) noexcept
    {
        __atomic_store_n(&target, value, __ATOMIC_RELEASE);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static uint32_t fetchSub(uint32_t& target, uint32_t value) noexcept
    {
        return __atomic_fetch_sub(&target, value, __ATOMIC_ACQ_REL);
    }
    static uint32_t fetchAdd(uint32_t& target, uint32_t value) noexcept
    {
        return __atomic_fetch_add(&target, value, __ATOMIC_ACQ_REL);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static uint32_t fetchOr(uint32_t& target, uint32_t value) noexcept
    {
        return __atomic_fetch_or(&target, value, __ATOMIC_ACQ_REL);
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ static bool compareExchange(uint32_t& target, uint32_t& expected,
                                                                         uint32_t desired) noexcept
    {
        return __atomic_compare_exchange_n(&target, &expected, desired, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult acquireSession(uint32_t generation) noexcept
    {
        if (broken()) return SlaveQueueBridgeResult::Broken;
        if (workerState() != SlaveQueueBridgeState::Active || loadAcquire(generation_) != generation) {
            return SlaveQueueBridgeResult::Rejected;
        }
        uint32_t gate = loadAcquire(isr_gate_);
        do {
            if ((gate & kIsrClosed) != 0 || (gate & kIsrRefMask) == kIsrRefMask)
                return SlaveQueueBridgeResult::Rejected;
        } while (!compareExchange(isr_gate_, gate, gate + 1));
        uint32_t idle = 0;
        if (!compareExchange(isr_busy_, idle, 1)) {
            fetchSub(isr_gate_, 1);
            return SlaveQueueBridgeResult::Rejected;
        }
        if (workerState() == SlaveQueueBridgeState::Active && loadAcquire(generation_) == generation && !broken()) {
            return SlaveQueueBridgeResult::Accepted;
        }
        storeRelease(isr_busy_, 0);
        fetchSub(isr_gate_, 1);
        return rejectionStatus();
    }

    SlaveQueueBridgeResult acquireWorkerSession(uint32_t generation, uint32_t& lease_token) noexcept
    {
        const auto state = workerState();
        if ((state != SlaveQueueBridgeState::Active && state != SlaveQueueBridgeState::Closing) ||
            loadAcquire(generation_) != generation)
            return SlaveQueueBridgeResult::Rejected;
        uint32_t gate = loadAcquire(worker_gate_);
        do {
            if ((gate & kIsrClosed) != 0 || (gate & kIsrRefMask) == kIsrRefMask)
                return SlaveQueueBridgeResult::Rejected;
        } while (!compareExchange(worker_gate_, gate, gate + 1));
        uint32_t token = fetchAdd(worker_lease_epoch_, 1) + 1;
        if (token == 0) {
            token = fetchAdd(worker_lease_epoch_, 1) + 1;
        }
        uint32_t idle = 0;
        if (!compareExchange(worker_busy_, idle, token)) {
            fetchSub(worker_gate_, 1);
            return SlaveQueueBridgeResult::Rejected;
        }
        const auto verified = workerState();
        if ((verified == SlaveQueueBridgeState::Active || verified == SlaveQueueBridgeState::Closing) &&
            loadAcquire(generation_) == generation && (loadAcquire(worker_gate_) & kIsrClosed) == 0) {
            lease_token = token;
            return SlaveQueueBridgeResult::Accepted;
        }
        releaseWorkerSession(token);
        return SlaveQueueBridgeResult::Rejected;
    }

    void releaseWorkerSession(uint32_t lease_token) noexcept
    {
        // This bridge-internal token, unlike the Accessor operation generation,
        // is not reset by workerReset(). Abandon/recovery clears the busy slot,
        // so a stale destructor cannot decrement a later session's gate even
        // when a new Accessor reuses the same operation generation.
        uint32_t expected = lease_token;
        if (compareExchange(worker_busy_, expected, 0)) {
            fetchSub(worker_gate_, 1);
        }
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult
    validateSession(const IsrSession& session) const noexcept
    {
        if (session.owner_ != this || !session.acquired_) return SlaveQueueBridgeResult::Rejected;
        return broken() ? SlaveQueueBridgeResult::Broken : SlaveQueueBridgeResult::Accepted;
    }

    SlaveQueueBridgeResult validateWorkerSession(const WorkerSession& session) const noexcept
    {
        if (session.owner_ != this || !session.acquired_ || loadAcquire(generation_) != session.generation_ ||
            loadAcquire(worker_busy_) != session.lease_token_) {
            return SlaveQueueBridgeResult::Rejected;
        }
        const auto state = workerState();
        return state == SlaveQueueBridgeState::Active || state == SlaveQueueBridgeState::Closing
                   ? SlaveQueueBridgeResult::Accepted
                   : SlaveQueueBridgeResult::Rejected;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult
    validatePending(const IsrSession& session, typename IsrSession::Pending pending) const noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        return session.pending_ == pending ? SlaveQueueBridgeResult::Accepted : SlaveQueueBridgeResult::Rejected;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueRxPrepareResult misuseRx(IsrSession&) noexcept
    {
        markBroken();
        return {SlaveQueueBridgeResult::Broken, {}, {}};
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueTxPrepareResult misuseTx(IsrSession&) noexcept
    {
        markBroken();
        return {SlaveQueueBridgeResult::Broken, 0, 0, 0};
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult internalBroken(IsrSession&) noexcept
    {
        return markAndReturnBroken();
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult markAndReturnBroken() noexcept
    {
        markBroken();
        return SlaveQueueBridgeResult::Broken;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ void clearPending(IsrSession& session) noexcept
    {
        session.pending_    = IsrSession::Pending::None;
        session.count_      = 0;
        session.real_       = 0;
        session.fill_       = 0;
        session.event_need_ = 0;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult publishSimple(IsrSession& session,
                                                                                  SlaveQueueRawEventKind kind,
                                                                                  uint32_t value) noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (session.pending_ != IsrSession::Pending::None) return markAndReturnBroken();
        if (loadAcquire(tx_paused_) != 0) return SlaveQueueBridgeResult::Rejected;
        if (!hasNormalEventSpace(1)) return SlaveQueueBridgeResult::NoSpace;
        publishEvent({kind, session.generation_, 0, value, 0});
        return SlaveQueueBridgeResult::Accepted;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult publishBoundary(IsrSession& session,
                                                                                    SlaveQueueRawEventKind kind,
                                                                                    uint32_t value) noexcept
    {
        const auto usable = validateSession(session);
        if (usable != SlaveQueueBridgeResult::Accepted) return usable;
        if (session.pending_ != IsrSession::Pending::None) return markAndReturnBroken();
        if (loadAcquire(tx_paused_) != 0) return SlaveQueueBridgeResult::Rejected;
        if (!hasBoundaryEventSpace()) return SlaveQueueBridgeResult::NoSpace;
        storeRelease(tx_paused_, 1);
        publishEvent({kind, session.generation_, 0, value, 0});
        return SlaveQueueBridgeResult::Accepted;
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ SlaveQueueBridgeResult rejectionStatus() const noexcept
    {
        return broken() ? SlaveQueueBridgeResult::Broken : SlaveQueueBridgeResult::Rejected;
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ void markBroken() noexcept
    {
        storeRelease(broken_, 1);
    }

    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ bool hasNormalEventSpace(uint32_t count) const noexcept
    {
        const uint32_t used = distance(loadRelaxed(event_write_), loadAcquire(event_read_));
        return used <= EventCapacity - 1 && count <= (EventCapacity - 1) - used;
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ bool hasBoundaryEventSpace() const noexcept
    {
        return distance(loadRelaxed(event_write_), loadAcquire(event_read_)) < EventCapacity;
    }
    M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_ void publishEvent(const SlaveQueueRawEvent& event) noexcept
    {
        const uint32_t write                 = loadRelaxed(event_write_);
        events_[write & (EventCapacity - 1)] = event;
        storeRelease(event_write_, write + 1);
    }
    static bool isBoundary(SlaveQueueRawEventKind kind) noexcept
    {
        return kind == SlaveQueueRawEventKind::StopBoundary || kind == SlaveQueueRawEventKind::TxEmptyBoundary ||
               kind == SlaveQueueRawEventKind::AbortBoundary;
    }
    void clearBoundary() noexcept
    {
        storeRelease(boundary_ready_, 0);
        storeRelease(tx_paused_, 0);
        storeRelease(boundary_required_, 0);
    }
    void resetIndices() noexcept
    {
        storeRelaxed(event_read_, 0);
        storeRelaxed(event_write_, 0);
        storeRelaxed(rx_read_, 0);
        storeRelaxed(rx_write_, 0);
        storeRelaxed(tx_head_, 0);
        storeRelaxed(tx_load_, 0);
        storeRelaxed(tx_tail_, 0);
        storeRelaxed(tx_paused_, 0);
        storeRelaxed(boundary_ready_, 0);
        storeRelaxed(boundary_required_, 0);
    }

    std::array<SlaveQueueRawEvent, EventCapacity> events_{};
    std::array<uint8_t, RxCapacity> rx_bytes_{};
    std::array<uint8_t, TxCapacity> tx_bytes_{};
    uint8_t fill_byte_                    = 0xFF;
    SlaveQueueRawEventKind boundary_kind_ = SlaveQueueRawEventKind::AbortBoundary;
    uint32_t state_ = encode(SlaveQueueBridgeState::Idle), generation_ = 0, broken_ = 0;
    uint32_t isr_gate_ = kIsrClosed, isr_busy_ = 0, worker_gate_ = kIsrClosed, worker_busy_ = 0;
    uint32_t worker_lease_epoch_ = 0;
    uint32_t event_read_ = 0, event_write_ = 0, rx_read_ = 0, rx_write_ = 0;
    uint32_t tx_head_ = 0, tx_load_ = 0, tx_tail_ = 0, tx_paused_ = 0, boundary_ready_ = 0;
    uint32_t boundary_required_ = 0, worker_peek_read_ = 0;
    bool worker_peeked_ = false;
};

}  // namespace m5::hal::v2::i2c::detail

#if defined(M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_LOCAL_)
#undef M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_LOCAL_
#undef M5HAL_DETAIL_I2C_SLAVE_QUEUE_ISR_INLINE_
#endif

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_QUEUE_BRIDGE_HPP_
