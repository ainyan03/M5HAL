// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SERVICE_SERVICE_HPP_
#define M5_HAL_HAL_V2_SERVICE_SERVICE_HPP_

#include "../../../../m5_hal_config.hpp"  // M5HAL_INLINE_V2

#include "../runtime/runtime.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

#include <atomic>
#if defined(ESP_PLATFORM)
#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif
#if __has_include(<esp_cpu.h>)
#include <esp_cpu.h>
#define M5HAL_SERVICE_HAS_ESP_CPU_H_ 1
#endif
#if __has_include(<esp_private/esp_clk.h>)
#include <esp_private/esp_clk.h>
#define M5HAL_SERVICE_HAS_ESP_CLK_CPU_FREQ_ 1
#endif
#endif

#include <stddef.h>
#include <stdint.h>

// Core placement for the auto-run task (runtime::Task::start `core`
// argument; see types.hpp sentinels). Default: scheduler-chosen
// (TASK_CORE_ANY). The relative-tick contract makes cross-core
// placement safe — elapsed streams carry a per-core domain guard, so a
// runner hopping cores degrades one sample to a gap-drop (elapsed=0,
// always erring toward "less time passed") instead of comparing
// foreign cycle counters. Measured on dual-core ESP32 against a
// SAME-pinned runner: software bit-bang reads show no regression, and
// a consumer that busy-polls transferBusy() from the runner's core no
// longer starves the transfer until endTransaction(). Pin with a core
// id (or types::TASK_CORE_SAME) when the runner must share the
// consumer's clock domain, e.g. under M5HAL_CONFIG_SERVICE_ASSUME_PINNED.
#ifndef M5HAL_CONFIG_SERVICE_AUTORUN_CORE
#define M5HAL_CONFIG_SERVICE_AUTORUN_CORE (::m5::hal::v2::types::TASK_CORE_ANY)
#endif

// Set to 1 ONLY when, for every runner, ALL tasks that default-clock
// drive it (auto-run, manual runOnce(), sole-pumper waits) run on the
// SAME core — or on single-core builds. "Every task is pinned" is NOT
// sufficient: two drivers pinned to different cores would collapse to
// one domain and subtract foreign ticks. fastTickDomain() becomes a
// constant and the per-pass core guard folds away. Default 0
// (correctness side) — the guard costs only a core-id read + compare
// (~a few cycles) and its branch predicts perfectly on pinned tasks.
#ifndef M5HAL_CONFIG_SERVICE_ASSUME_PINNED
#define M5HAL_CONFIG_SERVICE_ASSUME_PINNED 0
#endif

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    namespace service {

    enum class ServiceResult : uint8_t {
        Idle,
        Progress,
        Done,
        Error,
    };

    // `tick_nsec_t` holds NANOSECOND QUANTITIES (durations, configs);
    // `fast_tick_t` holds COMPARABLE TICKS — the raw mod-2^32 counter
    // values that ServiceContext carries and due-time math runs on.
    // The two are layout-identical; the names mark the unit.
    using tick_nsec_t = uint32_t;
    using fast_tick_t = uint32_t;

    struct ServiceContext {
        // Elapsed ticks since THIS service was last polled by the calling
        // poll stream (mod-2^32 duration, NOT an absolute tick). The
        // stream owner measures it; on the ESP32 family fastTick() is a
        // PER-CORE cycle counter, so absolute ticks from different
        // cores/streams are not comparable — the contract carries only
        // relative time, and a stream that cannot vouch for continuity
        // (core changed since its previous reading) reports 0 (gap-drop;
        // the error direction is always "runs late", which is safe for
        // open-drain I2C edges and SPI clock stretching). Converting an
        // elapsed DURATION with fastTickToNsec is fine; there is no
        // absolute tick here to misconvert across the 2^32 wrap.
        fast_tick_t elapsed = 0;
        // Raw tick at the time of this call, for intra-call spin
        // anchoring ONLY. Do not store it, do not compare it across
        // calls, do not mix it with `elapsed` — it is only meaningful
        // within this single poll on the calling core. For services that
        // spin against the live counter (software SPI edge pacing), this
        // must derive from the real fastTick() even under explicit-
        // context driving; services without intra-call spins (software
        // I2C) accept a purely virtual value.
        fast_tick_t local_tick = 0;

        // Not an aggregate on purpose: the pre-relative contract carried a
        // single absolute tick, and `ServiceContext{tick}` call sites must
        // fail to compile rather than silently reinterpret that tick as
        // `elapsed`.
        constexpr ServiceContext() = default;
        constexpr ServiceContext(fast_tick_t elapsed_ticks, fast_tick_t local)
            : elapsed{elapsed_ticks}, local_tick{local}
        {
        }
    };

    struct ServicePoll {
        ServiceResult result = ServiceResult::Idle;
        // Relative scheduling hint: "no progress possible for at least
        // this many ticks from now". 0 means "no scheduling hint" (poll
        // again next pass), matching the previous absolute-due sentinel.
        fast_tick_t next_due_delta = 0;

        constexpr ServicePoll() = default;
        constexpr ServicePoll(ServiceResult r) : result{r}
        {
        }
        constexpr ServicePoll(ServiceResult r, fast_tick_t due_delta) : result{r}, next_due_delta{due_delta}
        {
        }

        constexpr operator ServiceResult() const
        {
            return result;
        }
    };

    constexpr bool operator==(ServicePoll poll, ServiceResult result)
    {
        return poll.result == result;
    }

    constexpr bool operator==(ServiceResult result, ServicePoll poll)
    {
        return poll.result == result;
    }

    constexpr bool operator!=(ServicePoll poll, ServiceResult result)
    {
        return poll.result != result;
    }

    constexpr bool operator!=(ServiceResult result, ServicePoll poll)
    {
        return poll.result != result;
    }

    /*! @brief Half range of the mod-2^32 comparable space: the longest
        forward delay elapsedTicks()/hasReached() can represent. */
    constexpr fast_tick_t kMaxComparableDelayTicks = 0x7FFFFFFFu;

    constexpr fast_tick_t elapsedTicks(fast_tick_t now_tick, fast_tick_t since_tick)
    {
        return now_tick - since_tick;
    }

    constexpr bool hasReached(fast_tick_t now_tick, fast_tick_t due_tick)
    {
        return elapsedTicks(now_tick, due_tick) <= kMaxComparableDelayTicks;
    }

    /*!
      @brief Convert a tick DURATION to nanoseconds.

      Duration use only. Feeding it an absolute tick and comparing the
      results across a wrap is broken by construction: the fixed-point
      factor K = (1e9<<16)/f makes the mapping discontinuous at the
      2^32 boundary unless K is a multiple of 2^16 (at 240 MHz the
      timeline jumps ~0.717 s every ~17.9 s). Absolute time stays in
      ticks.
     */
    constexpr tick_nsec_t fastTickToNsec(fast_tick_t tick, uint32_t frequency_hz)
    {
        return frequency_hz ? static_cast<tick_nsec_t>(
                                  (static_cast<uint64_t>(tick) * ((uint64_t{1000000000} << 16) / frequency_hz)) >> 16)
                            : static_cast<tick_nsec_t>(tick);
    }

    constexpr fast_tick_t nsecToFastTickCeil(tick_nsec_t nsec, uint32_t frequency_hz)
    {
        if (frequency_hz == 0) {
            return static_cast<fast_tick_t>(nsec);
        }
        const uint64_t ticks =
            (static_cast<uint64_t>(nsec) * static_cast<uint64_t>(frequency_hz) + uint64_t{999999999}) /
            uint64_t{1000000000};
        return static_cast<fast_tick_t>((ticks == 0 && nsec != 0) ? 1 : ticks);
    }

    fast_tick_t fastTick();

    uint32_t fastTickFrequencyHz();

    /*! @brief Default poll-stream clock: the raw fastTick() count.
        (The former defaultNowNsec()/fastTickNsec() converted the absolute
        tick through fastTickToNsec, inheriting its wrap discontinuity —
        removed.) */
    fast_tick_t defaultNowTick();

    /*! @brief Identifier of the range within which fastTick() values are
        comparable. Per-core cycle counter targets (multi-core ESP32)
        return the current core id; shared-counter targets (host micros())
        return a constant. Two readings taken under different domain values
        must never be subtracted. */
    uint32_t fastTickDomain();

    constexpr uint32_t kInvalidTickDomain = 0xFFFFFFFFu;

    /*! @brief Cross-core monotonic microseconds, for whole-transfer
        DEADLINES only (never for edge scheduling — reading it costs ~100+
        cycles on ESP32, see esp_timer_get_time). Unlike fastTick() this
        clock is shared between cores, so durations measured across poll
        streams stay valid. Compare as a difference against a saved start
        (sharedNowUs() - start >= budget), never as absolute values. */
    uint64_t sharedNowUs();

    /*! @brief One poll stream's elapsed-time measurement point: (previous
        reading, domain it was taken under). step() returns the elapsed
        ticks since the previous reading, or 0 when the domain changed
        (gap-drop — the previous reading came from a counter the current
        one is not comparable with). Every place that measures elapsed
        against the default clock owns exactly one of these. */
    struct TickStream {
        fast_tick_t prev = 0;
        uint32_t domain  = kInvalidTickDomain;  // first step() always gap-drops

        fast_tick_t step(fast_tick_t now, uint32_t dom)
        {
            // kInvalidTickDomain never matches anything, ITSELF INCLUDED:
            // two consecutive torn samples (see sampleTickWithDomain) must
            // not be treated as comparable just because both were torn.
            const fast_tick_t e =
                (dom == domain && dom != kInvalidTickDomain) ? static_cast<fast_tick_t>(now - prev) : 0;
            prev   = now;
            domain = dom;
            return e;
        }
    };

    /*! @brief One (tick, domain) reading trustworthy as a pair: the domain
        is read on BOTH sides of the tick read, and a mismatch (the task
        migrated cores mid-sample) marks the sample torn
        (kInvalidTickDomain) so TickStream::step gap-drops instead of
        storing core A's tick under core B's domain. */
    struct TickSample {
        fast_tick_t tick = 0;
        uint32_t domain  = kInvalidTickDomain;
    };

    TickSample sampleTickWithDomain();

    class IService {
        friend class ServiceRunner;

    public:
        IService()                           = default;
        virtual ~IService()                  = default;
        IService(const IService&)            = delete;
        IService& operator=(const IService&) = delete;

    private:
        ServicePoll service(const ServiceContext& ctx);

    protected:
        virtual ServicePoll serviceImpl(const ServiceContext& ctx) = 0;
    };

    class ServiceRunner {
    public:
        static constexpr size_t kMaxServices = 16;

        ServiceRunner() = default;
        ~ServiceRunner();
        ServiceRunner(const ServiceRunner&)            = delete;
        ServiceRunner& operator=(const ServiceRunner&) = delete;

        bool add(IService& service);

        // remove() is SYNCHRONOUS: once it returns, the removed service's
        // serviceImpl() is guaranteed not to be called again, so the caller
        // may safely destroy the object right after. While auto-run owns the
        // table this waits until the runner task has observed the removal;
        // when the caller is the runner task itself (a service removing
        // itself or a sibling from inside its own serviceImpl) it applies
        // directly and does not wait.
        // Contract (undefined / forbidden otherwise):
        //  - Do not hold any lock the target's serviceImpl() may take while
        //    calling remove()/clear() (the runner task may need it to finish
        //    the in-flight poll before the removal is observed -> deadlock).
        //  - Concurrent add() and remove() of the SAME service from different
        //    threads is undefined.
        //  - Never wait (directly or via another task) for remove()/clear()
        //    to complete from inside a serviceImpl() -> circular wait.
        bool remove(IService& service);

        // clear() is SYNCHRONOUS like remove(): if auto-run is active it
        // stops (joins) the runner task, drains pending, and empties the
        // table; the runner stays stopped and the next add() restarts it.
        // The remove() contract above applies to clear() as well. Not
        // supported from inside a serviceImpl() (it would join its own task).
        void clear();

        // Concurrent runOnce callers are serialized through the control
        // mutex with a TRY-lock: a contender -- or any caller while auto-run
        // owns the table -- returns false without polling. Do not call
        // runOnce from inside a service() poll (it try-locks the same mutex
        // and returns false).
        // Services may add/remove (including themselves) during the pass;
        // remove() compensates the cursor, and a service added mid-pass is
        // polled in the same pass (it lands on the not-yet-visited tail).
        static ServicePoll run(IService& service, const ServiceContext& ctx);

        // Explicit-context pass (tests / simulation): ctx.elapsed advances
        // the runner's virtual timeline; the default-clock stream is not
        // touched. Do not interleave with default-clock driving (runOnce()
        // or auto-run) on the same runner: the first default-clock pass
        // after mixing gap-drops (elapsed = 0) and time continues on the
        // real clock — state does not corrupt, but timing guarantees are
        // void.
        bool runOnce(const ServiceContext& ctx);

        // Default-clock pass: measures elapsed on the runner's own
        // (prev, domain) stream with fastTick()/fastTickDomain().
        bool runOnce();

        // Approximate: reads the table count with relaxed ordering and does
        // not reflect additions/removals still queued in the pending slots
        // (applied by the runner task on its next pass). Exact only when the
        // caller holds the table still (no auto-run, no pending).
        size_t size() const;
        size_t capacity() const;

        bool startAutoRun();

        void stopAutoRun();

        bool autoRunActive() const;

    private:
        static constexpr size_t kMaxPending = 4;

        // Idle-wait backstop for the auto-run task. The runner normally
        // wakes through _wake (notified by add/remove/stop); this timeout
        // only bounds the damage of a missed notification (liveness, not
        // latency — a short-deadline transaction is already lost by the
        // time it fires). See autoRunLoop().
        static constexpr uint32_t kIdleWakeTimeoutMs = 100;

        void flushPending();

        size_t findInTable(const IService* service) const;

        void applyAdd(IService* service);

        void applyRemove(IService* service);

        static void autoRunEntry(void* arg);

        // refresh_local_each_poll: default-clock passes re-read fastTick()
        // right before EACH service so ctx.local_tick is genuinely
        // call-local (earlier services consume time, and an unpinned task
        // may migrate between polls). Explicit-context passes keep the
        // caller-supplied anchor for determinism.
        bool runOnceInternal(fast_tick_t elapsed_pass, fast_tick_t local_tick, bool refresh_local_each_poll);

        void autoRunLoop();

        // _control-held variants: the caller already owns _control. add() and
        // clear() drive auto-run from inside their own critical section, and
        // the mutex is non-recursive, so the start/stop bodies cannot re-take
        // it. The public startAutoRun()/stopAutoRun() take _control and defer
        // here.
        bool startAutoRunLocked();
        void stopAutoRunLocked();

        // True when the calling thread is the one currently authoritative for
        // direct table writes -- i.e. it is inside this runner's
        // runOnceInternal pass (the auto-run task, or a runOnce() caller).
        // Such a caller is the sole table writer (R1) and MUST NOT take
        // _control: a control thread may hold it while joining that very task
        // (clear/stopAutoRun), and a self-wait would deadlock.
        bool isCurrentWriter() const;

        // CAS a queued add of `service` back out of the pending-add slots.
        // Removing a service must also cancel a still-unconsumed add of it so
        // sequential add()->remove() leaves it absent (atomic; safe with or
        // without _control).
        void cancelPendingAdd(IService* service);

        IService* _services[kMaxServices] = {};
        // Virtual-timeline bookkeeping: _virtual_now advances only by the
        // per-pass elapsed the driving stream vouches for (gap-drops
        // excluded), so every value on this axis is comparable regardless
        // of which core executed which pass. _next_due[] holds absolute
        // positions ON THE VIRTUAL AXIS (skip check is the same
        // hasReached as before); _last_polled[] is each service's
        // previous-poll position, giving ctx.elapsed by subtraction. Both
        // arrays are part of the table invariant: add initializes,
        // remove shifts, clear resets (a stale baseline after a shift
        // would hand a service someone else's elapsed).
        fast_tick_t _next_due[kMaxServices]    = {};
        fast_tick_t _last_polled[kMaxServices] = {};
        fast_tick_t _virtual_now               = 0;
        // Default-clock measurement point. Owned by whichever thread runs
        // the pass; auto-run and manual runOnce() are mutually exclusive
        // (_control + _auto_running), so sharing one stream keeps elapsed
        // continuous across the auto-run <-> manual transition. Only
        // touched once a pass is committed (never on a try-lock back-off:
        // stepping there would advance `prev` and permanently lose that
        // interval from the timeline).
        TickStream _stream;
        std::atomic<size_t> _count{0};
        size_t _iter_index = kMaxServices;
        std::atomic<bool> _has_pending{false};
        std::atomic<IService*> _pending_add[kMaxPending]    = {};
        std::atomic<IService*> _pending_remove[kMaxPending] = {};
        runtime::Task _auto_task;
        std::atomic<bool> _auto_stop{false};
        std::atomic<bool> _auto_running{false};
        // Wakes the auto-run task out of its idle wait. Notified AFTER the
        // state it announces (_has_pending / _auto_stop) is stored and
        // BEFORE _control is released; latching, single waiter (the runner
        // task). Constructed before any task starts; destroyed only after
        // the dtor's stop/join, so no waiter or concurrent notify can
        // outlive it.
        runtime::Event _wake;
        // Counts idle waits that ended by timeout instead of a notify
        // (missed-notification telemetry, reported through M5HAL_DIAG).
        // Written only by the runner task.
        uint32_t _idle_wake_timeouts = 0;
        // Serializes the control plane (add/remove/clear/start/stop/runOnce);
        // the poll hot path (runOnceInternal / serviceImpl) never takes it.
        runtime::Mutex _control;
        // Identity of the thread currently running runOnceInternal (the table
        // owner), or null between passes. Written only by that thread; read by
        // control threads for the self-call check. See isCurrentWriter().
        std::atomic<void*> _writer_id{nullptr};
    };

    }  // namespace service
}
}  // namespace hal
}  // namespace m5

#ifdef M5HAL_SERVICE_HAS_ESP_CPU_H_
#undef M5HAL_SERVICE_HAS_ESP_CPU_H_
#endif
#ifdef M5HAL_SERVICE_HAS_ESP_CLK_CPU_FREQ_
#undef M5HAL_SERVICE_HAS_ESP_CLK_CPU_FREQ_
#endif

#endif  // M5_HAL_HAL_V2_SERVICE_SERVICE_HPP_
