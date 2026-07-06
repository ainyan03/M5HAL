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
        // 32-bit comparable tick; wrap-around is intentional. The runner
        // chooses the unit (the default runner passes raw fastTick()
        // counts; tests may pass plain numbers) — every service must keep
        // its due values in the SAME unit as this field and compare only
        // with elapsedTicks()/hasReached(). Converting an ABSOLUTE tick
        // to nanoseconds is forbidden here: the conversion is not
        // continuous across the 2^32 wrap unless the factor divides
        // exactly, which is why the context carries the raw
        // tick and durations are converted the other way
        // (nsecToFastTickCeil) instead.
        fast_tick_t now_tick = 0;
    };

    struct ServicePoll {
        ServiceResult result = ServiceResult::Idle;
        // 0 means "no scheduling hint". Services that cannot progress until
        // a known tick may set this so ServiceRunner can skip cheap-but-noisy
        // early polls.
        fast_tick_t next_due = 0;

        constexpr ServicePoll() = default;
        constexpr ServicePoll(ServiceResult r) : result{r}
        {
        }
        constexpr ServicePoll(ServiceResult r, fast_tick_t due) : result{r}, next_due{due}
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

    /*! @brief Default ServiceContext clock: the raw fastTick() count.
        (The former defaultNowNsec()/fastTickNsec() converted the absolute
        tick through fastTickToNsec, inheriting its wrap discontinuity —
        removed.) */
    fast_tick_t defaultNowTick();

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

        bool remove(IService& service);

        void clear();

        // Concurrent runOnce callers are serialized by _run_mutex; a contender
        // returns false without polling. Do not call runOnce from inside a
        // service() poll.
        // Services may add/remove (including themselves) during the pass;
        // remove() compensates the cursor, and a service added mid-pass is
        // polled in the same pass (it lands on the not-yet-visited tail).
        static ServicePoll run(IService& service, const ServiceContext& ctx);

        static ServicePoll run(IService& service, fast_tick_t now_tick);

        bool runOnce(const ServiceContext& ctx);

        bool runOnce(fast_tick_t now_tick);

        bool runOnce();

        size_t size() const;
        size_t capacity() const;

        bool startAutoRun();

        void stopAutoRun();

        bool autoRunActive() const;

    private:
        static constexpr size_t kMaxPending = 4;

        void flushPending();

        size_t findInTable(const IService* service) const;

        void applyAdd(IService* service);

        void applyRemove(IService* service);

        static void autoRunEntry(void* arg);

        bool runOnceInternal(const ServiceContext& ctx);

        void autoRunLoop();

        IService* _services[kMaxServices]   = {};
        fast_tick_t _next_due[kMaxServices] = {};
        size_t _count                       = 0;
        size_t _iter_index                  = kMaxServices;
        std::atomic<bool> _has_pending{false};
        std::atomic<IService*> _pending_add[kMaxPending]    = {};
        std::atomic<IService*> _pending_remove[kMaxPending] = {};
        runtime::Task _auto_task;
        std::atomic<bool> _auto_stop{false};
        std::atomic<bool> _auto_running{false};
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
