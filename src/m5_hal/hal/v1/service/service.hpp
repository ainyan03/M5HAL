// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V1_SERVICE_SERVICE_HPP_
#define M5_HAL_HAL_V1_SERVICE_SERVICE_HPP_

#include "../../../../m5_hal_config.hpp"  // M5HAL_INLINE_V1

#include <M5Utility.hpp>

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
M5HAL_INLINE_V1 namespace v1
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
        // exactly (S16 D2), which is why the context carries the raw
        // tick and durations are converted the other way
        // (nsecToFastTickCeil) instead.
        fast_tick_t now_tick = 0;
    };

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
      ticks (S16 D2).
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

    inline fast_tick_t fastTick()
    {
#if defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_HAS_ESP_CPU_H_)
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
        return static_cast<fast_tick_t>(esp_cpu_get_cycle_count());
#else
        return static_cast<fast_tick_t>(esp_cpu_get_ccount());
#endif
#else
        return static_cast<fast_tick_t>(m5::utility::micros());
#endif
    }

    inline uint32_t fastTickFrequencyHz()
    {
        // Branch on the SAME condition as fastTick(): the frequency must
        // describe whatever counter fastTick() actually reads.
#if defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_HAS_ESP_CPU_H_)
        // fastTick() reads the CPU cycle counter, so the frequency is the
        // CPU clock.
#if defined(M5HAL_SERVICE_HAS_ESP_CLK_CPU_FREQ_)
        const auto freq_hz = esp_clk_cpu_freq();
        if (freq_hz > 0) {
            return static_cast<uint32_t>(freq_hz);
        }
#endif
        // F_CPU and CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ are ESP-specific; keeping
        // them inside the ESP_PLATFORM guard prevents non-ESP targets (e.g. AVR)
        // that also define F_CPU from returning a CPU frequency here while
        // fastTick() actually returns micros() (1 MHz) — a guaranteed mismatch.
#if defined(F_CPU) && F_CPU > 0
        return static_cast<uint32_t>(F_CPU);
#elif defined(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) && CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ > 0
        return static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) * uint32_t{1000000};
#else
        return uint32_t{240000000};  // last-resort default for ESP without clock query
#endif
#else
        // Everywhere else (non-ESP, or ESP without esp_cpu.h) fastTick()
        // returns micros(), which is always 1 MHz regardless of the CPU
        // clock. F_CPU is intentionally not consulted here.
        return uint32_t{1000000};
#endif
    }

    /*! @brief Default ServiceContext clock: the raw fastTick() count.
        (The former defaultNowNsec()/fastTickNsec() converted the absolute
        tick through fastTickToNsec, inheriting its wrap discontinuity —
        removed, S16 D2.) */
    inline fast_tick_t defaultNowTick()
    {
        return fastTick();
    }

    class IService {
    public:
        virtual ~IService()                                      = default;
        virtual ServiceResult service(const ServiceContext& ctx) = 0;
    };

    class ServiceRunner {
    public:
        static constexpr size_t kMaxServices = 16;

        bool add(IService& service)
        {
            if (find(&service) != kMaxServices || _count >= kMaxServices) {
                return false;
            }
            _services[_count++] = &service;
            return true;
        }

        bool remove(IService& service)
        {
            const size_t index = find(&service);
            if (index == kMaxServices) {
                return false;
            }
            for (size_t i = index + 1; i < _count; ++i) {
                _services[i - 1] = _services[i];
            }
            --_count;
            _services[_count] = nullptr;
            // Removal from inside runOnce (typically a service removing
            // itself): the compaction shifted the not-yet-visited tail
            // down by one, so hold the cursor back to not skip the next
            // service this pass.
            if (_iter_index != kMaxServices && index <= _iter_index) {
                --_iter_index;
            }
            return true;
        }

        void clear()
        {
            for (size_t i = 0; i < _count; ++i) {
                _services[i] = nullptr;
            }
            _count = 0;
        }

        // Not reentrant: do not call runOnce from inside a service() poll.
        // Services may add/remove (including themselves) during the pass;
        // remove() compensates the cursor, and a service added mid-pass is
        // polled in the same pass (it lands on the not-yet-visited tail).
        bool runOnce(const ServiceContext& ctx)
        {
            bool progressed = false;
            for (_iter_index = 0; _iter_index < _count; ++_iter_index) {
                auto* s = _services[_iter_index];
                if (s == nullptr) {
                    continue;
                }
                const auto r = s->service(ctx);
                progressed   = progressed || r == ServiceResult::Progress || r == ServiceResult::Done;
            }
            _iter_index = kMaxServices;
            return progressed;
        }

        bool runOnce(fast_tick_t now_tick)
        {
            return runOnce(ServiceContext{now_tick});
        }

        bool runOnce()
        {
            return runOnce(defaultNowTick());
        }

        size_t size() const
        {
            return _count;
        }
        size_t capacity() const
        {
            return kMaxServices;
        }

    private:
        size_t find(const IService* service) const
        {
            for (size_t i = 0; i < _count; ++i) {
                if (_services[i] == service) {
                    return i;
                }
            }
            return kMaxServices;
        }

        IService* _services[kMaxServices] = {};
        size_t _count                     = 0;
        // Cursor of the in-progress runOnce pass; kMaxServices = not
        // iterating. remove() adjusts it so mid-pass removal (typically
        // self-removal) does not skip the next service.
        size_t _iter_index = kMaxServices;
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

#endif  // M5_HAL_HAL_V1_SERVICE_SERVICE_HPP_
