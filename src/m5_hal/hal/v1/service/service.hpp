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

    using tick_nsec_t = uint32_t;
    using fast_tick_t = uint32_t;

    struct ServiceContext {
        // 32-bit comparable tick. Wrap-around is intentional; a runner may
        // pass nanoseconds or raw fast ticks, but every service must keep its
        // due values in the same unit as this field and compare only with
        // elapsedNsec()/hasReached().
        tick_nsec_t now_nsec = 0;
    };

    constexpr tick_nsec_t kMaxComparableDelayNsec = 0x7FFFFFFFu;

    constexpr tick_nsec_t elapsedNsec(tick_nsec_t now_nsec, tick_nsec_t since_nsec)
    {
        return now_nsec - since_nsec;
    }

    constexpr bool hasReached(tick_nsec_t now_nsec, tick_nsec_t due_nsec)
    {
        return elapsedNsec(now_nsec, due_nsec) <= kMaxComparableDelayNsec;
    }

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
#if defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_HAS_ESP_CLK_CPU_FREQ_)
        const auto freq_hz = esp_clk_cpu_freq();
        if (freq_hz > 0) {
            return static_cast<uint32_t>(freq_hz);
        }
#endif
#if defined(F_CPU) && F_CPU > 0
        return static_cast<uint32_t>(F_CPU);
#elif defined(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) && CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ > 0
        return static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) * uint32_t{1000000};
#else
        return uint32_t{1000000};
#endif
    }

    inline tick_nsec_t fastTickNsec()
    {
        const static uint32_t frequency_hz = fastTickFrequencyHz();
        return fastTickToNsec(fastTick(), frequency_hz);
    }

    inline tick_nsec_t defaultNowNsec()
    {
        return fastTickNsec();
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
            return true;
        }

        void clear()
        {
            for (size_t i = 0; i < _count; ++i) {
                _services[i] = nullptr;
            }
            _count = 0;
        }

        bool runOnce(const ServiceContext& ctx)
        {
            bool progressed    = false;
            const size_t count = _count;
            for (size_t i = 0; i < count; ++i) {
                auto* s = _services[i];
                if (s == nullptr) {
                    continue;
                }
                const auto r = s->service(ctx);
                progressed   = progressed || r == ServiceResult::Progress || r == ServiceResult::Done;
            }
            return progressed;
        }

        bool runOnce(tick_nsec_t now_nsec)
        {
            return runOnce(ServiceContext{now_nsec});
        }

        bool runOnce()
        {
            return runOnce(defaultNowNsec());
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
