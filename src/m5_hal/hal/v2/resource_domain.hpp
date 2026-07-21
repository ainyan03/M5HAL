// SPDX-License-Identifier: MIT

#ifndef M5_HAL_RESOURCE_DOMAIN_HPP_
#define M5_HAL_RESOURCE_DOMAIN_HPP_

#include "bus/native_binding.hpp"
#include "bus/registry.hpp"
#include "bus/resource_interner.hpp"
#include "gpio/group.hpp"
#include "memory/allocator.hpp"
#include "service/service.hpp"

#include <cstdint>
#include <memory>
#include <new>

namespace m5::hal::v2 {

namespace bus {
class LocalBackend;
}

namespace detail {

/*! @brief Co-owned storage behind a local ResourceDomain facade. */
struct DomainState {
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity> native_identities;
    bus::BusRegistry registry;
    gpio::GPIOGroup gpio;
    service::ServiceRunner services;
    memory::Allocator memory;
    std::shared_ptr<bus::IPathInterner> paths;
    std::weak_ptr<void> local_connection;
    runtime::Mutex local_connection_mutex;

    explicit DomainState(memory::FallbackOps fallback = {}, std::shared_ptr<bus::IPathInterner> path_interner = {})
        : memory{fallback}, paths{std::move(path_interner)}
    {
        gpio.bindServiceRunner(&services);
    }

    ~DomainState()
    {
        // GPIOGroup and its poll service belong to the domain, not to any
        // individual Hal facade. Stop callbacks before member destruction.
        gpio.clearWatchers();
    }

    DomainState(const DomainState&)            = delete;
    DomainState& operator=(const DomainState&) = delete;
};

struct DomainStateDeleter {
    void operator()(DomainState* state) const
    {
        if (state == nullptr) {
            return;
        }
        state->~DomainState();
        void* raw = *(reinterpret_cast<void**>(state) - 1);
        ::operator delete(raw);
    }
};

inline std::shared_ptr<DomainState> makeDomainState(memory::FallbackOps fallback              = {},
                                                    std::shared_ptr<bus::IPathInterner> paths = {})
{
    constexpr size_t alignment = alignof(DomainState);
    static_assert((alignment & (alignment - 1u)) == 0u, "DomainState alignment must be a power of two");

    // Some embedded libstdc++ ports declare aligned new but cannot link it
    // (ESP8266 lacks memalign). Over-allocate through ordinary new, retain the
    // original pointer immediately before the aligned object, and let the
    // shared owner use the matching ordinary delete.
    void* raw                              = ::operator new(sizeof(DomainState) + alignment - 1u + sizeof(void*));
    const uintptr_t low                    = reinterpret_cast<uintptr_t>(raw) + sizeof(void*);
    const uintptr_t at                     = (low + alignment - 1u) & ~(static_cast<uintptr_t>(alignment) - 1u);
    auto* state                            = reinterpret_cast<DomainState*>(at);
    *(reinterpret_cast<void**>(state) - 1) = raw;
    new (state) DomainState{fallback, std::move(paths)};
    return std::shared_ptr<DomainState>{state, DomainStateDeleter{}};
}

}  // namespace detail

/*! @brief Local resource namespace and injectable backend dependencies.

  A facade is cheap to copy and co-owns its state. Buses created in this
  domain retain the same state, so GPIO/service/memory dependencies and the
  registry outlive a stack-local facade. Domain identity is structural: two
  independently constructed domains may use identical ResourceKey values
  without interning the same Bus.
 */
class ResourceDomain {
public:
    ResourceDomain() : _state{detail::makeDomainState()}
    {
    }
    explicit ResourceDomain(memory::FallbackOps fallback) : _state{detail::makeDomainState(fallback)}
    {
    }
    explicit ResourceDomain(std::shared_ptr<bus::IPathInterner> paths)
        : _state{detail::makeDomainState({}, std::move(paths))}
    {
    }
    ResourceDomain(memory::FallbackOps fallback, std::shared_ptr<bus::IPathInterner> paths)
        : _state{detail::makeDomainState(fallback, std::move(paths))}
    {
    }

    gpio::GPIOGroup& gpio(void) const
    {
        return _state->gpio;
    }
    service::ServiceRunner& services(void) const
    {
        return _state->services;
    }
    memory::Allocator& memory(void) const
    {
        return _state->memory;
    }
    bus::BusRegistry& busRegistry(void) const
    {
        return _state->registry;
    }
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& nativeInterner(void) const
    {
        return _state->native_identities;
    }

    bus::LocalResourceContext localResources(void) const
    {
        return {&_state->gpio, &_state->services, &_state->memory, _state};
    }
    bus::IPathInterner* pathInterner(void) const
    {
        return _state->paths.get();
    }

    bool sharesStateWith(const ResourceDomain& other) const
    {
        return _state == other._state;
    }

private:
    std::shared_ptr<detail::DomainState> _state;

    friend class Hal;
    friend class bus::LocalBackend;
};

}  // namespace m5::hal::v2

#endif  // M5_HAL_RESOURCE_DOMAIN_HPP_
