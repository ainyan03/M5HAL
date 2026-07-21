// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable, no include guard. The per-kind winner emitter behind
// offer_all.inl: every kind dispatches here with parameter macros
// instead of repeating the namespace plumbing. What CANNOT move here
// stays in offer_all.inl — the kind-specific marker macros
// (M5HAL_V2_SELECTED_VARIANT_<KIND>) and their #elif chains, because
// #define / #ifdef cannot compute macro names.
//
// Inputs (set by offer_all.inl, all consumed/undeffed here):
//   M5HAL_OFFER_KIND_NS_         — kind namespace leaf (e.g. i2c)
//   M5HAL_OFFER_KIND_EMIT_FLAT_  — defined: overall first hit for this kind;
//                                  emit the winner aliases below
//   M5HAL_OFFER_KIND_GPIO_       — defined: gpio-shaped kind (Port / GPIO /
//                                  getMCUGPIO / getGPIO) instead of Bus /
//                                  BusConfig
//   M5HAL_OFFER_KIND_RUNTIME_    — defined: runtime facility kind; the only
//                                  kind still injected via `using namespace`
//                                  (free functions + Mutex, see runtime.md)
//   M5HAL_OFFER_KIND_FACADE_     — defined: the unsuffixed `Bus` is a runtime
// facade class in the kind header, so the winner binds only its portable
// factory and NativeProvider (emitting `using Bus` would redefine the
// facade). Set for all facade-backed
//                                  kinds (i2c / spi / i2s / uart).
// Plus the current _offer.hpp's M5HAL_VARIANT_CURRENT_ALIAS_ (the variant
// short name, used as the provider-symbol suffix) and
// M5HAL_VARIANT_CURRENT_BASE_NS_ (used by the runtime injection only).
//
// Winner binding contract: facade bus variants define `Bus_<variant>`,
// `makePortableBackend_<variant>`, and `NativeProvider_<variant>` directly in
// m5::hal::v2::<kind>. The first eligible variant binds those provider seams;
// public `Bus` and portable `BusConfig` remain kind-level types. GPIO and the
// non-facade legacy path continue to use the aliases emitted below.

#ifdef M5HAL_OFFER_KIND_EMIT_FLAT_
#undef M5HAL_OFFER_KIND_EMIT_FLAT_

#define M5HAL_OFFER_PASTE2_(a, b) a##b
#define M5HAL_OFFER_PASTE_(a, b) M5HAL_OFFER_PASTE2_(a, b)

namespace m5 { namespace hal { namespace v2 { namespace M5HAL_OFFER_KIND_NS_ {

#if defined(M5HAL_OFFER_KIND_RUNTIME_)
    // runtime: free functions (millis / micros / delayMs / delayUs) plus
    // the Mutex class — names a type alias cannot carry, so this kind
    // keeps the namespace injection.
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v2::M5HAL_OFFER_KIND_NS_;
#elif defined(M5HAL_OFFER_KIND_GPIO_)
    using Port = M5HAL_OFFER_PASTE_(Port_, M5HAL_VARIANT_CURRENT_ALIAS_);
    using GPIO = M5HAL_OFFER_PASTE_(GPIO_, M5HAL_VARIANT_CURRENT_ALIAS_);
    inline const IGPIO* getMCUGPIO(void)
    {
        return M5HAL_OFFER_PASTE_(getMCUGPIO_, M5HAL_VARIANT_CURRENT_ALIAS_)();
    }
    inline const IGPIO* getGPIO(void)
    {
        return M5HAL_OFFER_PASTE_(getGPIO_, M5HAL_VARIANT_CURRENT_ALIAS_)();
    }
#elif defined(M5HAL_OFFER_KIND_FACADE_)
    // Facade kinds expose one portable BusConfig from the kind header.
    // The winner selects a provider, never a configuration type.
    template <class Policy>
    struct NativeProvider : M5HAL_OFFER_PASTE_(NativeProvider_, M5HAL_VARIANT_CURRENT_ALIAS_)<Policy> {};

    inline result_t<std::unique_ptr<IBus>> makeSelectedPortableBackend(
        const bus::LocalResourceContext& resources, const IBusConfig& config)
    {
        return M5HAL_OFFER_PASTE_(makePortableBackend_, M5HAL_VARIANT_CURRENT_ALIAS_)(resources, config);
    }
    inline result_t<void> Bus::init(const IBusConfig& config)
    {
        const auto& resources = bus::defaultLocalResources();
        auto backend = makeSelectedPortableBackend(resources, config);
        if (!backend.has_value()) {
            return m5::stl::make_unexpected(backend.error());
        }
        bindLocalResources(resources);
        return adoptPortableBackend(std::move(backend.value()), config);
    }
#else
    using Bus       = M5HAL_OFFER_PASTE_(Bus_, M5HAL_VARIANT_CURRENT_ALIAS_);
    using BusConfig = M5HAL_OFFER_PASTE_(BusConfig_, M5HAL_VARIANT_CURRENT_ALIAS_);
#endif

} } } }

#undef M5HAL_OFFER_PASTE_
#undef M5HAL_OFFER_PASTE2_

#endif  // M5HAL_OFFER_KIND_EMIT_FLAT_

#ifdef M5HAL_OFFER_KIND_GPIO_
#undef M5HAL_OFFER_KIND_GPIO_
#endif
#ifdef M5HAL_OFFER_KIND_RUNTIME_
#undef M5HAL_OFFER_KIND_RUNTIME_
#endif
#ifdef M5HAL_OFFER_KIND_FACADE_
#undef M5HAL_OFFER_KIND_FACADE_
#endif
#undef M5HAL_OFFER_KIND_NS_
