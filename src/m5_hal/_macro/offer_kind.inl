// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable, no include guard. The per-kind winner emitter behind
// offer_all.inl: every kind dispatches here with parameter macros
// instead of repeating the namespace plumbing. What CANNOT move here
// stays in offer_all.inl — the kind-specific marker macros
// (M5HAL_V1_SELECTED_VARIANT_<KIND>) and their #elif chains, because
// #define / #ifdef cannot compute macro names ([026]).
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
// Plus the current _offer.hpp's M5HAL_VARIANT_CURRENT_ALIAS_ (the variant
// short name, doubling as the `_<variant>` type suffix) and
// M5HAL_VARIANT_CURRENT_BASE_NS_ (used by the runtime injection only).
//
// Winner binding (D17): variants define their concrete types directly in
// m5::hal::v1::<kind> as `Bus_<variant>` / `BusConfig_<variant>` (gpio:
// `Port_<variant>` / `GPIO_<variant>` / `get*GPIO_<variant>`), and the
// first variant to offer a kind wins the unsuffixed name through the
// type aliases emitted here. `i2c::Bus` is therefore one alias hop away
// from its concrete `i2c::Bus_arduino`, every variant stays addressable
// by its suffixed name, and multiple variants coexist in one namespace.

#ifdef M5HAL_OFFER_KIND_EMIT_FLAT_
#undef M5HAL_OFFER_KIND_EMIT_FLAT_

#define M5HAL_OFFER_PASTE2_(a, b) a##b
#define M5HAL_OFFER_PASTE_(a, b) M5HAL_OFFER_PASTE2_(a, b)

namespace m5 { namespace hal { namespace v1 { namespace M5HAL_OFFER_KIND_NS_ {

#if defined(M5HAL_OFFER_KIND_RUNTIME_)
    // runtime: free functions (millis / micros / delayMs / delayUs) plus
    // the Mutex class — names a type alias cannot carry, so this kind
    // keeps the namespace injection.
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::M5HAL_OFFER_KIND_NS_;
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
#undef M5HAL_OFFER_KIND_NS_
