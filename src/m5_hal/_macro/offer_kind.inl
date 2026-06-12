// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable, no include guard. The per-kind alias emitter behind
// offer_all.inl: every bus kind dispatches here with parameter macros
// instead of repeating the namespace plumbing. What CANNOT move here
// stays in offer_all.inl — the kind-specific guard / marker macros
// (M5HAL_VARIANT_PLATFORM_<KIND>_BOUND_, M5HAL_V1_SELECTED_VARIANT_<KIND>)
// and their #elif chains, because #define / #ifdef cannot compute macro
// names ([026]).
//
// Inputs (set by offer_all.inl, all consumed/undeffed here):
//   M5HAL_OFFER_KIND_NS_            — kind namespace leaf (e.g. gpio): both the
//                                     variant's source namespace and the
//                                     m5::hal::v1 target use the same leaf
//   M5HAL_OFFER_KIND_EMIT_PLATFORM_ — defined: first platform hit for this kind,
//                                     also bind <kind>::variant::platform
//   M5HAL_OFFER_KIND_EMIT_FLAT_     — defined: overall first hit for this kind,
//                                     also flat-inject into m5::hal::v1::<kind>
// Plus the current _offer.hpp's M5HAL_VARIANT_CURRENT_ALIAS_ /
// M5HAL_VARIANT_CURRENT_BASE_NS_ (consumed later by offer_all.inl).

// m5::hal::v1::<kind>::variant::<ALIAS> — always created.
namespace m5 { namespace hal { namespace v1 { namespace M5HAL_OFFER_KIND_NS_ { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::M5HAL_OFFER_KIND_NS_;
}
} } } } }

#ifdef M5HAL_OFFER_KIND_EMIT_PLATFORM_
#undef M5HAL_OFFER_KIND_EMIT_PLATFORM_
namespace m5 { namespace hal { namespace v1 { namespace M5HAL_OFFER_KIND_NS_ { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::M5HAL_OFFER_KIND_NS_;
}
} } } } }
#endif

#ifdef M5HAL_OFFER_KIND_EMIT_FLAT_
#undef M5HAL_OFFER_KIND_EMIT_FLAT_
// Flat injection of the first hit (relies on m5::hal::v1::<kind> not
// containing a class named identically to one in the variant).
namespace m5 { namespace hal { namespace v1 { namespace M5HAL_OFFER_KIND_NS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::M5HAL_OFFER_KIND_NS_;
} } } }
#endif

#undef M5HAL_OFFER_KIND_NS_
