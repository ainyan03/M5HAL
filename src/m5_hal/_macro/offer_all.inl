// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable. M5HAL_v1.hpp re-includes this after every _offer.hpp
// during variant scanning: consumes the M5HAL_VARIANT_CURRENT_*_ macros,
// emits namespace aliases for offered HALs, then undefs every consumed
// macro so the next pass starts clean.
//
// Inputs (from the just-included _offer.hpp):
//   M5HAL_VARIANT_CURRENT_ALIAS_   — variant short name used as alias namespace (e.g. arduino)
//   M5HAL_VARIANT_CURRENT_BASE_NS_ — variant base namespace path (e.g. frameworks::arduino)
//   M5HAL_VARIANT_CURRENT_HAS_HAL_*_ — capability flag(s)
// Optional input (set by the M5HAL_v1.hpp scan loop, not the _offer.hpp):
//   M5HAL_VARIANT_PLATFORM_          — 1 only while scanning a platform variant
//
// Aliases generated when M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ is set:
//   m5::hal::v1::gpio::variant::<ALIAS>   — always created
//   m5::hal::v1::gpio::variant::platform  — only while scanning a platform variant, first hit only
//   m5::hal::v1::gpio                     — flat injection of the first hit across all variants
//                                            (relies on
//                                            m5::hal::v1::gpio not containing a class
//                                            named identically to one in the variant)
//
// Note: v0/v1 coexistence — target / source namespaces both pass through
// the explicit ::v1:: sub namespace. The shared bus base classes live in
// m5::hal::v1::bus::*. v0 (when offered via
// inline namespace v0) provides its own legacy structure and is not
// touched by this scan.

// ---------------------------------------------------------------------
// hal/gpio
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_) && M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

namespace m5 { namespace hal { namespace v1 { namespace gpio { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::gpio;
}
} } } } }  // namespace m5::hal::v1::gpio::variant

#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_GPIO_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_GPIO_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace gpio { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::gpio;
}
} } } } }
#  endif

#  ifndef M5HAL_VARIANT_OFFER_GPIO_BOUND_
#    define M5HAL_VARIANT_OFFER_GPIO_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace gpio {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::gpio;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

// ---------------------------------------------------------------------
// hal/i2c
//
// Source and target namespaces are symmetric: each variant nests its
// HAL kinds directly under the variant base namespace (e.g.
// m5::variants::frameworks::software::hal::v1::i2c), and the flat injection
// target is m5::hal::v1::i2c. There is no intermediate "bus" hop on either
// side — the shared bus base classes live in m5::hal::v1::bus::* but i2c
// concrete types do not pass through that namespace.
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_) && M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_

namespace m5 { namespace hal { namespace v1 { namespace i2c { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2c;
}
} } } } }  // namespace m5::hal::v1::i2c::variant

#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_I2C_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_I2C_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace i2c { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2c;
}
} } } } }
#  endif

#  ifndef M5HAL_VARIANT_OFFER_I2C_BOUND_
#    define M5HAL_VARIANT_OFFER_I2C_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace i2c {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2c;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_

// ---------------------------------------------------------------------
// hal/spi
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_) && M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_

namespace m5 { namespace hal { namespace v1 { namespace spi { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::spi;
}
} } } } }  // namespace m5::hal::v1::spi::variant

#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_SPI_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_SPI_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace spi { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::spi;
}
} } } } }
#  endif

#  ifndef M5HAL_VARIANT_OFFER_SPI_BOUND_
#    define M5HAL_VARIANT_OFFER_SPI_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace spi {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::spi;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_

// ---------------------------------------------------------------------
// hal/i2s
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_) && M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_

namespace m5 { namespace hal { namespace v1 { namespace i2s { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2s;
}
} } } } }  // namespace m5::hal::v1::i2s::variant

#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_I2S_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_I2S_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace i2s { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2s;
}
} } } } }
#  endif

#  ifndef M5HAL_VARIANT_OFFER_I2S_BOUND_
#    define M5HAL_VARIANT_OFFER_I2S_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace i2s {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::i2s;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_

// ---------------------------------------------------------------------
// hal/uart
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_UART_) && M5HAL_VARIANT_CURRENT_HAS_HAL_UART_

namespace m5 { namespace hal { namespace v1 { namespace uart { namespace variant {
namespace M5HAL_VARIANT_CURRENT_ALIAS_ {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::uart;
}
} } } } }  // namespace m5::hal::v1::uart::variant

#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_UART_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_UART_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace uart { namespace variant {
namespace platform {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::uart;
}
} } } } }
#  endif

#  ifndef M5HAL_VARIANT_OFFER_UART_BOUND_
#    define M5HAL_VARIANT_OFFER_UART_BOUND_ 1
namespace m5 { namespace hal { namespace v1 { namespace uart {
    using namespace ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v1::uart;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_UART_

// ---------------------------------------------------------------------
// cleanup: undef every macro the just-completed _offer.hpp may have
// declared, so the next pass starts clean.
// ---------------------------------------------------------------------
#undef M5HAL_VARIANT_CURRENT_ALIAS_
#undef M5HAL_VARIANT_CURRENT_BASE_NS_
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_UART_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_UART_
#endif
