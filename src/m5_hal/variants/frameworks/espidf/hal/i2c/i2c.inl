// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include "slave.hpp"  // defines M5HAL_ESPIDF_I2C_SLAVE_LL used by the gate below
#include "../../../../../hal/v2/resource_domain.hpp"

#include <new>

// The slave backend is provided whenever the SoC can drive a real SCL stretch
// (LL stretch primitive, e.g. ESP32-S3), the LL best-effort flavor (classic
// ESP32, no stretch cause), or the v2 slave driver is configured (fallback
// fill-only path). slave.hpp derives M5HAL_ESPIDF_I2C_SLAVE_LL /
// M5HAL_ESPIDF_I2C_SLAVE_LL_BE.
#if defined(ESP_PLATFORM) && \
    (M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE || M5HAL_ESPIDF_I2C_HAS_SLAVE_V2)
#include "slave.inl"
#endif

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
#include "backend_master_gen5.inl"
#elif defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
#include "backend_master_gen4.inl"

#endif

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
namespace m5::hal::v2::i2c {
namespace detail_espidf_i2c_native {

constexpr uint16_t kProvider = 0x0201;

struct PendingToken {
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* interner = nullptr;
    bus::NativeToken token{};

    ~PendingToken()
    {
        if (interner != nullptr && token.valid()) {
            (void)interner->release(token);
        }
    }
    void dismiss()
    {
        interner = nullptr;
        token    = {};
    }
};

uint32_t portableFingerprint(const IBusConfig& cfg)
{
    return static_cast<uint32_t>(static_cast<uint16_t>(cfg.pin_scl)) |
           (static_cast<uint32_t>(static_cast<uint16_t>(cfg.pin_sda)) << 16);
}

result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend& backend, const IBusConfig& cfg,
                                        const bus::NativeIdentity& identity, ::i2c_master_bus_handle_t borrowed_handle)
{
    const auto* domain = backend.localResourceDomain();
    if (domain == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0 || !identity.isValid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto& interner = domain->nativeInterner();
    auto interned  = interner.intern(identity);
    if (!interned.has_value()) {
        return m5::stl::make_unexpected(interned.error());
    }
    PendingToken pending{&interner, interned.value()};
    auto key = bus::ResourceKey::makeToken(types::bus_kind_t::I2C, bus::ResourceTag::Native, interned.value(),
                                           static_cast<uint32_t>(identity.kind));
    if (!key.has_value()) {
        return m5::stl::make_unexpected(key.error());
    }
    bus::BindingDescriptor binding;
    binding.provider       = kProvider;
    binding.ownership      = bus::Ownership::Borrowed;
    binding.native_kind    = bus::NativeBindingKind::Native;
    binding.native         = interned.value();
    binding.config_primary = portableFingerprint(cfg);

    auto acquired = backend.busRegistry().acquireOrFind(
        key.value(), binding,
        [&cfg](const std::shared_ptr<bus::IBus>& existing) -> result_t<void> {
            if (!BusTraits::configCompatible(static_cast<const IBusConfig&>(existing->getConfig()), cfg)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        },
        [&]() -> result_t<std::shared_ptr<bus::IBus>> {
            std::unique_ptr<Bus_espidf> concrete{new (std::nothrow) Bus_espidf()};
            if (!concrete) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            concrete->bindLocalResources(backend.localResources());
            auto adopted = concrete->adoptBorrowedNative(borrowed_handle, cfg, interner, interned.value());
            if (!adopted.has_value()) {
                return m5::stl::make_unexpected(adopted.error());
            }
            pending.dismiss();

            std::shared_ptr<Bus> facade{new (std::nothrow) Bus()};
            if (!facade) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            facade->bindLocalResources(backend.localResources());
            std::unique_ptr<IBus> native_backend{concrete.release()};
            auto facade_adopted = facade->adoptPortableBackend(std::move(native_backend), cfg);
            if (!facade_adopted.has_value()) {
                return m5::stl::make_unexpected(facade_adopted.error());
            }
            return std::shared_ptr<bus::IBus>{std::move(facade)};
        });
    if (!acquired.has_value()) {
        return m5::stl::make_unexpected(acquired.error());
    }
    return std::static_pointer_cast<IBus>(acquired.value());
}

}  // namespace detail_espidf_i2c_native

void Bus_espidf::retainNativeIdentity(
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token)
{
    _native_interner = &interner;
    _native_token    = token;
}

result_t<void> Bus_espidf::releaseNativeIdentity(void)
{
    if (_native_interner != nullptr && _native_token.valid()) {
        auto released = _native_interner->release(_native_token);
        if (!released.has_value()) {
            return released;
        }
    }
    _native_interner = nullptr;
    _native_token    = {};
    return {};
}

result_t<void> Bus_espidf::adoptBorrowedNative(
    ::i2c_master_bus_handle_t bus_handle, const IBusConfig& config,
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token)
{
    auto attached = attachBorrowedNative(bus_handle);
    if (error::isError(attached)) {
        return m5::stl::make_unexpected(attached);
    }
    _config = config;
    retainNativeIdentity(interner, token);
    return {};
}

result_t<std::shared_ptr<IBus>> NativeProvider_espidf<native::Borrowed<NativeMasterBus>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Borrowed<NativeMasterBus> policy)
{
    if (backend.localResourceDomain() == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    auto handle = policy.resource().value;
    if (handle == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto identity = bus::NativeIdentity::make(bus::NativeIdentityKind::ObjectAddress,
                                              {static_cast<uint64_t>(reinterpret_cast<uintptr_t>(handle))});
    if (!identity.has_value()) {
        return m5::stl::make_unexpected(identity.error());
    }
    return detail_espidf_i2c_native::acquire(backend, cfg, identity.value(), handle);
}

}  // namespace m5::hal::v2::i2c
#endif

#endif
