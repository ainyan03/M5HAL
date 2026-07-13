// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/local_backend.hpp"
#include "../../../../../hal/v2/i2c/i2c.hpp"
#include "../../../../../hal/v2/memory/allocator.hpp"
#include "../../../../../hal/v2/service/service.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER

#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
#include <driver/i2c_master.h>
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
#include <driver/i2c.h>
#endif

// LP_I2C pool support: IOMUX pad macros for the fixed-pin chips (C5/C6) and
// the RTC/LP GPIO membership check for the matrix chip (P4).
// Both are guarded by __has_include rather than a chip list, matching the
// codebase's existing optional-dependency convention (see
// espidf/hal/i2c/slave.hpp).
#if M5HAL_ESPIDF_I2C_LP_POOL
#if __has_include(<hal/i2c_ll.h>)
#include <hal/i2c_ll.h>
#endif
#if __has_include(<driver/rtc_io.h>)
#include <driver/rtc_io.h>
#endif
#endif

namespace m5::hal::v2::i2c {

struct BusConfig_espidf : public i2c::IBusConfig {
    // Inherit the tag-pin constructors (Scl / Sda, either order);
    // `i2c_port` keeps its member initializer and is set by assignment.
    using i2c::IBusConfig::IBusConfig;

#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    int i2c_port = -1;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t i2c_port = I2C_NUM_0;
#endif

    constexpr BusConfig_espidf(void) : i2c::IBusConfig{}
    {
    }
};

#if M5HAL_ESPIDF_I2C_LP_POOL
// True once `controller` is an LP_I2C port index. ESP-IDF numbers LP
// instances after all HP instances in the combined SOC_I2C_NUM count (see
// hardwareControllerCountForI2C below), so a simple threshold suffices.
inline bool isLowPowerControllerForI2C(int8_t controller)
{
    return controller >= static_cast<int8_t>(SOC_HP_I2C_NUM);
}
#endif

// ESP-IDF I2C bus. The public Bus_espidf type is stable inside the espidf
// variant; ESP-IDF driver generation differences live in gen4/gen5 backend
// implementations selected by detail/espidf_version.hpp.
class Bus_espidf : public i2c::IBus
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    ,
                   private service::IService
#endif
{
public:
    ~Bus_espidf() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig_espidf. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    result_t<void> init(const BusConfig_espidf& config);
    result_t<void> release(void) override;

    result_t<void> transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

    // This backend drives a dedicated ESP-IDF I2C peripheral. The
    // controller pool assigns the port through BusConfig_espidf::i2c_port; the
    // query API reports it so the resolver's incumbency check and a
    // holder watching for a downgrade both see the live state.
    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
        return static_cast<int8_t>(_controller_port);
#else
        return static_cast<int8_t>(_port);
#endif
    }
    uint32_t maxFrequency(void) const override
    {
#if M5HAL_ESPIDF_I2C_LP_POOL
        if (isLowPowerControllerForI2C(controllerId())) {
            // Hardware measurement on an LP_I2C-capable chip: LP_I2C
            // sustains 400 kHz (probe/read/write all verified). Still
            // respect a LOWER fail-safe ceiling if one is configured (see
            // master_clock_limit.hpp; 0 there means "no declared ceiling").
            constexpr uint32_t kLpMeasuredMaxHz = 400000u;
            if (M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ != 0 && M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ < kLpMeasuredMaxHz) {
                return M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ;
            }
            return kLpMeasuredMaxHz;
        }
#endif
        // The fail-safe master ceiling (master_clock_limit.hpp). 0 there means
        // "no declared ceiling", which matches the base default.
        return M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ;
    }

#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    error::error_t attach(::i2c_master_bus_handle_t bus_handle);
    ::i2c_master_bus_handle_t nativeHandle() const
    {
        return _bus_handle;
    }
#endif
#if !M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t nativePort() const
    {
        return _port;
    }
#endif

private:
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    result_t<void> ensureDevice(const i2c::MasterAccessConfig& cfg);
    result_t<void> removeDevice(void);
    void recoverBusAfterWireFault(error::error_t mapped, uint32_t wire_timeout_ms);

    ::i2c_master_bus_handle_t _bus_handle = nullptr;
    ::i2c_master_dev_handle_t _dev_handle = nullptr;
    bool _owns_bus                        = false;
    // Wire-fault recovery released the bus but could not rebuild it (e.g.
    // transient NO_MEM); transfer() retries the rebuild lazily.
    bool _rebuild_pending      = false;
    bool _dev_async            = false;
    uint16_t _dev_addr         = 0;
    uint32_t _dev_freq         = 0;
    uint32_t _dev_scl_wait_us  = 0;
    bool _dev_address_is_10bit = false;
    // The configured port (gen5 takes it via i2c_master_bus_config_t but keeps
    // no member); cached so controllerId() reports the pool's assignment.
    int _controller_port = -1;
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    static bool onTransferDone(::i2c_master_dev_handle_t dev, const ::i2c_master_event_data_t* evt, void* arg);
    service::ServiceResult serviceTransfer(const service::ServiceContext& ctx);
    void unregisterTransferService(void);
    void clearTransferState(void);

    bus::IAccessor* _transfer_owner = nullptr;
    data::Sink* _transfer_dst       = nullptr;
    memory::TempBuffer _transfer_tx_buf{};
    size_t _transfer_tx_count                         = 0;
    size_t _transfer_rx_count                         = 0;
    bool _transfer_active                             = false;
    bool _transfer_registered                         = false;
    bool _transfer_done                               = true;
    volatile error::error_t _transfer_callback_status = error::error_t::ASYNC_RUNNING;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    ::i2c_port_t _port = I2C_NUM_0;
    bool _installed    = false;
    // Last frequency pushed through i2c_param_config; skips the
    // re-config on every transfer when unchanged (0 = none applied).
    uint32_t _applied_freq = 0;
#endif
    bus::TransferTotals _transfer_totals{};
};

// Facade backend selection: i2c::Bus::init(BusConfig_espidf) -> Bus_espidf.
template <>
struct BackendFor<BusConfig_espidf> {
    using type = Bus_espidf;
};

// hardware backend factory. Builds a Bus_espidf for a logical
// request, binding the leased controller index to the ESP-IDF I2C port. This is
// the only code that knows about i2c_port, so the kind-generic BusView / pool
// stay variant-agnostic. M5HALCore wires this into i2c::BusView when this
// variant provides hardware I2C (M5HAL_DETAIL_I2C_HAS_HARDWARE_BACKEND_ below).
inline i2c::IBus* makeHardwareBackendForI2C(const i2c::LogicalBusConfig& logical, int8_t controller)
{
    auto* backend = new (std::nothrow) Bus_espidf();
    if (backend == nullptr) {
        return nullptr;
    }
    BusConfig_espidf cfg;
    cfg.pin_scl = logical.pin_scl;
    cfg.pin_sda = logical.pin_sda;
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
    cfg.i2c_port = controller;
#elif M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
    cfg.i2c_port = static_cast< ::i2c_port_t>(controller);
#endif
    auto r = backend->init(cfg);
    if (!r.has_value()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

// Silicon budget for I2C on this SoC. On chips with an LP_I2C (C5/C6/P4),
// SOC_I2C_NUM counts HP+LP combined. When the LP pool is active
// (M5HAL_ESPIDF_I2C_LP_POOL) the full combined count IS the silicon budget:
// the LP instance is opt-in-only (see controllerTopologyForI2C below), so
// the eligibility filter -- not a smaller capacity -- keeps automatic
// allocation off it. When the LP pool is NOT active (older IDF, or
// gen4/legacy), the pool must only ever hand out HP controllers: LP_I2C is
// master-only, pin-fixed, and needs an LP clock source, so it cannot serve
// an arbitrary-pin request (ESP-IDF's own port auto-select skips it for the
// same reason) -- SOC_HP_I2C_NUM is the HP-only count on those chips; older
// targets/IDF versions expose only SOC_I2C_NUM, which there means HP count.
inline uint8_t hardwareControllerCountForI2C(void)
{
#if M5HAL_ESPIDF_I2C_LP_POOL
    return static_cast<uint8_t>(SOC_I2C_NUM);
#elif defined(SOC_HP_I2C_NUM)
    return static_cast<uint8_t>(SOC_HP_I2C_NUM);
#elif defined(SOC_I2C_NUM)
    return static_cast<uint8_t>(SOC_I2C_NUM);
#else
    return 2;
#endif
}

#if M5HAL_ESPIDF_I2C_LP_POOL

// Per-controller capability bitmask: every HP controller offers plain
// HARDWARE; the LP_I2C instance additionally offers the opt-in LOW_POWER
// bit, so it never lands under a plain requireHardware()/automatic()
// request (see IAllocationKind::optInCaps).
inline types::backend_caps_t hardwareControllerCapsForI2C(int8_t controller)
{
    return isLowPowerControllerForI2C(controller) ? (caps::HARDWARE | caps::LOW_POWER) : caps::HARDWARE;
}

#if !defined(SOC_LP_GPIO_MATRIX_SUPPORTED) || !SOC_LP_GPIO_MATRIX_SUPPORTED

// Fixed-pin LP_I2C target (C5/C6, a single-element pin domain): the LP
// instance's SDA/SCL are hard-wired to one IOMUX pad pair -- there is no LP
// GPIO matrix to route through.
inline result_t<void> completeLogicalForI2C(LogicalBusConfig& cfg)
{
    if ((cfg.intent.require & caps::LOW_POWER) == 0) {
        // Only requireLowPower() triggers pin auto-fill. A plain
        // preferLowPower() may still fall back to a plain hardware
        // controller at commit time, but this identity is fixed NOW (before
        // that fallback is decided), so filling it in here would be wrong
        // for the fallback case.
        return {};
    }
#if defined(LP_I2C_SDA_IOMUX_PAD) && defined(LP_I2C_SCL_IOMUX_PAD)
    if (cfg.pin_scl < 0 && cfg.pin_sda < 0) {
        cfg.pin_scl = LP_I2C_SCL_IOMUX_PAD;
        cfg.pin_sda = LP_I2C_SDA_IOMUX_PAD;
        return {};
    }
    if (cfg.pin_scl != LP_I2C_SCL_IOMUX_PAD || cfg.pin_sda != LP_I2C_SDA_IOMUX_PAD) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif
    return {};
}

inline bool pinsAllowedForI2C(const LogicalBusConfig& cfg, int8_t controller)
{
    if (!isLowPowerControllerForI2C(controller)) {
        return true;
    }
#if defined(LP_I2C_SDA_IOMUX_PAD) && defined(LP_I2C_SCL_IOMUX_PAD)
    return cfg.pin_scl == LP_I2C_SCL_IOMUX_PAD && cfg.pin_sda == LP_I2C_SDA_IOMUX_PAD;
#else
    // hal/i2c_ll.h was not available at compile time (unexpected: every
    // chip that defines SOC_LP_I2C_SUPPORTED without the GPIO matrix ships
    // these pads) -- fail closed rather than silently accepting any wiring.
    (void)cfg;
    return false;
#endif
}

#else  // SOC_LP_GPIO_MATRIX_SUPPORTED: matrix chip (P4)

// GPIO-matrix LP_I2C target (P4, a multi-element pin domain): SDA/SCL may be
// any RTC/LP GPIO, so there is nothing to auto-fill -- only membership is
// enforced, and only once both pins are given (an omitted pin is left for
// the existing identity-derivation `id.valid()` check to reject, same as
// before this LP_I2C support existed).
inline result_t<void> completeLogicalForI2C(LogicalBusConfig& cfg)
{
    if ((cfg.intent.require & caps::LOW_POWER) == 0) {
        return {};
    }
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0) {
        return {};
    }
#if __has_include(<driver/rtc_io.h>)
    if (!::rtc_gpio_is_valid_gpio(static_cast< ::gpio_num_t>(cfg.pin_scl)) ||
        !::rtc_gpio_is_valid_gpio(static_cast< ::gpio_num_t>(cfg.pin_sda))) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif
    return {};
}

inline bool pinsAllowedForI2C(const LogicalBusConfig& cfg, int8_t controller)
{
    if (!isLowPowerControllerForI2C(controller)) {
        return true;
    }
#if __has_include(<driver/rtc_io.h>)
    return ::rtc_gpio_is_valid_gpio(static_cast< ::gpio_num_t>(cfg.pin_scl)) &&
           ::rtc_gpio_is_valid_gpio(static_cast< ::gpio_num_t>(cfg.pin_sda));
#else
    // driver/rtc_io.h was not available at compile time (unexpected: it
    // ships with esp_driver_gpio, an M5HAL dependency on every target) --
    // fail closed rather than silently accepting any wiring.
    (void)cfg;
    return false;
#endif
}

#endif  // SOC_LP_GPIO_MATRIX_SUPPORTED

// Non-uniform controller topology: wires the LP_I2C caps / opt-in bit /
// pin-domain hooks above into the shared `LocalKindAdapter<BusTraits>`.
inline bus::LocalKindAdapter<i2c::BusTraits>::Topology controllerTopologyForI2C(void)
{
    bus::LocalKindAdapter<i2c::BusTraits>::Topology topo;
    topo.controller_caps  = &hardwareControllerCapsForI2C;
    topo.opt_in           = caps::LOW_POWER;
    topo.complete_logical = &completeLogicalForI2C;
    topo.pins_allowed     = &pinsAllowedForI2C;
    return topo;
}

#else  // !M5HAL_ESPIDF_I2C_LP_POOL

// No LP pool on this build (older IDF, gen4/legacy, or no LP_I2C on this
// SoC): the default Topology{} reproduces the original uniform behaviour
// exactly (see LocalKindAdapter<Traits>::Topology).
inline bus::LocalKindAdapter<i2c::BusTraits>::Topology controllerTopologyForI2C(void)
{
    return {};
}

#endif  // M5HAL_ESPIDF_I2C_LP_POOL

// Tells M5HALCore that this build has a poolable hardware I2C backend, so the
// I2C BusView is wired with the hardware factory + controller pool.
#define M5HAL_DETAIL_I2C_HAS_HARDWARE_BACKEND_ 1

}  // namespace m5::hal::v2::i2c

#endif

#endif
