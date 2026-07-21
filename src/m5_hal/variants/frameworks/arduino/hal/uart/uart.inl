// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ARDUINO)

#include "../../../../../hal/v2/diag.hpp"
#include "../../../../../hal/v2/resource_domain.hpp"

#include <cstdlib>
#include <cstdint>
#include <new>

namespace m5::hal::v2::uart {

namespace {
namespace impl_arduino {

constexpr uint16_t kNativeStreamProvider         = 0x0301;
constexpr uint16_t kNativeHardwareSerialProvider = 0x0302;

struct PendingNativeToken {
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* interner = nullptr;
    bus::NativeToken token{};

    ~PendingNativeToken()
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

bus::BindingDescriptor makeNativeBinding(const IBusConfig& cfg, bus::NativeToken token, uint16_t provider)
{
    bus::BindingDescriptor binding;
    binding.provider    = provider;
    binding.ownership   = bus::Ownership::Borrowed;
    binding.native_kind = bus::NativeBindingKind::Native;
    binding.native      = token;
    binding.config_primary =
        static_cast<uint16_t>(cfg.pin_tx) | (static_cast<uint32_t>(static_cast<uint16_t>(cfg.pin_rx)) << 16u);
    binding.config_secondary =
        static_cast<uint16_t>(cfg.pin_rts) | (static_cast<uint32_t>(static_cast<uint16_t>(cfg.pin_cts)) << 16u);
    return binding;
}

#if !defined(ESP_PLATFORM) && !defined(ARDUINO_ARCH_ESP8266) && !defined(SERIAL_8N1)
#error "This Arduino core does not define the SERIAL_* uart config macros; extend serialConfig() for it."
#endif

result_t<uint32_t> serialConfig(const uart::AccessConfig& cfg)
{
    if (cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP8266)
    // arduino-esp32 and the ESP8266 core define the SERIAL_* configs as
    // SerialConfig enum values (not macros), with every 8-bit combination
    // available.
    switch (cfg.parity) {
        case uart::parity_t::None:
            return cfg.stop_bits == 1 ? SERIAL_8N1 : SERIAL_8N2;
        case uart::parity_t::Even:
            return cfg.stop_bits == 1 ? SERIAL_8E1 : SERIAL_8E2;
        case uart::parity_t::Odd:
            return cfg.stop_bits == 1 ? SERIAL_8O1 : SERIAL_8O2;
        default:
            break;
    }
#else
    // The non-ESP allowlisted cores define their SERIAL_* configs as macros,
    // and the set a core defines tracks the UART peripheral's real capability
    // (e.g. the Adafruit nRF52 core omits SERIAL_8O1/8O2 entirely — nRF52
    // UARTE has no odd-parity support). Guard each constant so a missing
    // combination rejects at runtime with INVALID_ARGUMENT (the spec's code
    // for 未対応値, uart.md) instead of failing the whole variant at compile
    // time. The #error above trips if a future core ships no SERIAL_8N1
    // macro at all (either a different constant style — like arduino-esp32's
    // enum — or none; both need explicit handling here, not silent
    // all-reject).
    switch (cfg.parity) {
        case uart::parity_t::None:
            if (cfg.stop_bits == 1) {
                return SERIAL_8N1;
            }
#if defined(SERIAL_8N2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8N2;
            }
#endif
            break;
        case uart::parity_t::Even:
#if defined(SERIAL_8E1)
            if (cfg.stop_bits == 1) {
                return SERIAL_8E1;
            }
#endif
#if defined(SERIAL_8E2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8E2;
            }
#endif
            break;
        case uart::parity_t::Odd:
#if defined(SERIAL_8O1)
            if (cfg.stop_bits == 1) {
                return SERIAL_8O1;
            }
#endif
#if defined(SERIAL_8O2)
            if (cfg.stop_bits == 2) {
                return SERIAL_8O2;
            }
#endif
            break;
        default:
            break;
    }
#endif
    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
}

bool sameConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

}  // namespace impl_arduino
}  // namespace

result_t<void> Bus_arduino::teardown(void)
{
    auto locked = _state_mutex.lock(types::TIMEOUT_FOREVER);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};

    if (_native_interner != nullptr && _native_token.valid()) {
        auto released = _native_interner->release(_native_token);
        if (!released.has_value()) {
            return released;
        }
    }
    _native_interner = nullptr;
    _native_token    = {};
    if (_serial != nullptr && _begun && !_attached && _hw_serial) {
        static_cast<::HardwareSerial*>(_serial)->end();
    }
    _serial   = nullptr;
    _begun    = false;
    _attached = false;
    return {};
}

bus::CloseOutcome Bus_arduino::closeBackend(void)
{
    auto closed = teardown();
    if (!closed.has_value()) {
        return bus::CloseOutcome::noMutation(closed.error());
    }
    return bus::CloseOutcome::success();
}

uint32_t Bus_arduino::reconfigSkips()
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER).has_value()) {
        return 0;
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};
    return _reconfig_skips;
}

result_t<void> Bus_arduino::adoptBorrowedNative(
    ::Stream& stream, bool hardware_serial, const IBusConfig& config,
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token)
{
    if (!token.valid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto initialized = initBorrowedDirect(config, stream, hardware_serial);
    if (!initialized.has_value()) {
        return initialized;
    }
    _native_interner = &interner;
    _native_token    = token;
    return {};
}

result_t<void> Bus_arduino::initBorrowedDirect(const IBusConfig& config, ::Stream& stream, bool hardware_serial)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto reset = teardown();
    if (!reset.has_value()) {
        return reset;
    }
    auto locked = _state_mutex.lock(types::TIMEOUT_FOREVER);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};
    _config          = config;
    _serial          = &stream;
    _hw_serial       = hardware_serial;
    _attached        = true;
    _begun           = false;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        _serial = nullptr;
        return initialized;
    }
    return {};
}

result_t<void> Bus_arduino::init(const IBusConfig& config, native::Borrowed<::HardwareSerial> policy)
{
    return initBorrowedDirect(config, policy.resource(), true);
}

result_t<void> Bus_arduino::init(const IBusConfig& config, native::Borrowed<::Stream> policy)
{
    return initBorrowedDirect(config, policy.resource(), false);
}

result_t<std::shared_ptr<IBus>> Bus_arduino::acquireBorrowed(bus::IHalBackend& backend, const IBusConfig& cfg,
                                                             ::Stream& stream, bool hardware_serial, uint16_t provider)
{
    const auto* domain = backend.localResourceDomain();
    if (domain == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    const uint64_t address = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&stream));
    auto identity          = bus::NativeIdentity::make(bus::NativeIdentityKind::ObjectAddress, {address});
    if (!identity.has_value()) {
        return m5::stl::make_unexpected(identity.error());
    }
    auto& interner = domain->nativeInterner();
    auto token     = interner.intern(identity.value());
    if (!token.has_value()) {
        return m5::stl::make_unexpected(token.error());
    }
    impl_arduino::PendingNativeToken pending{&interner, token.value()};
    auto key = bus::ResourceKey::makeToken(types::bus_kind_t::UART, bus::ResourceTag::Native, token.value(),
                                           static_cast<uint32_t>(bus::NativeIdentityKind::ObjectAddress));
    if (!key.has_value()) {
        return m5::stl::make_unexpected(key.error());
    }
    const auto binding = impl_arduino::makeNativeBinding(cfg, token.value(), provider);
    auto acquired      = backend.busRegistry().acquireOrFind(
        key.value(), binding,
        [&cfg](const std::shared_ptr<bus::IBus>& existing) -> result_t<void> {
            if (!BusTraits::configCompatible(static_cast<const IBusConfig&>(existing->getConfig()), cfg)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        },
        [&]() -> result_t<std::shared_ptr<bus::IBus>> {
            std::unique_ptr<Bus_arduino> concrete{new (std::nothrow) Bus_arduino()};
            if (!concrete) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            concrete->bindLocalResources(backend.localResources());
            auto adopted = concrete->adoptBorrowedNative(stream, hardware_serial, cfg, interner, token.value());
            if (!adopted.has_value()) {
                return m5::stl::make_unexpected(adopted.error());
            }
            pending.dismiss();

            std::shared_ptr<Bus> facade{new (std::nothrow) Bus()};
            if (!facade) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            facade->bindLocalResources(backend.localResources());
            std::unique_ptr<IBus> selected{concrete.release()};
            auto installed = facade->adoptPortableBackend(std::move(selected), cfg);
            if (!installed.has_value()) {
                return m5::stl::make_unexpected(installed.error());
            }
            return std::shared_ptr<bus::IBus>{std::move(facade)};
        });
    if (!acquired.has_value()) {
        return m5::stl::make_unexpected(acquired.error());
    }
    return std::static_pointer_cast<IBus>(acquired.value());
}

result_t<std::shared_ptr<IBus>> NativeProvider_arduino<native::Borrowed<::HardwareSerial>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Borrowed<::HardwareSerial> policy)
{
    return Bus_arduino::acquireBorrowed(backend, cfg, policy.resource(), true,
                                        impl_arduino::kNativeHardwareSerialProvider);
}

result_t<std::shared_ptr<IBus>> NativeProvider_arduino<native::Borrowed<::Stream>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Borrowed<::Stream> policy)
{
    return Bus_arduino::acquireBorrowed(backend, cfg, policy.resource(), false, impl_arduino::kNativeStreamProvider);
}

result_t<void> Bus_arduino::beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    const Channel entered = context.runtime.mode == bus::OperationMode::Tx ? Channel::Tx : Channel::Rx;
    return applyConfig(operationOwner(context), entered, context.config,
                       bus::remainingTimeout(context.runtime, runtime::millis()));
}

result_t<void> Bus_arduino::endOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    if (context.runtime.mode == bus::OperationMode::Tx && _serial != nullptr) {
        _serial->flush();
    }
    return {};
}

result_t<void> Bus_arduino::applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg,
                                        uint32_t timeout_ms)
{
    if (_serial == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    auto locked = _state_mutex.lock(timeout_ms);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};

    if (!_begun) {
        // First apply: no other owner can be mid-transfer yet
        // (spec/design/uart.md), so no quiescence gate is needed.
        return applyConfigLocked(cfg);
    }
    if (impl_arduino::sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    if (owner == nullptr) {
        // A reconfigure without an accessor identity cannot prove quiescence.
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig rejected: no accessor identity (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};
    }

    // Reconfigure: apply only when the opposite channel is quiescent for
    // `owner` (spec/design/uart.md; IBus::tryAcquireOppositeChannel is the
    // sanctioned exception to the channel-lock -> state-mutex ordering).
    // Applies uniformly regardless of `_hw_serial` (see applyConfigLocked).
    auto& ibus = static_cast<IBus&>(owner->getBus());
    auto grant = ibus.tryAcquireOppositeChannel(owner, entered);
    if (!grant.granted) {
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig rejected: opposite channel busy (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    auto applied = applyConfigLocked(cfg);
    ibus.releaseOppositeChannel(owner, grant);
    return applied;
}

result_t<void> Bus_arduino::applyConfigLocked(const uart::AccessConfig& cfg)
{
    if (_hw_serial) {
        if (cfg.baud_rate == 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        auto native_cfg = impl_arduino::serialConfig(cfg);
        if (!native_cfg.has_value()) {
            return m5::stl::make_unexpected(native_cfg.error());
        }
        auto* hw = static_cast<::HardwareSerial*>(_serial);
#if defined(ESP_PLATFORM)
        if (!_begun) {
            if (_config.rx_buffer_size > 0) {
                hw->setRxBufferSize(_config.rx_buffer_size);
            }
            if (_config.tx_buffer_size > 0) {
                hw->setTxBufferSize(_config.tx_buffer_size);
            }
        }
        hw->begin(cfg.baud_rate, native_cfg.value(), static_cast<int>(_config.pin_rx), static_cast<int>(_config.pin_tx),
                  cfg.invert);
#elif defined(ARDUINO_ARCH_ESP8266)
        // The ESP8266 core has an RX buffer setter (TX is FIFO+blocking
        // there, so tx_buffer_size has nothing to apply to) and its begin()
        // exposes signal inversion; mode and TX pin keep the overload's own
        // defaults (SERIAL_FULL, GPIO1 — pin selection is not honored on
        // this core, see variants.md).
        if (!_begun && _config.rx_buffer_size > 0) {
            hw->setRxBufferSize(_config.rx_buffer_size);
        }
        hw->begin(cfg.baud_rate, static_cast<decltype(SERIAL_8N1)>(native_cfg.value()), SERIAL_FULL, 1, cfg.invert);
        if (!*hw) {
            // Before begin(), setRxBufferSize() only records the size; the
            // core's uart_init() does the actual RX buffer malloc inside
            // begin() and leaves the port unusable on failure (operator
            // bool() == false). Surface that as the bounded-resource error
            // (uart.md) instead of recording a successful apply.
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
#else
        // Every other allowlisted core provides a begin(baud, config)
        // overload (uint8_t on STM32, uint16_t on the ArduinoCore-API
        // cores) — decltype(SERIAL_8N1) matches each core's constant type,
        // so the validated config is actually applied instead of silently
        // staying at the core-default 8N1. None of these cores can invert
        // the signal; reject instead of silently ignoring (the uart.md
        // contract for unsupported values).
        if (cfg.invert) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        hw->begin(cfg.baud_rate, static_cast<decltype(SERIAL_8N1)>(native_cfg.value()));
#endif
    }
    _serial->setTimeout(cfg.first_byte_timeout_ms);
    _applied_cfg = cfg;
    _begun       = true;
    return {};
}

result_t<size_t> Bus_arduino::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return _serial->write(data, len);
}

result_t<size_t> Bus_arduino::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    const uint32_t start = millis();
    while (_serial->available() <= 0) {
        if (millis() - start >= timeout_ms) {
            return static_cast<size_t>(0);
        }
        delay(1);
    }
    size_t count = 0;
    while (count < len && _serial->available() > 0) {
        const int value = _serial->read();
        if (value < 0) {
            break;
        }
        buf[count++] = static_cast<uint8_t>(value);
    }
    return count;
}

result_t<size_t> Bus_arduino::rawReadableBytes()
{
    return static_cast<size_t>(_serial->available());
}

result_t<size_t> Bus_arduino::writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                           size_t len)
{
    auto result = Bus_streaming::writeBackend(context, src, len);
    return result;
}

result_t<size_t> Bus_arduino::readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
                                          size_t len)
{
    return Bus_streaming::readBackend(context, dst, len);
}

result_t<size_t> Bus_arduino::readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    return Bus_streaming::readableBytesBackend(context);
}

}  // namespace m5::hal::v2::uart

#endif

#endif
