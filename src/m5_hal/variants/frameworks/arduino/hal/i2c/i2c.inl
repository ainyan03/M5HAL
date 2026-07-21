// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include "begin_result.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/resource_domain.hpp"
#include <M5Utility.hpp>

#include <cstdint>
#include <new>

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::i2c {

namespace {
namespace impl_arduino {

// Map TwoWire::endTransmission() return code to M5HAL error_t.
// 0 = success, 1 = data too long, 2 = NACK on address, 3 = NACK on data,
// 4 = other error, 5 = timeout (ESP32 extension). Other codes are
// collapsed into the generic bus error.
error::error_t mapWireEndTransmission(uint8_t code)
{
    switch (code) {
        case 0:
            return error::error_t::OK;
        case 2:
        case 3:
            return error::error_t::I2C_NO_ACK;
        case 5:
            return error::error_t::TIMEOUT_ERROR;
        default:
            return error::error_t::I2C_BUS_ERROR;
    }
}

constexpr uint16_t kNativeProvider = 0x0101;

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

bus::BindingDescriptor makeNativeBinding(const IBusConfig& cfg, bus::NativeToken token)
{
    bus::BindingDescriptor binding;
    binding.provider    = kNativeProvider;
    binding.ownership   = bus::Ownership::Borrowed;
    binding.native_kind = bus::NativeBindingKind::Native;
    binding.native      = token;
    binding.config_primary =
        static_cast<uint16_t>(cfg.pin_scl) | (static_cast<uint32_t>(static_cast<uint16_t>(cfg.pin_sda)) << 16u);
    return binding;
}

// The ESP8266 core's TwoWire has no end() — the peripheral cannot be
// deinitialized there and teardown leaves it configured (begin() re-inits).
void wireEnd(::TwoWire& wire)
{
#if !defined(ARDUINO_ARCH_ESP8266)
    wire.end();
#else
    (void)wire;
#endif
}

}  // namespace impl_arduino
}  // namespace

result_t<void> Bus_arduino::adoptBorrowedNative(
    ::TwoWire& wire, const IBusConfig& config,
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token)
{
    if (!token.valid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto reset = teardown();
    if (!reset.has_value()) {
        return reset;
    }
    _config          = config;
    _wire            = &wire;
    _owns_wire       = false;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        _wire = nullptr;
        return initialized;
    }
    _native_interner = &interner;
    _native_token    = token;
    return {};
}

result_t<void> Bus_arduino::init(const IBusConfig& config)
{
    auto* wire = &Wire;
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto reset = teardown();
    if (!reset.has_value()) {
        return reset;
    }
    bool began = false;

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP8266)
    // TwoWire::begin(sda, scl) is an Espressif extension (arduino-esp32 and
    // the ESP8266 core both have the int-pin overload); no other supported
    // Arduino core (see _checker.hpp) has a matching overload —
    // TwoWire::begin(uint8_t) means "start as I2C slave at this address" on
    // the portable Arduino Wire API, not pin selection.
    if (config.pin_sda >= 0 && config.pin_scl >= 0) {
        began = wire_begin_detail::invokeWireBegin(
            [&] { return wire->begin(static_cast<int>(config.pin_sda), static_cast<int>(config.pin_scl)); });
    } else {
        began = wire_begin_detail::invokeWireBegin([&] { return wire->begin(); });
    }
#else
    // Portable cores fix SDA/SCL per Wire instance (board variant file);
    // a configured pin pair cannot be honored here and is ignored.
    began = wire_begin_detail::invokeWireBegin([&] { return wire->begin(); });
#endif

    if (!began) {
        impl_arduino::wireEnd(*wire);
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }

    _config          = config;
    _wire            = wire;
    _owns_wire       = true;
    _last_freq       = 0;
    _last_timeout_ms = 0xFFFFFFFFu;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)teardown();
        return initialized;
    }
    return {};
}

result_t<void> Bus_arduino::init(const IBusConfig& config, native::Borrowed<::TwoWire> policy)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto reset = teardown();
    if (!reset.has_value()) {
        return reset;
    }
    _config          = config;
    _wire            = &policy.resource();
    _owns_wire       = false;
    _last_freq       = 0;
    _last_timeout_ms = 0xFFFFFFFFu;
    return markInitializationSucceeded(false);
}

result_t<void> Bus_arduino::teardown(void)
{
    if (_native_interner != nullptr && _native_token.valid()) {
        auto released = _native_interner->release(_native_token);
        if (!released.has_value()) {
            return released;
        }
    }
    _native_interner = nullptr;
    _native_token    = {};
    if (_wire && _owns_wire) {
        impl_arduino::wireEnd(*_wire);
    }
    _wire            = nullptr;
    _owns_wire       = false;
    _last_freq       = 0;
    _last_timeout_ms = 0xFFFFFFFFu;
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

result_t<std::shared_ptr<IBus>> NativeProvider_arduino<native::Borrowed<::TwoWire>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Borrowed<::TwoWire> policy)
{
    const auto* domain = backend.localResourceDomain();
    if (domain == nullptr) {
        // A remote backend has no local domain. Reject before touching the
        // caller's native object or issuing any transport request.
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    const uint64_t address = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&policy.resource()));
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
    auto key = bus::ResourceKey::makeToken(types::bus_kind_t::I2C, bus::ResourceTag::Native, token.value(),
                                           static_cast<uint32_t>(bus::NativeIdentityKind::ObjectAddress));
    if (!key.has_value()) {
        return m5::stl::make_unexpected(key.error());
    }
    const auto binding = impl_arduino::makeNativeBinding(cfg, token.value());
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
            auto adopted = concrete->adoptBorrowedNative(policy.resource(), cfg, interner, token.value());
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

result_t<void> Bus_arduino::transferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context,
                                            const i2c::TransferDesc& desc, data::Source* src, size_t tx_len,
                                            data::Sink* dst, size_t rx_len)
{
    const auto& cfg = context.config;
    _transfer_totals.clear();
    if (_wire == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // `Wire` drives 7-bit addressing only: reject a 10-bit request
    // instead of silently truncating the address (which would address
    // the wrong device). Same degradation as the espidf gen4 backend.
    if (cfg.address_is_10bit || cfg.i2c_addr > 0x007Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    // Apply per-accessor parameters lazily: only call setClock / setTimeOut
    // when the requested value differs from what was last issued. This mirrors
    // the _last_freq pattern to avoid redundant Wire API calls.
    //
    // clampMasterClockHz() caps the clock to the I2C master peripheral's valid
    // ceiling on ESP targets (the ceiling is 0 = disabled on non-ESP cores, so
    // ports to other Arduino platforms are passed through untouched).
    bool clamped        = false;
    const uint32_t freq = clampMasterClockHz(cfg.freq, &clamped);
    if (freq != _last_freq) {
        if (clamped) {
            M5_LIB_LOGW("I2C master clock %u Hz exceeds the peripheral ceiling; clamped to %u Hz",
                        static_cast<unsigned>(cfg.freq), static_cast<unsigned>(freq));
        }
        _wire->setClock(freq);
        _last_freq = freq;
    }
#if defined(ESP_PLATFORM)
    // arduino-esp32 TwoWire::setTimeOut() takes milliseconds, same unit as
    // cfg.wire_timeout_ms. Not part of the portable Arduino Wire API — no
    // other supported core (see _checker.hpp) has an equivalent, so
    // wire_timeout_ms is a no-op there.
    if (cfg.wire_timeout_ms != _last_timeout_ms) {
        _wire->setTimeOut(static_cast<uint32_t>(cfg.wire_timeout_ms));
        _last_timeout_ms = cfg.wire_timeout_ms;
    }
#endif

    // Pass `desc.prefix` into the legacy code path (header.data /
    // header.size) as a local `ConstDataSpan`. `desc` lives as a
    // const reference parameter so its lifetime covers this entire
    // function body.
    data::ConstDataSpan header{desc.prefix, desc.prefix_len};

    size_t transmitted = 0;
    size_t received    = 0;
    bool write_phase   = (header.size > 0) || (src != nullptr && tx_len > 0 && !src->eof());
    bool read_phase    = (dst != nullptr && rx_len > 0);

    // Probe contract (see i2c.hpp): an all-empty descriptor sends
    // only address+W and inspects the ACK. We implement this with
    // `beginTransmission` -> `endTransmission(true)` (`Wire` still
    // emits the address+W byte even at zero payload, and
    // `endTransmission`'s return value tells us ACK vs NACK).
    if (!write_phase && !read_phase) {
        _wire->beginTransmission(static_cast<uint8_t>(cfg.i2c_addr));
        auto err = impl_arduino::mapWireEndTransmission(_wire->endTransmission(true));
        if (error::isError(err)) {
            return m5::stl::make_unexpected(err);
        }
        return {};
    }

    if (write_phase) {
        _wire->beginTransmission(static_cast<uint8_t>(cfg.i2c_addr));
        if (header.size) {
            size_t w = _wire->write(header.data, header.size);
            if (w != header.size) {
                // Wire only buffers here; a short write means the local TX
                // buffer is full, not a wire fault. Wire has no abort, so the
                // endTransmission below sends the partial buffer (best effort).
                (void)_wire->endTransmission(true);
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            // Prefix bytes are NOT counted: the return value is the data
            // phase only (src + dst), matching SPI.
        }
        if (src) {
            // Drain Source via peek/advance loop. SIZE_MAX requests "all
            // remaining bytes"; memory-backed sources return them in one
            // peek, stream-backed sources may chunk.
            size_t tx_remaining = tx_len;
            while (tx_remaining > 0 && !src->eof()) {
                auto peeked = src->peek(tx_remaining);
                if (!peeked.has_value()) {
                    _wire->endTransmission(true);
                    return m5::stl::make_unexpected(peeked.error());
                }
                auto span = peeked.value().first(tx_remaining);
                if (span.size == 0) {
                    break;  // explicit end-of-stream
                }
                size_t w = _wire->write(span.data, span.size);
                if (w != span.size) {
                    // Local TX buffer full (see the header-write comment).
                    (void)_wire->endTransmission(true);
                    return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
                }
                auto adv = src->advance(w);
                if (!adv.has_value()) {
                    _wire->endTransmission(true);
                    return m5::stl::make_unexpected(adv.error());
                }
                tx_remaining -= w;
                transmitted += w;
            }
        }
        bool send_stop = (dst == nullptr) || !cfg.use_restart;
        auto err       = impl_arduino::mapWireEndTransmission(_wire->endTransmission(send_stop));
        if (error::isError(err)) {
            return m5::stl::make_unexpected(err);
        }
    }

    if (dst) {
        // Reserve "everything the sink can hold for this transfer". The
        // returned span size is what we request over the wire; len is no
        // longer carried by the signature.
        auto rsv = dst->reserve(rx_len);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        auto rx_span = rsv.value().first(rx_len);
        if (rx_span.size > 0) {
            // Reject before requestFrom(): some cores truncate, while others
            // trust the length and can overwrite their internal RX array.
            if (rx_span.size > arduino_i2c_capability_detail::wireRxBufferLength()) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            size_t got = _wire->requestFrom(static_cast<uint8_t>(cfg.i2c_addr), static_cast<size_t>(rx_span.size),
                                            static_cast<size_t>(true));
            if (got != rx_span.size) {
                // Zero bytes back means the address byte was NACKed (no
                // device answered); a partial count is a transfer that broke
                // mid-read. Wire exposes no finer diagnostics than this.
                return m5::stl::make_unexpected(got == 0 ? error::error_t::I2C_NO_ACK : error::error_t::I2C_BUS_ERROR);
            }
            for (size_t i = 0; i < rx_span.size; ++i) {
                int b = _wire->read();
                if (b < 0) {
                    return m5::stl::make_unexpected(error::error_t::I2C_BUS_ERROR);
                }
                rx_span.data[i] = static_cast<uint8_t>(b);
            }
            auto com = dst->commit(rx_span.size);
            if (!com.has_value()) {
                return m5::stl::make_unexpected(com.error());
            }
            received += rx_span.size;
        }
    }

    _transfer_totals.tx += transmitted;
    _transfer_totals.rx += received;
    return {};
}

result_t<bus::TransferTotals> Bus_arduino::waitTransferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context)
{
    const auto& cfg = context.config;
    (void)cfg;
    auto totals = _transfer_totals;
    _transfer_totals.clear();
    return totals;
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
