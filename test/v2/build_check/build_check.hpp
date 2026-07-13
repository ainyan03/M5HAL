// SPDX-License-Identifier: MIT
#ifndef M5HAL_TEST_V2_BUILD_CHECK_BUILD_CHECK_HPP_
#define M5HAL_TEST_V2_BUILD_CHECK_BUILD_CHECK_HPP_

#include <M5HAL_v2.hpp>

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#endif

#if defined(ESP_PLATFORM)
#include <m5_hal/variants/frameworks/espidf/detail/espidf_version.hpp>
#endif

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace m5hal_build_check::v2 {
namespace detail {

namespace bus   = ::m5::hal::v2::bus;
namespace data  = ::m5::hal::v2::data;
namespace error = ::m5::hal::v2::error;
namespace i2c   = ::m5::hal::v2::i2c;
namespace spi   = ::m5::hal::v2::spi;
namespace types = ::m5::hal::v2::types;
namespace uart  = ::m5::hal::v2::uart;

class DummyI2cBus : public i2c::IBus {
public:
    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    ::m5::hal::v2::result_t<void> init(const i2c::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::hal::v2::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v2::result_t<void> transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg,
                                           const i2c::TransferDesc& desc, data::Source* tx, size_t, data::Sink* rx,
                                           size_t) override
    {
        (void)owner;
        (void)cfg;
        (void)desc;
        size_t done = 0;  // data phase only
        while (tx != nullptr && !tx->eof()) {
            auto span = tx->peek(16);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto advanced = tx->advance(span->size);
            if (!advanced.has_value()) {
                return ::m5::stl::make_unexpected(advanced.error());
            }
        }
        while (rx != nullptr && !rx->closed()) {
            auto span = rx->reserve(16);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto committed = rx->commit(span->size);
            if (!committed.has_value()) {
                return ::m5::stl::make_unexpected(committed.error());
            }
        }
        (void)done;
        return {};
    }
};

class DummySpiBus : public spi::IBus {
public:
    ::m5::hal::v2::result_t<void> init(const spi::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::hal::v2::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v2::result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        return {};
    }

    ::m5::hal::v2::result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        return {};
    }

    ::m5::hal::v2::result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                                           const spi::TransferDesc& desc, data::Source* tx, size_t, data::Sink* rx,
                                           size_t) override
    {
        (void)owner;
        (void)cfg;
        (void)desc;
        size_t done = 0;
        while (tx != nullptr && !tx->eof()) {
            auto span = tx->peek(16);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto advanced = tx->advance(span->size);
            if (!advanced.has_value()) {
                return ::m5::stl::make_unexpected(advanced.error());
            }
        }
        while (rx != nullptr && !rx->closed()) {
            auto span = rx->reserve(16);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto committed = rx->commit(span->size);
            if (!committed.has_value()) {
                return ::m5::stl::make_unexpected(committed.error());
            }
        }
        (void)done;
        return {};
    }
};

class DummyUartBus : public uart::IBus {
public:
    ::m5::hal::v2::result_t<void> init(const uart::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::hal::v2::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v2::result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* tx,
                                          size_t len) override
    {
        (void)owner;
        (void)cfg;
        size_t done = 0;
        while (tx != nullptr && !tx->eof() && done < len) {
            auto span = tx->peek(len - done);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto advanced = tx->advance(span->size);
            if (!advanced.has_value()) {
                return ::m5::stl::make_unexpected(advanced.error());
            }
        }
        return done;
    }

    ::m5::hal::v2::result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* rx,
                                         size_t len) override
    {
        (void)owner;
        (void)cfg;
        size_t done = 0;
        while (rx != nullptr && !rx->closed() && done < len) {
            auto span = rx->reserve(len - done);
            if (!span.has_value()) {
                return ::m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            done += span->size;
            auto committed = rx->commit(span->size);
            if (!committed.has_value()) {
                return ::m5::stl::make_unexpected(committed.error());
            }
        }
        return done;
    }

    ::m5::hal::v2::result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        return size_t{0};
    }
};

template <typename T>
inline void useResult(const T& value)
{
    (void)value;
}

}  // namespace detail

// ---- Selected-variant markers -----------------------------------
// Fences for the scan-order assumptions, demonstrating both diagnosis
// idioms: the integer marker (usable in #if AND static_assert) and the
// entity-identity check (flat injection is a using-directive, so the flat
// name and the variant alias denote the same type).
// I2C is the runtime facade: the unsuffixed `i2c::Bus` is the facade
// class, NOT the winner variant alias. The winner is now expressed through the
// `BusConfig` alias + the `BackendFor` trait (`init(BusConfig)` creates that
// backend), so the scan-order winner is checked against that backend type.
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C != M5HAL_V2_VARIANT_ID_NONE, "some variant must provide I2C");
#if defined(ARDUINO)
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO,
              "scan order: arduino wins the I2C BusConfig alias when present");
static_assert(std::is_same<::m5::hal::v2::i2c::BackendFor<::m5::hal::v2::i2c::BusConfig>::type,
                           ::m5::hal::v2::i2c::Bus_arduino>::value,
              "the facade's default backend (BusConfig -> BackendFor) must be the winner variant");
#elif defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF,
              "scan order: espidf wins the I2C BusConfig alias on a plain IDF build");
static_assert(std::is_same<::m5::hal::v2::i2c::BackendFor<::m5::hal::v2::i2c::BusConfig>::type,
                           ::m5::hal::v2::i2c::Bus_espidf>::value,
              "the facade's default backend (BusConfig -> BackendFor) must be the winner variant");
#elif !defined(ESP_PLATFORM)
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE,
              "scan order: software provides I2C on a plain host build");
static_assert(std::is_same<::m5::hal::v2::i2c::BackendFor<::m5::hal::v2::i2c::BusConfig>::type,
                           ::m5::hal::v2::i2c::Bus_software>::value,
              "the facade's default backend (BusConfig -> BackendFor) must be the winner variant");
#endif
#if defined(ESP_PLATFORM)
static_assert(M5HAL_V2_SELECTED_VARIANT_GPIO == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32,
              "scan order: the platform variant wins GPIO on the ESP32 family");
// Detection and selection share one id registry, so the cross
// comparison is direct: the detected platform's variant wins GPIO.
static_assert(M5HAL_V2_SELECTED_VARIANT_GPIO == M5HAL_V2_TARGET_PLATFORM_VARIANT_ID,
              "the detected platform's variant should win GPIO on ESP32");
#endif

// ---- runtime kind -----------------------------------------------------
// runtime resolves in the EARLY scan (hal/v2/runtime/runtime.hpp) and can
// never be NONE — the stub fallback always offers it; the expectations
// below pin the per-environment winner and the flat/alias identity.
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME != M5HAL_V2_VARIANT_ID_NONE,
              "some variant must provide runtime (time functions)");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX != M5HAL_V2_VARIANT_ID_NONE,
              "some variant must provide runtime::Mutex (bus::IBus embeds it)");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK != M5HAL_V2_VARIANT_ID_NONE,
              "some variant must provide runtime::Task");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT != M5HAL_V2_VARIANT_ID_NONE,
              "some variant must provide runtime::Event (the ServiceRunner idle wait blocks on it)");
#if defined(ARDUINO) && defined(ESP_PLATFORM)
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO,
              "scan order: arduino wins the runtime time injection when present");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS,
              "scan order: freertos wins runtime::Mutex on arduino (FreeRTOS-hosted)");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Mutex, ::m5::variants::frameworks::freertos::hal::v2::runtime::Mutex>::value,
    "the unsuffixed name and the freertos variant type must be the same entity");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS,
              "scan order: freertos wins runtime::Event on arduino (a stub win here would turn the "
              "idle runner into a busy loop)");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::freertos::hal::v2::runtime::Event>::value,
    "the unsuffixed name and the freertos variant type must be the same entity");
#elif defined(ARDUINO)
// Non-ESP32 Arduino core (RP2040 / SAMD51, see _checker.hpp's variant
// allowlist): no FreeRTOS, no <thread> — arduino still wins the time
// injection, but Mutex/Event fall through to the stub fallback (single-task
// fakes; see stub/hal/runtime/runtime.hpp).
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO,
              "scan order: arduino wins the runtime time injection when present");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB,
              "scan order: no freertos/posix here, so the stub fake backs runtime::Mutex");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Mutex, ::m5::variants::frameworks::stub::hal::v2::runtime::Mutex>::value,
    "the unsuffixed name and the stub variant type must be the same entity");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB,
              "scan order: no freertos/posix here, so the stub fake backs runtime::Event");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::stub::hal::v2::runtime::Event>::value,
    "the unsuffixed name and the stub variant type must be the same entity");
#elif defined(ESP_PLATFORM)
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF,
              "scan order: espidf wins the runtime time injection on a plain IDF build");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS,
              "scan order: freertos wins runtime::Mutex on espidf (FreeRTOS-hosted)");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Mutex, ::m5::variants::frameworks::freertos::hal::v2::runtime::Mutex>::value,
    "the unsuffixed name and the freertos variant type must be the same entity");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS,
              "scan order: freertos wins runtime::Event on espidf (a stub win here would turn the "
              "idle runner into a busy loop)");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::freertos::hal::v2::runtime::Event>::value,
    "the unsuffixed name and the freertos variant type must be the same entity");
#elif M5HAL_FRAMEWORK_HAS_POSIX
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX,
              "scan order: posix provides runtime on a plain host build");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX,
              "scan order: posix provides runtime::Mutex on a plain host build");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Mutex, ::m5::variants::frameworks::posix::hal::v2::runtime::Mutex>::value,
    "the unsuffixed name and the posix variant type must be the same entity");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX,
              "scan order: posix provides runtime::Event on a plain host build");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::posix::hal::v2::runtime::Event>::value,
    "the unsuffixed name and the posix variant type must be the same entity");
#else
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB,
              "scan order: the stub fake backs runtime when no other variant offers it");
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB,
              "scan order: the stub fake backs runtime::Event when no other variant offers it");
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::stub::hal::v2::runtime::Event>::value,
    "the unsuffixed name and the stub variant type must be the same entity");
#endif

// ---- Typed registry mirror ------------------------------------
// The enum and name table derive from the ids.hpp X-macro list; pin the
// PP face and the typed face to the same values / spellings.
static_assert(static_cast<uint16_t>(::m5::hal::v2::variant_id_t::PLATFORM_ESP32) == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32,
              "variant_id_t must mirror the PP registry values");
static_assert(::m5::hal::v2::variantIdName(::m5::hal::v2::variant_id_t::FRAMEWORK_ARDUINO)[0] == 'F',
              "variantIdName must resolve registry names");
static_assert(::m5::hal::v2::variantIdName(static_cast<uint16_t>(0xFFFFu))[0] == 'U',
              "variantIdName must answer UNKNOWN for unregistered values");

inline void compileCommonApiSurface(void)
{
    uint8_t tx[]  = {0x10, 0x20, 0x30, 0x40};
    uint8_t rx[4] = {};

    detail::DummyI2cBus i2c_bus;
    detail::i2c::IBusConfig i2c_bus_cfg;
    i2c_bus_cfg.pin_scl = 22;
    i2c_bus_cfg.pin_sda = 21;
    detail::useResult(i2c_bus.init(i2c_bus_cfg));
    // tag-pin ctors on the kind base: either order, positional stays out.
    detail::useResult(i2c_bus.init(detail::i2c::IBusConfig{detail::i2c::Scl{22}, detail::i2c::Sda{21}}));
    detail::useResult(i2c_bus.init(detail::i2c::IBusConfig{detail::i2c::Sda{21}, detail::i2c::Scl{22}}));
    static_assert(!std::is_constructible<detail::i2c::IBusConfig, int, int>::value,
                  "no untagged positional pin ctor (I2C)");
    static_assert(!std::is_constructible<detail::uart::IBusConfig, int, int>::value,
                  "no untagged positional pin ctor (UART)");
    detail::i2c::MasterAccessConfig i2c_cfg;
    i2c_cfg.i2c_addr               = 0x3C;
    i2c_cfg.freq                   = 400000;
    i2c_cfg.register_address_bytes = 2;
    detail::i2c::MasterAccessor i2c_dev{i2c_bus, i2c_cfg};
    detail::useResult(i2c_dev.setConfig(i2c_cfg));
    detail::useResult(i2c_dev.transfer(detail::i2c::TransferDesc{uint8_t{0x00}},
                                       detail::data::ConstDataSpan{tx, sizeof(tx)},
                                       detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(i2c_dev.write(detail::data::ConstDataSpan{tx, sizeof(tx)}));
    detail::useResult(i2c_dev.read(detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(i2c_dev.write(tx, sizeof(tx)));
    detail::useResult(i2c_dev.read(rx, sizeof(rx)));
    detail::useResult(i2c_dev.writeRegister(uint8_t{0x01}, uint8_t{0x55}));
    detail::useResult(i2c_dev.writeRegister(uint16_t{0x1234}, tx, sizeof(tx)));
    detail::useResult(i2c_dev.readRegister(uint8_t{0x02}, rx, sizeof(rx)));
    detail::useResult(i2c_dev.readRegister(0x1234, detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(i2c_dev.readRegister(0x1234));
    detail::useResult(i2c_dev.probe());

    detail::DummySpiBus spi_bus;
    detail::spi::IBusConfig spi_bus_cfg;
    spi_bus_cfg.pin_clk  = 18;
    spi_bus_cfg.pin_mosi = 23;
    spi_bus_cfg.pin_miso = 19;
    detail::useResult(spi_bus.init(spi_bus_cfg));
    detail::spi::MasterAccessConfig spi_cfg;
    spi_cfg.pin_cs                = 5;
    spi_cfg.freq                  = 40000000;
    spi_cfg.spi_mode              = 0;
    spi_cfg.spi_order             = 0;
    spi_cfg.spi_command_length    = 8;
    spi_cfg.spi_address_length    = 16;
    spi_cfg.spi_read_dummy_cycle  = 8;
    spi_cfg.spi_write_dummy_cycle = 0;
    spi_cfg.spi_data_mode         = detail::spi::spi_data_mode_t::FullDuplex;
    detail::spi::MasterAccessor spi_dev{spi_bus, spi_cfg};
    detail::useResult(spi_dev.setConfig(spi_cfg));
    detail::useResult(spi_dev.beginTransaction());
    detail::useResult(spi_dev.transfer(detail::spi::TransferDesc{}, detail::data::ConstDataSpan{tx, sizeof(tx)},
                                       detail::data::DataSpan{rx, sizeof(rx)}));
    detail::data::MemorySource spi_source{detail::data::ConstDataSpan{tx, sizeof(tx)}};
    detail::data::MemorySink spi_sink{detail::data::DataSpan{rx, sizeof(rx)}};
    detail::data::LimitedSource limited_source{&spi_source, sizeof(tx)};
    detail::data::LimitedSink limited_sink{&spi_sink, sizeof(rx)};
    detail::useResult(
        i2c_dev.transfer(detail::i2c::TransferDesc{}, &limited_source, sizeof(tx), &limited_sink, sizeof(rx)));
    detail::useResult(spi_dev.transfer(detail::spi::TransferDesc{}, &limited_source, &limited_sink, sizeof(tx)));
    detail::useResult(spi_dev.write(detail::data::ConstDataSpan{tx, sizeof(tx)}));
    detail::useResult(spi_dev.read(detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(spi_dev.write(tx, sizeof(tx)));
    detail::useResult(spi_dev.read(rx, sizeof(rx)));
    detail::useResult(spi_dev.writeCommand(uint32_t{0x2C}));
    detail::useResult(spi_dev.writeCommand(detail::data::ConstDataSpan{tx, 1}));
    detail::useResult(spi_dev.writeCommandAddress(uint32_t{0x0B}, uint32_t{0x001234}));
    detail::useResult(spi_dev.writeCommandData(uint32_t{0x2C}, detail::data::ConstDataSpan{tx, sizeof(tx)}));
    detail::useResult(spi_dev.writeCommandAddressData(uint32_t{0x0C}, uint32_t{0x001234},
                                                      detail::data::ConstDataSpan{tx, sizeof(tx)}));
    detail::useResult(spi_dev.readCommandData(uint32_t{0x0F}, detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(
        spi_dev.readCommandAddressData(uint32_t{0x0F}, uint32_t{0x001234}, detail::data::DataSpan{rx, sizeof(rx)}));
    detail::useResult(spi_dev.sendDummyClock(8));
    detail::useResult(spi_dev.endTransaction());

    detail::DummyUartBus uart_bus;
    detail::uart::IBusConfig uart_bus_cfg;
    uart_bus_cfg.pin_tx = 17;
    uart_bus_cfg.pin_rx = 16;
    detail::useResult(uart_bus.init(uart_bus_cfg));
    // tag-pin ctors on the kind base: either order.
    detail::useResult(uart_bus.init(detail::uart::IBusConfig{detail::uart::Tx{17}, detail::uart::Rx{16}}));
    detail::useResult(uart_bus.init(detail::uart::IBusConfig{detail::uart::Rx{16}, detail::uart::Tx{17}}));
    detail::uart::AccessConfig uart_cfg;
    uart_cfg.baud_rate             = 921600;
    uart_cfg.first_byte_timeout_ms = 10;
    uart_cfg.inter_byte_timeout_ms = 2;
    uart_cfg.write_timeout_ms      = 10;
    uart_cfg.data_bits             = 8;
    uart_cfg.stop_bits             = 1;
    uart_cfg.parity                = detail::uart::parity_t::None;
    detail::uart::Accessor uart_dev{uart_bus, uart_cfg};
    detail::useResult(uart_dev.setConfig(uart_cfg));
    detail::useResult(uart_dev.beginAccess());
    detail::useResult(uart_dev.endAccess());
    detail::useResult(uart_dev.write(detail::data::ConstDataSpan{tx, sizeof(tx)}));
    detail::useResult(uart_dev.read(detail::data::DataSpan{rx, sizeof(rx)}));
    detail::data::MemorySource uart_source{detail::data::ConstDataSpan{tx, sizeof(tx)}};
    detail::data::MemorySink uart_sink{detail::data::DataSpan{rx, sizeof(rx)}};
    detail::useResult(uart_dev.write(uart_source, sizeof(tx)));
    detail::useResult(uart_dev.read(uart_sink, sizeof(rx)));
    detail::useResult(uart_dev.write(tx, sizeof(tx)));
    detail::useResult(uart_dev.read(rx, sizeof(rx)));
    detail::useResult(uart_dev.readableBytes());

    detail::uart::TxAccessor uart_tx{uart_bus, uart_cfg};
    detail::uart::RxAccessor uart_rx{uart_bus, uart_cfg};
    detail::useResult(uart_tx.setConfig(uart_cfg));
    detail::useResult(uart_rx.setConfig(uart_cfg));
    detail::useResult(uart_tx.beginAccess());
    detail::useResult(uart_tx.endAccess());
    detail::useResult(uart_rx.beginAccess());
    detail::useResult(uart_rx.endAccess());
    detail::useResult(uart_tx.write(tx, sizeof(tx)));
    detail::useResult(uart_rx.read(rx, sizeof(rx)));
    detail::useResult(uart_rx.readableBytes());

    // Stream adapters: the RX/TX accessors bind to the minimal stream
    // interfaces, and the adapters lift them into Source / Sink.
    detail::data::StreamReader& stream_reader = uart_rx;
    detail::data::StreamWriter& stream_writer = uart_tx;
    uint8_t stream_scratch[8];
    detail::data::StreamSource stream_source{stream_reader, detail::data::DataSpan{stream_scratch, 0}};
    detail::data::StreamSink stream_sink{stream_writer, detail::data::DataSpan{stream_scratch, 0}};
    detail::data::Source& stream_as_source = stream_source;
    detail::data::Sink& stream_as_sink     = stream_sink;
    // peek/reserve use max_len 1 (0 is a contract violation); the size-0
    // scratch makes both return INVALID_ARGUMENT before any I/O happens.
    detail::useResult(stream_as_source.peek(1));
    detail::useResult(stream_as_source.advance(0));
    (void)stream_as_source.eof();
    (void)stream_source.buffered();
    (void)stream_source.pendingSkip();
    detail::useResult(stream_as_sink.reserve(1));
    detail::useResult(stream_as_sink.commit(0));
    (void)stream_as_sink.closed();

    // BlockSource: block-queue Source backed by Allocator.
    {
        detail::data::BlockSource block_src{::m5::hal::v2::memory::defaultAllocator()};
        (void)block_src.eof();
        (void)block_src.blockCount();
        (void)block_src.totalBuffered();
        detail::useResult(block_src.peek(1));
        detail::useResult(block_src.advance(0));
    }

    // Frame codec: pure encode/decode plus the Source/Sink-driven
    // reader/writer. All calls below stay I/O-free (empty inputs).
    namespace frame = ::m5::hal::v2::frame;
    uint8_t frame_buf[frame::checkedFrameWireSize(1)];
    detail::useResult(frame::encodeDelimiter(detail::data::DataSpan{frame_buf, sizeof(frame_buf)}));
    detail::useResult(frame::encodeChecked(detail::data::DataSpan{frame_buf, sizeof(frame_buf)}, frame::Kind::Control,
                                           0, detail::data::ConstDataSpan{}));
    detail::useResult(
        frame::encodeData(detail::data::DataSpan{frame_buf, sizeof(frame_buf)}, 0, detail::data::ConstDataSpan{}));
    (void)frame::crc8AtmUpdate(0x00, 0x00);
    (void)frame::check8(0x04, 0x01);
    (void)frame::isCheckedKind(frame::Kind::Data);
    (void)frame::isKnownKind(0x01);
    (void)frame::isDelimiter(detail::data::ConstDataSpan{});
    frame::View frame_view;
    (void)frame::decode(detail::data::ConstDataSpan{}, frame_view);  // need_more, no I/O
    detail::data::MemorySource frame_src{detail::data::ConstDataSpan{}};
    detail::data::MemorySink frame_snk{detail::data::DataSpan{}};
    frame::FrameReader frame_reader{frame_src};
    frame::FrameWriter frame_writer{frame_snk};
    detail::useResult(frame_reader.next(frame_view));  // empty source -> END_OF_STREAM
    detail::useResult(frame_writer.writeDelimiter());  // closed sink -> CLOSED

    // Bytecode: encoder + runner surface. All calls stay I/O-free
    // (closed sinks, empty scripts, no registered targets touched).
    namespace bytecode = ::m5::hal::v2::bytecode;
    uint8_t lenvar_buf[5];
    (void)bytecode::encodeLenVar(lenvar_buf, bytecode::kMaxStoreSlots);
    (void)bytecode::decodeLenVar(detail::data::ConstDataSpan{});
    (void)bytecode::lenVarSize(0x1234);
    detail::data::MemorySink bytecode_snk{detail::data::DataSpan{}};  // closed sink
    bytecode::BytecodeEncoder bytecode_enc{bytecode_snk};
    detail::useResult(bytecode_enc.delayMs(0));
    detail::useResult(bytecode_enc.gpioWriteHigh(nullptr, 0));
    detail::useResult(bytecode_enc.storeData(0, detail::data::ConstDataSpan{}));
    detail::useResult(bytecode_enc.end());
    bytecode::BytecodeRunner bytecode_runner;
    detail::useResult(bytecode_runner.registerI2C(0, i2c_dev));
    detail::useResult(bytecode_runner.registerUART(0, uart_dev));
    detail::useResult(bytecode_runner.run(detail::data::ConstDataSpan{}));  // empty script -> no-op
    (void)bytecode_runner.storedData(0);
    (void)bytecode_runner.storedCount();
    (void)bytecode_runner.statusReported();
    (void)bytecode_runner.lastOffset();
    (void)bytecode_runner.unknownSkipped();
    detail::useResult(bytecode_runner.writeResponse(bytecode_snk, detail::error::error_t::OK));
}

#if defined(ARDUINO)
inline void compileArduinoApiSurface(void)
{
    ::m5::hal::v2::i2c::BusConfig_arduino i2c_cfg;
    i2c_cfg.wire    = &Wire;
    i2c_cfg.pin_scl = 22;
    i2c_cfg.pin_sda = 21;
    ::m5::hal::v2::spi::BusConfig_arduino spi_cfg;
    spi_cfg.spi = &SPI;
    ::m5::hal::v2::uart::BusConfig_arduino uart_cfg;
    uart_cfg.setSerial(Serial1);

    // tag-pin ctors, inherited by the variant configs; the
    // variant-specific field stays assignable after tag construction.
    ::m5::hal::v2::i2c::BusConfig_arduino i2c_tag_cfg{::m5::hal::v2::i2c::Scl{22}, ::m5::hal::v2::i2c::Sda{21}};
    i2c_tag_cfg.wire = &Wire;
    ::m5::hal::v2::uart::BusConfig_arduino uart_tag_cfg{::m5::hal::v2::uart::Rx{16}, ::m5::hal::v2::uart::Tx{17}};
    uart_tag_cfg.setSerial(Serial1);

    static_assert(sizeof(::m5::hal::v2::i2c::Bus_arduino) > 0, "Arduino I2C Bus type must be visible");
    static_assert(sizeof(::m5::hal::v2::spi::Bus_arduino) > 0, "Arduino SPI Bus type must be visible");
    static_assert(sizeof(::m5::hal::v2::uart::Bus_arduino) > 0, "Arduino UART Bus type must be visible");
    detail::useResult(i2c_cfg);
    detail::useResult(spi_cfg);
    detail::useResult(uart_cfg);
    detail::useResult(i2c_tag_cfg);
    detail::useResult(uart_tag_cfg);
}
#endif

#if defined(ESP_PLATFORM)
inline void compileEspidfApiSurface(void)
{
#if M5HAL_ESPIDF_I2C_HAS_MASTER
    ::m5::hal::v2::i2c::BusConfig_espidf i2c_cfg;
    i2c_cfg.pin_scl = 22;
    i2c_cfg.pin_sda = 21;
    static_assert(sizeof(::m5::hal::v2::i2c::Bus_espidf) > 0, "ESP-IDF I2C Bus type must be visible");
    detail::useResult(i2c_cfg);

    // tag-pin ctors, inherited by the variant config.
    ::m5::hal::v2::i2c::BusConfig_espidf i2c_tag_cfg{::m5::hal::v2::i2c::Scl{22}, ::m5::hal::v2::i2c::Sda{21}};
    detail::useResult(i2c_tag_cfg);
#endif

#if M5HAL_ESPIDF_SPI_HAS_MASTER
    ::m5::hal::v2::spi::BusConfig_espidf spi_cfg;
    static_assert(sizeof(::m5::hal::v2::spi::Bus_espidf) > 0, "ESP-IDF SPI Bus type must be visible");
    detail::useResult(spi_cfg);
#endif

    ::m5::hal::v2::uart::BusConfig_espidf uart_cfg;
    static_assert(sizeof(::m5::hal::v2::uart::Bus_espidf) > 0, "ESP-IDF UART Bus type must be visible");
    detail::useResult(uart_cfg);

    // tag-pin ctors; `port_num` stays assignable after tag construction.
    ::m5::hal::v2::uart::BusConfig_espidf uart_tag_cfg{::m5::hal::v2::uart::Tx{17}, ::m5::hal::v2::uart::Rx{16}};
    uart_tag_cfg.port_num = 1;
    detail::useResult(uart_tag_cfg);
}
#endif

#if defined(ESP_PLATFORM)
// Chip-level GPIO capabilities (PinBackup / ScopedPinBackup) are exposed under
// the public `m5::hal::v2::gpio` namespace directly from the platform layer,
// independent of which variant wins the GPIO HAL flat injection. Referencing
// them here via the public flat name keeps CI honest about that exposure
// (it would catch a regression where the symbols stop resolving).
inline void compileGpioCapabilityApiSurface(void)
{
    namespace gpio  = ::m5::hal::v2::gpio;
    namespace types = ::m5::hal::v2::types;

    static_assert(sizeof(gpio::PinBackup) > 0, "PinBackup must be visible under m5::hal::v2::gpio");
    static_assert(sizeof(gpio::ScopedPinBackup) > 0, "ScopedPinBackup must be visible under m5::hal::v2::gpio");

    gpio::PinBackup backup{types::gpio_number_t{21}};
    backup.setPin(types::gpio_number_t{22});
    detail::useResult(backup.getPin());
    backup.backup();
    backup.backup(types::gpio_number_t{23});
    detail::useResult(backup.captured());
    backup.restore();

    gpio::ScopedPinBackup guard{types::gpio_number_t{21}};
    detail::useResult(guard.getPin());
    detail::useResult(guard.armed());
    detail::useResult(guard.captured());
    gpio::ScopedPinBackup moved{static_cast<gpio::ScopedPinBackup&&>(guard)};  // move-only
    moved.dismiss();
}
#endif

#if M5HAL_FRAMEWORK_HAS_POSIX
// Host POSIX UART variant surface. On a plain POSIX host build posix is the
// flat-injected UART provider; this references its suffixed variant type to
// keep CI honest about the exposure. Compile/surface only: it never opens a
// device (no open()/attach()), so it is safe to run in CI.
inline void compilePosixApiSurface(void)
{
    static_assert(sizeof(::m5::hal::v2::uart::Bus_posix) > 0, "POSIX UART Bus type must be visible");

    ::m5::hal::v2::uart::Bus_posix bus;
    ::m5::hal::v2::uart::BusConfig_posix uart_cfg;
    uart_cfg.device_path = nullptr;  // lazy open; nothing is opened here
    detail::useResult(bus.init(uart_cfg));
    detail::useResult(bus.nativeHandle());
}
#endif

// runtime kind surface: free functions + the mutex contract, all
// through the flat-injected names so every target proves its variant
// (FreeRTOS detail on device, std::timed_mutex on the posix host,
// the fake on a bare native build) actually compiles.
inline void compileRuntimeApiSurface(void)
{
    namespace runtime = ::m5::hal::v2::runtime;
    detail::useResult(runtime::millis());
    detail::useResult(runtime::micros());
    runtime::delayMs(0);
    runtime::delayUs(0);
    runtime::Mutex mutex;
    if (mutex.lock(0)) {
        mutex.unlock();
    }
    runtime::Event event;
    event.notify();
    detail::useResult(event.wait(0));
}

inline void compileApiSurface(void)
{
    compileCommonApiSurface();
    compileRuntimeApiSurface();
#if defined(ARDUINO)
    compileArduinoApiSurface();
#endif
#if defined(ESP_PLATFORM)
    compileEspidfApiSurface();
    compileGpioCapabilityApiSurface();
#endif
#if M5HAL_FRAMEWORK_HAS_POSIX
    compilePosixApiSurface();
#endif
}

}  // namespace m5hal_build_check::v2

#endif
