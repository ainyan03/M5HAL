// SPDX-License-Identifier: MIT
#ifndef M5HAL_TEST_V1_BUILD_CHECK_BUILD_CHECK_HPP_
#define M5HAL_TEST_V1_BUILD_CHECK_BUILD_CHECK_HPP_

#include <M5HAL_v1.hpp>

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

namespace m5hal_build_check::v1 {
namespace detail {

namespace bus   = ::m5::hal::v1::bus;
namespace data  = ::m5::hal::v1::data;
namespace error = ::m5::hal::v1::error;
namespace i2c   = ::m5::hal::v1::i2c;
namespace spi   = ::m5::hal::v1::spi;
namespace types = ::m5::hal::v1::types;
namespace uart  = ::m5::hal::v1::uart;

class DummyI2CBus : public i2c::I2CBus {
public:
    // Typed init (S17 E1): the fake adds no fields, so it takes the
    // abstract kind config.
    ::m5::stl::expected<void, error::error_t> init(const i2c::I2CBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::stl::expected<void, error::error_t> release(void) override
    {
        return {};
    }

    ::m5::stl::expected<size_t, error::error_t> transfer(bus::Accessor* owner, const i2c::I2CMasterAccessConfig& cfg,
                                                         const i2c::TransferDesc& desc, data::Source* tx,
                                                         data::Sink* rx) override
    {
        (void)owner;
        (void)cfg;
        size_t done = 0;  // data phase only (S16 D4)
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
        return done;
    }
};

class DummySPIBus : public spi::SPIBus {
public:
    ::m5::stl::expected<void, error::error_t> init(const spi::SPIBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::stl::expected<void, error::error_t> release(void) override
    {
        return {};
    }

    ::m5::stl::expected<void, error::error_t> beginTransaction(bus::Accessor* owner,
                                                               const spi::SPIMasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        return {};
    }

    ::m5::stl::expected<void, error::error_t> endTransaction(bus::Accessor* owner,
                                                             const spi::SPIMasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        return {};
    }

    ::m5::stl::expected<size_t, error::error_t> transfer(bus::Accessor* owner, const spi::SPIMasterAccessConfig& cfg,
                                                         const spi::TransferDesc& desc, data::Source* tx,
                                                         data::Sink* rx) override
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
        return done;
    }
};

class DummyUARTBus : public uart::UARTBus {
public:
    ::m5::stl::expected<void, error::error_t> init(const uart::UARTBusConfig& config)
    {
        _config = config;
        return {};
    }

    ::m5::stl::expected<void, error::error_t> release(void) override
    {
        return {};
    }

    ::m5::stl::expected<size_t, error::error_t> write(bus::Accessor* owner, const uart::UARTAccessConfig& cfg,
                                                      data::Source* tx, size_t len) override
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

    ::m5::stl::expected<size_t, error::error_t> read(bus::Accessor* owner, const uart::UARTAccessConfig& cfg,
                                                     data::Sink* rx, size_t len) override
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

    ::m5::stl::expected<size_t, error::error_t> readableBytes(bus::Accessor* owner,
                                                              const uart::UARTAccessConfig& cfg) override
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

// ---- Selected-variant markers (S17 E3) -----------------------------------
// Fences for the scan-order assumptions, demonstrating both diagnosis
// idioms: the integer marker (usable in #if AND static_assert) and the
// entity-identity check (flat injection is a using-directive, so the flat
// name and the variant alias denote the same type).
static_assert(M5HAL_V1_SELECTED_VARIANT_I2C != M5HAL_V1_VARIANT_ID_NONE, "some variant must provide I2C");
#if defined(ARDUINO)
static_assert(M5HAL_V1_SELECTED_VARIANT_I2C == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO,
              "scan order: arduino wins the I2C flat injection when present");
static_assert(std::is_same<::m5::hal::v1::i2c::Bus, ::m5::hal::v1::i2c::variant::arduino::Bus>::value,
              "the flat name and the variant alias must be the same entity");
#elif defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER
static_assert(M5HAL_V1_SELECTED_VARIANT_I2C == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF,
              "scan order: espidf wins the I2C flat injection on a plain IDF build");
static_assert(std::is_same<::m5::hal::v1::i2c::Bus, ::m5::hal::v1::i2c::variant::espidf::Bus>::value,
              "the flat name and the variant alias must be the same entity");
#elif !defined(ESP_PLATFORM)
static_assert(M5HAL_V1_SELECTED_VARIANT_I2C == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE,
              "scan order: software provides I2C on a plain host build");
static_assert(std::is_same<::m5::hal::v1::i2c::Bus, ::m5::hal::v1::i2c::variant::software::Bus>::value,
              "the flat name and the variant alias must be the same entity");
#endif
#if defined(ESP_PLATFORM)
static_assert(M5HAL_V1_SELECTED_VARIANT_GPIO == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32,
              "scan order: the platform variant wins GPIO on the ESP32 family");
// Detection and selection share one id registry (S18), so the cross
// comparison is direct: the detected platform's variant wins GPIO.
static_assert(M5HAL_V1_SELECTED_VARIANT_GPIO == M5HAL_V1_TARGET_PLATFORM_VARIANT_ID,
              "the detected platform's variant should win GPIO on ESP32");
#endif

inline void compileCommonApiSurface(void)
{
    uint8_t tx[]  = {0x10, 0x20, 0x30, 0x40};
    uint8_t rx[4] = {};

    detail::DummyI2CBus i2c_bus;
    detail::i2c::I2CBusConfig i2c_bus_cfg{22, 21};
    detail::useResult(i2c_bus.init(i2c_bus_cfg));
    detail::i2c::I2CMasterAccessConfig i2c_cfg;
    i2c_cfg.i2c_addr               = 0x3C;
    i2c_cfg.freq                   = 400000;
    i2c_cfg.timeout_ms             = 25;
    i2c_cfg.register_address_bytes = 2;
    detail::i2c::I2CMasterAccessor i2c_dev{i2c_bus, i2c_cfg};
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

    detail::DummySPIBus spi_bus;
    detail::spi::SPIBusConfig spi_bus_cfg;
    spi_bus_cfg.pin_clk  = 18;
    spi_bus_cfg.pin_mosi = 23;
    spi_bus_cfg.pin_miso = 19;
    detail::useResult(spi_bus.init(spi_bus_cfg));
    detail::spi::SPIMasterAccessConfig spi_cfg;
    spi_cfg.pin_cs                = 5;
    spi_cfg.freq                  = 40000000;
    spi_cfg.timeout_ms            = 50;
    spi_cfg.spi_mode              = 0;
    spi_cfg.spi_order             = 0;
    spi_cfg.spi_command_length    = 8;
    spi_cfg.spi_address_length    = 16;
    spi_cfg.spi_read_dummy_cycle  = 8;
    spi_cfg.spi_write_dummy_cycle = 0;
    spi_cfg.spi_data_mode         = detail::spi::spi_data_mode_t::spi_fullduplex;
    detail::spi::SPIMasterAccessor spi_dev{spi_bus, spi_cfg};
    detail::useResult(spi_dev.setConfig(spi_cfg));
    detail::useResult(spi_dev.beginTransaction());
    detail::useResult(spi_dev.transfer(detail::spi::TransferDesc{}, detail::data::ConstDataSpan{tx, sizeof(tx)},
                                       detail::data::DataSpan{rx, sizeof(rx)}));
    detail::data::MemorySource spi_source{detail::data::ConstDataSpan{tx, sizeof(tx)}};
    detail::data::MemorySink spi_sink{detail::data::DataSpan{rx, sizeof(rx)}};
    detail::data::LimitedSource limited_source{&spi_source, sizeof(tx)};
    detail::data::LimitedSink limited_sink{&spi_sink, sizeof(rx)};
    detail::useResult(i2c_dev.transfer(detail::i2c::TransferDesc{}, &limited_source, &limited_sink));
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

    detail::DummyUARTBus uart_bus;
    detail::uart::UARTBusConfig uart_bus_cfg;
    uart_bus_cfg.pin_tx = 17;
    uart_bus_cfg.pin_rx = 16;
    detail::useResult(uart_bus.init(uart_bus_cfg));
    detail::uart::UARTAccessConfig uart_cfg;
    uart_cfg.baud_rate             = 921600;
    uart_cfg.first_byte_timeout_ms = 10;
    uart_cfg.inter_byte_timeout_ms = 2;
    uart_cfg.write_timeout_ms      = 10;
    uart_cfg.data_bits             = 8;
    uart_cfg.stop_bits             = 1;
    uart_cfg.parity                = detail::uart::parity_t::none;
    detail::uart::UARTAccessor uart_dev{uart_bus, uart_cfg};
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

    detail::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    detail::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};
    detail::useResult(uart_tx.setConfig(uart_cfg));
    detail::useResult(uart_rx.setConfig(uart_cfg));
    detail::useResult(uart_tx.beginTxAccess());
    detail::useResult(uart_tx.endTxAccess());
    detail::useResult(uart_rx.beginRxAccess());
    detail::useResult(uart_rx.endRxAccess());
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

    // Frame codec: pure encode/decode plus the Source/Sink-driven
    // reader/writer. All calls below stay I/O-free (empty inputs).
    namespace frame = ::m5::hal::v1::frame;
    uint8_t frame_buf[frame::checkedFrameWireSize(1)];
    detail::useResult(frame::encodeDelimiter(detail::data::DataSpan{frame_buf, sizeof(frame_buf)}));
    detail::useResult(frame::encodeChecked(detail::data::DataSpan{frame_buf, sizeof(frame_buf)},
                                           frame::Kind::control, detail::data::ConstDataSpan{}));
    detail::useResult(
        frame::encodeData(detail::data::DataSpan{frame_buf, sizeof(frame_buf)}, 0, detail::data::ConstDataSpan{}));
    (void)frame::crc16CcittUpdate(0xFFFF, 0x00);
    (void)frame::check16(detail::data::ConstDataSpan{frame_buf, sizeof(frame_buf)});
    (void)frame::isCheckedKind(frame::Kind::data);
    (void)frame::isKnownKind(0x01);
    (void)frame::isDelimiter(detail::data::ConstDataSpan{});
    frame::View frame_view;
    (void)frame::decode(detail::data::ConstDataSpan{}, frame_view);  // need_more, no I/O
    detail::data::MemorySource frame_src{detail::data::ConstDataSpan{}};
    detail::data::MemorySink frame_snk{detail::data::DataSpan{}};
    frame::FrameReader frame_reader{frame_src};
    frame::FrameWriter frame_writer{frame_snk};
    detail::useResult(frame_reader.next(frame_view));      // empty source -> END_OF_STREAM
    detail::useResult(frame_writer.writeDelimiter());      // closed sink -> CLOSED

    // Bytecode: encoder + runner surface. All calls stay I/O-free
    // (closed sinks, empty scripts, no registered targets touched).
    namespace bytecode = ::m5::hal::v1::bytecode;
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
    namespace i2c_arduino  = ::m5::hal::v1::i2c::variant::arduino;
    namespace spi_arduino  = ::m5::hal::v1::spi::variant::arduino;
    namespace uart_arduino = ::m5::hal::v1::uart::variant::arduino;

    i2c_arduino::BusConfig i2c_cfg{&Wire, 22, 21};
    spi_arduino::BusConfig spi_cfg;
    spi_cfg.spi = &SPI;
    uart_arduino::BusConfig uart_cfg;
    uart_cfg.serial = &Serial1;

    static_assert(sizeof(i2c_arduino::Bus) > 0, "Arduino I2C Bus alias must be visible");
    static_assert(sizeof(spi_arduino::Bus) > 0, "Arduino SPI Bus alias must be visible");
    static_assert(sizeof(uart_arduino::Bus) > 0, "Arduino UART Bus alias must be visible");
    detail::useResult(i2c_cfg);
    detail::useResult(spi_cfg);
    detail::useResult(uart_cfg);
}
#endif

#if defined(ESP_PLATFORM)
inline void compileEspidfApiSurface(void)
{
#if M5HAL_ESPIDF_I2C_HAS_MASTER
    namespace i2c_espidf = ::m5::hal::v1::i2c::variant::espidf;
    i2c_espidf::BusConfig i2c_cfg{22, 21};
    static_assert(sizeof(i2c_espidf::Bus) > 0, "ESP-IDF I2C Bus alias must be visible");
    detail::useResult(i2c_cfg);
#endif

#if M5HAL_ESPIDF_SPI_HAS_MASTER
    namespace spi_espidf = ::m5::hal::v1::spi::variant::espidf;
    spi_espidf::BusConfig spi_cfg;
    static_assert(sizeof(spi_espidf::Bus) > 0, "ESP-IDF SPI Bus alias must be visible");
    detail::useResult(spi_cfg);
#endif

    namespace uart_espidf = ::m5::hal::v1::uart::variant::espidf;
    uart_espidf::BusConfig uart_cfg;
    static_assert(sizeof(uart_espidf::Bus) > 0, "ESP-IDF UART Bus alias must be visible");
    detail::useResult(uart_cfg);
}
#endif

#if defined(ESP_PLATFORM)
// Chip-level GPIO capabilities (PinBackup / ScopedPinBackup) are exposed under
// the public `m5::hal::v1::gpio` namespace directly from the platform layer,
// independent of which variant wins the GPIO HAL flat injection. Referencing
// them here via the public flat name keeps CI honest about that exposure
// (it would catch a regression where the symbols stop resolving).
inline void compileGpioCapabilityApiSurface(void)
{
    namespace gpio  = ::m5::hal::v1::gpio;
    namespace types = ::m5::hal::v1::types;

    static_assert(sizeof(gpio::PinBackup) > 0, "PinBackup must be visible under m5::hal::v1::gpio");
    static_assert(sizeof(gpio::ScopedPinBackup) > 0, "ScopedPinBackup must be visible under m5::hal::v1::gpio");

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
// flat-injected UART provider; this references its variant-qualified alias to
// keep CI honest about the exposure. Compile/surface only: it never opens a
// device (no open()/attach()), so it is safe to run in CI.
inline void compilePosixApiSurface(void)
{
    namespace uart_posix = ::m5::hal::v1::uart::variant::posix;

    static_assert(sizeof(uart_posix::Bus) > 0, "POSIX UART Bus alias must be visible");

    uart_posix::Bus bus;
    uart_posix::BusConfig uart_cfg;
    uart_cfg.device_path = nullptr;  // lazy open; nothing is opened here
    detail::useResult(bus.init(uart_cfg));
    detail::useResult(bus.nativeHandle());
}
#endif

inline void compileApiSurface(void)
{
    compileCommonApiSurface();
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

}  // namespace m5hal_build_check::v1

#endif
