// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_INL

#include "spi.hpp"

#include <M5Utility.hpp>

#include <cstddef>

#if defined(ARDUINO)

namespace m5::hal::v2::spi {

namespace {
namespace impl_arduino {

// Return type deduced: MSBFIRST/LSBFIRST are plain int constants on
// arduino-esp32 but a `BitOrder` enum on arduino-pico (RP2040) — SPISettings'
// constructor expects whichever type the core actually declared.
auto spiBitOrder(uint8_t order)
{
    return (order == 0) ? MSBFIRST : LSBFIRST;
}

// Same rationale as spiBitOrder() above: SPI_MODEn's type varies by core.
auto spiDataMode(uint8_t mode)
{
    switch (mode & 0x03) {
        case 0:
            return SPI_MODE0;
        case 1:
            return SPI_MODE1;
        case 2:
            return SPI_MODE2;
        case 3:
        default:
            return SPI_MODE3;
    }
}

void setPinLevel(types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        digitalWrite(static_cast<int>(pin), level ? HIGH : LOW);
    }
}

void setPinOutput(types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        pinMode(static_cast<int>(pin), OUTPUT);
        digitalWrite(static_cast<int>(pin), level ? HIGH : LOW);
    }
}

void setDC(types::gpio_number_t dc_pin, int8_t level)
{
    if (level >= 0 && dc_pin >= 0) {
        setPinLevel(dc_pin, level != 0);
    }
}

uint8_t metaByte(uint32_t value, uint8_t remaining)
{
    const uint8_t shift = static_cast<uint8_t>((remaining - 1u) * 8u);
    return static_cast<uint8_t>(value >> shift);
}

result_t<void> transferByte(::SPIClass& spi, uint8_t tx_byte, uint8_t* rx_byte)
{
    const uint8_t value = spi.transfer(tx_byte);
    if (rx_byte != nullptr) {
        *rx_byte = value;
    }
    return {};
}

result_t<void> sendMeta(::SPIClass& spi, uint32_t value, uint8_t bytes)
{
    if (bytes > 4) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    while (bytes > 0) {
        auto result = transferByte(spi, metaByte(value, bytes), nullptr);
        if (!result.has_value()) {
            return result;
        }
        --bytes;
    }
    return {};
}

result_t<void> sendDummy(::SPIClass& spi, uint8_t cycles)
{
    if (cycles == 0) {
        return {};
    }
#if defined(ESP_PLATFORM)
    // The arduino-esp32 SPIClass (whole ESP32 family) can clock an arbitrary bit
    // count, so honor sub-byte dummy directly (see the dummy_cycles portability
    // note in hal/v2/spi/spi.hpp). We are already inside `#if defined(ARDUINO)`,
    // so ESP_PLATFORM here means arduino-esp32. transferBits sends 1..32 bits per
    // call; loop for larger counts. MOSI is don't-care during dummy, driven high
    // (0xFFFFFFFF) to match the byte path. `out == nullptr` is safe (guarded in
    // esp32-hal-spi spiTransferBitsNL).
    uint16_t remaining = cycles;
    while (remaining > 0) {
        const uint8_t n = (remaining > 32) ? 32 : static_cast<uint8_t>(remaining);
        spi.transferBits(0xFFFFFFFF, nullptr, n);
        remaining -= n;
    }
    return {};
#else
    // Portable Arduino SPIClass clocks whole bytes only. Reject sub-byte counts
    // rather than silently rounding.
    if ((cycles & 0x07) != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    for (uint8_t i = 0; i < (cycles >> 3); ++i) {
        auto result = transferByte(spi, 0xFF, nullptr);
        if (!result.has_value()) {
            return result;
        }
    }
    return {};
#endif
}

result_t<void> transferChunk(::SPIClass& spi, data::ConstDataSpan tx_span, data::DataSpan rx_span)
{
#if defined(ESP_PLATFORM)
    // arduino-esp32 SPIClass extensions: separate tx/rx buffers in one call.
    const size_t common = (tx_span.size < rx_span.size) ? tx_span.size : rx_span.size;
    if (common > 0) {
        spi.transferBytes(tx_span.data, rx_span.data, static_cast<uint32_t>(common));
    }
    if (tx_span.size > common) {
        spi.writeBytes(tx_span.data + common, static_cast<uint32_t>(tx_span.size - common));
    }
    if (rx_span.size > common) {
        spi.transferBytes(nullptr, rx_span.data + common, static_cast<uint32_t>(rx_span.size - common));
    }
#else
    // Portable Arduino SPIClass exposes only transfer(uint8_t) (one byte,
    // simultaneous tx+rx) and transfer(void*, size_t) (in-place, same
    // buffer for tx and rx). Neither fits separate/differently-sized tx
    // and rx spans, so fall back to a byte loop through transferByte().
    const size_t common = (tx_span.size < rx_span.size) ? tx_span.size : rx_span.size;
    for (size_t i = 0; i < common; ++i) {
        auto result = transferByte(spi, tx_span.data[i], &rx_span.data[i]);
        if (!result.has_value()) {
            return result;
        }
    }
    for (size_t i = common; i < tx_span.size; ++i) {
        auto result = transferByte(spi, tx_span.data[i], nullptr);
        if (!result.has_value()) {
            return result;
        }
    }
    for (size_t i = common; i < rx_span.size; ++i) {
        auto result = transferByte(spi, 0xFF, &rx_span.data[i]);
        if (!result.has_value()) {
            return result;
        }
    }
#endif
    return {};
}

}  // namespace impl_arduino
}  // namespace

error::error_t Bus_arduino::attach(::SPIClass& spi)
{
    if (_spi) {
        (void)release();
    }
    _spi      = &spi;
    _owns_spi = false;
    return error::error_t::OK;
}

result_t<void> Bus_arduino::init(const BusConfig_arduino& config)
{
    _config = config;
    if (_spi) {
        (void)release();
    }
    auto* spi = config.spi != nullptr ? config.spi : &SPI;

#if defined(ESP_PLATFORM)
    // arduino-esp32 SPIClass::begin(sck, miso, mosi, ss) is an Espressif
    // extension; no other supported Arduino core (see _checker.hpp) has a
    // matching overload — pins are fixed per SPIClass instance there.
    const int clk  = static_cast<int>(_config.pin_clk);
    const int miso = static_cast<int>(_config.pin_miso);
    const int mosi = static_cast<int>(_config.pin_mosi);
    if (_config.pin_clk >= 0 || _config.pin_miso >= 0 || _config.pin_mosi >= 0) {
        spi->begin(clk, miso, mosi, -1);
    } else {
        spi->begin();
    }
#else
    // Portable cores fix SCK/MISO/MOSI per SPIClass instance (board variant
    // file); configured pins cannot be honored here and are ignored.
    spi->begin();
#endif
    if (_config.pin_dc >= 0) {
        impl_arduino::setPinOutput(_config.pin_dc, true);
    }

    auto err = attach(*spi);
    if (error::isError(err)) {
        spi->end();
        return m5::stl::make_unexpected(err);
    }
    _owns_spi = true;
    return {};
}

result_t<void> Bus_arduino::release(void)
{
    if (_spi && _owns_spi) {
        _spi->end();
    }
    _spi      = nullptr;
    _owns_spi = false;
    return {};
}

result_t<void> Bus_arduino::beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    if (_spi == nullptr || cfg.freq == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _spi->beginTransaction(
        ::SPISettings(cfg.freq, impl_arduino::spiBitOrder(cfg.spi_order), impl_arduino::spiDataMode(cfg.spi_mode)));
    impl_arduino::setPinOutput(cfg.pin_cs, false);
    const auto dc_pin = cfg.pin_dc >= 0 ? cfg.pin_dc : _config.pin_dc;
    impl_arduino::setDC(dc_pin, 1);
    return {};
}

result_t<void> Bus_arduino::endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    impl_arduino::setPinLevel(cfg.pin_cs, true);
    if (_spi != nullptr) {
        _spi->endTransaction();
    }
    return {};
}

result_t<void> Bus_arduino::transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                                     const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                     size_t rx_len)
{
    (void)owner;
    _transfer_totals.clear();
    // This variant drives a single-lane MOSI/MISO pair through SPIClass.
    // Multi-lane modes (dual/quad/octal) are physically unimplemented:
    // always reject. Half-duplex modes share the full-duplex wire shape
    // as long as a transfer carries data in only ONE direction (the
    // command/address meta phase is already sent sequentially — the DC
    // demos rely on that); what cannot be honored is half-duplex with
    // BOTH src and dst data, which full-duplex clocking would corrupt.
    {
        using spi::spi_data_mode_t;
        const auto mode       = cfg.spi_data_mode;
        const bool multi_lane = mode == spi_data_mode_t::DualOutput || mode == spi_data_mode_t::DualIo ||
                                mode == spi_data_mode_t::QuadOutput || mode == spi_data_mode_t::QuadIo ||
                                mode == spi_data_mode_t::OctalOutput || mode == spi_data_mode_t::OctalIo;
        const bool half_duplex = mode == spi_data_mode_t::HalfDuplex || mode == spi_data_mode_t::HalfDuplexWithDcPin ||
                                 mode == spi_data_mode_t::HalfDuplexWithDcBit;
        if (multi_lane ||
            (half_duplex && src != nullptr && tx_len > 0 && !src->eof() && dst != nullptr && rx_len > 0)) {
            return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
        }
    }
    if (_spi == nullptr || desc.command_bytes > 4 || desc.address_bytes > 4) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    // Per-device D/C override: a non-negative accessor pin_dc beats the
    // bus-level default. The override pin is switched to output once and
    // cached (the bus-level pin was set up in init()).
    const types::gpio_number_t dc_pin = cfg.pin_dc >= 0 ? cfg.pin_dc : _config.pin_dc;
    if (cfg.pin_dc >= 0 && cfg.pin_dc != _last_acc_dc) {
        impl_arduino::setPinOutput(cfg.pin_dc, true);
        _last_acc_dc = cfg.pin_dc;
    }

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && desc.dc_level_valid) {
        impl_arduino::setDC(dc_pin, desc.dc_level ? 1 : 0);
    }

    impl_arduino::setDC(dc_pin, desc.command_dc_level);
    auto meta = impl_arduino::sendMeta(*_spi, desc.command, desc.command_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    impl_arduino::setDC(dc_pin, desc.address_dc_level);
    meta = impl_arduino::sendMeta(*_spi, desc.address, desc.address_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    auto dummy = impl_arduino::sendDummy(*_spi, desc.dummy_cycles);
    if (!dummy.has_value()) {
        return m5::stl::make_unexpected(dummy.error());
    }

    impl_arduino::setDC(dc_pin, desc.data_dc_level);
    if (has_phase_dc && desc.data_dc_level < 0) {
        impl_arduino::setDC(dc_pin, 1);
    }

    size_t transferred  = 0;
    size_t received     = 0;
    size_t tx_remaining = tx_len;
    size_t rx_remaining = rx_len;
    while ((src != nullptr && tx_remaining > 0 && !src->eof()) ||
           (dst != nullptr && rx_remaining > 0 && !dst->closed())) {
        data::ConstDataSpan tx_span{};
        data::DataSpan rx_span{};
        if (src != nullptr && tx_remaining > 0 && !src->eof()) {
            auto peeked = src->peek(tx_remaining);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            tx_span = peeked.value().first(tx_remaining);
        }
        if (dst != nullptr && rx_remaining > 0 && !dst->closed()) {
            auto reserved = dst->reserve(rx_remaining);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            rx_span = reserved.value().first(rx_remaining);
        }

        const size_t chunk_len = (tx_span.size > rx_span.size) ? tx_span.size : rx_span.size;
        if (chunk_len == 0) {
            break;
        }
        auto chunk = impl_arduino::transferChunk(*_spi, tx_span, rx_span);
        if (!chunk.has_value()) {
            return m5::stl::make_unexpected(chunk.error());
        }
        if (tx_span.size > 0) {
            auto advanced = src->advance(tx_span.size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            tx_remaining -= tx_span.size;
        }
        if (rx_span.size > 0) {
            auto committed = dst->commit(rx_span.size);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
            rx_remaining -= rx_span.size;
        }
        transferred += tx_span.size;
        received += rx_span.size;
    }

    _transfer_totals.tx += transferred;
    _transfer_totals.rx += received;
    return {};
}

result_t<bus::TransferTotals> Bus_arduino::waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    auto totals = _transfer_totals;
    _transfer_totals.clear();
    return totals;
}

}  // namespace m5::hal::v2::spi

#endif

#endif
