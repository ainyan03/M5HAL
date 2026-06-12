// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_SPI_SPI_INL

#include "spi.hpp"

#include <M5Utility.hpp>

#include <cstddef>

#if defined(ARDUINO)

namespace m5::variants::frameworks::arduino::hal::v1::spi {

using namespace ::m5::hal::v1;

namespace {

uint8_t spiBitOrder(uint8_t order)
{
    return (order == 0) ? MSBFIRST : LSBFIRST;
}

uint8_t spiDataMode(uint8_t mode)
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

void setPinLevel(::m5::hal::v1::types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        digitalWrite(static_cast<int>(pin), level ? HIGH : LOW);
    }
}

void setPinOutput(::m5::hal::v1::types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        pinMode(static_cast<int>(pin), OUTPUT);
        digitalWrite(static_cast<int>(pin), level ? HIGH : LOW);
    }
}

void setDC(const ::m5::hal::v1::spi::SPIBusConfig& bus_cfg, int8_t level)
{
    if (level >= 0 && bus_cfg.pin_dc >= 0) {
        setPinLevel(bus_cfg.pin_dc, level != 0);
    }
}

uint8_t metaByte(uint32_t value, uint8_t remaining)
{
    const uint8_t shift = static_cast<uint8_t>((remaining - 1u) * 8u);
    return static_cast<uint8_t>(value >> shift);
}

::m5::hal::v1::result_t<void> transferByte(::SPIClass& spi, uint8_t tx_byte, uint8_t* rx_byte)
{
    const uint8_t value = spi.transfer(tx_byte);
    if (rx_byte != nullptr) {
        *rx_byte = value;
    }
    return {};
}

::m5::hal::v1::result_t<void> sendMeta(::SPIClass& spi, uint32_t value, uint8_t bytes)
{
    if (bytes > 4) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
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

::m5::hal::v1::result_t<void> sendDummy(::SPIClass& spi, uint8_t cycles)
{
    if (cycles == 0) {
        return {};
    }
#if defined(ESP_PLATFORM)
    // The arduino-esp32 SPIClass (whole ESP32 family) can clock an arbitrary bit
    // count, so honor sub-byte dummy directly (see the dummy_cycles portability
    // note in hal/v1/spi/spi.hpp). We are already inside `#if defined(ARDUINO)`,
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
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
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

::m5::hal::v1::result_t<void> transferChunk(::SPIClass& spi, ::m5::hal::v1::data::ConstDataSpan tx_span,
                                            ::m5::hal::v1::data::DataSpan rx_span)
{
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
    return {};
}

}  // namespace

::m5::hal::v1::error::error_t Bus::attach(::SPIClass& spi)
{
    if (_spi) {
        (void)release();
    }
    _spi      = &spi;
    _owns_spi = false;
    return ::m5::hal::v1::error::error_t::OK;
}

::m5::hal::v1::result_t<void> Bus::init(const BusConfig& config)
{
    _config = config;
    if (_spi) {
        (void)release();
    }
    auto* spi = config.spi;
    if (spi == nullptr) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    const int clk  = static_cast<int>(_config.pin_clk);
    const int miso = static_cast<int>(_config.pin_miso);
    const int mosi = static_cast<int>(_config.pin_mosi);
    if (_config.pin_clk >= 0 || _config.pin_miso >= 0 || _config.pin_mosi >= 0) {
        spi->begin(clk, miso, mosi, -1);
    } else {
        spi->begin();
    }
    if (_config.pin_dc >= 0) {
        setPinOutput(_config.pin_dc, true);
    }

    auto err = attach(*spi);
    if (::m5::hal::v1::error::isError(err)) {
        spi->end();
        return m5::stl::make_unexpected(err);
    }
    _owns_spi = true;
    return {};
}

::m5::hal::v1::result_t<void> Bus::release(void)
{
    if (_spi && _owns_spi) {
        _spi->end();
    }
    _spi      = nullptr;
    _owns_spi = false;
    return {};
}

::m5::hal::v1::result_t<void> Bus::beginTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                    const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg)
{
    (void)owner;
    if (_spi == nullptr || cfg.freq == 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _spi->beginTransaction(::SPISettings(cfg.freq, spiBitOrder(cfg.spi_order), spiDataMode(cfg.spi_mode)));
    setPinOutput(cfg.pin_cs, false);
    return {};
}

::m5::hal::v1::result_t<void> Bus::endTransaction(::m5::hal::v1::bus::Accessor* owner,
                                                  const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg)
{
    (void)owner;
    setPinLevel(cfg.pin_cs, true);
    if (_spi != nullptr) {
        _spi->endTransaction();
    }
    return {};
}

::m5::hal::v1::result_t<size_t> Bus::transfer(::m5::hal::v1::bus::Accessor* owner,
                                              const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
                                              const ::m5::hal::v1::spi::TransferDesc& desc,
                                              ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    (void)owner;
    // This variant drives a single-lane MOSI/MISO pair through SPIClass.
    // Multi-lane modes (dual/quad/octal) are physically unimplemented:
    // always reject. Half-duplex modes share the full-duplex wire shape
    // as long as a transfer carries data in only ONE direction (the
    // command/address meta phase is already sent sequentially — the DC
    // demos rely on that); what cannot be honored is half-duplex with
    // BOTH tx and rx data, which full-duplex clocking would corrupt
    // (S16 D10).
    {
        using ::m5::hal::v1::spi::spi_data_mode_t;
        const auto mode       = cfg.spi_data_mode;
        const bool multi_lane = mode == spi_data_mode_t::spi_dual_output || mode == spi_data_mode_t::spi_dual_io ||
                                mode == spi_data_mode_t::spi_quad_output || mode == spi_data_mode_t::spi_quad_io ||
                                mode == spi_data_mode_t::spi_octal_output || mode == spi_data_mode_t::spi_octal_io;
        const bool half_duplex = mode == spi_data_mode_t::spi_halfduplex ||
                                 mode == spi_data_mode_t::spi_halfduplex_with_dc_pin ||
                                 mode == spi_data_mode_t::spi_halfduplex_with_dc_bit;
        if (multi_lane || (half_duplex && tx != nullptr && !tx->eof() && rx != nullptr)) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
        }
    }
    if (_spi == nullptr || desc.command_bytes > 4 || desc.address_bytes > 4) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && desc.dc_level_valid) {
        setDC(_config, desc.dc_level ? 1 : 0);
    }

    setDC(_config, desc.command_dc_level);
    auto meta = sendMeta(*_spi, desc.command, desc.command_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    setDC(_config, desc.address_dc_level);
    meta = sendMeta(*_spi, desc.address, desc.address_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    auto dummy = sendDummy(*_spi, desc.dummy_cycles);
    if (!dummy.has_value()) {
        return m5::stl::make_unexpected(dummy.error());
    }

    setDC(_config, desc.data_dc_level);

    size_t transferred = 0;
    while ((tx != nullptr && !tx->eof()) || (rx != nullptr && !rx->closed())) {
        ::m5::hal::v1::data::ConstDataSpan tx_span{};
        ::m5::hal::v1::data::DataSpan rx_span{};
        if (tx != nullptr && !tx->eof()) {
            auto peeked = tx->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            tx_span = peeked.value();
        }
        if (rx != nullptr && !rx->closed()) {
            auto reserved = rx->reserve(SIZE_MAX);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            rx_span = reserved.value();
        }

        const size_t chunk_len = (tx_span.size > rx_span.size) ? tx_span.size : rx_span.size;
        if (chunk_len == 0) {
            break;
        }
        auto chunk = transferChunk(*_spi, tx_span, rx_span);
        if (!chunk.has_value()) {
            return m5::stl::make_unexpected(chunk.error());
        }
        if (tx_span.size > 0) {
            auto advanced = tx->advance(tx_span.size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
        }
        if (rx_span.size > 0) {
            auto committed = rx->commit(rx_span.size);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
        }
        transferred += chunk_len;
    }

    return transferred;
}

}  // namespace m5::variants::frameworks::arduino::hal::v1::spi

#endif

#endif
