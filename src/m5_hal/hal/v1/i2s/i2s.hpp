#ifndef M5_HAL_I2S_I2S_HPP_
#define M5_HAL_I2S_I2S_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::i2s {

/// I2S は DMA 駆動の連続ストリームであり、I2C / SPI のようなトランザクション境界を
/// 持たない。API は uart と同型 (write / writableBytes)。当面は master TX (再生) のみで、
/// スロット形式は Philips standard 固定。RX (録音) は pin_din を予約済みの将来拡張。

struct I2SBusConfig : public bus::BusConfig {
    types::gpio_number_t pin_bclk = -1;
    types::gpio_number_t pin_ws   = -1;
    types::gpio_number_t pin_dout = -1;
    types::gpio_number_t pin_din  = -1;    ///< RX (録音) 用。当面未使用
    types::gpio_number_t pin_mclk = -1;    ///< 外部 MCLK を要するコーデック用。-1 = 出力しない
    size_t tx_buffer_size         = 8192;  ///< DMA バッファ総量の目安 (backend が記述子構成へ丸める)

    constexpr I2SBusConfig(void) : bus::BusConfig{types::bus_kind_t::I2S}
    {
    }
};

struct I2SAccessConfig : public bus::AccessConfig {
    uint32_t sample_rate_hz   = 44100;
    uint32_t timeout_ms       = 1000;  ///< Bus lock timeout.
    uint32_t write_timeout_ms = 1000;  ///< write が DMA の空きを待つ上限。0 = non-blocking
    uint8_t bits_per_sample   = 16;    ///< 当面 16 のみ
    uint8_t channels          = 2;     ///< 1 = mono / 2 = stereo

    constexpr I2SAccessConfig(void) : bus::AccessConfig{types::bus_kind_t::I2S}
    {
    }
};

struct I2SBus;

struct I2STxAccessor : public bus::Accessor, public data::StreamWriter {
    I2STxAccessor(I2SBus& bus, const I2SAccessConfig& access_config);

    const I2SAccessConfig& getConfig(void) const override
    {
        return _access_config;
    }
    I2SBus& getI2SBus(void) const;

    m5::stl::expected<void, error::error_t> setConfig(const I2SAccessConfig& cfg);

    m5::stl::expected<size_t, error::error_t> write(data::ConstDataSpan tx_bytes) override;
    m5::stl::expected<size_t, error::error_t> write(data::Source& tx, size_t len);
    m5::stl::expected<size_t, error::error_t> write(const uint8_t* tx, size_t len);

    /// いま write してもブロックせず受理される量 (DMA バッファの空きバイト数)。
    m5::stl::expected<size_t, error::error_t> writableBytes(void);

protected:
    I2SAccessConfig _access_config;
};

struct I2SBus : public bus::Bus {
    const I2SBusConfig& getConfig(void) const override
    {
        return _config;
    }

    /// 受理できた分のバイト数を返す (write_timeout_ms 内に DMA へ受理できた量。短い戻りは
    /// 正常)。underrun はエラーにしない (DMA が枯れたら無音を出力し、次の write で再開する)。
    virtual m5::stl::expected<size_t, error::error_t> write(bus::Accessor* owner, const I2SAccessConfig& cfg,
                                                            data::Source* tx, size_t len);
    virtual m5::stl::expected<size_t, error::error_t> writableBytes(bus::Accessor* owner, const I2SAccessConfig& cfg);

protected:
    I2SBusConfig _config;
};

}  // namespace m5::hal::v1::i2s

#endif
