// SPDX-License-Identifier: MIT
#ifndef M5_HAL_BYTECODE_BYTECODE_INL_
#define M5_HAL_BYTECODE_BYTECODE_INL_

#include "bytecode.hpp"

#include <string.h>

namespace m5::hal::v2::bytecode {

namespace {
using error_t = m5::hal::v2::error::error_t;

void putU16(uint8_t* dst, uint16_t v)
{
    dst[0] = static_cast<uint8_t>(v & 0xFF);
    dst[1] = static_cast<uint8_t>(v >> 8);
}

void putU32(uint8_t* dst, uint32_t v)
{
    dst[0] = static_cast<uint8_t>(v & 0xFF);
    dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

// Sequential little-endian field reader. Each accessor returns false
// when the remaining bytes cannot hold the field - the basis of the
// tolerant config decode (read the fields you know, skip the rest).
struct FieldReader {
    const uint8_t* p = nullptr;
    size_t n         = 0;

    explicit FieldReader(data::ConstDataSpan src) : p{src.data}, n{src.size}
    {
    }

    bool u8(uint8_t& v)
    {
        if (n < 1) {
            return false;
        }
        v = p[0];
        p += 1;
        n -= 1;
        return true;
    }
    bool u16(uint16_t& v)
    {
        if (n < 2) {
            return false;
        }
        v = static_cast<uint16_t>(p[0] | (p[1] << 8));
        p += 2;
        n -= 2;
        return true;
    }
    bool u32(uint32_t& v)
    {
        if (n < 4) {
            return false;
        }
        v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
            (static_cast<uint32_t>(p[3]) << 24);
        p += 4;
        n -= 4;
        return true;
    }
    bool i16(int16_t& v)
    {
        uint16_t raw = 0;
        if (!u16(raw)) {
            return false;
        }
        v = static_cast<int16_t>(raw);
        return true;
    }
    bool boolean(bool& v)
    {
        uint8_t raw = 0;
        if (!u8(raw)) {
            return false;
        }
        v = raw != 0;
        return true;
    }
};

// ---- config payloads --------------------------------------------------------
// Sizes and the timeout-field offsets live in bytecode.hpp (shared with
// the server prescan); the remaining offsets are local to the
// encode/decode pairs below.

void encodeConfig(uint8_t* dst, const i2c::MasterAccessConfig& cfg)
{
    putU32(dst + 0, cfg.freq);
    putU32(dst + kI2CConfigWireTimeoutOffset, cfg.wire_timeout_ms);
    putU16(dst + 8, cfg.i2c_addr);
    dst[10] = static_cast<uint8_t>((cfg.address_is_10bit ? 0x01 : 0x00) | (cfg.use_restart ? 0x02 : 0x00));
    dst[11] = cfg.register_address_bytes;
}

void decodeConfig(data::ConstDataSpan src, i2c::MasterAccessConfig& cfg)
{
    FieldReader r{src};
    uint8_t flags = 0;
    if (!r.u32(cfg.freq) || !r.u32(cfg.wire_timeout_ms) || !r.u16(cfg.i2c_addr) || !r.u8(flags)) {
        return;
    }
    cfg.address_is_10bit = (flags & 0x01) != 0;
    cfg.use_restart      = (flags & 0x02) != 0;
    (void)r.u8(cfg.register_address_bytes);
}

void encodeConfig(uint8_t* dst, const spi::MasterAccessConfig& cfg)
{
    putU16(dst + 0, static_cast<uint16_t>(cfg.pin_cs));
    putU16(dst + 2, static_cast<uint16_t>(cfg.pin_dc));
    putU32(dst + 4, cfg.freq);
    dst[8]  = static_cast<uint8_t>(cfg.spi_data_mode);
    dst[9]  = static_cast<uint8_t>((cfg.spi_mode & 0x03) | (cfg.spi_order ? 0x04 : 0x00));
    dst[10] = cfg.spi_command_length;
    dst[11] = cfg.spi_address_length;
    dst[12] = cfg.spi_read_dummy_cycle;
    dst[13] = cfg.spi_write_dummy_cycle;
}

void decodeConfig(data::ConstDataSpan src, spi::MasterAccessConfig& cfg)
{
    FieldReader r{src};
    uint8_t data_mode = 0;
    uint8_t mode_bits = 0;
    // pin_cs/pin_dc default to the registered (server-side) values: the remote
    // SPI proxy sends -1 because it does not know the device's physical wiring
    // (spec/design/remote.md §SPI — CS is realized server-side). A host that
    // sets them explicitly (>= 0) still overrides.
    int16_t pin_cs = cfg.pin_cs;
    int16_t pin_dc = cfg.pin_dc;
    if (!r.i16(pin_cs) || !r.i16(pin_dc) || !r.u32(cfg.freq) || !r.u8(data_mode) || !r.u8(mode_bits)) {
        return;
    }
    if (pin_cs >= 0) {
        cfg.pin_cs = static_cast<types::gpio_number_t>(pin_cs);
    }
    if (pin_dc >= 0) {
        cfg.pin_dc = static_cast<types::gpio_number_t>(pin_dc);
    }
    cfg.spi_data_mode = static_cast<spi::spi_data_mode_t>(data_mode);
    cfg.spi_mode      = mode_bits & 0x03;
    cfg.spi_order     = (mode_bits & 0x04) ? 1 : 0;
    if (!r.u8(cfg.spi_command_length) || !r.u8(cfg.spi_address_length)) {
        return;
    }
    if (!r.u8(cfg.spi_read_dummy_cycle)) {
        return;
    }
    (void)r.u8(cfg.spi_write_dummy_cycle);
}

void encodeConfig(uint8_t* dst, const uart::AccessConfig& cfg)
{
    putU32(dst + 0, cfg.baud_rate);
    putU32(dst + kUARTConfigFirstByteTimeoutOffset, cfg.first_byte_timeout_ms);
    putU32(dst + kUARTConfigInterByteTimeoutOffset, cfg.inter_byte_timeout_ms);
    putU32(dst + kUARTConfigWriteTimeoutOffset, cfg.write_timeout_ms);
    dst[16] = cfg.data_bits;
    dst[17] = cfg.stop_bits;
    dst[18] = static_cast<uint8_t>(cfg.parity);
    dst[19] = cfg.invert ? 1 : 0;
}

void decodeConfig(data::ConstDataSpan src, uart::AccessConfig& cfg)
{
    FieldReader r{src};
    uint8_t parity = 0;
    if (!r.u32(cfg.baud_rate) || !r.u32(cfg.first_byte_timeout_ms) || !r.u32(cfg.inter_byte_timeout_ms) ||
        !r.u32(cfg.write_timeout_ms)) {
        return;
    }
    if (!r.u8(cfg.data_bits) || !r.u8(cfg.stop_bits) || !r.u8(parity)) {
        return;
    }
    cfg.parity = static_cast<uart::parity_t>(parity);
    (void)r.boolean(cfg.invert);
}

void encodeConfig(uint8_t* dst, const i2s::AccessConfig& cfg)
{
    putU32(dst + 0, cfg.sample_rate_hz);
    putU32(dst + kI2SConfigWriteTimeoutOffset, cfg.write_timeout_ms);
    dst[8] = cfg.bits_per_sample;
    dst[9] = cfg.channels;
    putU32(dst + kI2SConfigReadTimeoutOffset, cfg.read_timeout_ms);
}

void decodeConfig(data::ConstDataSpan src, i2s::AccessConfig& cfg)
{
    FieldReader r{src};
    if (!r.u32(cfg.sample_rate_hz) || !r.u32(cfg.write_timeout_ms)) {
        return;
    }
    if (!r.u8(cfg.bits_per_sample)) {
        return;
    }
    if (!r.u8(cfg.channels)) {
        return;
    }
    (void)r.u32(cfg.read_timeout_ms);
}

// ---- transfer meta ----------------------------------------------------------

constexpr size_t kSPIMetaSize = 15;

size_t i2cMetaSize(const i2c::TransferDesc& desc)
{
    return 1 + desc.prefix_len;
}

void encodeMeta(uint8_t* dst, const i2c::TransferDesc& desc)
{
    dst[0] = desc.prefix_len;
    ::memcpy(dst + 1, desc.prefix, desc.prefix_len);
}

bool decodeMeta(data::ConstDataSpan src, i2c::TransferDesc& desc)
{
    FieldReader r{src};
    if (!r.u8(desc.prefix_len) || desc.prefix_len > i2c::TransferDesc::PREFIX_CAPACITY || r.n < desc.prefix_len) {
        return false;
    }
    ::memcpy(desc.prefix, r.p, desc.prefix_len);
    return true;
}

void encodeMeta(uint8_t* dst, const spi::TransferDesc& desc)
{
    dst[0] = static_cast<uint8_t>((desc.dc_level_valid ? 0x01 : 0x00) | (desc.dc_level ? 0x02 : 0x00));
    dst[1] = static_cast<uint8_t>(desc.command_dc_level);
    dst[2] = static_cast<uint8_t>(desc.address_dc_level);
    dst[3] = static_cast<uint8_t>(desc.data_dc_level);
    putU32(dst + 4, desc.command);
    putU32(dst + 8, desc.address);
    dst[12] = desc.command_bytes;
    dst[13] = desc.address_bytes;
    dst[14] = desc.dummy_cycles;
}

bool decodeMeta(data::ConstDataSpan src, spi::TransferDesc& desc)
{
    if (src.size < kSPIMetaSize) {
        return false;
    }
    FieldReader r{src};
    uint8_t flags = 0, cmd_dc = 0, addr_dc = 0, data_dc = 0;
    (void)r.u8(flags);
    (void)r.u8(cmd_dc);
    (void)r.u8(addr_dc);
    (void)r.u8(data_dc);
    (void)r.u32(desc.command);
    (void)r.u32(desc.address);
    (void)r.u8(desc.command_bytes);
    (void)r.u8(desc.address_bytes);
    (void)r.u8(desc.dummy_cycles);
    desc.dc_level_valid   = (flags & 0x01) != 0;
    desc.dc_level         = (flags & 0x02) != 0;
    desc.command_dc_level = static_cast<int8_t>(cmd_dc);
    desc.address_dc_level = static_cast<int8_t>(addr_dc);
    desc.data_dc_level    = static_cast<int8_t>(data_dc);
    return true;
}

// Grow a peek until it can lend `need` bytes. Returns BUFFER_UNDERFLOW
// when two consecutive peeks make no progress: a truncated script, a
// scratch smaller than the instruction, or a stream that stayed silent
// for a whole timeout period (StreamSource::peek blocks for the missing
// bytes up to the reader's timeouts, so byte gaps don't trip this).
result_t<data::ConstDataSpan> peekAtLeast(data::Source& src, size_t need)
{
    size_t prev = static_cast<size_t>(-1);
    for (;;) {
        auto peeked = src.peek(need);
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        if (peeked.value().size >= need) {
            return peeked.value();
        }
        if (peeked.value().size == prev) {
            return m5::stl::make_unexpected(error_t::BUFFER_UNDERFLOW);
        }
        prev = peeked.value().size;
    }
}

result_t<size_t> checkedAdd(size_t lhs, size_t rhs)
{
    if (lhs > static_cast<size_t>(-1) - rhs) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    return lhs + rhs;
}

result_t<size_t> checkedPayload(size_t base, size_t extra)
{
    return checkedAdd(base, extra);
}

result_t<size_t> checkedPayload(size_t base, size_t extra0, size_t extra1)
{
    auto sum = checkedAdd(base, extra0);
    if (!sum.has_value()) {
        return m5::stl::make_unexpected(sum.error());
    }
    return checkedAdd(sum.value(), extra1);
}

}  // namespace

namespace detail {

// Thin public entry points over the anonymous-namespace packing above
// (kept next to their decode counterparts). Qualified lookup resolves
// to the unnamed-namespace definitions, not back into detail.
void encodeConfig(uint8_t* dst, const i2c::MasterAccessConfig& cfg)
{
    m5::hal::v2::bytecode::encodeConfig(dst, cfg);
}
void encodeConfig(uint8_t* dst, const spi::MasterAccessConfig& cfg)
{
    m5::hal::v2::bytecode::encodeConfig(dst, cfg);
}
void encodeConfig(uint8_t* dst, const uart::AccessConfig& cfg)
{
    m5::hal::v2::bytecode::encodeConfig(dst, cfg);
}
void encodeConfig(uint8_t* dst, const i2s::AccessConfig& cfg)
{
    m5::hal::v2::bytecode::encodeConfig(dst, cfg);
}

}  // namespace detail

LenVar decodeLenVar(data::ConstDataSpan src)
{
    LenVar out;
    if (src.data == nullptr || src.size < 1) {
        return out;
    }
    const uint8_t head = src.data[0];
    if (head <= 0xFC) {
        out.value    = head;
        out.consumed = 1;
        return out;
    }
    if (head == 0xFF) {
        out.valid = false;
        return out;
    }
    if (head == 0xFD) {
        if (src.size < 3) {
            return out;
        }
        out.value    = static_cast<size_t>(src.data[1] | (src.data[2] << 8));
        out.consumed = 3;
        return out;
    }
    // head == 0xFE
    if (src.size < 5) {
        return out;
    }
    out.value = static_cast<size_t>(src.data[1]) | (static_cast<size_t>(src.data[2]) << 8) |
                (static_cast<size_t>(src.data[3]) << 16) | (static_cast<size_t>(src.data[4]) << 24);
    out.consumed = 5;
    return out;
}

size_t encodeLenVar(uint8_t* dst, size_t value)
{
    if (value <= 0xFC) {
        dst[0] = static_cast<uint8_t>(value);
        return 1;
    }
    if (value <= 0xFFFF) {
        dst[0] = 0xFD;
        putU16(dst + 1, static_cast<uint16_t>(value));
        return 3;
    }
    dst[0] = 0xFE;
    putU32(dst + 1, static_cast<uint32_t>(value));
    return 5;
}

// ---- BytecodeEncoder --------------------------------------------------------

result_t<data::DataSpan> BytecodeEncoder::beginInstruction(OpCode opcode, size_t payload_size)
{
    // A payload near SIZE_MAX (broken caller span) would wrap the size
    // arithmetic below; reject it before any reservation happens.
    if (payload_size > static_cast<size_t>(-1) - 8) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const size_t size_field = 1 + payload_size;
    const size_t prefix     = lenVarSize(size_field);
    const size_t total      = prefix + size_field;
    auto reserved           = _sink->reserve(total);
    if (!reserved.has_value()) {
        return m5::stl::make_unexpected(reserved.error());
    }
    if (reserved.value().size < total) {
        return m5::stl::make_unexpected(_sink->closed() ? error_t::CLOSED : error_t::BUFFER_OVERFLOW);
    }
    uint8_t* p = reserved.value().data;
    encodeLenVar(p, size_field);
    p[prefix]   = static_cast<uint8_t>(opcode);
    _instr_size = total;
    return data::DataSpan{p + prefix + 1, payload_size};
}

result_t<void> BytecodeEncoder::emit(void)
{
    return _sink->commit(_instr_size);
}

result_t<void> BytecodeEncoder::delayMs(uint32_t ms)
{
    auto payload = beginInstruction(OpCode::DelayMs, 4);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    putU32(payload.value().data, ms);
    return emit();
}

result_t<void> BytecodeEncoder::configure(uint8_t bus_id, const i2c::MasterAccessConfig& cfg)
{
    auto payload = beginInstruction(OpCode::BusConfigure, 2 + kI2CConfigSize);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::I2C);
    p[1]       = bus_id;
    encodeConfig(p + 2, cfg);
    return emit();
}

result_t<void> BytecodeEncoder::configure(uint8_t bus_id, const spi::MasterAccessConfig& cfg)
{
    auto payload = beginInstruction(OpCode::BusConfigure, 2 + kSPIConfigSize);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::SPI);
    p[1]       = bus_id;
    encodeConfig(p + 2, cfg);
    return emit();
}

result_t<void> BytecodeEncoder::configure(uint8_t bus_id, const uart::AccessConfig& cfg)
{
    auto payload = beginInstruction(OpCode::BusConfigure, 2 + kUARTConfigSize);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::UART);
    p[1]       = bus_id;
    encodeConfig(p + 2, cfg);
    return emit();
}

result_t<void> BytecodeEncoder::configure(types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan cfg_bytes)
{
    if (cfg_bytes.data == nullptr && cfg_bytes.size != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedAdd(static_cast<size_t>(2), cfg_bytes.size);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::BusConfigure, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(kind);
    p[1]       = bus_id;
    if (cfg_bytes.size != 0) {
        ::memcpy(p + 2, cfg_bytes.data, cfg_bytes.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::i2sConfig(uint8_t bus_id, const i2s::AccessConfig& cfg)
{
    auto payload = beginInstruction(OpCode::BusConfigure, 2 + kI2SConfigSize);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::I2S);
    p[1]       = bus_id;
    encodeConfig(p + 2, cfg);
    return emit();
}

result_t<void> BytecodeEncoder::transfer(uint8_t bus_id, const i2c::TransferDesc& desc, data::ConstDataSpan src,
                                         size_t rx_len, uint8_t store_id)
{
    if (desc.prefix_len > i2c::TransferDesc::PREFIX_CAPACITY || (src.data == nullptr && src.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const size_t meta_size = i2cMetaSize(desc);
    auto payload_size      = checkedPayload(3 + lenVarSize(rx_len) + 1, meta_size, src.size);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::BusTransfer, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::I2C);
    p[1]       = bus_id;
    p[2]       = store_id;
    size_t at  = 3 + encodeLenVar(p + 3, rx_len);
    p[at++]    = static_cast<uint8_t>(meta_size);
    encodeMeta(p + at, desc);
    at += meta_size;
    if (src.size != 0) {
        ::memcpy(p + at, src.data, src.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::transfer(uint8_t bus_id, const spi::TransferDesc& desc, data::ConstDataSpan src,
                                         size_t rx_len, uint8_t store_id)
{
    if (src.data == nullptr && src.size != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedPayload(3 + lenVarSize(rx_len) + 1, kSPIMetaSize, src.size);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::BusTransfer, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::SPI);
    p[1]       = bus_id;
    p[2]       = store_id;
    size_t at  = 3 + encodeLenVar(p + 3, rx_len);
    p[at++]    = static_cast<uint8_t>(kSPIMetaSize);
    encodeMeta(p + at, desc);
    at += kSPIMetaSize;
    if (src.size != 0) {
        ::memcpy(p + at, src.data, src.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::uartTransfer(uint8_t bus_id, data::ConstDataSpan src, size_t rx_len, uint8_t store_id)
{
    if (src.data == nullptr && src.size != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedPayload(3 + lenVarSize(rx_len) + 1, src.size);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::BusTransfer, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(types::bus_kind_t::UART);
    p[1]       = bus_id;
    p[2]       = store_id;
    size_t at  = 3 + encodeLenVar(p + 3, rx_len);
    p[at++]    = 0;  // no meta
    if (src.size != 0) {
        ::memcpy(p + at, src.data, src.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioSetMode(types::gpio_mode_t mode, const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto pin_bytes = checkedAdd(count, count);
    if (!pin_bytes.has_value()) {
        return m5::stl::make_unexpected(pin_bytes.error());
    }
    auto payload_size = checkedPayload(1, pin_bytes.value());
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioSetMode, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(mode);
    for (size_t i = 0; i < count; ++i) {
        putU16(p + 1 + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioWriteHigh(const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedAdd(count, count);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioWriteHigh, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    for (size_t i = 0; i < count; ++i) {
        putU16(payload.value().data + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioWriteLow(const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedAdd(count, count);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioWriteLow, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    for (size_t i = 0; i < count; ++i) {
        putU16(payload.value().data + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioRead(uint8_t store_id, const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto pin_bytes = checkedAdd(count, count);
    if (!pin_bytes.has_value()) {
        return m5::stl::make_unexpected(pin_bytes.error());
    }
    auto payload_size = checkedPayload(1, pin_bytes.value());
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioRead, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = store_id;
    for (size_t i = 0; i < count; ++i) {
        putU16(p + 1 + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioPortRead(uint8_t store_id, types::gpio_slot_t slot, uint8_t port_index)
{
    auto payload = beginInstruction(OpCode::GpioPortRead, 3);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = store_id;
    p[1]       = static_cast<uint8_t>(slot);
    p[2]       = port_index;
    return emit();
}

result_t<void> BytecodeEncoder::gpioPortWrite(types::gpio_slot_t slot, uint8_t port_index, uint32_t set_mask,
                                              uint32_t clear_mask)
{
    auto payload = beginInstruction(OpCode::GpioPortWrite, 10);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(slot);
    p[1]       = port_index;
    putU32(p + 2, set_mask);
    putU32(p + 6, clear_mask);
    return emit();
}

result_t<void> BytecodeEncoder::gpioSubscribe(const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedAdd(count, count);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioSubscribe, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    for (size_t i = 0; i < count; ++i) {
        putU16(payload.value().data + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::gpioUnsubscribe(const types::gpio_number_t* pins, size_t count)
{
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload_size = checkedAdd(count, count);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::GpioUnsubscribe, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    for (size_t i = 0; i < count; ++i) {
        putU16(payload.value().data + i * 2, static_cast<uint16_t>(pins[i]));
    }
    return emit();
}

result_t<void> BytecodeEncoder::evtGpioState(const types::gpio_number_t* pins, const bool* levels, size_t count)
{
    if ((pins == nullptr || levels == nullptr) && count != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    // Each entry: 2 bytes pin + 1 byte level = 3 bytes
    auto pin_bytes = checkedAdd(count, count);
    if (!pin_bytes.has_value()) {
        return m5::stl::make_unexpected(pin_bytes.error());
    }
    auto payload_size = checkedAdd(pin_bytes.value(), count);
    if (!payload_size.has_value()) {
        return m5::stl::make_unexpected(payload_size.error());
    }
    auto payload = beginInstruction(OpCode::EvtGpioState, payload_size.value());
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    for (size_t i = 0; i < count; ++i) {
        putU16(p, static_cast<uint16_t>(pins[i]));
        p[2] = levels[i] ? 1 : 0;
        p += 3;
    }
    return emit();
}

result_t<void> BytecodeEncoder::streamTransfer(types::bus_kind_t kind, uint8_t bus_id, uint8_t stream_id,
                                               uint32_t tx_len, uint32_t rx_len, data::ConstDataSpan meta)
{
    const size_t payload_size = 4 + 8 + meta.size;  // kind+bus_id+stream_id+meta_size + tx_len(4) + rx_len(4) + meta
    auto payload              = beginInstruction(OpCode::BusStreamTransfer, payload_size);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(kind);
    p[1]       = bus_id;
    p[2]       = stream_id;
    p[3]       = static_cast<uint8_t>(meta.size);
    p[4]       = static_cast<uint8_t>(tx_len & 0xFF);
    p[5]       = static_cast<uint8_t>((tx_len >> 8) & 0xFF);
    p[6]       = static_cast<uint8_t>((tx_len >> 16) & 0xFF);
    p[7]       = static_cast<uint8_t>((tx_len >> 24) & 0xFF);
    p[8]       = static_cast<uint8_t>(rx_len & 0xFF);
    p[9]       = static_cast<uint8_t>((rx_len >> 8) & 0xFF);
    p[10]      = static_cast<uint8_t>((rx_len >> 16) & 0xFF);
    p[11]      = static_cast<uint8_t>((rx_len >> 24) & 0xFF);
    if (meta.size > 0 && meta.data != nullptr) {
        ::memcpy(p + 12, meta.data, meta.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::busCreate(types::bus_kind_t kind, uint8_t bus_id, uint8_t store_id,
                                          data::ConstDataSpan pin_config)
{
    auto payload = beginInstruction(OpCode::BusCreate, 3 + pin_config.size);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    payload.value().data[0] = static_cast<uint8_t>(kind);
    payload.value().data[1] = bus_id;
    payload.value().data[2] = store_id;
    if (pin_config.size > 0 && pin_config.data != nullptr) {
        ::memcpy(payload.value().data + 3, pin_config.data, pin_config.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::busRelease(types::bus_kind_t kind, uint8_t bus_id)
{
    auto payload = beginInstruction(OpCode::BusRelease, 2);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    payload.value().data[0] = static_cast<uint8_t>(kind);
    payload.value().data[1] = bus_id;
    return emit();
}

result_t<void> BytecodeEncoder::busBeginTransaction(types::bus_kind_t kind, uint8_t bus_id)
{
    auto payload = beginInstruction(OpCode::BusBeginTransaction, 2);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    payload.value().data[0] = static_cast<uint8_t>(kind);
    payload.value().data[1] = bus_id;
    return emit();
}

result_t<void> BytecodeEncoder::busEndTransaction(types::bus_kind_t kind, uint8_t bus_id)
{
    auto payload = beginInstruction(OpCode::BusEndTransaction, 2);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    payload.value().data[0] = static_cast<uint8_t>(kind);
    payload.value().data[1] = bus_id;
    return emit();
}

result_t<void> BytecodeEncoder::storeData(uint8_t store_id, data::ConstDataSpan bytes)
{
    if (bytes.data == nullptr && bytes.size != 0) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    auto payload = beginInstruction(OpCode::StoreData, 1 + bytes.size);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = store_id;
    if (bytes.size != 0) {
        ::memcpy(p + 1, bytes.data, bytes.size);
    }
    return emit();
}

result_t<void> BytecodeEncoder::reportError(error_t err, size_t offset)
{
    auto payload = beginInstruction(OpCode::ReportError, 1 + lenVarSize(offset));
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    uint8_t* p = payload.value().data;
    p[0]       = static_cast<uint8_t>(static_cast<int8_t>(err));
    encodeLenVar(p + 1, offset);
    return emit();
}

result_t<void> BytecodeEncoder::reportComplete(error_t status)
{
    auto payload = beginInstruction(OpCode::ReportComplete, 1);
    if (!payload.has_value()) {
        return m5::stl::make_unexpected(payload.error());
    }
    payload.value().data[0] = static_cast<uint8_t>(static_cast<int8_t>(status));
    return emit();
}

result_t<void> BytecodeEncoder::end(void)
{
    auto reserved = _sink->reserve(1);
    if (!reserved.has_value()) {
        return m5::stl::make_unexpected(reserved.error());
    }
    if (reserved.value().size < 1) {
        return m5::stl::make_unexpected(_sink->closed() ? error_t::CLOSED : error_t::BUFFER_OVERFLOW);
    }
    reserved.value().data[0] = 0x00;  // LenVar 0 = terminator
    return _sink->commit(1);
}

// ---- BytecodeRunner ---------------------------------------------------------

namespace {

using RunnerBusBinding = detail::RunnerBusBinding;

result_t<void> requireAccessor(const void* ptr)
{
    if (ptr == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    return {};
}

result_t<void> i2cConfigure(RunnerBusBinding& binding, data::ConstDataSpan cfg_bytes)
{
    auto* acc = static_cast<i2c::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    i2c::MasterAccessConfig cfg = acc->getConfig();
    decodeConfig(cfg_bytes, cfg);
    return acc->setConfig(cfg);
}

result_t<void> spiConfigure(RunnerBusBinding& binding, data::ConstDataSpan cfg_bytes)
{
    auto* acc = static_cast<spi::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    spi::MasterAccessConfig cfg = acc->getConfig();
    decodeConfig(cfg_bytes, cfg);
    return acc->setConfig(cfg);
}

result_t<void> uartConfigure(RunnerBusBinding& binding, data::ConstDataSpan cfg_bytes)
{
    auto* acc = static_cast<uart::Accessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    uart::AccessConfig cfg = acc->getConfig();
    decodeConfig(cfg_bytes, cfg);
    return acc->setConfig(cfg);
}

result_t<void> i2sConfigure(RunnerBusBinding& binding, data::ConstDataSpan cfg_bytes)
{
    auto* tx = static_cast<i2s::TxAccessor*>(binding.tx);
    auto* rx = static_cast<i2s::RxAccessor*>(binding.rx);
    if (tx == nullptr && rx == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    i2s::AccessConfig cfg = (tx != nullptr) ? tx->getConfig() : rx->getConfig();
    decodeConfig(cfg_bytes, cfg);
    if (tx != nullptr) {
        auto s = tx->setConfig(cfg);
        if (!s.has_value()) {
            return s;
        }
    }
    if (rx != nullptr) {
        auto s = rx->setConfig(cfg);
        if (!s.has_value()) {
            return s;
        }
    }
    return {};
}

result_t<void> i2cTransfer(RunnerBusBinding& binding, data::ConstDataSpan meta, data::ConstDataSpan src,
                           data::DataSpan dst, size_t& actual_tx_len, size_t& actual_rx_len)
{
    auto* acc = static_cast<i2c::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    i2c::TransferDesc desc;
    if (!decodeMeta(meta, desc)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto begin = acc->beginTransaction();
    if (!begin.has_value()) {
        return m5::stl::make_unexpected(begin.error());
    }
    auto result = acc->transfer(desc, src, dst);
    auto end    = acc->endTransaction();
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    if (!end.has_value()) {
        return m5::stl::make_unexpected(end.error());
    }
    actual_tx_len = src.size;
    actual_rx_len = end->rx;
    return {};
}

result_t<void> spiTransfer(RunnerBusBinding& binding, data::ConstDataSpan meta, data::ConstDataSpan src,
                           data::DataSpan dst, size_t& actual_tx_len, size_t& actual_rx_len)
{
    auto* acc = static_cast<spi::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    spi::TransferDesc desc;
    if (!decodeMeta(meta, desc)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto begin = acc->beginTransaction();
    if (!begin.has_value()) {
        return m5::stl::make_unexpected(begin.error());
    }
    auto result = acc->transfer(desc, src, dst);
    auto end    = acc->endTransaction();
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    if (!end.has_value()) {
        return m5::stl::make_unexpected(end.error());
    }
    actual_tx_len = src.size;
    actual_rx_len = end->rx;
    return {};
}

result_t<void> uartTransfer(RunnerBusBinding& binding, data::ConstDataSpan, data::ConstDataSpan src, data::DataSpan dst,
                            size_t& actual_tx_len, size_t& actual_rx_len)
{
    auto* acc = static_cast<uart::Accessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    actual_tx_len = 0;
    actual_rx_len = 0;
    if (src.size != 0) {
        auto written = acc->write(src);
        if (!written.has_value()) {
            return m5::stl::make_unexpected(written.error());
        }
        actual_tx_len = written.value();
    }
    if (dst.size != 0) {
        auto got = acc->read(dst);
        if (!got.has_value()) {
            return m5::stl::make_unexpected(got.error());
        }
        actual_rx_len = got.value() <= dst.size ? got.value() : dst.size;
    }
    return {};
}

result_t<void> spiBeginTransaction(RunnerBusBinding& binding)
{
    auto* acc = static_cast<spi::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    return acc->beginTransaction();
}

result_t<void> spiEndTransaction(RunnerBusBinding& binding)
{
    auto* acc = static_cast<spi::MasterAccessor*>(binding.main);
    auto ok   = requireAccessor(acc);
    if (!ok.has_value()) {
        return ok;
    }
    auto end = acc->endTransaction();
    if (!end.has_value()) {
        return m5::stl::make_unexpected(end.error());
    }
    return {};
}

const detail::RunnerBusOps kI2COps = {i2cConfigure, i2cTransfer, nullptr, nullptr};

const detail::RunnerBusOps kSPIOps = {spiConfigure, spiTransfer, spiBeginTransaction, spiEndTransaction};

const detail::RunnerBusOps kUARTOps = {uartConfigure, uartTransfer, nullptr, nullptr};

result_t<void> i2sTransfer(RunnerBusBinding& binding, data::ConstDataSpan, data::ConstDataSpan src, data::DataSpan dst,
                           size_t& actual_tx_len, size_t& actual_rx_len)
{
    auto* tx = static_cast<i2s::TxAccessor*>(binding.tx);
    auto* rx = static_cast<i2s::RxAccessor*>(binding.rx);
    if (tx == nullptr && rx == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    actual_tx_len = 0;
    actual_rx_len = 0;
    if (src.size != 0) {
        if (tx == nullptr) {
            return m5::stl::make_unexpected(error_t::NOT_IMPLEMENTED);
        }
        auto written = tx->write(src);
        if (!written.has_value()) {
            return m5::stl::make_unexpected(written.error());
        }
        actual_tx_len = written.value() <= src.size ? written.value() : src.size;
    }
    if (dst.size != 0) {
        if (rx == nullptr) {
            return m5::stl::make_unexpected(error_t::NOT_IMPLEMENTED);
        }
        auto got = rx->read(dst);
        if (!got.has_value()) {
            return m5::stl::make_unexpected(got.error());
        }
        actual_rx_len = got.value() <= dst.size ? got.value() : dst.size;
    }
    return {};
}

const detail::RunnerBusOps kI2SOps = {i2sConfigure, i2sTransfer, nullptr, nullptr};

}  // namespace

bool BytecodeRunner::hasBinding(types::bus_kind_t kind, uint8_t bus_id) const
{
    return binding(kind, bus_id) != nullptr;
}

result_t<void> BytecodeRunner::streamTransferChunk(types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan meta,
                                                   data::ConstDataSpan src, data::DataSpan dst, size_t& actual_tx_len,
                                                   size_t& actual_rx_len)
{
    auto* bus = binding(kind, bus_id);
    if (bus == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    actual_tx_len = 0;
    actual_rx_len = 0;

    if (bus->ops == nullptr || bus->ops->transfer == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    auto result = bus->ops->transfer(*bus, meta, src, dst, actual_tx_len, actual_rx_len);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    if (actual_rx_len > dst.size) {
        actual_rx_len = dst.size;
    }
    return {};
}

detail::RunnerBusBinding* BytecodeRunner::binding(types::bus_kind_t kind, uint8_t bus_id)
{
    for (size_t i = 0; i < kMaxBindingSlots; ++i) {
        if (_bindings[i].matches(kind, bus_id)) {
            return &_bindings[i];
        }
    }
    return nullptr;
}

const detail::RunnerBusBinding* BytecodeRunner::binding(types::bus_kind_t kind, uint8_t bus_id) const
{
    for (size_t i = 0; i < kMaxBindingSlots; ++i) {
        if (_bindings[i].matches(kind, bus_id)) {
            return &_bindings[i];
        }
    }
    return nullptr;
}

result_t<void> BytecodeRunner::registerBinding(types::bus_kind_t kind, uint8_t bus_id, const detail::RunnerBusOps& ops,
                                               void* main, void* tx, void* rx)
{
    if (bus_id >= kMaxBusBindings || ops.configure == nullptr || (main == nullptr && tx == nullptr && rx == nullptr)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    if (binding(kind, bus_id) != nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    detail::RunnerBusBinding* slot = nullptr;
    for (size_t i = 0; i < kMaxBindingSlots; ++i) {
        if (!_bindings[i].registered()) {
            slot = &_bindings[i];
            break;
        }
    }
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    slot->kind   = kind;
    slot->bus_id = bus_id;
    slot->main   = main;
    slot->tx     = tx;
    slot->rx     = rx;
    slot->ops    = &ops;
    return {};
}

void BytecodeRunner::unregisterBinding(types::bus_kind_t kind, uint8_t bus_id)
{
    auto* slot = binding(kind, bus_id);
    if (slot == nullptr) {
        return;
    }
    slot->clear();
}

result_t<void> BytecodeRunner::registerI2C(uint8_t bus_id, i2c::MasterAccessor& acc)
{
    return registerBinding(types::bus_kind_t::I2C, bus_id, kI2COps, &acc, nullptr, nullptr);
}

result_t<void> BytecodeRunner::registerSPI(uint8_t bus_id, spi::MasterAccessor& acc)
{
    return registerBinding(types::bus_kind_t::SPI, bus_id, kSPIOps, &acc, nullptr, nullptr);
}

result_t<void> BytecodeRunner::registerUART(uint8_t bus_id, uart::Accessor& acc)
{
    return registerBinding(types::bus_kind_t::UART, bus_id, kUARTOps, &acc, &acc.tx(), &acc.rx());
}

void BytecodeRunner::unregisterI2C(uint8_t bus_id)
{
    unregisterBinding(types::bus_kind_t::I2C, bus_id);
}

void BytecodeRunner::unregisterSPI(uint8_t bus_id)
{
    unregisterBinding(types::bus_kind_t::SPI, bus_id);
}

void BytecodeRunner::unregisterUART(uint8_t bus_id)
{
    unregisterBinding(types::bus_kind_t::UART, bus_id);
}

result_t<void> BytecodeRunner::registerI2S(uint8_t bus_id, i2s::Accessor& acc)
{
    return registerBinding(types::bus_kind_t::I2S, bus_id, kI2SOps, nullptr, &acc.tx(), &acc.rx());
}

result_t<void> BytecodeRunner::registerI2S(uint8_t bus_id, i2s::TxAccessor& acc)
{
    return registerBinding(types::bus_kind_t::I2S, bus_id, kI2SOps, nullptr, &acc, nullptr);
}

result_t<void> BytecodeRunner::registerI2S(uint8_t bus_id, i2s::RxAccessor& acc)
{
    return registerBinding(types::bus_kind_t::I2S, bus_id, kI2SOps, nullptr, nullptr, &acc);
}

void BytecodeRunner::unregisterI2S(uint8_t bus_id)
{
    unregisterBinding(types::bus_kind_t::I2S, bus_id);
}

result_t<void> BytecodeRunner::opBusStreamTransfer(data::ConstDataSpan payload)
{
    if (payload.size < 12) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    if (_stream_transfer_fn == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    StreamTransferDesc desc;
    desc.kind         = static_cast<types::bus_kind_t>(payload.data[0]);
    desc.bus_id       = payload.data[1];
    desc.stream_id    = payload.data[2];
    uint8_t meta_size = payload.data[3];
    desc.tx_len       = static_cast<uint32_t>(payload.data[4]) | (static_cast<uint32_t>(payload.data[5]) << 8) |
                  (static_cast<uint32_t>(payload.data[6]) << 16) | (static_cast<uint32_t>(payload.data[7]) << 24);
    desc.rx_len = static_cast<uint32_t>(payload.data[8]) | (static_cast<uint32_t>(payload.data[9]) << 8) |
                  (static_cast<uint32_t>(payload.data[10]) << 16) | (static_cast<uint32_t>(payload.data[11]) << 24);
    if (12 + meta_size > payload.size) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    desc.meta = {payload.data + 12, meta_size};
    return _stream_transfer_fn(_stream_transfer_ctx, desc);
}

result_t<void> BytecodeRunner::opBusCreate(data::ConstDataSpan payload)
{
    if (payload.size < 3) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (_bus_create_fn == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    const auto kind   = static_cast<types::bus_kind_t>(payload.data[0]);
    const uint8_t bid = payload.data[1];
    // payload[2] = store_id (reserved, skip)
    data::ConstDataSpan pin_config{payload.data + 3, payload.size - 3};
    return _bus_create_fn(_bus_create_ctx, true, kind, bid, pin_config);
}

result_t<void> BytecodeRunner::opBusRelease(data::ConstDataSpan payload)
{
    if (payload.size < 2) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (_bus_create_fn == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    const auto kind   = static_cast<types::bus_kind_t>(payload.data[0]);
    const uint8_t bid = payload.data[1];
    return _bus_create_fn(_bus_create_ctx, false, kind, bid, data::ConstDataSpan{nullptr, 0});
}

result_t<void> BytecodeRunner::opGpioAllowlist(data::ConstDataSpan payload)
{
    if (_gpio_allowlist_fn == nullptr) {
        return {};
    }
    return _gpio_allowlist_fn(_gpio_allowlist_ctx, payload.data, payload.size);
}

result_t<void> BytecodeRunner::opGpioPortRead(data::ConstDataSpan payload)
{
    if (_gpio_group == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    FieldReader r{payload};
    uint8_t store_id = kDiscardStoreId, slot_u8 = 0, port_index = 0;
    if (!r.u8(store_id) || !r.u8(slot_u8) || !r.u8(port_index)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto pa = _gpio_group->getPort(static_cast<types::gpio_slot_t>(slot_u8), port_index);
    if (!pa.has_value()) {
        return m5::stl::make_unexpected(pa.error());
    }
    uint32_t val = pa.value().port->readPort() & ~pa.value().deny_mask;
    if (store_id != kDiscardStoreId) {
        auto allocated = allocStore(store_id, 4);
        if (!allocated.has_value()) {
            return m5::stl::make_unexpected(allocated.error());
        }
        auto* buf              = static_cast<uint8_t*>(allocated.value()->buf.data());
        buf[0]                 = static_cast<uint8_t>(val);
        buf[1]                 = static_cast<uint8_t>(val >> 8);
        buf[2]                 = static_cast<uint8_t>(val >> 16);
        buf[3]                 = static_cast<uint8_t>(val >> 24);
        allocated.value()->len = 4;
    }
    return {};
}

result_t<void> BytecodeRunner::opGpioPortWrite(data::ConstDataSpan payload)
{
    if (_gpio_group == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    FieldReader r{payload};
    uint8_t slot_u8 = 0, port_index = 0;
    uint32_t set_mask = 0, clear_mask = 0;
    if (!r.u8(slot_u8) || !r.u8(port_index) || !r.u32(set_mask) || !r.u32(clear_mask)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto pa = _gpio_group->getPort(static_cast<types::gpio_slot_t>(slot_u8), port_index);
    if (!pa.has_value()) {
        return m5::stl::make_unexpected(pa.error());
    }
    set_mask &= ~pa.value().deny_mask;
    clear_mask &= ~pa.value().deny_mask;
    pa.value().port->writePort(set_mask, clear_mask);
    return {};
}

data::ConstDataSpan BytecodeRunner::storedData(uint8_t store_id) const
{
    for (const auto& slot : _slots) {
        if (slot.id == store_id) {
            return {static_cast<const uint8_t*>(slot.buf.data()), slot.len};
        }
    }
    return {};
}

size_t BytecodeRunner::storedCount(void) const
{
    size_t count = 0;
    for (const auto& slot : _slots) {
        count += (slot.id != kDiscardStoreId) ? 1 : 0;
    }
    return count;
}

uint8_t BytecodeRunner::storedIdAt(size_t index) const
{
    for (const auto& slot : _slots) {
        if (slot.id == kDiscardStoreId) {
            continue;
        }
        if (index == 0) {
            return slot.id;
        }
        --index;
    }
    return kDiscardStoreId;
}

void BytecodeRunner::clearStored(void)
{
    for (auto& slot : _slots) {
        slot.id  = kDiscardStoreId;
        slot.len = 0;
        slot.buf = memory::TempBuffer{};
    }
}

result_t<BytecodeRunner::Slot*> BytecodeRunner::allocStore(uint8_t store_id, size_t size)
{
    Slot* free_slot = nullptr;
    Slot* target    = nullptr;
    for (auto& slot : _slots) {
        if (slot.id == store_id) {
            target = &slot;
            break;
        }
        if (slot.id == kDiscardStoreId && free_slot == nullptr) {
            free_slot = &slot;
        }
    }
    if (target == nullptr) {
        target = free_slot;
    }
    if (target == nullptr) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    if (size > target->buf.size()) {
        target->buf = memory::TempBuffer{*_alloc, size};
        if (size != 0 && target->buf.data() == nullptr) {
            target->id  = kDiscardStoreId;
            target->len = 0;
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
    }
    target->id  = store_id;
    target->len = 0;
    return target;
}

result_t<size_t> BytecodeRunner::run(data::ConstDataSpan script)
{
    data::MemorySource source{script};
    return run(source);
}

result_t<size_t> BytecodeRunner::run(data::Source& script)
{
    clearStored();
    _status_reported = false;
    _reported_status = error_t::OK;
    _reported_offset = 0;
    _event_mode      = false;
    return runLoop(script);
}

result_t<size_t> BytecodeRunner::runEvent(data::ConstDataSpan script)
{
    // Event scripts execute WITHOUT touching the request state: no slot
    // clear, no report reset, and store/report instructions inside the
    // script are ignored by their handlers while _event_mode is set.
    // No early-return path exists between here and runLoop's exit, so
    // the flag reset below is reliable.
    data::MemorySource source{script};
    _event_mode = true;
    auto r      = runLoop(source);
    _event_mode = false;
    return r;
}

result_t<size_t> BytecodeRunner::runLoop(data::Source& script)
{
    _last_offset     = 0;
    _unknown_skipped = 0;

    size_t offset = 0;
    for (;;) {
        // Ask for a single byte first: most LenVars are the 1-byte form,
        // and requesting more would make a StreamSource block a whole
        // timeout when the next instruction is already buffered but
        // shorter than the request. Wider LenVars grow below.
        auto head = script.peek(1);
        if (!head.has_value()) {
            return m5::stl::make_unexpected(head.error());
        }
        if (head.value().size == 0) {
            break;  // clean end at end-of-input
        }
        auto prefix = decodeLenVar(head.value());
        if (!prefix.valid) {
            _last_offset = offset;
            return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
        }
        if (prefix.consumed == 0) {
            // 3- or 5-byte LenVar split across the current peek.
            const size_t need = (head.value().data[0] == 0xFD) ? 3 : 5;
            auto grown        = peekAtLeast(script, need);
            if (!grown.has_value()) {
                _last_offset = offset;
                return m5::stl::make_unexpected(grown.error());
            }
            prefix = decodeLenVar(grown.value());
        }
        if (prefix.value == 0) {
            auto advanced = script.advance(prefix.consumed);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            offset += prefix.consumed;
            break;  // explicit terminator
        }
        // Reject sizes that would wrap `need` on 32-bit targets (a hostile
        // u32 LenVar like FE FF FF FF FF would otherwise alias to a tiny
        // value and read past the peeked span).
        if (prefix.value > static_cast<size_t>(-1) - prefix.consumed) {
            _last_offset = offset;
            return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
        }

        const size_t need = prefix.consumed + prefix.value;
        auto instruction  = peekAtLeast(script, need);
        if (!instruction.has_value()) {
            _last_offset = offset;
            return m5::stl::make_unexpected(instruction.error());
        }
        const uint8_t opcode = instruction.value().data[prefix.consumed];
        const data::ConstDataSpan payload{instruction.value().data + prefix.consumed + 1, prefix.value - 1};

        _last_offset = offset;
        auto status  = dispatch(opcode, payload);
        if (!status.has_value()) {
            return m5::stl::make_unexpected(status.error());
        }
        auto advanced = script.advance(need);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        offset += need;
    }
    return offset;
}

result_t<void> BytecodeRunner::dispatch(uint8_t opcode, data::ConstDataSpan payload)
{
    // Receive-only runners (the host session's) accept only the
    // receive-side opcodes: a peer's script fills data and reports
    // status, it never drives local buses, pins, or the clock.
    // Unknown opcodes still reach the forward-compatibility default.
    if (_receive_only) {
        switch (static_cast<OpCode>(opcode)) {
            case OpCode::DelayMs:
            case OpCode::BusConfigure:
            case OpCode::BusTransfer:
            case OpCode::BusStreamTransfer:
            case OpCode::GpioSetMode:
            case OpCode::GpioWriteHigh:
            case OpCode::GpioWriteLow:
            case OpCode::GpioRead:
            case OpCode::GpioSubscribe:
            case OpCode::GpioUnsubscribe:
            case OpCode::GpioAllowlist:
            case OpCode::GpioPortRead:
            case OpCode::GpioPortWrite:
            case OpCode::BusCreate:
            case OpCode::BusRelease:
            case OpCode::BusBeginTransaction:
            case OpCode::BusEndTransaction:
                return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
            default:
                break;
        }
    }
    switch (static_cast<OpCode>(opcode)) {
        case OpCode::DelayMs:
            return opDelay(payload);
        case OpCode::BusConfigure:
            return opBusConfigure(payload);
        case OpCode::BusTransfer:
            return opBusTransfer(payload);
        case OpCode::BusStreamTransfer:
            return opBusStreamTransfer(payload);
        case OpCode::GpioSetMode:
        case OpCode::GpioWriteHigh:
        case OpCode::GpioWriteLow:
        case OpCode::GpioRead:
            return opGpio(opcode, payload);
        case OpCode::GpioSubscribe:
        case OpCode::GpioUnsubscribe:
            return opGpioSubscribe(opcode, payload);
        case OpCode::GpioAllowlist:
            return opGpioAllowlist(payload);
        case OpCode::GpioPortRead:
            return opGpioPortRead(payload);
        case OpCode::GpioPortWrite:
            return opGpioPortWrite(payload);
        case OpCode::BusCreate:
            return opBusCreate(payload);
        case OpCode::BusRelease:
            return opBusRelease(payload);
        case OpCode::EvtGpioState:
            return opEvtGpioState(payload);
        case OpCode::BusBeginTransaction:
            return opBusBeginTransaction(payload);
        case OpCode::BusEndTransaction:
            return opBusEndTransaction(payload);
        case OpCode::StoreData:
            return opStoreData(payload);
        case OpCode::ReportError:
        case OpCode::ReportComplete:
            return opReport(opcode, payload);
        default:
            if (opcode & kCriticalOpcodeBit) {
                return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
            }
            ++_unknown_skipped;
            return {};
    }
}

result_t<void> BytecodeRunner::opDelay(data::ConstDataSpan payload)
{
    FieldReader r{payload};
    uint32_t ms = 0;
    if (!r.u32(ms)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (_delay_fn != nullptr) {
        _delay_fn(ms);
    } else {
        m5::utility::delay(ms);
    }
    return {};
}

result_t<void> BytecodeRunner::opBusConfigure(data::ConstDataSpan payload)
{
    FieldReader r{payload};
    uint8_t kind = 0, bus_id = 0;
    if (!r.u8(kind) || !r.u8(bus_id) || bus_id >= kMaxBusBindings) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const data::ConstDataSpan cfg_bytes{r.p, r.n};
    const auto bus_kind = static_cast<types::bus_kind_t>(kind);
    auto* slot          = binding(bus_kind, bus_id);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    if (slot->ops->configure == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    return slot->ops->configure(*slot, cfg_bytes);
}

result_t<void> BytecodeRunner::opBusTransfer(data::ConstDataSpan payload)
{
    FieldReader r{payload};
    uint8_t kind = 0, bus_id = 0, store_id = 0;
    if (!r.u8(kind) || !r.u8(bus_id) || !r.u8(store_id) || bus_id >= kMaxBusBindings) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto rx_len_var = decodeLenVar({r.p, r.n});
    if (!rx_len_var.valid || rx_len_var.consumed == 0) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    r.p += rx_len_var.consumed;
    r.n -= rx_len_var.consumed;
    const size_t rx_len = rx_len_var.value;

    uint8_t meta_size = 0;
    if (!r.u8(meta_size) || r.n < meta_size) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const data::ConstDataSpan meta{r.p, meta_size};
    const data::ConstDataSpan src{r.p + meta_size, r.n - meta_size};

    // Resolve the dst buffer: a labeled slot, or a discard buffer.
    Slot* slot = nullptr;
    memory::TempBuffer discard;
    uint8_t* rx_ptr = nullptr;
    if (rx_len > 0) {
        if (store_id != kDiscardStoreId) {
            auto allocated = allocStore(store_id, rx_len);
            if (!allocated.has_value()) {
                return m5::stl::make_unexpected(allocated.error());
            }
            slot   = allocated.value();
            rx_ptr = static_cast<uint8_t*>(slot->buf.data());
        } else {
            discard = memory::TempBuffer{*_alloc, rx_len};
            if (discard.data() == nullptr) {
                return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
            }
            rx_ptr = static_cast<uint8_t*>(discard.data());
        }
    }
    const data::DataSpan dst{rx_ptr, rx_len};

    const auto bus_kind = static_cast<types::bus_kind_t>(kind);
    auto* bus           = binding(bus_kind, bus_id);
    if (bus == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    if (bus->ops->transfer == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    size_t actual_tx_len = 0;
    size_t actual_rx_len = 0;
    auto result          = bus->ops->transfer(*bus, meta, src, dst, actual_tx_len, actual_rx_len);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    if (slot != nullptr) {
        slot->len = actual_rx_len;
    }
    return {};
}

result_t<void> BytecodeRunner::opGpio(uint8_t opcode, data::ConstDataSpan payload)
{
    if (_gpio_group == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    FieldReader r{payload};
    uint8_t mode = 0, store_id = kDiscardStoreId;
    const auto op = static_cast<OpCode>(opcode);
    if (op == OpCode::GpioSetMode && !r.u8(mode)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (op == OpCode::GpioRead && !r.u8(store_id)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if ((r.n % 2) != 0) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const size_t count = r.n / 2;

    Slot* slot = nullptr;
    if (op == OpCode::GpioRead && store_id != kDiscardStoreId) {
        auto allocated = allocStore(store_id, (count + 7) / 8);
        if (!allocated.has_value()) {
            return m5::stl::make_unexpected(allocated.error());
        }
        slot = allocated.value();
        ::memset(slot->buf.data(), 0, slot->buf.size());
    }

    for (size_t i = 0; i < count; ++i) {
        int16_t num = 0;
        (void)r.i16(num);
        auto pin = _gpio_group->tryGetPin(static_cast<types::gpio_number_t>(num));
        if (!pin.has_value()) {
            return m5::stl::make_unexpected(pin.error());
        }
        switch (op) {
            case OpCode::GpioSetMode:
                pin.value().setMode(static_cast<types::gpio_mode_t>(mode));
                if (_gpio_mode_fn != nullptr) {
                    auto mode_set = _gpio_mode_fn(_gpio_mode_ctx, static_cast<types::gpio_number_t>(num),
                                                  static_cast<types::gpio_mode_t>(mode));
                    if (!mode_set.has_value()) {
                        return m5::stl::make_unexpected(mode_set.error());
                    }
                }
                break;
            case OpCode::GpioWriteHigh:
                pin.value().writeHigh();
                break;
            case OpCode::GpioWriteLow:
                pin.value().writeLow();
                break;
            case OpCode::GpioRead:
            default:
                if (slot != nullptr && pin.value().read()) {
                    static_cast<uint8_t*>(slot->buf.data())[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
                }
                break;
        }
    }
    if (slot != nullptr) {
        slot->len = (count + 7) / 8;
    }
    return {};
}

result_t<void> BytecodeRunner::opStoreData(data::ConstDataSpan payload)
{
    if (_event_mode) {
        return {};  // events must not clobber request slots
    }
    FieldReader r{payload};
    uint8_t store_id = 0;
    if (!r.u8(store_id) || store_id == kDiscardStoreId) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    auto allocated = allocStore(store_id, r.n);
    if (!allocated.has_value()) {
        return m5::stl::make_unexpected(allocated.error());
    }
    if (r.n != 0) {
        ::memcpy(allocated.value()->buf.data(), r.p, r.n);
    }
    allocated.value()->len = r.n;
    return {};
}

result_t<void> BytecodeRunner::opReport(uint8_t opcode, data::ConstDataSpan payload)
{
    if (_event_mode) {
        return {};  // events must not clobber the report state
    }
    FieldReader r{payload};
    uint8_t raw_status = 0;
    if (!r.u8(raw_status)) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    _status_reported = true;
    _reported_status = static_cast<error_t>(static_cast<int8_t>(raw_status));
    _reported_offset = 0;
    if (static_cast<OpCode>(opcode) == OpCode::ReportError) {
        auto offset_var = decodeLenVar({r.p, r.n});
        if (!offset_var.valid || offset_var.consumed == 0) {
            return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
        }
        _reported_offset = offset_var.value;
    }
    return {};
}

result_t<void> BytecodeRunner::opGpioSubscribe(uint8_t opcode, data::ConstDataSpan payload)
{
    if ((payload.size % 2) != 0) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (_gpio_subscribe_fn == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    const bool subscribe = (static_cast<OpCode>(opcode) == OpCode::GpioSubscribe);
    if (payload.size == 0) {
        // Empty payload: unsubscribe all (pins=nullptr, count=0)
        return _gpio_subscribe_fn(_gpio_subscribe_ctx, subscribe, nullptr, 0);
    }
    // Decode the whole pin list and hand it to the handler in ONE call,
    // so the handler can apply all-or-nothing semantics (a per-pin loop
    // here would leave a prefix subscribed when a later pin fails). One
    // remote message bounds the payload (BODY <= 238 bytes), so the
    // stack list stays small; larger hand-built scripts are rejected.
    const size_t count         = payload.size / 2;
    constexpr size_t kMaxBatch = 120;
    if (count > kMaxBatch) {
        return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
    }
    types::gpio_number_t pins[kMaxBatch];
    for (size_t i = 0; i < count; ++i) {
        const uint16_t raw = static_cast<uint16_t>(static_cast<uint16_t>(payload.data[i * 2]) |
                                                   (static_cast<uint16_t>(payload.data[i * 2 + 1]) << 8));
        pins[i]            = static_cast<types::gpio_number_t>(raw);
    }
    return _gpio_subscribe_fn(_gpio_subscribe_ctx, subscribe, pins, count);
}

result_t<void> BytecodeRunner::opEvtGpioState(data::ConstDataSpan payload)
{
    if ((payload.size % 3) != 0) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    if (_gpio_event_fn == nullptr) {
        // Silently ignore when no handler installed
        return {};
    }
    const size_t count = payload.size / 3;
    for (size_t i = 0; i < count; ++i) {
        const uint16_t raw             = static_cast<uint16_t>(static_cast<uint16_t>(payload.data[i * 3]) |
                                                               (static_cast<uint16_t>(payload.data[i * 3 + 1]) << 8));
        const types::gpio_number_t pin = static_cast<types::gpio_number_t>(raw);
        const bool level               = payload.data[i * 3 + 2] != 0;
        _gpio_event_fn(_gpio_event_ctx, pin, level);
    }
    return {};
}

result_t<void> BytecodeRunner::opBusBeginTransaction(data::ConstDataSpan payload)
{
    FieldReader r{payload};
    uint8_t kind = 0, bus_id = 0;
    if (!r.u8(kind) || !r.u8(bus_id) || bus_id >= kMaxBusBindings) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const auto bus_kind = static_cast<types::bus_kind_t>(kind);
    auto* slot          = binding(bus_kind, bus_id);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    if (slot->ops->begin_transaction == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    return slot->ops->begin_transaction(*slot);
}

result_t<void> BytecodeRunner::opBusEndTransaction(data::ConstDataSpan payload)
{
    FieldReader r{payload};
    uint8_t kind = 0, bus_id = 0;
    if (!r.u8(kind) || !r.u8(bus_id) || bus_id >= kMaxBusBindings) {
        return m5::stl::make_unexpected(error_t::PROTOCOL_ERROR);
    }
    const auto bus_kind = static_cast<types::bus_kind_t>(kind);
    auto* slot          = binding(bus_kind, bus_id);
    if (slot == nullptr) {
        return m5::stl::make_unexpected(error_t::INVALID_STATE);
    }
    if (slot->ops->end_transaction == nullptr) {
        return m5::stl::make_unexpected(error_t::UNSUPPORTED);
    }
    return slot->ops->end_transaction(*slot);
}

result_t<void> BytecodeRunner::writeResponse(data::Sink& dst, error_t status)
{
    BytecodeEncoder encoder{dst};
    for (const auto& slot : _slots) {
        if (slot.id == kDiscardStoreId) {
            continue;
        }
        auto written = encoder.storeData(slot.id, {static_cast<const uint8_t*>(slot.buf.data()), slot.len});
        if (!written.has_value()) {
            return written;
        }
    }
    auto reported =
        (status == error_t::OK) ? encoder.reportComplete(status) : encoder.reportError(status, _last_offset);
    if (!reported.has_value()) {
        return reported;
    }
    return encoder.end();
}

}  // namespace m5::hal::v2::bytecode

#endif
