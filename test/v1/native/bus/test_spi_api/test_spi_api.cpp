// SPDX-License-Identifier: MIT
#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

using ::m5::hal::v1::result_t;

namespace {

namespace spi   = m5::hal::v1::spi;
namespace bus   = m5::hal::v1::bus;
namespace data  = m5::hal::v1::data;
namespace error = m5::hal::v1::error;
namespace gpio  = m5::hal::v1::gpio;
namespace types = m5::hal::v1::types;

class StubIBus : public spi::IBus {
public:
    struct Call {
        const bus::IAccessor* owner = nullptr;
        spi::MasterAccessConfig cfg;
        spi::TransferDesc desc;
        std::vector<uint8_t> tx;
        size_t rx_len = 0;
    };

    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    result_t<void> init(const spi::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        if (fail_begin_transaction) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
        }
        transaction_owners.push_back(owner);
        transaction_cfgs.push_back(cfg);
        ++begin_transaction_count;
        return {};
    }

    bool fail_begin_transaction = false;

    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        transaction_owners.push_back(owner);
        transaction_cfgs.push_back(cfg);
        ++end_transaction_count;
        return {};
    }

    result_t<size_t> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                              const spi::TransferDesc& desc, data::Source* tx, data::Sink* rx) override
    {
        Call call;
        call.owner = owner;
        call.cfg   = cfg;
        call.desc  = desc;

        if (tx != nullptr) {
            while (!tx->eof()) {
                auto span = tx->peek(16);
                if (!span.has_value()) {
                    return m5::stl::make_unexpected(span.error());
                }
                call.tx.insert(call.tx.end(), span->data, span->data + span->size);
                auto adv = tx->advance(span->size);
                if (!adv.has_value()) {
                    return m5::stl::make_unexpected(adv.error());
                }
            }
        }

        if (rx != nullptr) {
            uint8_t next = rx_seed;
            while (!rx->closed()) {
                auto span = rx->reserve(16);
                if (!span.has_value()) {
                    return m5::stl::make_unexpected(span.error());
                }
                for (size_t i = 0; i < span->size; ++i) {
                    span->data[i] = next++;
                }
                call.rx_len += span->size;
                auto commit = rx->commit(span->size);
                if (!commit.has_value()) {
                    return m5::stl::make_unexpected(commit.error());
                }
            }
        }

        calls.push_back(call);
        return call.tx.size() + call.rx_len;
    }

    std::vector<Call> calls;
    std::vector<const bus::IAccessor*> transaction_owners;
    std::vector<spi::MasterAccessConfig> transaction_cfgs;
    size_t begin_transaction_count = 0;
    size_t end_transaction_count   = 0;
    uint8_t rx_seed                = 0x40;
};

class RecordingPort : public gpio::IPort {
public:
    enum class Kind : uint8_t { Write, Read, SetMode };
    struct Event {
        types::gpio_number_t gpio_num = -1;
        Kind kind                     = Kind::Write;
        bool value                    = false;
        types::gpio_mode_t mode       = types::gpio_mode_t::Input;
    };

    void clear()
    {
        events.clear();
    }
    void setReadValue(bool value)
    {
        read_value = value;
    }

    std::vector<Event> events;
    bool read_value = false;

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        events.push_back({static_cast<types::gpio_number_t>(encoded_num), Kind::Write, v, types::gpio_mode_t::Input});
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        events.push_back(
            {static_cast<types::gpio_number_t>(encoded_num), Kind::Read, read_value, types::gpio_mode_t::Input});
        return read_value;
    }
    void _setPinModeEncoded(uint32_t encoded_num, types::gpio_mode_t mode) override
    {
        events.push_back({static_cast<types::gpio_number_t>(encoded_num), Kind::SetMode, false, mode});
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override
    {
        return pin_index;
    }
};

class RecordingGPIO : public gpio::IGPIO {
public:
    gpio::IPort* portForPin(types::gpio_local_pin_t local_pin) const override
    {
        (void)local_pin;
        return &_port;
    }
    gpio::IPort* getPort(uint8_t port_index) const override
    {
        (void)port_index;
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return 16;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

    mutable RecordingPort _port;
};

RecordingGPIO& softwareSpiGPIO()
{
    static RecordingGPIO gpio;
    static bool registered = [] {
        auto r = m5::hal::v1::M5_Hal.Gpio.addGPIO(&gpio, 42);
        (void)r;
        return true;
    }();
    (void)registered;
    return gpio;
}

types::gpio_number_t softPin(uint8_t local)
{
    return types::makeGpioNumber(42, local);
}

size_t countWrites(const std::vector<RecordingPort::Event>& events, types::gpio_number_t pin, bool value)
{
    const auto local = static_cast<types::gpio_number_t>(types::extractLocalPin(pin));
    return static_cast<size_t>(std::count_if(events.begin(), events.end(), [local, value](const auto& event) {
        return event.kind == RecordingPort::Kind::Write && event.gpio_num == local && event.value == value;
    }));
}

size_t firstWriteIndex(const std::vector<RecordingPort::Event>& events, types::gpio_number_t pin, bool value)
{
    const auto local = static_cast<types::gpio_number_t>(types::extractLocalPin(pin));
    const auto it    = std::find_if(events.begin(), events.end(), [local, value](const auto& event) {
        return event.kind == RecordingPort::Kind::Write && event.gpio_num == local && event.value == value;
    });
    return static_cast<size_t>(std::distance(events.begin(), it));
}

}  // namespace

TEST(MasterAccessConfig, SetupWithDCPinSelectsPinModeAndCommandLength)
{
    spi::AccessConfig cfg;
    cfg.setupWithDCPin(27).pin_cs = 5;

    EXPECT_EQ(cfg.pin_dc, 27);
    EXPECT_EQ(cfg.spi_data_mode, spi::spi_data_mode_t::halfduplex_with_dc_pin);
    EXPECT_EQ(cfg.spi_command_length, 8);
    EXPECT_EQ(cfg.pin_cs, 5);  // the preset chains into further assignments
}

TEST(MasterAccessConfig, SetupWithDCBitSelectsBitModeAndClearsPin)
{
    spi::AccessConfig cfg;
    cfg.pin_dc = 27;  // a previously set pin must be cleared by the bit preset
    cfg.setupWithDCBit();

    EXPECT_EQ(cfg.pin_dc, -1);
    EXPECT_EQ(cfg.spi_data_mode, spi::spi_data_mode_t::halfduplex_with_dc_bit);
    EXPECT_EQ(cfg.spi_command_length, 8);
}

TEST(MasterAccessor, WriteWrapsTransferAndCopiesTx)
{
    StubIBus bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = 18;
    bus_cfg.pin_mosi = 23;
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = 5;
    cfg.freq   = 40000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x9F, 0x00, 0x01};
    auto result        = accessor.write(tx, sizeof(tx));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(tx));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].owner, &accessor);
    EXPECT_EQ(bus.calls[0].cfg.pin_cs, 5);
    EXPECT_EQ(bus.calls[0].cfg.freq, 40000000u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x9F, 0x00, 0x01}));
    EXPECT_EQ(bus.calls[0].rx_len, 0u);
}

TEST(MasterAccessor, ReadWrapsTransferAndFillsRx)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    uint8_t rx[4] = {};
    auto result   = accessor.read(rx, sizeof(rx));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(rx));
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0x42, 0x43}));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].rx_len, sizeof(rx));
}

TEST(MasterAccessor, WriteSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    const uint8_t bytes[] = {0x10, 0x11, 0x12, 0x13, 0x14};
    data::MemorySource src{data::ConstDataSpan{bytes, sizeof(bytes)}};

    auto result = accessor.write(src, 3);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x10, 0x11, 0x12}));

    auto next = src.peek(8);
    ASSERT_TRUE(next.has_value());
    ASSERT_EQ(next->size, 2u);
    EXPECT_EQ(next->data[0], 0x13);
}

TEST(MasterAccessor, ReadSinkLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    uint8_t rx[5] = {};
    data::MemorySink sink{data::DataSpan{rx, sizeof(rx)}};

    auto result = accessor.read(sink, 2);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 2u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].rx_len, 2u);
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0, 0, 0}));
}

TEST(MasterAccessor, SetConfigRejectsWhileAccessIsHeld)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    ASSERT_TRUE(accessor.beginAccess().has_value());

    spi::MasterAccessConfig next;
    next.freq   = 2000000;
    auto result = accessor.setConfig(next);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::INVALID_ARGUMENT);
    ASSERT_TRUE(accessor.endAccess().has_value());
    ASSERT_TRUE(accessor.setConfig(next).has_value());
    EXPECT_EQ(accessor.getConfig().freq, 2000000u);
}

TEST(MasterAccessor, TransferWrapsTransaction)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = 5;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x12};
    auto result        = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.begin_transaction_count, 1u);
    EXPECT_EQ(bus.end_transaction_count, 1u);
    ASSERT_EQ(bus.transaction_owners.size(), 2u);
    EXPECT_EQ(bus.transaction_owners[0], &accessor);
    EXPECT_EQ(bus.transaction_owners[1], &accessor);
    EXPECT_EQ(bus.transaction_cfgs[0].pin_cs, 5);
}

TEST(MasterAccessor, ExplicitTransactionSpansNestedTransfers)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    const uint8_t first[]  = {0x12};
    const uint8_t second[] = {0x34};
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{first, sizeof(first)}).has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{second, sizeof(second)}).has_value());
    ASSERT_TRUE(accessor.endTransaction().has_value());

    ASSERT_EQ(bus.calls.size(), 2u);
    EXPECT_EQ(bus.begin_transaction_count, 1u);
    EXPECT_EQ(bus.end_transaction_count, 1u);
}

TEST(MasterAccessor, WriteCommandDataSplitsDcLevel)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x2A, 0x00, 0xEF};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(tx));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_TRUE(bus.calls[0].desc.dc_level_valid);
    EXPECT_TRUE(bus.calls[0].desc.dc_level);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Au);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.command_dc_level, 0);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x00, 0xEF}));
}

TEST(MasterAccessor, WriteCommandDataCanTakeSeparateCommand)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x11, 0x22};
    auto result        = accessor.writeCommandData(0x2C, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Cu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x11, 0x22}));
}

TEST(MasterAccessor, WriteCommandDataSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t bytes[] = {0xA0, 0xA1, 0xA2, 0xA3};
    data::MemorySource src{data::ConstDataSpan{bytes, sizeof(bytes)}};
    auto result = accessor.writeCommandData(0x2C, src, 2);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Cu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xA0, 0xA1}));
}

TEST(MasterAccessor, WriteCommandDataUsesWriteDummyCycle)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x01};
    auto result        = accessor.writeCommandData(0x2C, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
}

TEST(MasterAccessor, ReadCommandDataUsesReadDummyCycle)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[1] = {};
    auto result   = accessor.readCommandData(0x0B, data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 8u);
}

TEST(MasterAccessor, WriteCommandAddressDataBuildsSingleTransfer)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xDE, 0xAD};
    auto result        = accessor.writeCommandAddressData(0x02, 0x00123456, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x02u);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x00123456u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 3u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
    EXPECT_EQ(bus.calls[0].desc.command_dc_level, 0);
    EXPECT_EQ(bus.calls[0].desc.address_dc_level, 1);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xDE, 0xAD}));
}

TEST(MasterAccessor, WriteCommandAddressDataSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xA0, 0xA1, 0xA2};
    data::MemorySource src{data::ConstDataSpan{tx, sizeof(tx)}};
    auto result = accessor.writeCommandAddressData(0x02, 0x00123456, src, 2);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x02u);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x00123456u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 3u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xA0, 0xA1}));
}

TEST(MasterAccessor, ReadCommandAddressDataBuildsSingleTransfer)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 16;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[3] = {};
    auto result   = accessor.readCommandAddressData(0x0B, 0x1234, data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6u);
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0x42}));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x0Bu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x1234u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 2u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 8u);
    EXPECT_EQ(bus.calls[0].rx_len, 3u);
}

TEST(MasterAccessor, SendDummyClockUsesTransferDesc)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    auto result = accessor.sendDummyClock(12);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 12);
    EXPECT_TRUE(bus.calls[0].tx.empty());
    EXPECT_EQ(bus.calls[0].rx_len, 0u);
}

// Regression: writeCommandData(span) must reject spi_command_length == 0
// rather than silently degrading to write(). Setting a command length is
// mandatory for the command-sugar family; callers that want a plain write
// must call write() directly.
TEST(MasterAccessor, WriteCommandDataSpanRejectsZeroCommandLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    // spi_command_length defaults to 0 — no command configured.
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xDE, 0xAD};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::INVALID_ARGUMENT);
    // Nothing must have been sent to the bus.
    EXPECT_TRUE(bus.calls.empty());
}

TEST(IBus, DefaultTransferReturnsNotImplemented)
{
    spi::IBus bus;
    spi::MasterAccessConfig cfg;
    spi::TransferDesc desc;
    auto result = bus.transfer(nullptr, cfg, desc, nullptr, nullptr);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::NOT_IMPLEMENTED);
}

// ---- ScopedTransaction (CS scope RAII) ---------------------------------

TEST(ScopedTransaction, ClosesTheTransactionOnScopeExit)
{
    StubIBus bus;
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor dev{bus, cfg};

    {
        spi::ScopedTransaction scope{dev};
        ASSERT_FALSE(scope.has_error());
        EXPECT_EQ(scope.error(), m5::hal::v1::error::error_t::OK);
        EXPECT_EQ(bus.begin_transaction_count, 1u);
        EXPECT_EQ(bus.end_transaction_count, 0u);
    }  // scope exit = endTransaction, even on an early return
    EXPECT_EQ(bus.end_transaction_count, 1u);
}

TEST(ScopedTransaction, SurfacesABeginFailureWithoutClosing)
{
    StubIBus bus;
    bus.fail_begin_transaction = true;
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor dev{bus, cfg};

    {
        spi::ScopedTransaction scope{dev};
        EXPECT_TRUE(scope.has_error());
        EXPECT_EQ(scope.error(), m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
    }
    EXPECT_EQ(bus.end_transaction_count, 0u);  // nothing to close
}

TEST(SoftwareIBus, WriteDrivesCsDcClockAndMosi)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs             = softPin(3);
    cfg.freq               = 20000000;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xA5, 0x5A};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(tx));
    EXPECT_GE(countWrites(port.events, softPin(0), true), 8u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 8u);
    EXPECT_GT(countWrites(port.events, softPin(2), true), 0u);
    EXPECT_GT(countWrites(port.events, softPin(2), false), 0u);
    EXPECT_EQ(countWrites(port.events, softPin(1), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(1), true), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), true), 1u);
}

// Per-device D/C override: a non-negative MasterAccessConfig::pin_dc
// beats the bus-level pin_dc — the bus-level pin must stay untouched.
TEST(SoftwareIBus, AccessorPinDcOverridesBusPinDc)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);  // bus-level default D/C
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs             = softPin(3);
    cfg.pin_dc             = softPin(4);  // device-level override
    cfg.freq               = 20000000;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xA5, 0x5A};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    // The override pin carries the command/data D/C swing...
    EXPECT_EQ(countWrites(port.events, softPin(4), false), 1u);
    EXPECT_GE(countWrites(port.events, softPin(4), true), 1u);
    // ...and the bus-level D/C pin stays silent.
    EXPECT_EQ(countWrites(port.events, softPin(1), false), 0u);
    EXPECT_EQ(countWrites(port.events, softPin(1), true), 0u);
}

// Rejection boundary for the single-lane variants: multi-lane
// modes (dual/quad/octal) are always NOT_IMPLEMENTED; half-duplex modes
// are rejected only when one transfer carries BOTH tx and rx data
// (full-duplex clocking would corrupt the response). One-directional
// half-duplex transfers keep working — the DC-pin demos depend on them.
TEST(SoftwareIBus, UnimplementedDataModesAreRejected)
{
    auto& gpio = softwareSpiGPIO();
    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_miso = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    gpio._port.clear();

    const uint8_t tx[] = {0x01};
    uint8_t rx[1]      = {};

    // Multi-lane: rejected even for a one-directional write.
    {
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::quad_output;
        spi::MasterAccessor accessor{bus, cfg};
        auto result = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), error::error_t::NOT_IMPLEMENTED);
    }

    // Half-duplex carrying both tx and rx data: rejected.
    {
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::halfduplex;
        spi::MasterAccessor accessor{bus, cfg};
        auto result =
            accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{rx, sizeof(rx)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), error::error_t::NOT_IMPLEMENTED);
    }

    // Half-duplex one-directional write: still works.
    {
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::halfduplex_with_dc_pin;
        spi::MasterAccessor accessor{bus, cfg};
        auto result = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});
        EXPECT_TRUE(result.has_value());
    }
}

TEST(SoftwareIBus, ReadSamplesMiso)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_miso = softPin(4);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();
    port.setReadValue(true);

    spi::MasterAccessConfig cfg;
    cfg.freq = 20000000;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[1] = {};
    auto result   = accessor.read(rx, sizeof(rx));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xFF);
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 8u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 8u);
}

TEST(SoftwareIBus, DummyClockCountIsCycleCount)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk = softPin(0);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.freq = 20000000;
    spi::MasterAccessor accessor{bus, cfg};

    auto result = accessor.sendDummyClock(16);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 16u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 16u);
}

TEST(SoftwareIBus, CpolHighIsAppliedBeforeCsAssert)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs   = softPin(3);
    cfg.freq     = 20000000;
    cfg.spi_mode = 2;  // CPOL=1, CPHA=0
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x00};
    auto result        = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_LT(firstWriteIndex(port.events, softPin(0), true), port.events.size());
    ASSERT_LT(firstWriteIndex(port.events, softPin(3), false), port.events.size());
    EXPECT_LT(firstWriteIndex(port.events, softPin(0), true), firstWriteIndex(port.events, softPin(3), false));
}

TEST(SoftwareIBus, ExplicitTransactionKeepsCsAssertedAcrossTransfers)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v1::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = softPin(3);
    cfg.freq   = 20000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t first[]  = {0x12};
    const uint8_t second[] = {0x34};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{first, sizeof(first)}).has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{second, sizeof(second)}).has_value());
    ASSERT_TRUE(accessor.endTransaction().has_value());

    EXPECT_EQ(countWrites(port.events, softPin(3), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), true), 1u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
