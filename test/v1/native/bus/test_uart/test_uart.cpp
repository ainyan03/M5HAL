#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace {

class RecordingUARTBus : public m5::hal::v1::uart::UARTBus {
public:
    m5::stl::expected<void, m5::hal::v1::error::error_t> init(const m5::hal::v1::bus::BusConfig& config) override
    {
        if (config.getBusKind() != m5::hal::v1::types::bus_kind_t::UART) {
            return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        _config = static_cast<const m5::hal::v1::uart::UARTBusConfig&>(config);
        return {};
    }

    m5::stl::expected<void, m5::hal::v1::error::error_t> release(void) override
    {
        tx_recorded.clear();
        rx_queue.clear();
        return {};
    }

    m5::stl::expected<size_t, m5::hal::v1::error::error_t> write(m5::hal::v1::bus::Accessor* owner,
                                                                 const m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                 m5::hal::v1::data::Source* tx, size_t len) override
    {
        last_owner  = owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (tx != nullptr && !tx->eof() && done < len) {
            auto span = tx->peek(len - done);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span.value().size == 0) {
                break;
            }
            tx_recorded.insert(tx_recorded.end(), span.value().data, span.value().data + span.value().size);
            auto advanced = tx->advance(span.value().size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            done += span.value().size;
        }
        return done;
    }

    m5::stl::expected<size_t, m5::hal::v1::error::error_t> read(m5::hal::v1::bus::Accessor* owner,
                                                                const m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                m5::hal::v1::data::Sink* rx, size_t len) override
    {
        last_owner  = owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (rx != nullptr && !rx->closed() && done < len && !rx_queue.empty()) {
            auto span = rx->reserve(len - done);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span.value().size == 0) {
                break;
            }
            size_t count = span.value().size;
            if (count > rx_queue.size()) {
                count = rx_queue.size();
            }
            for (size_t i = 0; i < count; ++i) {
                span.value().data[i] = rx_queue[i];
            }
            rx_queue.erase(rx_queue.begin(),
                           rx_queue.begin() + static_cast<std::vector<uint8_t>::difference_type>(count));
            auto committed = rx->commit(count);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
            done += count;
        }
        return done;
    }

    m5::stl::expected<size_t, m5::hal::v1::error::error_t> readableBytes(
        m5::hal::v1::bus::Accessor* owner, const m5::hal::v1::uart::UARTAccessConfig& cfg) override
    {
        last_owner = owner;
        last_cfg   = cfg;
        return rx_queue.size();
    }

    std::vector<uint8_t> tx_recorded;
    std::vector<uint8_t> rx_queue;
    m5::hal::v1::bus::Accessor* last_owner = nullptr;
    m5::hal::v1::uart::UARTAccessConfig last_cfg;
};

}  // namespace

TEST(UARTBusConfig, DefaultCtorSetsUARTKind)
{
    m5::hal::v1::uart::UARTBusConfig cfg;
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v1::types::bus_kind_t::UART);
    EXPECT_EQ(cfg.pin_tx, -1);
    EXPECT_EQ(cfg.pin_rx, -1);
    EXPECT_EQ(cfg.rx_buffer_size, 256u);
    EXPECT_EQ(cfg.tx_buffer_size, 0u);
}

TEST(UARTAccessor, WriteConsumesSource)
{
    RecordingUARTBus bus;
    m5::hal::v1::uart::UARTAccessConfig cfg;
    cfg.baud_rate = 921600;
    m5::hal::v1::uart::UARTAccessor dev{bus, cfg};

    const uint8_t payload[] = {0x11, 0x22, 0x33};
    auto result             = dev.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[0], 0x11);
    EXPECT_EQ(bus.tx_recorded[2], 0x33);
    EXPECT_EQ(bus.last_owner, &dev);
    EXPECT_EQ(bus.last_cfg.baud_rate, 921600u);
}

TEST(UARTAccessor, ReadFillsSink)
{
    RecordingUARTBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2};
    m5::hal::v1::uart::UARTAccessConfig cfg;
    m5::hal::v1::uart::UARTAccessor dev{bus, cfg};

    uint8_t dst[2] = {};
    auto result    = dev.read(dst, sizeof(dst));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(dst));
    EXPECT_EQ(dst[0], 0xA0);
    EXPECT_EQ(dst[1], 0xA1);
    ASSERT_EQ(bus.rx_queue.size(), 1u);
    EXPECT_EQ(bus.rx_queue[0], 0xA2);
}

TEST(UARTAccessor, ReadableBytesUsesBus)
{
    RecordingUARTBus bus;
    bus.rx_queue = {1, 2, 3, 4};
    m5::hal::v1::uart::UARTAccessConfig cfg;
    m5::hal::v1::uart::UARTAccessor dev{bus, cfg};

    auto result = dev.readableBytes();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 4u);
}

TEST(UARTAccessor, SetConfigRejectsInsideAccess)
{
    RecordingUARTBus bus;
    m5::hal::v1::uart::UARTAccessConfig cfg;
    m5::hal::v1::uart::UARTAccessor dev{bus, cfg};

    auto locked = dev.beginAccess(0);
    ASSERT_TRUE(locked.has_value());
    cfg.baud_rate = 57600;
    auto result   = dev.setConfig(cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    ASSERT_TRUE(dev.endAccess().has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
