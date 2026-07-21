// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/bus/local_backend.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <vector>

namespace m5::hal::v2::pdm {

using FakeBusConfig = IBusConfig;

class FakeBus : public IBus {
public:
    result_t<void> init(const FakeBusConfig& cfg)
    {
        if (cfg.pin_clk < 0 || cfg.pin_din < 0 || cfg.rx_buffer_size == 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _config = cfg;
        return {};
    }

    result_t<void> lockFor(bus::IAccessor& owner)
    {
        return acquireAccessLock(owner, 0);
    }

    result_t<void> unlockFor(bus::IAccessor& owner)
    {
        return releaseAccessLock(owner);
    }

    result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context) override
    {
        last_owner = &bus::OperationSlot::contextOwner(context);
        last_cfg   = context.config;
        ++begin_count;
        return {};
    }

    result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context) override
    {
        last_owner = &bus::OperationSlot::contextOwner(context);
        last_cfg   = context.config;
        ++end_count;
        return {};
    }

    result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len) override
    {
        last_owner = &bus::OperationSlot::contextOwner(context);
        last_cfg   = context.config;
        ++read_calls;
        const size_t count = std::min(len, samples.size());
        if (count == 0 || dst == nullptr) {
            return size_t{0};
        }
        auto span = dst->reserve(count);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        const size_t copied = std::min(count, span->size);
        std::copy_n(samples.begin(), copied, span->data);
        auto committed = dst->commit(copied);
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        samples.erase(samples.begin(), samples.begin() + static_cast<std::vector<uint8_t>::difference_type>(copied));
        return copied;
    }

    result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context) override
    {
        last_owner = &bus::OperationSlot::contextOwner(context);
        last_cfg   = context.config;
        ++readable_calls;
        return samples.size();
    }

    std::vector<uint8_t> samples;
    bus::IAccessor* last_owner = nullptr;
    AccessConfig last_cfg;
    size_t begin_count    = 0;
    size_t end_count      = 0;
    size_t read_calls     = 0;
    size_t readable_calls = 0;
};

class InspectableAccessor : public bus::IAccessor {
public:
    InspectableAccessor(FakeBus& bus, const AccessConfig& config = {})
        : bus::IAccessor{bus}, _typed_bus{bus}, _context{makeOperationContext(config)}
    {
    }

    const AccessConfig& getConfig() const override
    {
        return _context.config;
    }

    bus::OperationContext<AccessConfig>& context()
    {
        return _context;
    }

    result_t<void> begin()
    {
        return _beginOperationAccess(_context, 0, bus::OperationMode::Rx,
                                     [&](auto& context) { return _typed_bus.beginOperation(context); });
    }

    result_t<void> end()
    {
        return _endOperationAccess(_context, 0, [&](auto& context) { return _typed_bus.endOperation(context); });
    }

private:
    FakeBus& _typed_bus;
    bus::OperationContext<AccessConfig> _context;
};

result_t<std::unique_ptr<IBus>> makeFakeBackend(const bus::LocalResourceContext&, const IBusConfig& cfg)
{
    std::unique_ptr<FakeBus> backend{new (std::nothrow) FakeBus()};
    if (!backend) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto initialized = backend->init(cfg);
    if (!initialized.has_value()) {
        return m5::stl::make_unexpected(initialized.error());
    }
    return std::unique_ptr<IBus>{std::move(backend)};
}

}  // namespace m5::hal::v2::pdm

namespace {
namespace v2 = m5::hal::v2;

#define ASSERT_RESULT_OK(expr)                                                                           \
    do {                                                                                                 \
        auto result_ok = (expr);                                                                         \
        ASSERT_TRUE(result_ok.has_value()) << "err=" << m5::hal::v2::error::toString(result_ok.error()); \
    } while (false)

TEST(PdmContract, AppendsWireKindWithoutRenumberingExistingKinds)
{
    EXPECT_EQ(static_cast<uint8_t>(v2::types::bus_kind_t::I2S), 3u);
    EXPECT_EQ(static_cast<uint8_t>(v2::types::bus_kind_t::UART), 4u);
    EXPECT_EQ(static_cast<uint8_t>(v2::types::bus_kind_t::DAC), 8u);
    EXPECT_EQ(static_cast<uint8_t>(v2::types::bus_kind_t::PDM), 9u);
}

TEST(PdmContract, TaggedConfigUsesIndependentKindAndPins)
{
    v2::pdm::FakeBusConfig cfg{v2::pdm::Clk{4}, v2::pdm::Din{5}};
    EXPECT_EQ(cfg.getBusKind(), v2::types::bus_kind_t::PDM);
    EXPECT_EQ(cfg.pin_clk, 4);
    EXPECT_EQ(cfg.pin_din, 5);
    v2::pdm::AccessConfig access;
    EXPECT_EQ(access.getBusKind(), v2::types::bus_kind_t::PDM);
    EXPECT_EQ(access.bits_per_sample, 16);
    EXPECT_EQ(access.channels, 1);
}

TEST(PDMCheckedFacade, RejectsInactiveEndedWrongBusAndWrongAccessorContexts)
{
    v2::pdm::FakeBus first_bus;
    v2::pdm::FakeBus second_bus;
    v2::pdm::InspectableAccessor first{first_bus};
    v2::pdm::InspectableAccessor other{first_bus};

    auto inactive_read = first_bus.read(first.context(), nullptr, 0);
    ASSERT_FALSE(inactive_read.has_value());
    EXPECT_EQ(inactive_read.error(), v2::error::error_t::INVALID_STATE);
    auto inactive_readable = first_bus.readableBytes(first.context());
    ASSERT_FALSE(inactive_readable.has_value());
    EXPECT_EQ(inactive_readable.error(), v2::error::error_t::INVALID_STATE);

    first.context().runtime.begin(0, 0, v2::bus::OperationMode::Rx);
    ASSERT_RESULT_OK(first_bus.lockFor(other));
    auto wrong_accessor = first_bus.beginOperation(first.context());
    ASSERT_FALSE(wrong_accessor.has_value());
    EXPECT_EQ(wrong_accessor.error(), v2::error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(first_bus.unlockFor(other));

    ASSERT_RESULT_OK(first.begin());
    auto wrong_bus = second_bus.readableBytes(first.context());
    ASSERT_FALSE(wrong_bus.has_value());
    EXPECT_EQ(wrong_bus.error(), v2::error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(first.end());

    auto ended = first_bus.read(first.context(), nullptr, 0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(first_bus.read_calls, 0u);
    EXPECT_EQ(first_bus.readable_calls, 0u);
}

TEST(PDMCheckedFacade, RejectsCorruptRuntimeAndRecoversSlotAndLock)
{
    v2::pdm::FakeBus pdm_bus;
    v2::pdm::InspectableAccessor accessor{pdm_bus};

    ASSERT_RESULT_OK(accessor.begin());
    const auto registered_generation = accessor.context().runtime.generation;
    ++accessor.context().runtime.generation;
    auto stale = pdm_bus.readableBytes(accessor.context());
    ASSERT_FALSE(stale.has_value());
    EXPECT_EQ(stale.error(), v2::error::error_t::INVALID_STATE);
    auto stale_end = accessor.end();
    ASSERT_FALSE(stale_end.has_value());
    EXPECT_EQ(stale_end.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_FALSE(accessor.inAccess());

    ASSERT_RESULT_OK(accessor.begin());
    EXPECT_GT(accessor.context().runtime.generation, registered_generation);
    accessor.context().runtime.mode = v2::bus::OperationMode::Tx;
    auto wrong_mode                 = pdm_bus.read(accessor.context(), nullptr, 0);
    ASSERT_FALSE(wrong_mode.has_value());
    EXPECT_EQ(wrong_mode.error(), v2::error::error_t::INVALID_STATE);
    auto wrong_mode_end = accessor.end();
    ASSERT_FALSE(wrong_mode_end.has_value());
    EXPECT_EQ(wrong_mode_end.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_FALSE(accessor.inAccess());

    pdm_bus.samples = {1, 2};
    ASSERT_RESULT_OK(accessor.begin());
    auto recovered = pdm_bus.readableBytes(accessor.context());
    ASSERT_TRUE(recovered.has_value()) << "err=" << v2::error::toString(recovered.error());
    EXPECT_EQ(recovered.value(), 2u);
    ASSERT_RESULT_OK(accessor.end());
}

TEST(PdmPortableProvider, RejectsIncompleteWiring)
{
    v2::pdm::FakeBusConfig cfg;
    cfg.pin_clk      = 4;
    auto initialized = v2::pdm::makeFakeBackend({}, cfg);
    ASSERT_FALSE(initialized.has_value());
    EXPECT_EQ(initialized.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(PdmAccessor, ReadAndReadableForwardOwnerAndConfig)
{
    v2::pdm::FakeBus bus;
    v2::pdm::FakeBusConfig bus_cfg{v2::pdm::Clk{4}, v2::pdm::Din{5}};
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    bus.samples = {1, 2, 3, 4};
    v2::pdm::AccessConfig cfg;
    cfg.sample_rate_hz = 32000;
    v2::pdm::RxAccessor acc{bus, cfg};

    auto available = acc.readableBytes();
    ASSERT_TRUE(available.has_value());
    EXPECT_EQ(available.value(), 4u);
    uint8_t dst[4] = {};
    auto read      = acc.read(dst, sizeof(dst));
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(read.value(), sizeof(dst));
    EXPECT_EQ(dst[0], 1u);
    EXPECT_EQ(dst[3], 4u);
    EXPECT_EQ(bus.last_owner, &acc);
    EXPECT_EQ(bus.last_cfg.sample_rate_hz, 32000u);
}

TEST(PdmAccessor, AccessIsNonNestingAndTracksEachRead)
{
    v2::pdm::FakeBus bus;
    v2::pdm::FakeBusConfig bus_cfg{v2::pdm::Clk{6}, v2::pdm::Din{7}};
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    bus.samples = {1, 2, 3, 4};
    v2::pdm::RxAccessor acc{bus, {}};
    ASSERT_TRUE(acc.beginAccess().has_value());
    auto nested = acc.beginAccess();
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), v2::error::error_t::INVALID_STATE);
    uint8_t first[2]  = {};
    uint8_t second[2] = {};
    ASSERT_TRUE(acc.read(first, sizeof(first)).has_value());
    ASSERT_TRUE(acc.read(second, sizeof(second)).has_value());
    ASSERT_TRUE(acc.endAccess().has_value());
    EXPECT_EQ(bus.begin_count, 1u);
    EXPECT_EQ(bus.end_count, 1u);
    auto status = acc.getLastTransferStatus();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->transfer_id, 2u);
    EXPECT_EQ(status->totals.rx, sizeof(second));
    EXPECT_EQ(status->completion, v2::bus::CompletionLevel::Complete);
}

TEST(PdmBusView, InternsIdentityAndRejectsConfigDrift)
{
    v2::bus::LocalBackend backend;
    v2::bus::LocalPortableProvider<v2::pdm::BusTraits> provider{&v2::pdm::makeFakeBackend};
    backend.registerPortableProvider(provider);
    v2::pdm::BusView view{&backend};
    v2::pdm::FakeBusConfig first{v2::pdm::Clk{20}, v2::pdm::Din{21}};
    auto a = view.acquire(first);
    ASSERT_TRUE(a.has_value());
    auto b = view.acquire(first);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a->get(), b->get());

    auto changed           = first;
    changed.rx_buffer_size = 4096;
    auto rejected          = view.acquire(changed);
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), v2::error::error_t::INVALID_STATE);
}

TEST(PdmBytecode, ConfigHasIndependentLayout)
{
    v2::pdm::AccessConfig cfg;
    cfg.sample_rate_hz                          = 48000;
    cfg.read_timeout_ms                         = 321;
    uint8_t bytes[v2::bytecode::kPDMConfigSize] = {};
    v2::bytecode::detail::encodeConfig(bytes, cfg);
    EXPECT_EQ(bytes[0], 0x80u);
    EXPECT_EQ(bytes[1], 0xBBu);
    EXPECT_EQ(bytes[4], 0x41u);
    EXPECT_EQ(bytes[5], 0x01u);
    EXPECT_EQ(bytes[8], 16u);
    EXPECT_EQ(bytes[9], 1u);
}

TEST(PdmBytecode, RunnerUsesIndependentRxBinding)
{
    v2::pdm::FakeBus bus;
    v2::pdm::FakeBusConfig bus_cfg{v2::pdm::Clk{30}, v2::pdm::Din{31}};
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    bus.samples = {0x10, 0x11, 0x20, 0x21};
    v2::pdm::RxAccessor accessor{bus, {}};
    v2::bytecode::BytecodeRunner runner;
    ASSERT_TRUE(runner.registerPDM(2, accessor).has_value());

    uint8_t dst[4]   = {};
    size_t actual_tx = 99;
    size_t actual_rx = 0;
    auto transferred =
        runner.streamTransferChunk(v2::types::bus_kind_t::PDM, 2, {}, {}, {dst, sizeof(dst)}, actual_tx, actual_rx);
    ASSERT_TRUE(transferred.has_value());
    EXPECT_EQ(actual_tx, 0u);
    EXPECT_EQ(actual_rx, sizeof(dst));
    EXPECT_EQ(dst[0], 0x10u);
    EXPECT_EQ(dst[3], 0x21u);
}

TEST(PdmRemote, HasIndependentBusIdPool)
{
    v2::remote::RemoteBusIdState ids;
    EXPECT_EQ(ids.reserve(v2::types::bus_kind_t::PDM), 0u);
    EXPECT_EQ(ids.reserve(v2::types::bus_kind_t::PDM), 1u);
    ids.release(v2::types::bus_kind_t::PDM, 0);
    EXPECT_EQ(ids.reserve(v2::types::bus_kind_t::PDM), 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
