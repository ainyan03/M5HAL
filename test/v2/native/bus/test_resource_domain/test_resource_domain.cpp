// SPDX-License-Identifier: MIT

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/i2c/virtual_bus.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <cstdint>
#include <memory>

namespace {
namespace v2 = m5::hal::v2;

class FakeI2cBus : public v2::i2c::IBus {
public:
    ~FakeI2cBus() override
    {
        if (_registry != nullptr && _registry->beginAbandon(_token, _key, this)) {
            _registry->finishAbandon(_token, _key, this, true);
        }
    }
    bool bindRegistryRegistration(v2::bus::BusRegistry& registry, const v2::bus::ResourceKey& key, uint16_t slot,
                                  uint32_t generation) override
    {
        _registry = &registry;
        _key      = key;
        _token    = {slot, 0, generation};
        return _token.valid();
    }

private:
    v2::bus::BusRegistry* _registry = nullptr;
    v2::bus::ResourceKey _key;
    v2::bus::RegistryEntryToken _token;
};

v2::bus::ResourceKey key(void)
{
    return v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21});
}

TEST(ResourceDomain, CopySharesStateAndIndependentDomainSeparatesEveryResource)
{
    v2::ResourceDomain first;
    v2::ResourceDomain alias = first;
    v2::ResourceDomain second;

    EXPECT_TRUE(first.sharesStateWith(alias));
    EXPECT_FALSE(first.sharesStateWith(second));
    EXPECT_EQ(&first.busRegistry(), &alias.busRegistry());
    EXPECT_EQ(&first.gpio(), &alias.gpio());
    EXPECT_EQ(&first.services(), &alias.services());
    EXPECT_EQ(&first.memory(), &alias.memory());
    EXPECT_NE(&first.busRegistry(), &second.busRegistry());
    EXPECT_NE(&first.gpio(), &second.gpio());
    EXPECT_NE(&first.services(), &second.services());
    EXPECT_NE(&first.memory(), &second.memory());
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&first.memory()) % alignof(v2::memory::Allocator), 0u);
}

TEST(ResourceDomain, SameKeyInternsWithinOneDomainButNotAcrossDomains)
{
    v2::ResourceDomain first;
    v2::ResourceDomain second;
    int makes = 0;
    auto make = [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        ++makes;
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<FakeI2cBus>()};
    };

    auto a = first.busRegistry().acquireOrFind(key(), make);
    auto b = first.busRegistry().acquireOrFind(key(), make);
    auto c = second.busRegistry().acquireOrFind(key(), make);

    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());
    EXPECT_NE(a.value().get(), c.value().get());
    EXPECT_EQ(makes, 2);
}

TEST(ResourceDomain, LocalBackendUsesInjectedDomainRegistry)
{
    v2::ResourceDomain domain;
    v2::bus::LocalBackend backend{domain};
    EXPECT_EQ(&backend.busRegistry(), &domain.busRegistry());
}

TEST(ResourceDomain, LocalConnectPropagatesGpioBootstrapFailureAndCanRetry)
{
    v2::ResourceDomain domain;
    const auto* gpio = v2::gpio::getGPIO();
    ASSERT_NE(gpio, nullptr);
    for (size_t i = 0; i < v2::gpio::GPIOGroup::kMaxEntries; ++i) {
        ASSERT_TRUE(domain.gpio().addGPIO(gpio, static_cast<v2::types::gpio_slot_t>(i + 1u)).has_value());
    }

    v2::Hal hal{domain};
    auto full = hal.init();
    ASSERT_FALSE(full.has_value());
    EXPECT_EQ(full.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(hal.backend(), nullptr);
    EXPECT_FALSE(domain.gpio().hasGPIO(0));

    ASSERT_TRUE(domain.gpio().removeGPIO(1).has_value());
    ASSERT_TRUE(hal.init().has_value());
    EXPECT_NE(hal.backend(), nullptr);
    EXPECT_TRUE(domain.gpio().hasGPIO(0));
}

TEST(ResourceDomain, HalInstancesSharingDomainAlsoShareLocalBackendAndBus)
{
    v2::ResourceDomain domain;
    v2::Hal first{domain};
    v2::Hal second{domain};
    ASSERT_TRUE(first.init().has_value());
    ASSERT_TRUE(second.init().has_value());
    ASSERT_NE(first.backend(), nullptr);
    EXPECT_EQ(first.backend(), second.backend());

    v2::i2c::BusConfig cfg;
    cfg.pin_scl     = 22;
    cfg.pin_sda     = 21;
    auto first_bus  = first.I2C.acquire(cfg);
    auto second_bus = second.I2C.acquire(cfg);
    ASSERT_TRUE(first_bus.has_value());
    ASSERT_TRUE(second_bus.has_value());
    EXPECT_EQ(first_bus.value().get(), second_bus.value().get());
}

TEST(ResourceDomain, DestroyingOneLocalHalDoesNotClearDomainWatchService)
{
    v2::ResourceDomain domain;
    auto first = std::unique_ptr<v2::Hal>{new v2::Hal{domain}};
    v2::Hal second{domain};
    ASSERT_TRUE(first->init().has_value());
    ASSERT_TRUE(second.init().has_value());
    ASSERT_TRUE(domain.gpio()
                    .setWatchSink([](void*, v2::types::gpio_number_t, bool, v2::gpio::GPIOGroup::Edge) {}, nullptr)
                    .has_value());
    ASSERT_EQ(domain.services().size(), 1u);

    first.reset();
    EXPECT_EQ(domain.services().size(), 1u);
    EXPECT_TRUE(domain.gpio().setWatchSink(nullptr, nullptr).has_value());
}

TEST(ResourceDomain, DestroyingLastHalDoesNotClearDomainOwnedWatchService)
{
    v2::ResourceDomain domain;
    {
        v2::Hal only{domain};
        ASSERT_TRUE(only.init().has_value());
        ASSERT_TRUE(domain.gpio()
                        .setWatchSink([](void*, v2::types::gpio_number_t, bool, v2::gpio::GPIOGroup::Edge) {}, nullptr)
                        .has_value());
        ASSERT_EQ(domain.services().size(), 1u);
    }

    EXPECT_EQ(domain.services().size(), 1u);
    EXPECT_TRUE(domain.gpio().setWatchSink(nullptr, nullptr).has_value());
}

TEST(ResourceDomain, IndependentHalUsesItsOwnGpioServicesAndRegistry)
{
    v2::ResourceDomain first_domain;
    v2::ResourceDomain second_domain;
    v2::Hal first{first_domain};
    v2::Hal second{second_domain};
    ASSERT_TRUE(first.init().has_value());
    ASSERT_TRUE(second.init().has_value());

    v2::i2c::VirtualOpenDrainBus first_wire;
    v2::i2c::VirtualOpenDrainBus second_wire;
    v2::i2c::VirtualI2CPort first_scl{first_wire, v2::i2c::VirtualI2CPort::Line::SCL};
    v2::i2c::VirtualI2CPort first_sda{first_wire, v2::i2c::VirtualI2CPort::Line::SDA};
    v2::i2c::VirtualI2CPort second_scl{second_wire, v2::i2c::VirtualI2CPort::Line::SCL};
    v2::i2c::VirtualI2CPort second_sda{second_wire, v2::i2c::VirtualI2CPort::Line::SDA};
    v2::i2c::ScopedVirtualI2CGPIO first_gpio{first_scl, first_sda, first.Gpio};
    v2::i2c::ScopedVirtualI2CGPIO second_gpio{second_scl, second_sda, second.Gpio};
    ASSERT_EQ(first_gpio.scl(), second_gpio.scl());
    ASSERT_EQ(first_gpio.sda(), second_gpio.sda());

    v2::i2c::SlaveEndpoint slave{first_wire, 0x42};
    v2::service::ServiceRunner slave_runner;
    const auto added = slave_runner.add(slave.service());
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());
    first_wire.setRunner(&slave_runner);

    v2::i2c::BusConfig first_cfg;
    first_cfg.pin_scl = first_gpio.scl();
    first_cfg.pin_sda = first_gpio.sda();
    auto second_cfg   = first_cfg;
    auto first_bus    = first.I2C.acquire(first_cfg);
    auto second_bus   = second.I2C.acquire(second_cfg);
    ASSERT_TRUE(first_bus.has_value());
    ASSERT_TRUE(second_bus.has_value());
    EXPECT_NE(first_bus.value().get(), second_bus.value().get());
    EXPECT_EQ(first_bus.value()->localResourceContext().gpio, &first.Gpio);
    EXPECT_EQ(second_bus.value()->localResourceContext().gpio, &second.Gpio);
    EXPECT_TRUE(first_bus.value()->probe(0x42).has_value());
    EXPECT_FALSE(second_bus.value()->probe(0x42).has_value());
}

TEST(ResourceDomain, AcquiredBusCoOwnsStateAfterFacadeAndHalDestruction)
{
    std::shared_ptr<v2::i2c::IBus> bus;
    std::weak_ptr<void> connection_lifetime;
    v2::gpio::GPIOGroup* gpio = nullptr;
    {
        v2::ResourceDomain domain;
        v2::Hal hal{domain};
        ASSERT_TRUE(hal.init().has_value());
        v2::i2c::BusConfig cfg;
        cfg.pin_scl   = 22;
        cfg.pin_sda   = 21;
        auto acquired = hal.I2C.acquire(cfg);
        ASSERT_TRUE(acquired.has_value());
        bus                 = acquired.value();
        gpio                = &hal.Gpio;
        connection_lifetime = bus->localResourceContext().lifetime;
    }

    ASSERT_TRUE(bus);
    EXPECT_TRUE(bus->localResourceContext().valid());
    EXPECT_EQ(bus->localResourceContext().gpio, gpio);
    EXPECT_TRUE(gpio->hasGPIO(0));
    EXPECT_FALSE(connection_lifetime.expired());

    bus.reset();
    EXPECT_TRUE(connection_lifetime.expired());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
