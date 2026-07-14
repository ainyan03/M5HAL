// SPDX-License-Identifier: MIT
#ifndef TEST_V2_NATIVE_BUS_STATIC_BUS_VIEW_CONTRACT_HPP_
#define TEST_V2_NATIVE_BUS_STATIC_BUS_VIEW_CONTRACT_HPP_

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

namespace m5::hal::v2::test::bus_contract {

template <class View>
void expectStaticCommitSurface(View& view)
{
    EXPECT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(view.hardwareInUse(), 0u);
}

template <class View, class LogicalConfig>
void expectStaticLogicalAcquireContract(View& view, LogicalConfig valid_req, LogicalConfig unset_pin_req)
{
    auto r = view.acquire(valid_req);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::NOT_IMPLEMENTED);

    // Identity preserves unset pin roles; the static backend, not BusView,
    // decides whether it can create such a bus. This backend offers no logical
    // creation at all, so both requests reach the same NOT_IMPLEMENTED result.
    auto unset_pin = view.acquire(unset_pin_req);
    ASSERT_FALSE(unset_pin.has_value());
    EXPECT_EQ(unset_pin.error(), error::error_t::NOT_IMPLEMENTED);

    LogicalConfig invalid_intent  = valid_req;
    invalid_intent.intent.require = types::backend_caps::HARDWARE;
    invalid_intent.intent.forbid  = types::backend_caps::HARDWARE;
    auto invalid_intent_result    = view.acquire(invalid_intent);
    ASSERT_FALSE(invalid_intent_result.has_value());
    EXPECT_EQ(invalid_intent_result.error(), error::error_t::INVALID_ARGUMENT);
}

}  // namespace m5::hal::v2::test::bus_contract

#endif  // TEST_V2_NATIVE_BUS_STATIC_BUS_VIEW_CONTRACT_HPP_
