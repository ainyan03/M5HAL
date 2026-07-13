// SPDX-License-Identifier: MIT
// Native gtest for service::CompletionGate / SpinBackoff
// (hal/v2/service/completion_gate.hpp): the standard producer/consumer
// handoff primitive for a service-polled transfer publishing completion
// to a waiting consumer thread.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <m5_hal/hal/v2/service/completion_gate.hpp>

#include <thread>

namespace {

namespace service = ::m5::hal::v2::service;
using service::CompletionGate;
using service::SpinBackoff;
using GateState = CompletionGate::State;

TEST(CompletionGate, InitialStateIsIdle)
{
    CompletionGate gate;
    EXPECT_EQ(gate.state(), GateState::Idle);
    EXPECT_FALSE(gate.busy());
    EXPECT_EQ(gate.wait(), GateState::Idle);  // never armed -> returns immediately
}

TEST(CompletionGate, ArmTransitionsToBusy)
{
    CompletionGate gate;
    gate.arm();
    EXPECT_EQ(gate.state(), GateState::Busy);
    EXPECT_TRUE(gate.busy());
}

TEST(CompletionGate, FinishDoneIsObservedByStateAndWait)
{
    CompletionGate gate;
    gate.arm();
    gate.finish(GateState::Done);
    EXPECT_EQ(gate.state(), GateState::Done);
    EXPECT_FALSE(gate.busy());
    EXPECT_EQ(gate.wait(), GateState::Done);
}

TEST(CompletionGate, FinishErrorIsObservedByState)
{
    CompletionGate gate;
    gate.arm();
    gate.finish(GateState::Error);
    EXPECT_EQ(gate.state(), GateState::Error);
    EXPECT_FALSE(gate.busy());
}

TEST(CompletionGate, ResetReturnsToIdle)
{
    CompletionGate gate;
    gate.arm();
    gate.finish(GateState::Done);
    gate.reset();
    EXPECT_EQ(gate.state(), GateState::Idle);
    EXPECT_FALSE(gate.busy());
}

// Producer writes a payload, then publishes completion via finish(); the
// consumer must only observe the payload after wait() reports Done (the
// gate's release/acquire contract). join() makes the ordering deterministic
// for the test itself while the gate is what the production code relies on.
TEST(CompletionGate, ProducerConsumerHandoffPublishesPayload)
{
    CompletionGate gate;
    int payload = 0;
    gate.arm();

    std::thread producer([&] {
        payload = 42;
        gate.finish(GateState::Done);
    });

    EXPECT_EQ(gate.wait(), GateState::Done);
    EXPECT_EQ(payload, 42);

    producer.join();
}

TEST(SpinBackoffSmoke, StepAndResetDoNotCrash)
{
    SpinBackoff backoff;
    for (int i = 0; i < 16; ++i) {
        backoff.step();
    }
    backoff.reset();
    backoff.step();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
