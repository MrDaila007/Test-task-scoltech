#include "fcstub/mode_machine.hpp"

#include <gtest/gtest.h>

#include <tuple>

namespace {

using fcstub::AckResult;
using fcstub::Mode;
using fcstub::ModeEvent;
using fcstub::ModeMachine;
using fcstub::TimeNs;
using fcstub::kNsPerMs;

constexpr TimeNs kReady = 2000 * kNsPerMs;
constexpr TimeNs kTimeout = 500 * kNsPerMs;
constexpr TimeNs kNoSetpoint = ModeMachine::kNoSetpoint;

ModeMachine booted() {
    ModeMachine m({kReady, kTimeout});
    m.boot(0);
    return m;
}

// Drives a fresh machine into `target` at time kReady with a live setpoint stream.
ModeMachine machine_in(Mode target) {
    ModeMachine m = booted();
    const TimeNs t = kReady;
    (void)m.tick(t, kNoSetpoint);
    if (target == Mode::NotReady) {
        return booted();
    }
    if (target == Mode::Ready) {
        return m;
    }
    EXPECT_EQ(m.request_arm(true, t).result, AckResult::Accepted);
    if (target == Mode::Offboard || target == Mode::Hold) {
        EXPECT_EQ(m.request_mode(target, t, t).result, AckResult::Accepted);
    }
    EXPECT_EQ(m.mode(), target);
    return m;
}

}  // namespace

TEST(ModeMachine, BootsNotReadyThenBecomesReady) {
    ModeMachine m = booted();
    EXPECT_EQ(m.mode(), Mode::NotReady);
    EXPECT_FALSE(m.armed());
    EXPECT_EQ(m.next_deadline(kNoSetpoint), kReady);
    EXPECT_EQ(m.tick(kReady - 1, kNoSetpoint), ModeEvent::None);
    EXPECT_EQ(m.tick(kReady, kNoSetpoint), ModeEvent::BecameReady);
    EXPECT_EQ(m.mode(), Mode::Ready);
}

TEST(ModeMachine, ArmFromNotReadyIsTemporarilyRejected) {
    ModeMachine m = booted();
    const auto r = m.request_arm(true, kReady / 2);
    EXPECT_EQ(r.result, AckResult::TemporarilyRejected);
    EXPECT_EQ(m.mode(), Mode::NotReady);
}

TEST(ModeMachine, ArmGoesToManualAndDisarmBackToReady) {
    ModeMachine m = machine_in(Mode::Ready);
    EXPECT_EQ(m.request_arm(true, kReady).result, AckResult::Accepted);
    EXPECT_EQ(m.mode(), Mode::Manual);
    EXPECT_TRUE(m.armed());
    EXPECT_EQ(m.request_arm(false, kReady).result, AckResult::Accepted);
    EXPECT_EQ(m.mode(), Mode::Ready);
    EXPECT_FALSE(m.armed());
}

// Full (mode x request) table for mode changes, setpoint stream alive.
class ModeTable : public ::testing::TestWithParam<std::tuple<Mode, Mode, AckResult, Mode>> {};

TEST_P(ModeTable, Transition) {
    const auto [from, request, expected_result, expected_mode] = GetParam();
    ModeMachine m = machine_in(from);
    const TimeNs t = kReady + kNsPerMs;
    EXPECT_EQ(m.request_mode(request, t, t).result, expected_result);
    EXPECT_EQ(m.mode(), expected_mode);
}

INSTANTIATE_TEST_SUITE_P(
    AllPairs, ModeTable,
    ::testing::Values(
        std::make_tuple(Mode::NotReady, Mode::Manual, AckResult::Denied, Mode::NotReady),
        std::make_tuple(Mode::NotReady, Mode::Offboard, AckResult::Denied, Mode::NotReady),
        std::make_tuple(Mode::NotReady, Mode::Hold, AckResult::Denied, Mode::NotReady),
        std::make_tuple(Mode::Ready, Mode::Manual, AckResult::Denied, Mode::Ready),
        std::make_tuple(Mode::Ready, Mode::Offboard, AckResult::Denied, Mode::Ready),
        std::make_tuple(Mode::Ready, Mode::Hold, AckResult::Denied, Mode::Ready),
        std::make_tuple(Mode::Manual, Mode::Manual, AckResult::Accepted, Mode::Manual),
        std::make_tuple(Mode::Manual, Mode::Offboard, AckResult::Accepted, Mode::Offboard),
        std::make_tuple(Mode::Manual, Mode::Hold, AckResult::Accepted, Mode::Hold),
        std::make_tuple(Mode::Offboard, Mode::Manual, AckResult::Accepted, Mode::Manual),
        std::make_tuple(Mode::Offboard, Mode::Offboard, AckResult::Accepted, Mode::Offboard),
        std::make_tuple(Mode::Offboard, Mode::Hold, AckResult::Accepted, Mode::Hold),
        std::make_tuple(Mode::Hold, Mode::Manual, AckResult::Accepted, Mode::Manual),
        std::make_tuple(Mode::Hold, Mode::Offboard, AckResult::Accepted, Mode::Offboard),
        std::make_tuple(Mode::Hold, Mode::Hold, AckResult::Accepted, Mode::Hold),
        // NotReady/Ready are not requestable flight modes.
        std::make_tuple(Mode::Manual, Mode::Ready, AckResult::Unsupported, Mode::Manual),
        std::make_tuple(Mode::Hold, Mode::NotReady, AckResult::Unsupported, Mode::Hold)));

TEST(ModeMachine, OffboardNeedsALiveSetpointStream) {
    ModeMachine m = machine_in(Mode::Manual);
    const TimeNs t = 10 * kReady;
    auto r = m.request_mode(Mode::Offboard, t, kNoSetpoint);
    EXPECT_EQ(r.result, AckResult::Denied);
    EXPECT_EQ(r.reason, fcstub::DenyReason::NoSetpointStream);
    r = m.request_mode(Mode::Offboard, t, t - kTimeout - 1);  // stale
    EXPECT_EQ(r.result, AckResult::Denied);
    EXPECT_EQ(m.request_mode(Mode::Offboard, t, t - kTimeout).result, AckResult::Accepted);
}

TEST(ModeMachine, OffboardLossBoundaryIsStrictlyGreaterThan500ms) {
    ModeMachine m = machine_in(Mode::Manual);
    const TimeNs t0 = 3 * kReady;
    ASSERT_EQ(m.request_mode(Mode::Offboard, t0, t0).result, AckResult::Accepted);
    EXPECT_EQ(m.next_deadline(t0), t0 + kTimeout + 1);
    EXPECT_EQ(m.tick(t0 + 499 * kNsPerMs, t0), ModeEvent::None);
    EXPECT_EQ(m.tick(t0 + kTimeout, t0), ModeEvent::None);
    EXPECT_EQ(m.mode(), Mode::Offboard);
    EXPECT_EQ(m.tick(t0 + 501 * kNsPerMs, t0), ModeEvent::OffboardLost);
    EXPECT_EQ(m.mode(), Mode::Hold);
    EXPECT_TRUE(m.armed());
}

TEST(ModeMachine, NoAutomaticReturnFromHoldWhenSetpointsResume) {
    ModeMachine m = machine_in(Mode::Manual);
    const TimeNs t0 = 3 * kReady;
    ASSERT_EQ(m.request_mode(Mode::Offboard, t0, t0).result, AckResult::Accepted);
    ASSERT_EQ(m.tick(t0 + kNsPerMs * 600, t0), ModeEvent::OffboardLost);
    for (TimeNs t = t0 + 600 * kNsPerMs; t < t0 + 5000 * kNsPerMs; t += 50 * kNsPerMs) {
        EXPECT_EQ(m.tick(t, t), ModeEvent::None);  // fresh setpoint every tick
    }
    EXPECT_EQ(m.mode(), Mode::Hold);
}

TEST(ModeMachine, ArmAndDisarmAreIdempotent) {
    ModeMachine m = machine_in(Mode::Ready);
    EXPECT_EQ(m.request_arm(false, kReady).result, AckResult::Accepted);
    EXPECT_EQ(m.mode(), Mode::Ready);
    m = machine_in(Mode::Hold);
    EXPECT_EQ(m.request_arm(true, kReady).result, AckResult::Accepted);
    EXPECT_EQ(m.mode(), Mode::Hold);
}

TEST(ModeMachine, RebootReturnsToNotReadyDisarmed) {
    ModeMachine m = machine_in(Mode::Offboard);
    m.boot(100 * kReady);
    EXPECT_EQ(m.mode(), Mode::NotReady);
    EXPECT_FALSE(m.armed());
    EXPECT_EQ(m.tick(101 * kReady, kNoSetpoint), ModeEvent::BecameReady);
}
