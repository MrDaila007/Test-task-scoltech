#include "fcstub/setpoint_gate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using fcstub::kNsPerMs;
using fcstub::RejectReason;
using fcstub::SetpointGate;
using fcstub::SetpointKind;
using fcstub::SetpointMsg;

constexpr std::uint16_t kVelocityOnly = 3527;
constexpr std::uint16_t kPositionOnly = 3576;
constexpr std::uint16_t kPositionVelocity = 3520;

SetpointGate make_gate() { return SetpointGate({1, 1, 12.0, 1000.0, 500.0}); }

SetpointMsg msg(std::uint16_t mask) {
    SetpointMsg m;
    m.target_system = 1;
    m.target_component = 1;
    m.coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
    m.type_mask = mask;
    m.x = 10.0F;
    m.y = -5.0F;
    m.z = -20.0F;
    m.vx = 2.0F;
    m.vy = 0.0F;
    m.vz = -1.0F;
    return m;
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

}  // namespace

TEST(SetpointGate, AcceptsTheThreeSupportedMasks) {
    SetpointGate g = make_gate();
    auto r = g.submit(0, msg(kVelocityOnly));
    EXPECT_EQ(r.reason, RejectReason::None);
    EXPECT_EQ(g.last().kind, SetpointKind::Velocity);
    EXPECT_DOUBLE_EQ(g.last().vel[0], 2.0);

    r = g.submit(0, msg(kPositionOnly));
    EXPECT_EQ(r.reason, RejectReason::None);
    EXPECT_EQ(g.last().kind, SetpointKind::Position);
    EXPECT_DOUBLE_EQ(g.last().pos[2], -20.0);

    r = g.submit(0, msg(kPositionVelocity));
    EXPECT_EQ(r.reason, RejectReason::None);
    EXPECT_EQ(g.last().kind, SetpointKind::PositionVelocity);
}

TEST(SetpointGate, YawBitsAreOptional) {
    SetpointGate g = make_gate();
    auto m = msg(kVelocityOnly & ~1024U);  // yaw used
    m.yaw = 1.5F;
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::None);
    EXPECT_TRUE(g.last().yaw_valid);
    EXPECT_FLOAT_EQ(g.last().yaw, 1.5F);
    EXPECT_FALSE(g.last().yaw_rate_valid);
}

TEST(SetpointGate, RejectsUnsupportedMasks) {
    SetpointGate g = make_gate();
    EXPECT_EQ(g.submit(0, msg(0)).reason, RejectReason::TypeMask);  // acceleration used
    EXPECT_EQ(g.submit(0, msg(kVelocityOnly | 512U)).reason, RejectReason::TypeMask);  // force
    EXPECT_EQ(g.submit(0, msg(kVelocityOnly | 8U)).reason, RejectReason::TypeMask);    // half vel
    EXPECT_EQ(g.stats().rejected[static_cast<int>(RejectReason::TypeMask)], 3U);
}

TEST(SetpointGate, RejectsOtherFrames) {
    SetpointGate g = make_gate();
    auto m = msg(kVelocityOnly);
    m.coordinate_frame = 8;  // MAV_FRAME_BODY_NED
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::Frame);
}

TEST(SetpointGate, NonFiniteOnlyMattersInUsedFields) {
    SetpointGate g = make_gate();
    auto m = msg(kVelocityOnly);
    m.vx = kNaN;
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::NonFinite);
    m = msg(kVelocityOnly);
    m.x = kNaN;    // position is ignored by this mask
    m.yaw = kNaN;  // yaw ignored too
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::None);
    m = msg(kVelocityOnly & ~1024U);
    m.yaw = std::numeric_limits<float>::infinity();
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::NonFinite);
}

TEST(SetpointGate, RangeChecks) {
    SetpointGate g = make_gate();
    auto m = msg(kVelocityOnly);
    m.vx = 10.0F;
    m.vy = 10.0F;  // |v| = 14.1 > 12
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::Range);
    m = msg(kPositionOnly);
    m.x = 900.0F;
    m.y = 500.0F;  // 1030 m > 1000 m
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::Range);
    m = msg(kPositionOnly);
    m.z = 1.0F;  // below ground (NED down positive)
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::Range);
    m = msg(kPositionOnly);
    m.z = -501.0F;
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::Range);
}

TEST(SetpointGate, MessagesForOtherSystemsAreIgnoredSilently) {
    SetpointGate g = make_gate();
    auto m = msg(kVelocityOnly);
    m.target_system = 7;
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::NotForUs);
    m = msg(kVelocityOnly);
    m.target_system = 0;  // broadcast
    m.target_component = 0;
    EXPECT_EQ(g.submit(0, m).reason, RejectReason::None);
}

TEST(SetpointGate, OnlyAcceptedSetpointsRefreshTheTimestamp) {
    SetpointGate g = make_gate();
    EXPECT_EQ(g.last_valid_time(), SetpointGate::kNoSetpoint);
    ASSERT_EQ(g.submit(100 * kNsPerMs, msg(kVelocityOnly)).reason, RejectReason::None);
    EXPECT_EQ(g.last_valid_time(), 100 * kNsPerMs);
    auto bad = msg(kVelocityOnly);
    bad.vx = kNaN;
    ASSERT_EQ(g.submit(400 * kNsPerMs, bad).reason, RejectReason::NonFinite);
    EXPECT_EQ(g.last_valid_time(), 100 * kNsPerMs);
    EXPECT_DOUBLE_EQ(g.last().vel[0], 2.0);  // last good setpoint kept
}
