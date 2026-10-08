#include "fcstub/dynamics.hpp"
#include "fcstub/battery.hpp"
#include "fcstub/geo.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using fcstub::Dynamics;
using fcstub::MotionKind;
using fcstub::MotionTarget;

constexpr double kDt = 0.004;
constexpr double kTau = 0.3;

Dynamics make_dynamics() { return Dynamics({kTau, 12.0, 1.0}); }

MotionTarget velocity(double n, double e, double d) {
    MotionTarget t;
    t.kind = MotionKind::Velocity;
    t.vel = {n, e, d};
    return t;
}

MotionTarget position(double n, double e, double d) {
    MotionTarget t;
    t.kind = MotionKind::Position;
    t.pos = {n, e, d};
    return t;
}

void run(Dynamics& dyn, const MotionTarget& target, double seconds) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
        dyn.step(kDt, target);
    }
}

}  // namespace

TEST(Dynamics, VelocityStepSettlesWithinOnePercentAfterFiveTau) {
    Dynamics dyn = make_dynamics();
    run(dyn, velocity(0, 0, -1), 2.0);  // climb first so the ground clamp is not involved
    run(dyn, velocity(2.0, 0, 0), 5 * kTau);
    EXPECT_NEAR(dyn.state().vel[0], 2.0, 0.02);
}

TEST(Dynamics, PositionTargetIsReached) {
    Dynamics dyn = make_dynamics();
    run(dyn, position(10.0, 0.0, -5.0), 20.0);
    EXPECT_NEAR(dyn.state().pos[0], 10.0, 0.1);
    EXPECT_NEAR(dyn.state().pos[1], 0.0, 0.1);
    EXPECT_NEAR(dyn.state().pos[2], -5.0, 0.1);
}

TEST(Dynamics, SpeedIsLimited) {
    Dynamics dyn = make_dynamics();
    run(dyn, position(900.0, 0.0, -5.0), 10.0);
    EXPECT_LE(std::hypot(dyn.state().vel[0], dyn.state().vel[1], dyn.state().vel[2]), 12.0 + 1e-9);
}

TEST(Dynamics, AcceleratingNorthPitchesNoseDown) {
    Dynamics dyn = make_dynamics();
    run(dyn, velocity(0, 0, -1), 2.0);
    dyn.step(kDt, velocity(5.0, 0, -1));
    EXPECT_LT(dyn.state().pitch, 0.0);
    EXPECT_NEAR(dyn.state().roll, 0.0, 1e-9);
    // Accelerating east (yaw 0) rolls right.
    Dynamics dyn2 = make_dynamics();
    run(dyn2, velocity(0, 0, -1), 2.0);
    dyn2.step(kDt, velocity(0, 5.0, -1));
    EXPECT_GT(dyn2.state().roll, 0.0);
}

TEST(Dynamics, StopHoldsPositionAndDisarmedFreezes) {
    Dynamics dyn = make_dynamics();
    run(dyn, velocity(3.0, 0, -1), 3.0);
    MotionTarget stop;
    stop.kind = MotionKind::Stop;
    run(dyn, stop, 3.0);
    EXPECT_NEAR(dyn.state().vel[0], 0.0, 1e-3);
    const double n = dyn.state().pos[0];
    run(dyn, stop, 3.0);
    EXPECT_NEAR(dyn.state().pos[0], n, 1e-3);

    MotionTarget off;
    off.kind = MotionKind::Disarmed;
    Dynamics ground = make_dynamics();
    run(ground, off, 5.0);
    EXPECT_DOUBLE_EQ(ground.state().pos[0], 0.0);
    EXPECT_DOUBLE_EQ(ground.state().vel[2], 0.0);
}

TEST(Dynamics, CannotGoBelowGround) {
    Dynamics dyn = make_dynamics();
    run(dyn, velocity(0, 0, 2.0), 3.0);  // commanded down from the ground
    EXPECT_LE(dyn.state().pos[2], 0.0);
}

TEST(Dynamics, YawFollowsYawTarget) {
    Dynamics dyn = make_dynamics();
    MotionTarget t = velocity(0, 0, -1);
    t.yaw_valid = true;
    t.yaw = 1.0;
    run(dyn, t, 5.0);
    EXPECT_NEAR(dyn.state().yaw, 1.0, 0.01);
}

TEST(Geo, OneKilometreNorth) {
    const fcstub::GeoOrigin origin{55.7558, 37.6173, 150.0};
    const auto p = fcstub::ned_to_wgs84(origin, 1000.0, 0.0, -20.0);
    EXPECT_NEAR(p.lat_deg - origin.lat_deg, 0.0089932, 1e-6);
    EXPECT_NEAR(p.lon_deg, origin.lon_deg, 1e-12);
    EXPECT_DOUBLE_EQ(p.alt_msl_m, 170.0);
}

TEST(Geo, EastIsScaledByLatitude) {
    const fcstub::GeoOrigin origin{60.0, 30.0, 0.0};
    const auto p = fcstub::ned_to_wgs84(origin, 0.0, 1000.0, 0.0);
    EXPECT_NEAR(p.lon_deg - 30.0, 2 * 0.0089932, 1e-6);  // cos 60 = 0.5
}

TEST(Battery, DrainsByCoulombCounting) {
    fcstub::Battery bat({6, 10000.0, 0.02, 0.5, 18.0, 0.05, 1.0});
    for (int i = 0; i < 250 * 600; ++i) {  // 600 s armed, hovering
        bat.step(kDt, true, 0.0);
    }
    const double expected_drop = 18.0 * 600.0 / (10000.0 * 3.6);
    EXPECT_NEAR(1.0 - bat.soc(), expected_drop, expected_drop * 1e-3);
    EXPECT_NEAR(bat.consumed_mah(), 18.0 * 600.0 / 3.6, 1.0);
    EXPECT_DOUBLE_EQ(bat.current_a(), 18.0);
}

TEST(Battery, VoltageSagsUnderLoadAndDropsWithCharge) {
    fcstub::Battery bat({6, 10000.0, 0.02, 0.5, 18.0, 0.05, 1.0});
    bat.step(kDt, false, 0.0);
    const double idle = bat.voltage_v();
    bat.step(kDt, true, 0.0);
    EXPECT_NEAR(idle - bat.voltage_v(), (18.0 - 0.5) * 0.02, 1e-3);
    EXPECT_NEAR(idle, 6 * 4.2 - 0.5 * 0.02, 1e-3);
    double prev = bat.soc();
    for (int i = 0; i < 1000; ++i) {
        bat.step(kDt, true, 5.0);
        ASSERT_LE(bat.soc(), prev);
        prev = bat.soc();
    }
    EXPECT_DOUBLE_EQ(bat.current_a(), 18.0 + 0.05 * 25.0);
}

TEST(Battery, NeverBelowEmpty) {
    fcstub::Battery bat({6, 100.0, 0.02, 0.5, 200.0, 0.0, 0.01});
    for (int i = 0; i < 250 * 60; ++i) {
        bat.step(kDt, true, 0.0);
    }
    EXPECT_DOUBLE_EQ(bat.soc(), 0.0);
}

// The autopilot closes its loops on its own estimate, not on the truth: a biased
// estimate moves the real vehicle, and the estimate itself follows the setpoint.
TEST(Dynamics, VelocityLoopClosesOnTheEstimate) {
    Dynamics dyn = make_dynamics();
    run(dyn, velocity(0, 0, -1), 2.0);
    const int steps = static_cast<int>(std::lround(5.0 / kDt));
    for (int i = 0; i < steps; ++i) {
        fcstub::VehicleState est = dyn.state();
        est.vel[0] += 0.2;  // estimate reads 0.2 m/s too fast northwards
        dyn.step(kDt, velocity(2.0, 0, -1), est);
    }
    EXPECT_NEAR(dyn.state().vel[0], 1.8, 0.01);
}

TEST(Dynamics, PositionLoopClosesOnTheEstimate) {
    Dynamics dyn = make_dynamics();
    const int steps = static_cast<int>(std::lround(20.0 / kDt));
    for (int i = 0; i < steps; ++i) {
        fcstub::VehicleState est = dyn.state();
        est.pos[0] += 5.0;  // estimate is 5 m north of the truth
        dyn.step(kDt, position(10.0, 0.0, -5.0), est);
    }
    EXPECT_NEAR(dyn.state().pos[0], 5.0, 0.1);
}
