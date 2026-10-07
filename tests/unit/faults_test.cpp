#include "fcstub/estimator_tap.hpp"
#include "fcstub/fault_schedule.hpp"
#include "fcstub/fc_clock.hpp"
#include "fcstub/lifecycle.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

using namespace fcstub;

FaultSpec fault(FaultType type, double start_s, double duration_s, FaultParams params) {
    FaultSpec f;
    f.type = type;
    f.start_s = start_s;
    f.duration_s = duration_s;
    f.params = params;
    return f;
}

VehicleState moving_truth(TimeNs now) {
    VehicleState s;
    const double t = ns_to_seconds(now);
    s.pos = {2.0 * t, 0.0, -10.0};
    s.vel = {2.0, 0.0, 0.0};
    s.pitch = -0.1;
    return s;
}

constexpr TimeNs kS = kNsPerS;

}  // namespace

// --- schedule ----------------------------------------------------------------------

TEST(FaultSchedule, ActiveWindowsAndBoundaries) {
    const FaultSchedule sched({fault(FaultType::Link, 10, 5, LinkFaultParams{}),
                               fault(FaultType::Gnss, 12, 0, GnssFaultParams{}),
                               fault(FaultType::FcReboot, 30, 0, FcRebootParams{2000})},
                              7);
    EXPECT_EQ(sched.active(FaultType::Link, 10 * kS - 1), nullptr);
    ASSERT_NE(sched.active(FaultType::Link, 10 * kS), nullptr);
    EXPECT_EQ(sched.active(FaultType::Link, 15 * kS), nullptr);  // half-open window
    EXPECT_NE(sched.active(FaultType::Gnss, 1000 * kS), nullptr);  // duration 0 = until the end
    EXPECT_NE(sched.active(FaultType::FcReboot, 31 * kS), nullptr);
    EXPECT_EQ(sched.active(FaultType::FcReboot, 32 * kS), nullptr);  // boot_ms window
    EXPECT_EQ(sched.next_boundary(0), 10 * kS);
    EXPECT_EQ(sched.next_boundary(10 * kS), 12 * kS);
    EXPECT_EQ(sched.next_boundary(12 * kS), 15 * kS);
    EXPECT_EQ(sched.next_boundary(15 * kS), 30 * kS);
    EXPECT_EQ(sched.next_boundary(30 * kS), 32 * kS);
    EXPECT_EQ(sched.next_boundary(32 * kS), FaultSchedule::kNever);
}

TEST(FaultSchedule, EachFaultHasItsOwnRandomStream) {
    FaultSchedule a({fault(FaultType::Link, 1, 1, LinkFaultParams{}),
                     fault(FaultType::Gnss, 1, 1, GnssFaultParams{})},
                    42);
    FaultSchedule b({fault(FaultType::Link, 1, 1, LinkFaultParams{})}, 42);
    EXPECT_EQ(a.rng(0).next_u64(), b.rng(0).next_u64());  // adding a fault changes nothing
    EXPECT_NE(a.rng(0).next_u64(), a.rng(1).next_u64());
}

// --- F1 ------------------------------------------------------------------------------

TEST(EstimatorFreeze, PositionAndAttitudeFreezeWhileVelocityStaysLive) {
    const FaultSchedule sched(
        {fault(FaultType::EstimatorFreeze, 10, 5, EstimatorFreezeParams{false})}, 1);
    EstimatorTap tap(sched);
    EXPECT_DOUBLE_EQ(tap.update(moving_truth(9 * kS), 9 * kS).pos[0], 18.0);
    EXPECT_DOUBLE_EQ(tap.update(moving_truth(10 * kS), 10 * kS).pos[0], 20.0);  // snapshot
    auto rep = tap.update(moving_truth(14 * kS), 14 * kS);
    EXPECT_DOUBLE_EQ(rep.pos[0], 20.0);
    EXPECT_DOUBLE_EQ(rep.pitch, -0.1);
    EXPECT_DOUBLE_EQ(rep.vel[0], 2.0);  // velocity still from the live estimate
    rep = tap.update(moving_truth(15 * kS), 15 * kS);
    EXPECT_DOUBLE_EQ(rep.pos[0], 30.0);  // window over: jumps back to truth
}

TEST(EstimatorFreeze, FreezeVelocityFreezesEverything) {
    const FaultSchedule sched(
        {fault(FaultType::EstimatorFreeze, 10, 5, EstimatorFreezeParams{true})}, 1);
    EstimatorTap tap(sched);
    (void)tap.update(moving_truth(10 * kS), 10 * kS);
    VehicleState accelerating = moving_truth(12 * kS);
    accelerating.vel = {7.0, 1.0, 0.0};
    const auto rep = tap.update(accelerating, 12 * kS);
    EXPECT_DOUBLE_EQ(rep.vel[0], 2.0);
    EXPECT_DOUBLE_EQ(rep.vel[1], 0.0);
    EXPECT_DOUBLE_EQ(rep.pos[0], 20.0);
}

// --- F5 ------------------------------------------------------------------------------

TEST(GnssFault, JumpAppliesOnlyInsideTheWindow) {
    GnssFaultParams p;
    p.kind = GnssKind::Jump;
    p.offset_m = {15.0, -5.0, 0.0};
    const FaultSchedule sched({fault(FaultType::Gnss, 10, 5, p)}, 1);
    EstimatorTap tap(sched);
    EXPECT_DOUBLE_EQ(tap.update(moving_truth(9 * kS), 9 * kS).pos[0], 18.0);
    const auto in = tap.update(moving_truth(10 * kS), 10 * kS);
    EXPECT_DOUBLE_EQ(in.pos[0], 35.0);
    EXPECT_DOUBLE_EQ(in.pos[1], -5.0);
    EXPECT_DOUBLE_EQ(in.vel[0], 2.0);  // a jump does not show up in velocity
    EXPECT_DOUBLE_EQ(tap.update(moving_truth(15 * kS), 15 * kS).pos[0], 30.0);
}

TEST(GnssFault, DriftIsConsistentInPositionAndVelocity) {
    GnssFaultParams p;
    p.kind = GnssKind::Drift;
    p.drift_mps = {0.2, 0.0, 0.0};
    const FaultSchedule sched({fault(FaultType::Gnss, 20, 30, p)}, 1);
    EstimatorTap tap(sched);
    const auto rep = tap.update(moving_truth(50 * kS - 1), 50 * kS - 1);
    EXPECT_NEAR(rep.pos[0] - moving_truth(50 * kS - 1).pos[0], 6.0, 0.01);
    EXPECT_DOUBLE_EQ(rep.vel[0], 2.2);
    EXPECT_DOUBLE_EQ(tap.update(moving_truth(50 * kS), 50 * kS).pos[0], 100.0);
}

// --- F3 ------------------------------------------------------------------------------

TEST(ClockFault, DriftAccumulatesInsideWindowAndPersistsAfter) {
    ClockFaultParams p;
    p.drift_ppm = 100.0;
    const FaultSchedule sched({fault(FaultType::ClockFault, 10, 10, p)}, 1);
    FcClock clock(sched);
    clock.boot(0);
    EXPECT_EQ(clock.fc_ns(10 * kS), 10 * kS);
    EXPECT_EQ(clock.fc_ns(20 * kS), 20 * kS + kNsPerMs);  // 100 ppm * 10 s = 1 ms
    EXPECT_EQ(clock.fc_ns(30 * kS), 30 * kS + kNsPerMs);  // no further drift, offset stays
}

TEST(ClockFault, StepHappensAtStart) {
    ClockFaultParams p;
    p.step_ms = 50.0;
    const FaultSchedule sched({fault(FaultType::ClockFault, 10, 1, p)}, 1);
    FcClock clock(sched);
    clock.boot(0);
    EXPECT_EQ(clock.fc_ns(10 * kS - 1), 10 * kS - 1);
    EXPECT_EQ(clock.fc_ns(10 * kS), 10 * kS + 50 * kNsPerMs);
}

TEST(ClockFault, RebootRestartsTheClockFromZero) {
    const FaultSchedule sched({}, 1);
    FcClock clock(sched);
    clock.boot(0);
    EXPECT_EQ(clock.fc_ns(5 * kS), 5 * kS);
    clock.boot(7 * kS);
    EXPECT_EQ(clock.fc_ns(8 * kS), kS);
}

// --- F2 ------------------------------------------------------------------------------

TEST(Lifecycle, RebootWindowIsSilentAndEndsWithOneRebootedEvent) {
    const FaultSchedule sched({fault(FaultType::FcReboot, 10, 0, FcRebootParams{3000})}, 1);
    Lifecycle life(sched);
    EXPECT_EQ(life.update(9 * kS), LifeEvent::None);
    EXPECT_FALSE(life.silent());
    EXPECT_EQ(life.update(10 * kS), LifeEvent::RebootStarted);
    EXPECT_TRUE(life.silent());
    EXPECT_EQ(life.update(12 * kS), LifeEvent::None);
    EXPECT_TRUE(life.silent());
    EXPECT_EQ(life.update(13 * kS), LifeEvent::Rebooted);
    EXPECT_FALSE(life.silent());
    EXPECT_EQ(life.update(20 * kS), LifeEvent::None);
}

TEST(Lifecycle, LateUpdateStillDeliversBothEventsInOrder) {
    const FaultSchedule sched({fault(FaultType::FcReboot, 10, 0, FcRebootParams{3000})}, 1);
    Lifecycle life(sched);
    EXPECT_EQ(life.update(100 * kS), LifeEvent::RebootStarted);
    EXPECT_EQ(life.update(100 * kS), LifeEvent::Rebooted);
    EXPECT_EQ(life.update(100 * kS), LifeEvent::None);
}
