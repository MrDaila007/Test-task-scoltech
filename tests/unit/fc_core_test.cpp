#include "../support/fc_harness.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using namespace fcstub;
using fcstub::test::base_config;
using fcstub::test::Harness;
using fcstub::test::Received;

constexpr TimeNs kS = kNsPerS;
constexpr TimeNs kMs = kNsPerMs;

mavlink_heartbeat_t last_heartbeat(const Harness& h) {
    const auto hbs = h.station.of(MAVLINK_MSG_ID_HEARTBEAT);
    mavlink_heartbeat_t hb{};
    if (!hbs.empty()) {
        mavlink_msg_heartbeat_decode(&hbs.back().msg, &hb);
    }
    return hb;
}

std::vector<mavlink_command_ack_t> acks(const Harness& h) {
    std::vector<mavlink_command_ack_t> out;
    for (const auto& r : h.station.of(MAVLINK_MSG_ID_COMMAND_ACK)) {
        mavlink_command_ack_t a{};
        mavlink_msg_command_ack_decode(&r.msg, &a);
        out.push_back(a);
    }
    return out;
}

bool has_text(const Harness& h, const std::string& needle) {
    const auto texts = h.station.texts();
    return std::any_of(texts.begin(), texts.end(),
                       [&](const std::string& t) { return t.find(needle) != std::string::npos; });
}

// Boots, arms and enters OFFBOARD with a 20 Hz north 2 m/s stream running until `until`.
void fly_offboard(Harness& h, TimeNs until) {
    h.run_until(3 * kS);
    h.send(h.client.arm());
    h.send(h.client.velocity(2, 0, -1));
    h.send(h.client.set_mode(kPx4MainOffboard));
    h.stream_velocity(until, 20.0, 2, 0, -1);
}

}  // namespace

TEST(FcCore, BootsNotReadyThenReportsReady) {
    Harness h(base_config());
    h.run_until(1 * kS);
    EXPECT_TRUE(has_text(h, "FC stub boot"));
    EXPECT_EQ(last_heartbeat(h).system_status, MAV_STATE_UNINIT);
    EXPECT_EQ(last_heartbeat(h).autopilot, MAV_AUTOPILOT_PX4);
    h.run_until(3 * kS);
    EXPECT_EQ(last_heartbeat(h).system_status, MAV_STATE_STANDBY);
    EXPECT_TRUE(has_text(h, "Ready to arm"));
    EXPECT_EQ(last_heartbeat(h).base_mode & MAV_MODE_FLAG_SAFETY_ARMED, 0);
}

TEST(FcCore, TelemetryRatesOverTenSeconds) {
    Harness h(base_config());
    h.run_until(10 * kS - 1);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_HEARTBEAT).size(), 10U);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_ATTITUDE).size(), 500U);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_GLOBAL_POSITION_INT).size(), 100U);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_BATTERY_STATUS).size(), 20U);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_SYS_STATUS).size(), 10U);
    EXPECT_EQ(h.station.of(MAVLINK_MSG_ID_EXTENDED_SYS_STATE).size(), 10U);
    for (const auto& r : h.station.of(MAVLINK_MSG_ID_ATTITUDE)) {
        ASSERT_EQ(r.t, r.tag.deadline);  // the sim emits exactly at the deadline
        ASSERT_EQ(r.tag.deadline % (20 * kMs), 0);
    }
}

TEST(FcCore, ArmReportsArmedManual) {
    Harness h(base_config());
    h.run_until(3 * kS);
    h.send(h.client.arm());
    ASSERT_EQ(acks(h).size(), 1U);
    EXPECT_EQ(acks(h)[0].result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(acks(h)[0].target_system, 255);
    h.run_until(5 * kS);
    EXPECT_NE(last_heartbeat(h).base_mode & MAV_MODE_FLAG_SAFETY_ARMED, 0);
    EXPECT_EQ(last_heartbeat(h).custom_mode, px4_custom_mode(kPx4MainManual, 0));
}

TEST(FcCore, OffboardWithoutStreamIsDenied) {
    Harness h(base_config());
    h.run_until(3 * kS);
    h.send(h.client.arm());
    h.send(h.client.set_mode(kPx4MainOffboard));
    ASSERT_EQ(acks(h).size(), 2U);
    EXPECT_EQ(acks(h)[1].result, MAV_RESULT_DENIED);
    EXPECT_TRUE(has_text(h, "Denied: OFFBOARD needs setpoints"));
}

TEST(FcCore, OffboardSetpointsMoveTheVehicle) {
    Harness h(base_config());
    fly_offboard(h, 10 * kS);
    EXPECT_EQ(h.core.mode(), Mode::Offboard);
    EXPECT_GT(h.core.truth().pos[0], 10.0);
    const auto pos = h.station.of(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    mavlink_global_position_int_t first{};
    mavlink_global_position_int_t last{};
    mavlink_msg_global_position_int_decode(&pos.front().msg, &first);
    mavlink_msg_global_position_int_decode(&pos.back().msg, &last);
    EXPECT_GT(last.lat, first.lat + 900);  // > 10 m north (1e-7 deg ~ 1.1 cm)
    EXPECT_NEAR(last.vx, 200, 5);
}

TEST(FcCore, SetpointLossSwitchesToHoldJustAfter500ms) {
    Harness h(base_config());
    fly_offboard(h, 10 * kS);
    const TimeNs last_setpoint = h.core.last_setpoint_time();
    h.run_until(last_setpoint + 500 * kMs);
    EXPECT_EQ(h.core.mode(), Mode::Offboard);
    h.run_until(last_setpoint + 501 * kMs);
    EXPECT_EQ(h.core.mode(), Mode::Hold);
    EXPECT_TRUE(has_text(h, "Offboard lost >500ms: HOLD"));
    h.run_until(last_setpoint + 2 * kS);
    EXPECT_EQ(last_heartbeat(h).custom_mode, px4_custom_mode(kPx4MainAuto, kPx4SubAutoLoiter));
    const auto crit = h.station.of(MAVLINK_MSG_ID_STATUSTEXT);
    EXPECT_TRUE(std::any_of(crit.begin(), crit.end(), [](const auto& r) {
        return mavlink_msg_statustext_get_severity(&r.msg) == MAV_SEVERITY_CRITICAL;
    }));
}

TEST(FcCore, TimesyncIsAnsweredWithAutopilotTime) {
    Harness h(base_config());
    h.run_until(4 * kS);
    h.send(h.client.timesync(123456789));
    const auto ts = h.station.of(MAVLINK_MSG_ID_TIMESYNC);
    ASSERT_EQ(ts.size(), 1U);
    mavlink_timesync_t t{};
    mavlink_msg_timesync_decode(&ts[0].msg, &t);
    EXPECT_EQ(t.ts1, 123456789);
    EXPECT_EQ(t.tc1, 4 * kS);
    EXPECT_EQ(t.target_system, 255);
}

TEST(FcCore, RetransmittedCommandGetsTheSameAckOnce) {
    Harness h(base_config());
    h.run_until(3 * kS);
    h.send(h.client.command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0F, 0, 0, 0));
    h.send(h.client.command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0F, 0, 0, 1));
    ASSERT_EQ(acks(h).size(), 2U);
    EXPECT_EQ(acks(h)[1].result, MAV_RESULT_ACCEPTED);
    const auto texts = h.station.texts();
    EXPECT_EQ(std::count(texts.begin(), texts.end(), "Mode: MANUAL"), 1);
}

TEST(FcCore, RejectedSetpointsAreReportedAtMostOncePerSecond) {
    Harness h(base_config());
    h.run_until(3 * kS);
    for (int i = 0; i < 40; ++i) {  // 2 s of NaN setpoints at 20 Hz
        h.send(h.client.velocity(std::nanf(""), 0, 0));
        h.run_until(h.now + 50 * kMs);
    }
    const auto texts = h.station.texts();
    const auto n = std::count(texts.begin(), texts.end(), "Setpoint rejected: nonfinite");
    EXPECT_GE(n, 2);
    EXPECT_LE(n, 3);
    EXPECT_EQ(h.core.stats().gate.rejected[static_cast<int>(RejectReason::NonFinite)], 40U);
}

TEST(FcCore, UnknownCommandIsUnsupported) {
    Harness h(base_config());
    h.run_until(3 * kS);
    h.send(h.client.command(MAV_CMD_NAV_TAKEOFF, 0));
    ASSERT_EQ(acks(h).size(), 1U);
    EXPECT_EQ(acks(h)[0].result, MAV_RESULT_UNSUPPORTED);
}

TEST(FcCore, OversizedDatagramAndTimeRegressionAreCounted) {
    Harness h(base_config());
    h.run_until(kS);
    const std::vector<std::uint8_t> big(3000, 0xFD);
    h.core.on_rx_bytes(kS, big.data(), big.size());
    EXPECT_EQ(h.core.stats().rx_oversize, 1U);
    const auto b = h.client.arm();
    h.core.on_rx_bytes(kS / 2, b.data.data(), b.len);
    EXPECT_EQ(h.core.stats().rx_time_regressions, 1U);
}

TEST(FcCore, NextDeadlineIsAlwaysInTheFuture) {
    Harness h(base_config());
    EXPECT_EQ(h.core.next_deadline(), 0);
    h.core.advance_to(0, h.station);
    for (int i = 0; i < 5000; ++i) {
        const TimeNs next = h.core.next_deadline();
        ASSERT_GT(next, h.now);
        h.now = next;
        h.core.advance_to(next, h.station);
    }
}

TEST(FcCore, RebootSilencesThenRestartsEverything) {
    Config cfg = base_config();
    FaultSpec reboot;
    reboot.type = FaultType::FcReboot;
    reboot.start_s = 10.0;
    reboot.params = FcRebootParams{3000};
    cfg.faults.push_back(reboot);
    Harness h(cfg);
    fly_offboard(h, 9 * kS);
    h.run_until(20 * kS);
    for (const auto& r : h.station.log) {
        ASSERT_FALSE(r.t >= 10 * kS && r.t < 13 * kS) << "frame during reboot at " << r.t;
    }
    std::vector<Received> after;
    for (const auto& r : h.station.log) {
        if (r.t >= 13 * kS) {
            after.push_back(r);
        }
    }
    ASSERT_FALSE(after.empty());
    EXPECT_EQ(after.front().msg.seq, 0);
    EXPECT_EQ(h.core.mode(), Mode::Ready);  // not ready for 2 s, then ready; never re-armed
    mavlink_heartbeat_t hb{};
    for (const auto& r : after) {
        if (r.msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_msg_heartbeat_decode(&r.msg, &hb);
            EXPECT_EQ(hb.system_status, MAV_STATE_BOOT);
            EXPECT_EQ(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED, 0);
            break;
        }
    }
    const auto att = h.station.of(MAVLINK_MSG_ID_ATTITUDE);
    std::uint32_t first_after_ms = 0;
    for (const auto& r : att) {
        if (r.t >= 13 * kS) {
            first_after_ms = mavlink_msg_attitude_get_time_boot_ms(&r.msg);
            break;
        }
    }
    EXPECT_LT(first_after_ms, 100U);  // the autopilot clock restarted
}

// F7: the autopilot leaves OFFBOARD on its own (pilot takeover, battery or
// geofence failsafe) while setpoints are still streaming, and refuses OFFBOARD
// until the window ends.
TEST(FcCore, ModeOverrideLeavesOffboardAndLocksItOut) {
    Config cfg = base_config();
    FaultSpec f;
    f.type = FaultType::ModeOverride;
    f.start_s = 10.0;
    f.duration_s = 5.0;
    f.params = ModeOverrideParams{OverrideTarget::Manual, OverrideCause::RcOverride};
    cfg.faults.push_back(f);
    Harness h(cfg);
    fly_offboard(h, 10 * kS - kMs);
    EXPECT_EQ(h.core.mode(), Mode::Offboard);
    h.stream_velocity(11 * kS, 20.0, 2, 0, -1);
    EXPECT_EQ(h.core.mode(), Mode::Manual);
    EXPECT_TRUE(has_text(h, "Pilot took over: MANUAL"));
    h.send(h.client.set_mode(kPx4MainOffboard));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_DENIED);
    EXPECT_TRUE(has_text(h, "Denied: pilot has control"));
    h.stream_velocity(16 * kS, 20.0, 2, 0, -1);
    h.send(h.client.set_mode(kPx4MainOffboard));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(h.core.mode(), Mode::Offboard);
}

TEST(FcCore, ModeOverrideOnTheGroundDoesNothing) {
    Config cfg = base_config();
    FaultSpec f;
    f.type = FaultType::ModeOverride;
    f.start_s = 5.0;
    f.params = ModeOverrideParams{OverrideTarget::Hold, OverrideCause::LowBattery};
    cfg.faults.push_back(f);
    Harness h(cfg);
    h.run_until(6 * kS);
    EXPECT_EQ(h.core.mode(), Mode::Ready);
    EXPECT_FALSE(has_text(h, "Low battery"));
}

// As PX4: a plain disarm is refused in the air, the forced one (21196) is not.
TEST(FcCore, DisarmInTheAirNeedsForce) {
    Harness h(base_config());
    fly_offboard(h, 8 * kS);  // climbing at 1 m/s
    h.send(h.client.arm(false));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_DENIED);
    EXPECT_TRUE(has_text(h, "Denied: in air"));
    EXPECT_TRUE(h.core.armed());
    h.send(h.client.command(MAV_CMD_COMPONENT_ARM_DISARM, 0.0F, 21196.0F));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_ACCEPTED);
    EXPECT_FALSE(h.core.armed());
}

TEST(FcCore, ExtendedSysStateReportsTheLandedState) {
    Harness h(base_config());
    h.run_until(3 * kS);
    auto landed = [&h] {
        const auto es = h.station.of(MAVLINK_MSG_ID_EXTENDED_SYS_STATE);
        return es.empty() ? -1 : mavlink_msg_extended_sys_state_get_landed_state(&es.back().msg);
    };
    EXPECT_EQ(landed(), MAV_LANDED_STATE_ON_GROUND);
    fly_offboard(h, 8 * kS);
    EXPECT_EQ(landed(), MAV_LANDED_STATE_IN_AIR);
}

TEST(FcCore, SysStatusCarriesBatteryAndHealthySensors) {
    Harness h(base_config());
    h.run_until(3 * kS);
    const auto ss = h.station.of(MAVLINK_MSG_ID_SYS_STATUS);
    ASSERT_FALSE(ss.empty());
    mavlink_sys_status_t s{};
    mavlink_msg_sys_status_decode(&ss.back().msg, &s);
    EXPECT_EQ(s.battery_remaining, 100);
    EXPECT_NEAR(s.voltage_battery, 6 * 4200 - 10, 20);
    const std::uint32_t needed =
        MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_GPS;
    EXPECT_EQ(s.onboard_control_sensors_health & needed, needed);
}

// Armed on the ground with a noisy estimate: the vehicle must stay on the
// ground, report ON_GROUND and accept a plain disarm.
TEST(FcCore, ArmedOnTheGroundWithEstimatorNoiseStaysLanded) {
    Config cfg = base_config();
    cfg.estimator = {0.5, 0.05, 0.01, 1.0};
    Harness h(cfg);
    h.run_until(3 * kS);
    h.send(h.client.arm());
    double max_alt = 0.0;
    double max_slide = 0.0;
    for (TimeNs t = 3 * kS; t < 63 * kS; t += 100 * kMs) {
        h.run_until(t);
        max_alt = std::max(max_alt, -h.core.truth().pos[2]);
        max_slide = std::max(max_slide, std::hypot(h.core.truth().pos[0], h.core.truth().pos[1]));
        const auto es = h.station.of(MAVLINK_MSG_ID_EXTENDED_SYS_STATE);
        ASSERT_EQ(mavlink_msg_extended_sys_state_get_landed_state(&es.back().msg),
                  MAV_LANDED_STATE_ON_GROUND)
            << "at " << ns_to_seconds(t) << " s, altitude " << -h.core.truth().pos[2];
    }
    EXPECT_LT(max_alt, 0.05);
    EXPECT_LT(max_slide, 0.01);
    h.send(h.client.arm(false));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_ACCEPTED);
}

// PX4 flies an over-limit velocity at the limit; the stream stays alive.
TEST(FcCore, OverLimitVelocityIsFlownAtTheLimitWithoutLosingOffboard) {
    Harness h(base_config());
    fly_offboard(h, 5 * kS);
    h.stream_velocity(15 * kS, 20.0, 15, 0, -1);
    EXPECT_EQ(h.core.mode(), Mode::Offboard);
    const auto& v = h.core.truth().vel;
    EXPECT_NEAR(std::hypot(v[0], v[1], v[2]), 12.0, 0.05);
}

// A rebooted autopilot remembers nothing: a retransmission that arrives after a
// short reboot is executed afresh, not answered from the pre-reboot cache.
TEST(FcCore, RebootForgetsTheCommandRetransmissionCache) {
    Config cfg = base_config();
    FaultSpec f;
    f.type = FaultType::FcReboot;
    f.start_s = 3.1;
    f.params = FcRebootParams{300};
    cfg.faults.push_back(f);
    Harness h(cfg);
    h.run_until(3 * kS);
    h.send(h.client.command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0F, 0, 0, 0));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_ACCEPTED);
    h.run_until(3 * kS + 600 * kMs);  // rebooted at 3.4 s, not ready again yet
    h.send(h.client.command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0F, 0, 0, 1));
    EXPECT_EQ(acks(h).back().result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_FALSE(h.core.armed());
}
