#include "../support/fc_harness.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using namespace fcstub;
using fcstub::test::Harness;
using fcstub::test::Received;
using fcstub::test::base_config;

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
