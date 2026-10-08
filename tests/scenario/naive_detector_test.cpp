// The reference detector itself, on hand-made journals: the tracking check must
// fire when the reported velocity does not follow the commanded one and stay
// silent when it does.

#include "../support/naive_detector.hpp"

#include <common/mavlink.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

using fcstub::test::Alarm;
using fcstub::test::JournalRecord;

constexpr std::uint32_t kOffboardMode = 6U << 16U;
constexpr std::uint16_t kVelocityMask = 3527;

JournalRecord record(double t_s, std::uint8_t dir, const mavlink_message_t& m) {
    JournalRecord r{static_cast<std::int64_t>(t_s * 1e9), dir, {}};
    r.bytes.resize(MAVLINK_MAX_PACKET_LEN);
    r.bytes.resize(mavlink_msg_to_send_buffer(r.bytes.data(), &m));
    return r;
}

// 20 s of OFFBOARD flight north at `cmd_vn` while the autopilot reports `rep_vn`.
std::vector<JournalRecord> flight(double cmd_vn, double rep_vn) {
    std::vector<JournalRecord> j;
    // Separate channels keep the downlink sequence free of gaps.
    mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq = 0;
    auto down = [&](double t, const mavlink_message_t& m) { j.push_back(record(t, 0, m)); };
    mavlink_message_t m{};
    for (int i = 0; i <= 200; ++i) {  // 10 Hz
        const double t = i * 0.1;
        if (i % 10 == 0) {
            mavlink_msg_heartbeat_pack_chan(
                1, 1, MAVLINK_COMM_0, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED, kOffboardMode,
                MAV_STATE_ACTIVE);
            down(t, m);
        }
        mavlink_msg_set_position_target_local_ned_pack_chan(
            255, 190, MAVLINK_COMM_1, &m, static_cast<std::uint32_t>(t * 1000), 1, 1,
            MAV_FRAME_LOCAL_NED, kVelocityMask, 0, 0, 0, static_cast<float>(cmd_vn), 0, 0, 0, 0, 0,
            0, 0);
        j.push_back(record(t, 1, m));
        const auto lat = static_cast<std::int32_t>(557558000 + rep_vn * t / 111195.0 * 1e7);
        mavlink_msg_global_position_int_pack_chan(
            1, 1, MAVLINK_COMM_0, &m, static_cast<std::uint32_t>(t * 1000), lat, 376173000, 150000,
            10000, static_cast<std::int16_t>(rep_vn * 100), 0, 0, 0);
        down(t, m);
    }
    return j;
}

bool has(const std::vector<fcstub::test::AlarmEvent>& alarms, Alarm kind) {
    for (const auto& a : alarms) {
        if (a.kind == kind) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(NaiveDetector, TrackingErrorFiresWhenTheVehicleDoesNotFollow) {
    const auto alarms = fcstub::test::detect(flight(2.0, 1.5));
    EXPECT_TRUE(has(alarms, Alarm::TrackingError));
}

TEST(NaiveDetector, TrackingIsSilentWhenTheVehicleFollows) {
    const auto alarms = fcstub::test::detect(flight(2.0, 2.0));
    EXPECT_TRUE(alarms.empty());
}
