// Smoke test: the pinned MAVLink headers compile under the project's warning
// policy and a PX4-style HEARTBEAT survives an encode/parse round trip.
#include <gtest/gtest.h>

#include <common/mavlink.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kOffboardCustomMode = 6U << 16U;

}  // namespace

TEST(Smoke, MavlinkHeartbeatRoundTrip) {
    mavlink_message_t out{};
    mavlink_msg_heartbeat_pack(1, 1, &out, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, kOffboardCustomMode,
                               MAV_STATE_STANDBY);

    std::uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {};
    const std::uint16_t len = mavlink_msg_to_send_buffer(buf, &out);
    ASSERT_EQ(len, 21U);

    mavlink_message_t in{};
    mavlink_status_t status{};
    std::uint8_t parsed = 0;
    for (std::uint16_t i = 0; i < len; ++i) {
        parsed = mavlink_parse_char(MAVLINK_COMM_0, buf[i], &in, &status);
    }
    ASSERT_EQ(parsed, MAVLINK_FRAMING_OK);
    EXPECT_EQ(in.msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
    EXPECT_EQ(mavlink_msg_heartbeat_get_custom_mode(&in), kOffboardCustomMode);
}
