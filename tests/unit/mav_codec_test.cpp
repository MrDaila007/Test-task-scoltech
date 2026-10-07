#include "fcstub/mav_codec.hpp"
#include "fcstub/rng.hpp"

#include <common/mavlink.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace fcstub;

// Parses a frame produced by the encoder with an independent, test-local parser.
mavlink_message_t parse_one(const FrameBuf& frame) {
    mavlink_message_t rx{};
    mavlink_status_t st{};
    mavlink_message_t out{};
    mavlink_status_t out_st{};
    std::uint8_t result = MAVLINK_FRAMING_INCOMPLETE;
    for (std::uint16_t i = 0; i < frame.len; ++i) {
        result = mavlink_frame_char_buffer(&rx, &st, frame.data[i], &out, &out_st);
    }
    EXPECT_EQ(result, MAVLINK_FRAMING_OK);
    return out;
}

// Builds a raw client frame (as an external client would send it).
std::vector<std::uint8_t> client_setpoint(std::uint8_t seq, std::uint16_t type_mask, float vx) {
    mavlink_status_t st{};
    st.current_tx_seq = seq;
    mavlink_message_t msg{};
    mavlink_msg_set_position_target_local_ned_pack_status(
        255, 190, &st, &msg, 1234, 1, 1, MAV_FRAME_LOCAL_NED, type_mask, 1.0F, 2.0F, -3.0F, vx,
        0.5F, -0.25F, 0.0F, 0.0F, 0.0F, 0.1F, 0.2F);
    std::vector<std::uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    bytes.resize(mavlink_msg_to_send_buffer(bytes.data(), &msg));
    return bytes;
}

struct Collector : RxHandler {
    std::vector<RxMessage> messages;
    void on_message(const RxMessage& msg) noexcept override { messages.push_back(msg); }
};

}  // namespace

TEST(MavCodec, Px4CustomModeValues) {
    EXPECT_EQ(px4_custom_mode(kPx4MainPosctl, 0), 196608U);
    EXPECT_EQ(px4_custom_mode(kPx4MainManual, 0), 65536U);
    EXPECT_EQ(px4_custom_mode(kPx4MainOffboard, 0), 393216U);
    EXPECT_EQ(px4_custom_mode(kPx4MainAuto, kPx4SubAutoLoiter), 50593792U);
}

TEST(MavCodec, HeartbeatRoundTrip) {
    MavEncoder enc(1, 1);
    const FrameBuf f = enc.heartbeat({MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                      px4_custom_mode(kPx4MainOffboard, 0), MAV_STATE_STANDBY});
    EXPECT_EQ(f.len, 21U);
    const mavlink_message_t m = parse_one(f);
    EXPECT_EQ(m.msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
    EXPECT_EQ(m.sysid, 1);
    EXPECT_EQ(m.compid, 1);
    EXPECT_EQ(mavlink_msg_heartbeat_get_type(&m), MAV_TYPE_QUADROTOR);
    EXPECT_EQ(mavlink_msg_heartbeat_get_autopilot(&m), MAV_AUTOPILOT_PX4);
    EXPECT_EQ(mavlink_msg_heartbeat_get_custom_mode(&m), 393216U);
    EXPECT_EQ(mavlink_msg_heartbeat_get_system_status(&m), MAV_STATE_STANDBY);
}

TEST(MavCodec, AttitudeRoundTrip) {
    MavEncoder enc(1, 1);
    const mavlink_message_t m =
        parse_one(enc.attitude({4242, 0.1F, -0.2F, 3.0F, 0.01F, 0.02F, 0.03F}));
    mavlink_attitude_t a{};
    mavlink_msg_attitude_decode(&m, &a);
    EXPECT_EQ(a.time_boot_ms, 4242U);
    EXPECT_FLOAT_EQ(a.roll, 0.1F);
    EXPECT_FLOAT_EQ(a.pitch, -0.2F);
    EXPECT_FLOAT_EQ(a.yaw, 3.0F);
    EXPECT_FLOAT_EQ(a.yawspeed, 0.03F);
}

TEST(MavCodec, GlobalPositionUnitsAndSaturation) {
    MavEncoder enc(1, 1);
    GlobalPositionData d{};
    d.time_boot_ms = 100;
    d.lat_deg = 55.75580004;
    d.lon_deg = -37.61730006;
    d.alt_msl_m = 160.1234;
    d.relative_alt_m = 10.1234;
    d.vn_mps = 1.234;
    d.ve_mps = 400.0;  // beyond int16 cm/s
    d.vd_mps = -0.5;
    d.yaw_rad = -0.5 * M_PI;  // west -> 270 deg
    mavlink_global_position_int_t p{};
    const mavlink_message_t m = parse_one(enc.global_position_int(d));
    mavlink_msg_global_position_int_decode(&m, &p);
    EXPECT_EQ(p.lat, 557558000);
    EXPECT_EQ(p.lon, -376173001);
    EXPECT_EQ(p.alt, 160123);
    EXPECT_EQ(p.relative_alt, 10123);
    EXPECT_EQ(p.vx, 123);
    EXPECT_EQ(p.vy, std::numeric_limits<std::int16_t>::max());
    EXPECT_EQ(p.vz, -50);
    EXPECT_EQ(p.hdg, 27000);
}

TEST(MavCodec, BatteryStatusFields) {
    MavEncoder enc(1, 1);
    const mavlink_message_t m = parse_one(enc.battery_status({6, 24.6, 18.25, 1234.4, 0.255}));
    mavlink_battery_status_t b{};
    mavlink_msg_battery_status_decode(&m, &b);
    EXPECT_EQ(b.battery_function, MAV_BATTERY_FUNCTION_ALL);
    EXPECT_EQ(b.type, MAV_BATTERY_TYPE_LION);
    EXPECT_EQ(b.temperature, std::numeric_limits<std::int16_t>::max());
    for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(b.voltages[i], 4100);
    }
    for (int i = 6; i < 10; ++i) {
        EXPECT_EQ(b.voltages[i], std::numeric_limits<std::uint16_t>::max());
    }
    EXPECT_EQ(b.current_battery, 1825);
    EXPECT_EQ(b.current_consumed, 1234);
    EXPECT_EQ(b.energy_consumed, -1);
    EXPECT_EQ(b.battery_remaining, 26);
    EXPECT_EQ(b.charge_state, MAV_BATTERY_CHARGE_STATE_LOW);
}

TEST(MavCodec, StatusTextIsTruncatedTo50) {
    MavEncoder enc(1, 1);
    const std::string long_text(60, 'x');
    const mavlink_message_t m = parse_one(enc.statustext({MAV_SEVERITY_WARNING, long_text}));
    char text[51] = {};
    mavlink_msg_statustext_get_text(&m, text);
    EXPECT_EQ(std::string(text), std::string(50, 'x'));
    EXPECT_EQ(mavlink_msg_statustext_get_severity(&m), MAV_SEVERITY_WARNING);
}

TEST(MavCodec, CommandAckAndTimesyncCarryTargets) {
    MavEncoder enc(1, 1);
    mavlink_command_ack_t ack{};
    const mavlink_message_t ack_msg = parse_one(enc.command_ack({400, MAV_RESULT_DENIED, 255, 190}));
    mavlink_msg_command_ack_decode(&ack_msg, &ack);
    EXPECT_EQ(ack.command, 400);
    EXPECT_EQ(ack.result, MAV_RESULT_DENIED);
    EXPECT_EQ(ack.target_system, 255);
    EXPECT_EQ(ack.target_component, 190);

    mavlink_timesync_t ts{};
    const mavlink_message_t ts_msg = parse_one(enc.timesync({111, 222, 255, 190}));
    mavlink_msg_timesync_decode(&ts_msg, &ts);
    EXPECT_EQ(ts.tc1, 111);
    EXPECT_EQ(ts.ts1, 222);
    EXPECT_EQ(ts.target_system, 255);
}

TEST(MavCodec, SequenceIncrementsAndResets) {
    MavEncoder enc(1, 1);
    const HeartbeatData hb{MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY};
    EXPECT_EQ(parse_one(enc.heartbeat(hb)).seq, 0);
    EXPECT_EQ(parse_one(enc.heartbeat(hb)).seq, 1);
    EXPECT_EQ(parse_one(enc.heartbeat(hb)).seq, 2);
    enc.reset_sequence();
    EXPECT_EQ(parse_one(enc.heartbeat(hb)).seq, 0);
}

TEST(MavCodec, EncodersAreIndependent) {
    // No shared global channel state: a second encoder starts from seq 0 again.
    MavEncoder a(1, 1);
    const HeartbeatData hb{MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY};
    (void)a.heartbeat(hb);
    (void)a.heartbeat(hb);
    MavEncoder b(1, 1);
    EXPECT_EQ(parse_one(b.heartbeat(hb)).seq, 0);
}

TEST(MavCodec, DecodesSetpoint) {
    MavDecoder dec;
    Collector out;
    const auto bytes = client_setpoint(7, 3527, 2.5F);
    dec.feed(bytes.data(), bytes.size(), out);
    ASSERT_EQ(out.messages.size(), 1U);
    const auto& sp = std::get<SetpointMsg>(out.messages[0]);
    EXPECT_EQ(sp.source_system, 255);
    EXPECT_EQ(sp.source_component, 190);
    EXPECT_EQ(sp.seq, 7);
    EXPECT_EQ(sp.target_system, 1);
    EXPECT_EQ(sp.coordinate_frame, MAV_FRAME_LOCAL_NED);
    EXPECT_EQ(sp.type_mask, 3527);
    EXPECT_FLOAT_EQ(sp.vx, 2.5F);
    EXPECT_FLOAT_EQ(sp.z, -3.0F);
    EXPECT_FLOAT_EQ(sp.yaw_rate, 0.2F);
    EXPECT_EQ(dec.stats().frames_ok, 1U);
}

TEST(MavCodec, DecodesCommandLongAndTimesyncAndIgnoresOthers) {
    mavlink_status_t st{};
    mavlink_message_t msg{};
    std::vector<std::uint8_t> stream;
    auto append = [&stream, &msg] {
        std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const std::uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        stream.insert(stream.end(), buf, buf + n);
    };
    mavlink_msg_command_long_pack_status(255, 190, &st, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1.0F, 0, 0, 0, 0, 0, 0);
    append();
    mavlink_msg_timesync_pack_status(255, 190, &st, &msg, 0, 987654321, 1, 1);
    append();
    mavlink_msg_heartbeat_pack_status(255, 190, &st, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0,
                                      0, MAV_STATE_ACTIVE);
    append();

    MavDecoder dec;
    Collector out;
    dec.feed(stream.data(), stream.size(), out);
    ASSERT_EQ(out.messages.size(), 3U);
    const auto& cmd = std::get<CommandLongMsg>(out.messages[0]);
    EXPECT_EQ(cmd.command, MAV_CMD_COMPONENT_ARM_DISARM);
    EXPECT_FLOAT_EQ(cmd.param[0], 1.0F);
    const auto& ts = std::get<TimesyncMsg>(out.messages[1]);
    EXPECT_EQ(ts.tc1, 0);
    EXPECT_EQ(ts.ts1, 987654321);
    EXPECT_EQ(std::get<OtherMsg>(out.messages[2]).msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
}

TEST(MavCodec, AcceptsMavlink1Input) {
    mavlink_status_t st{};
    st.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack_status(255, 190, &st, &msg, 1, 1, MAV_CMD_DO_SET_MODE, 0,
                                         1.0F, 6.0F, 0, 0, 0, 0, 0);
    std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const std::uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
    ASSERT_EQ(buf[0], MAVLINK_STX_MAVLINK1);

    MavDecoder dec;
    Collector out;
    dec.feed(buf, n, out);
    ASSERT_EQ(out.messages.size(), 1U);
    EXPECT_EQ(std::get<CommandLongMsg>(out.messages[0]).command, MAV_CMD_DO_SET_MODE);
}

TEST(MavCodec, CorruptedFrameIsCountedNotDelivered) {
    auto bytes = client_setpoint(0, 3527, 1.0F);
    bytes[20] ^= 0x01U;
    MavDecoder dec;
    Collector out;
    dec.feed(bytes.data(), bytes.size(), out);
    EXPECT_TRUE(out.messages.empty());
    EXPECT_EQ(dec.stats().crc_errors, 1U);
}

TEST(MavCodec, ResynchronizesAfterGarbage) {
    Rng rng(2024, 0);
    for (int trial = 0; trial < 200; ++trial) {
        std::vector<std::uint8_t> stream;
        for (int i = 0; i < 1000; ++i) {
            stream.push_back(static_cast<std::uint8_t>(rng.next_u64()));
        }
        for (std::uint8_t seq = 0; seq < 10; ++seq) {
            const auto frame = client_setpoint(seq, 3527, 1.0F);
            stream.insert(stream.end(), frame.begin(), frame.end());
        }
        MavDecoder dec;
        Collector out;
        dec.feed(stream.data(), stream.size(), out);
        // A start byte in the garbage can swallow at most one max-size frame (280 bytes)
        // worth of valid frames; everything after that must decode.
        ASSERT_GE(out.messages.size(), 5U) << "trial " << trial;
        EXPECT_EQ(std::get<SetpointMsg>(out.messages.back()).seq, 9) << "trial " << trial;
    }
}

TEST(MavCodec, FramesFitMaxPacket) {
    MavEncoder enc(1, 1);
    EXPECT_LE(enc.statustext({MAV_SEVERITY_INFO, std::string(50, 'a')}).len, kMaxFrameLen);
    EXPECT_LE(enc.battery_status({6, 25.0, 1.0, 1.0, 1.0}).len, kMaxFrameLen);
}
