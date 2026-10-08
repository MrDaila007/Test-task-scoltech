#include "fcstub/mav_codec.hpp"

#include <common/mavlink.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>

namespace fcstub {

static_assert(sizeof(mavlink_status_t) <= 64 && alignof(mavlink_status_t) <= 8,
              "status storage too small");
static_assert(sizeof(mavlink_message_t) <= 320 && alignof(mavlink_message_t) <= 8,
              "message storage too small");
static_assert(kSeverityCritical == MAV_SEVERITY_CRITICAL &&
                  kSeverityWarning == MAV_SEVERITY_WARNING && kSeverityInfo == MAV_SEVERITY_INFO,
              "severity constants");
static_assert(kStateUninit == MAV_STATE_UNINIT && kStateBoot == MAV_STATE_BOOT &&
                  kStateStandby == MAV_STATE_STANDBY && kStateActive == MAV_STATE_ACTIVE,
              "MAV_STATE constants");
static_assert(kModeFlagCustomModeEnabled == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED &&
                  kModeFlagSafetyArmed == MAV_MODE_FLAG_SAFETY_ARMED,
              "mode flag constants");
static_assert(kCmdComponentArmDisarm == MAV_CMD_COMPONENT_ARM_DISARM &&
                  kCmdDoSetMode == MAV_CMD_DO_SET_MODE,
              "command ids");
static_assert(kMaxFrameLen >= MAVLINK_MAX_PACKET_LEN, "frame buffer must hold any MAVLink packet");

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kStatusTextLen = 50;

mavlink_status_t* as_status(std::array<unsigned char, 64>& storage) noexcept {
    return std::launder(reinterpret_cast<mavlink_status_t*>(storage.data()));
}

// Round to nearest and clamp into T's range (wire fields have fixed widths).
template <typename T>
T saturate(double value) noexcept {
    constexpr double lo = static_cast<double>(std::numeric_limits<T>::min());
    constexpr double hi = static_cast<double>(std::numeric_limits<T>::max());
    if (!(value > lo)) {  // also catches NaN
        return std::numeric_limits<T>::min();
    }
    if (value >= hi) {
        return std::numeric_limits<T>::max();
    }
    return static_cast<T>(std::llround(value));
}

// Yaw (rad, any range) -> heading in centidegrees 0..35999.
std::uint16_t heading_cdeg(double yaw_rad) noexcept {
    double deg = std::fmod(yaw_rad * 180.0 / kPi, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    const long long cdeg = std::llround(deg * 100.0) % 36000;
    return static_cast<std::uint16_t>(cdeg);
}

std::uint8_t charge_state(double remaining) noexcept {
    if (remaining > 0.30) {
        return MAV_BATTERY_CHARGE_STATE_OK;
    }
    return remaining > 0.15 ? MAV_BATTERY_CHARGE_STATE_LOW : MAV_BATTERY_CHARGE_STATE_CRITICAL;
}

FrameBuf to_frame(const mavlink_message_t& msg) noexcept {
    FrameBuf frame;
    frame.len = mavlink_msg_to_send_buffer(frame.data.data(), &msg);
    return frame;
}

template <typename Msg>
void fill_header(Msg& out, const mavlink_message_t& msg) noexcept {
    // The library widens sysid to 32 bits; a MAVLink 2 frame carries 8.
    out.source_system = static_cast<std::uint8_t>(msg.sysid);
    out.source_component = msg.compid;
    out.seq = msg.seq;
}

RxMessage decode_setpoint(const mavlink_message_t& msg) noexcept {
    mavlink_set_position_target_local_ned_t p{};
    mavlink_msg_set_position_target_local_ned_decode(&msg, &p);
    SetpointMsg out;
    fill_header(out, msg);
    out.time_boot_ms = p.time_boot_ms;
    out.target_system = p.target_system;
    out.target_component = p.target_component;
    out.coordinate_frame = p.coordinate_frame;
    out.type_mask = p.type_mask;
    out.x = p.x;
    out.y = p.y;
    out.z = p.z;
    out.vx = p.vx;
    out.vy = p.vy;
    out.vz = p.vz;
    out.afx = p.afx;
    out.afy = p.afy;
    out.afz = p.afz;
    out.yaw = p.yaw;
    out.yaw_rate = p.yaw_rate;
    return out;
}

RxMessage decode_command_long(const mavlink_message_t& msg) noexcept {
    mavlink_command_long_t c{};
    mavlink_msg_command_long_decode(&msg, &c);
    CommandLongMsg out;
    fill_header(out, msg);
    out.target_system = c.target_system;
    out.target_component = c.target_component;
    out.command = c.command;
    out.confirmation = c.confirmation;
    out.param = {c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7};
    return out;
}

RxMessage decode_timesync(const mavlink_message_t& msg) noexcept {
    mavlink_timesync_t t{};
    mavlink_msg_timesync_decode(&msg, &t);
    TimesyncMsg out;
    fill_header(out, msg);
    out.tc1 = t.tc1;
    out.ts1 = t.ts1;
    out.target_system = t.target_system;
    out.target_component = t.target_component;
    return out;
}

RxMessage decode(const mavlink_message_t& msg) noexcept {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            return decode_setpoint(msg);
        case MAVLINK_MSG_ID_COMMAND_LONG:
            return decode_command_long(msg);
        case MAVLINK_MSG_ID_TIMESYNC:
            return decode_timesync(msg);
        default: {
            OtherMsg out;
            fill_header(out, msg);
            out.msgid = msg.msgid;
            return out;
        }
    }
}

}  // namespace

// --- encoder -------------------------------------------------------------------------

MavEncoder::MavEncoder(std::uint8_t system_id, std::uint8_t component_id) noexcept
    : system_id_(system_id), component_id_(component_id) {
    new (status_.data()) mavlink_status_t{};
}

void MavEncoder::reset_sequence() noexcept { as_status(status_)->current_tx_seq = 0; }

FrameBuf MavEncoder::heartbeat(const HeartbeatData& d) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack_status(system_id_, component_id_, as_status(status_), &msg,
                                      MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, d.base_mode,
                                      d.custom_mode, d.system_status);
    return to_frame(msg);
}

FrameBuf MavEncoder::attitude(const AttitudeData& d) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_attitude_pack_status(system_id_, component_id_, as_status(status_), &msg,
                                     d.time_boot_ms, d.roll, d.pitch, d.yaw, d.rollspeed,
                                     d.pitchspeed, d.yawspeed);
    return to_frame(msg);
}

FrameBuf MavEncoder::global_position_int(const GlobalPositionData& d) noexcept {
    mavlink_global_position_int_t p{};
    p.time_boot_ms = d.time_boot_ms;
    p.lat = saturate<std::int32_t>(d.lat_deg * 1e7);
    p.lon = saturate<std::int32_t>(d.lon_deg * 1e7);
    p.alt = saturate<std::int32_t>(d.alt_msl_m * 1000.0);
    p.relative_alt = saturate<std::int32_t>(d.relative_alt_m * 1000.0);
    p.vx = saturate<std::int16_t>(d.vn_mps * 100.0);
    p.vy = saturate<std::int16_t>(d.ve_mps * 100.0);
    p.vz = saturate<std::int16_t>(d.vd_mps * 100.0);
    p.hdg = heading_cdeg(d.yaw_rad);
    mavlink_message_t msg{};
    mavlink_msg_global_position_int_encode_status(system_id_, component_id_, as_status(status_),
                                                  &msg, &p);
    return to_frame(msg);
}

FrameBuf MavEncoder::battery_status(const BatteryData& d) noexcept {
    mavlink_battery_status_t b{};
    b.id = 0;
    b.battery_function = MAV_BATTERY_FUNCTION_ALL;
    b.type = MAV_BATTERY_TYPE_LION;
    b.temperature = std::numeric_limits<std::int16_t>::max();  // unknown
    const int cells = std::clamp(d.cells, 1, 10);
    const std::uint16_t cell_mv =
        std::min(saturate<std::uint16_t>(d.voltage_v / cells * 1000.0),
                 static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() - 1));
    for (int i = 0; i < 10; ++i) {
        b.voltages[i] = i < cells ? cell_mv : std::numeric_limits<std::uint16_t>::max();
    }
    b.current_battery = saturate<std::int16_t>(d.current_a * 100.0);
    b.current_consumed = saturate<std::int32_t>(d.consumed_mah);
    b.energy_consumed = -1;
    b.battery_remaining = saturate<std::int8_t>(std::clamp(d.remaining, 0.0, 1.0) * 100.0);
    b.charge_state = charge_state(d.remaining);
    mavlink_message_t msg{};
    mavlink_msg_battery_status_encode_status(system_id_, component_id_, as_status(status_), &msg,
                                             &b);
    return to_frame(msg);
}

FrameBuf MavEncoder::sys_status(const SysStatusData& d) noexcept {
    // The sensors a PX4 multicopter reports; the stub has no sensor failures, so
    // all present ones are enabled and healthy.
    constexpr std::uint32_t kSensors =
        MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
        MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_BATTERY;
    mavlink_sys_status_t s{};
    s.onboard_control_sensors_present = kSensors;
    s.onboard_control_sensors_enabled = kSensors;
    s.onboard_control_sensors_health = kSensors;
    s.voltage_battery = saturate<std::uint16_t>(d.voltage_v * 1000.0);
    s.current_battery = saturate<std::int16_t>(d.current_a * 100.0);
    s.battery_remaining = saturate<std::int8_t>(std::clamp(d.remaining, 0.0, 1.0) * 100.0);
    mavlink_message_t msg{};
    mavlink_msg_sys_status_encode_status(system_id_, component_id_, as_status(status_), &msg, &s);
    return to_frame(msg);
}

FrameBuf MavEncoder::extended_sys_state(std::uint8_t landed_state) noexcept {
    mavlink_extended_sys_state_t e{};
    e.vtol_state = MAV_VTOL_STATE_UNDEFINED;
    e.landed_state = landed_state;
    mavlink_message_t msg{};
    mavlink_msg_extended_sys_state_encode_status(system_id_, component_id_, as_status(status_),
                                                 &msg, &e);
    return to_frame(msg);
}

FrameBuf MavEncoder::statustext(const StatusTextData& d) noexcept {
    std::array<char, kStatusTextLen> text{};
    std::memcpy(text.data(), d.text.data(), std::min(d.text.size(), kStatusTextLen));
    mavlink_message_t msg{};
    mavlink_msg_statustext_pack_status(system_id_, component_id_, as_status(status_), &msg,
                                       d.severity, text.data(), 0, 0);
    return to_frame(msg);
}

FrameBuf MavEncoder::command_ack(const CommandAckData& d) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_command_ack_pack_status(system_id_, component_id_, as_status(status_), &msg,
                                        d.command, d.result, 0, 0, d.target_system,
                                        d.target_component);
    return to_frame(msg);
}

FrameBuf MavEncoder::timesync(const TimesyncData& d) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_timesync_pack_status(system_id_, component_id_, as_status(status_), &msg, d.tc1,
                                     d.ts1, d.target_system, d.target_component);
    return to_frame(msg);
}

// --- client encoder ------------------------------------------------------------------

MavClientEncoder::MavClientEncoder(std::uint8_t system_id, std::uint8_t component_id,
                                   std::uint8_t target_system,
                                   std::uint8_t target_component) noexcept
    : system_id_(system_id),
      component_id_(component_id),
      target_system_(target_system),
      target_component_(target_component) {
    new (status_.data()) mavlink_status_t{};
}

FrameBuf MavClientEncoder::command_long(std::uint16_t command, const std::array<float, 7>& p,
                                        std::uint8_t confirmation) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack_status(system_id_, component_id_, as_status(status_), &msg,
                                         target_system_, target_component_, command, confirmation,
                                         p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
    return to_frame(msg);
}

FrameBuf MavClientEncoder::setpoint_local_ned(std::uint32_t time_boot_ms, std::uint16_t type_mask,
                                              const std::array<float, 3>& pos,
                                              const std::array<float, 3>& vel) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_set_position_target_local_ned_pack_status(
        system_id_, component_id_, as_status(status_), &msg, time_boot_ms, target_system_,
        target_component_, MAV_FRAME_LOCAL_NED, type_mask, pos[0], pos[1], pos[2], vel[0], vel[1],
        vel[2], 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    return to_frame(msg);
}

FrameBuf MavClientEncoder::timesync_request(std::int64_t ts1) noexcept {
    mavlink_message_t msg{};
    mavlink_msg_timesync_pack_status(system_id_, component_id_, as_status(status_), &msg, 0, ts1,
                                     target_system_, target_component_);
    return to_frame(msg);
}

// --- decoder -------------------------------------------------------------------------

MavDecoder::MavDecoder() noexcept {
    new (status_.data()) mavlink_status_t{};
    new (rx_buffer_.data()) mavlink_message_t{};
}

void MavDecoder::feed(const std::uint8_t* data, std::size_t len, RxHandler& handler) noexcept {
    auto* rx = std::launder(reinterpret_cast<mavlink_message_t*>(rx_buffer_.data()));
    mavlink_status_t* status = as_status(status_);
    mavlink_message_t msg{};
    mavlink_status_t msg_status{};
    for (std::size_t i = 0; i < len; ++i) {
        const std::uint8_t result =
            mavlink_frame_char_buffer(rx, status, data[i], &msg, &msg_status);
        if (result == MAVLINK_FRAMING_OK) {
            ++stats_.frames_ok;
            handler.on_message(decode(msg));
        } else if (result == MAVLINK_FRAMING_BAD_CRC) {
            ++stats_.crc_errors;
        } else if (result != MAVLINK_FRAMING_INCOMPLETE) {
            ++stats_.other_errors;
        }
    }
}

}  // namespace fcstub
