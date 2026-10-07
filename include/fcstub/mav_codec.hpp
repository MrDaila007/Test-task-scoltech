#pragma once

// MAVLink 2 encoding and decoding for the PX4-like stub.
//
// Encoder and decoder each own their MAVLink channel state (tx sequence, rx
// parser buffer) instead of using the library's global channel table, so
// independent instances in one process never influence each other. That keeps
// two runs of the same scenario byte-identical.
//
// Only mav_codec.cpp includes the MAVLink headers; the library structs are held
// here as opaque, suitably aligned storage.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <variant>

namespace fcstub {

// >= MAVLINK_MAX_PACKET_LEN of the pinned headers (287: the header reserves room for
// extended fields and a signature), checked by a static_assert in mav_codec.cpp.
inline constexpr std::size_t kMaxFrameLen = 288;

struct FrameBuf {
    std::array<std::uint8_t, kMaxFrameLen> data{};
    std::uint16_t len = 0;
};

// PX4 custom_mode encoding: main mode in byte 2, sub mode in byte 3.
inline constexpr std::uint8_t kPx4MainManual = 1;
inline constexpr std::uint8_t kPx4MainPosctl = 3;
inline constexpr std::uint8_t kPx4MainAuto = 4;
inline constexpr std::uint8_t kPx4MainOffboard = 6;
inline constexpr std::uint8_t kPx4SubAutoLoiter = 3;

constexpr std::uint32_t px4_custom_mode(std::uint8_t main_mode, std::uint8_t sub_mode) noexcept {
    return (static_cast<std::uint32_t>(main_mode) << 16U) |
           (static_cast<std::uint32_t>(sub_mode) << 24U);
}

// --- outgoing payloads, in engineering units; the encoder converts to wire units ---

struct HeartbeatData {
    std::uint8_t base_mode;
    std::uint32_t custom_mode;
    std::uint8_t system_status;
};

struct AttitudeData {
    std::uint32_t time_boot_ms;
    float roll, pitch, yaw;  // rad
    float rollspeed, pitchspeed, yawspeed;  // rad/s
};

struct GlobalPositionData {
    std::uint32_t time_boot_ms;
    double lat_deg, lon_deg;
    double alt_msl_m, relative_alt_m;
    double vn_mps, ve_mps, vd_mps;
    double yaw_rad;
};

struct BatteryData {
    int cells;
    double voltage_v;      // pack voltage under load
    double current_a;
    double consumed_mah;
    double remaining;      // 0..1
};

struct StatusTextData {
    std::uint8_t severity;  // MAV_SEVERITY
    std::string_view text;  // truncated to 50 characters
};

struct CommandAckData {
    std::uint16_t command;
    std::uint8_t result;  // MAV_RESULT
    std::uint8_t target_system, target_component;
};

struct TimesyncData {
    std::int64_t tc1, ts1;  // ns
    std::uint8_t target_system, target_component;
};

class MavEncoder {
public:
    MavEncoder(std::uint8_t system_id, std::uint8_t component_id) noexcept;

    FrameBuf heartbeat(const HeartbeatData& d) noexcept;
    FrameBuf attitude(const AttitudeData& d) noexcept;
    FrameBuf global_position_int(const GlobalPositionData& d) noexcept;
    FrameBuf battery_status(const BatteryData& d) noexcept;
    FrameBuf statustext(const StatusTextData& d) noexcept;
    FrameBuf command_ack(const CommandAckData& d) noexcept;
    FrameBuf timesync(const TimesyncData& d) noexcept;

    // Restart the tx sequence at 0, as a rebooted autopilot does.
    void reset_sequence() noexcept;

private:
    std::uint8_t system_id_;
    std::uint8_t component_id_;
    alignas(8) std::array<unsigned char, 64> status_{};  // mavlink_status_t
};

// --- incoming messages ---------------------------------------------------------

struct RxHeader {
    std::uint8_t source_system = 0;
    std::uint8_t source_component = 0;
    std::uint8_t seq = 0;
};

struct SetpointMsg : RxHeader {
    std::uint32_t time_boot_ms = 0;
    std::uint8_t target_system = 0, target_component = 0;
    std::uint8_t coordinate_frame = 0;
    std::uint16_t type_mask = 0;
    float x = 0, y = 0, z = 0;
    float vx = 0, vy = 0, vz = 0;
    float afx = 0, afy = 0, afz = 0;
    float yaw = 0, yaw_rate = 0;
};

struct CommandLongMsg : RxHeader {
    std::uint8_t target_system = 0, target_component = 0;
    std::uint16_t command = 0;
    std::uint8_t confirmation = 0;
    std::array<float, 7> param{};
};

struct TimesyncMsg : RxHeader {
    std::int64_t tc1 = 0, ts1 = 0;
    std::uint8_t target_system = 0, target_component = 0;  // 0 when sent without extensions
};

struct OtherMsg : RxHeader {
    std::uint32_t msgid = 0;
};

using RxMessage = std::variant<SetpointMsg, CommandLongMsg, TimesyncMsg, OtherMsg>;

class RxHandler {
public:
    virtual void on_message(const RxMessage& msg) noexcept = 0;

protected:
    ~RxHandler() = default;
};

struct DecoderStats {
    std::uint64_t frames_ok = 0;
    std::uint64_t crc_errors = 0;   // complete frame, bad checksum
    std::uint64_t other_errors = 0; // bad signature / malformed
};

class MavDecoder {
public:
    MavDecoder() noexcept;

    // Parse a byte stream; every decoded message is delivered synchronously.
    // A partial frame at the end is kept and completed by the next call.
    void feed(const std::uint8_t* data, std::size_t len, RxHandler& handler) noexcept;

    const DecoderStats& stats() const noexcept { return stats_; }

private:
    DecoderStats stats_;
    alignas(8) std::array<unsigned char, 64> status_{};       // mavlink_status_t (parser state)
    alignas(8) std::array<unsigned char, 320> rx_buffer_{};   // mavlink_message_t (in progress)
};

}  // namespace fcstub
