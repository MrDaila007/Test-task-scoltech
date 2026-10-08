#include "fcstub/setpoint_gate.hpp"

#include <cmath>

namespace fcstub {

namespace {

// POSITION_TARGET_TYPEMASK bits.
constexpr std::uint16_t kPosIgnore = 1U | 2U | 4U;
constexpr std::uint16_t kVelIgnore = 8U | 16U | 32U;
constexpr std::uint16_t kAccIgnore = 64U | 128U | 256U;
constexpr std::uint16_t kYawIgnore = 1024U;
constexpr std::uint16_t kYawRateIgnore = 2048U;
constexpr std::uint8_t kFrameLocalNed = 1;  // MAV_FRAME_LOCAL_NED

bool kind_from_mask(std::uint16_t type_mask, SetpointKind& kind) noexcept {
    const auto m = static_cast<std::uint16_t>(type_mask & ~(kYawIgnore | kYawRateIgnore));
    if (m == (kPosIgnore | kAccIgnore)) {
        kind = SetpointKind::Velocity;
    } else if (m == (kVelIgnore | kAccIgnore)) {
        kind = SetpointKind::Position;
    } else if (m == kAccIgnore) {
        kind = SetpointKind::PositionVelocity;
    } else {
        return false;
    }
    return true;
}

bool finite3(float a, float b, float c) noexcept {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

bool uses_position(SetpointKind k) noexcept { return k != SetpointKind::Velocity; }
bool uses_velocity(SetpointKind k) noexcept { return k != SetpointKind::Position; }

}  // namespace

SetpointGate::SetpointGate(const GateLimits& limits) noexcept : limits_(limits) {}

void SetpointGate::reset() noexcept {
    last_ = Setpoint{};
    last_valid_ = kNoSetpoint;
}

RejectReason SetpointGate::classify(const SetpointMsg& m, Setpoint& sp) const noexcept {
    if ((m.target_system != 0 && m.target_system != limits_.system_id) ||
        (m.target_component != 0 && m.target_component != limits_.component_id)) {
        return RejectReason::NotForUs;
    }
    if (m.coordinate_frame != kFrameLocalNed) {
        return RejectReason::Frame;
    }
    if (!kind_from_mask(m.type_mask, sp.kind)) {
        return RejectReason::TypeMask;
    }
    sp.yaw_valid = (m.type_mask & kYawIgnore) == 0;
    sp.yaw_rate_valid = (m.type_mask & kYawRateIgnore) == 0;
    const bool pos_ok = !uses_position(sp.kind) || finite3(m.x, m.y, m.z);
    const bool vel_ok = !uses_velocity(sp.kind) || finite3(m.vx, m.vy, m.vz);
    const bool yaw_ok = (!sp.yaw_valid || std::isfinite(m.yaw)) &&
                        (!sp.yaw_rate_valid || std::isfinite(m.yaw_rate));
    if (!pos_ok || !vel_ok || !yaw_ok) {
        return RejectReason::NonFinite;
    }
    sp.pos = {m.x, m.y, m.z};
    sp.vel = {m.vx, m.vy, m.vz};
    sp.yaw = m.yaw;
    sp.yaw_rate = m.yaw_rate;
    // A velocity above v_max is accepted and flown at v_max, as PX4 does.
    // Position targets outside the fence or below ground are refused instead of
    // triggering a geofence failsafe: a deliberate simplification.
    if (uses_position(sp.kind) && (std::hypot(sp.pos[0], sp.pos[1]) > limits_.geofence_m ||
                                   sp.pos[2] > 0.0 || sp.pos[2] < -limits_.geofence_alt_m)) {
        return RejectReason::Range;
    }
    return RejectReason::None;
}

GateOutcome SetpointGate::submit(TimeNs now, const SetpointMsg& msg) noexcept {
    Setpoint candidate;
    const RejectReason reason = classify(msg, candidate);
    if (reason == RejectReason::None) {
        last_ = candidate;
        last_valid_ = now;
        ++stats_.accepted;
    } else {
        ++stats_.rejected[static_cast<std::size_t>(reason)];
    }
    return {reason};
}

const char* reject_reason_name(RejectReason reason) noexcept {
    switch (reason) {
        case RejectReason::None:
            return "none";
        case RejectReason::NotForUs:
            return "target";
        case RejectReason::Frame:
            return "frame";
        case RejectReason::TypeMask:
            return "type_mask";
        case RejectReason::NonFinite:
            return "nonfinite";
        case RejectReason::Range:
            return "range";
        case RejectReason::Count:
            break;
    }
    return "?";
}

}  // namespace fcstub
