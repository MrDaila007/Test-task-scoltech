#pragma once

// Validation of incoming SET_POSITION_TARGET_LOCAL_NED setpoints.
//
// Only an accepted setpoint becomes the current target and refreshes the
// "last valid setpoint" time that the offboard-loss timeout watches; a rejected
// one never keeps offboard alive. Accepted type_mask forms (yaw bits free):
//   velocity only (e.g. 3527), position only (3576), position + velocity (3520).

#include "fcstub/config.hpp"
#include "fcstub/mav_codec.hpp"
#include "fcstub/time.hpp"

#include <array>
#include <cstdint>
#include <limits>

namespace fcstub {

enum class SetpointKind : std::uint8_t { Velocity, Position, PositionVelocity };

struct Setpoint {
    SetpointKind kind = SetpointKind::Velocity;
    Vec3 pos{};  // NED, m
    Vec3 vel{};  // NED, m/s
    bool yaw_valid = false;
    float yaw = 0.0F;
    bool yaw_rate_valid = false;
    float yaw_rate = 0.0F;
};

enum class RejectReason : std::uint8_t { None, NotForUs, Frame, TypeMask, NonFinite, Range, Count };

struct GateLimits {
    std::uint8_t system_id;
    std::uint8_t component_id;
    double v_max_mps;
    double geofence_m;
    double geofence_alt_m;
};

struct GateStats {
    std::uint64_t accepted = 0;
    std::array<std::uint64_t, static_cast<std::size_t>(RejectReason::Count)> rejected{};
};

struct GateOutcome {
    RejectReason reason;
};

class SetpointGate {
public:
    static constexpr TimeNs kNoSetpoint = std::numeric_limits<TimeNs>::min();

    explicit SetpointGate(const GateLimits& limits) noexcept;

    GateOutcome submit(TimeNs now, const SetpointMsg& msg) noexcept;

    TimeNs last_valid_time() const noexcept { return last_valid_; }
    bool has_setpoint() const noexcept { return last_valid_ != kNoSetpoint; }
    const Setpoint& last() const noexcept { return last_; }
    const GateStats& stats() const noexcept { return stats_; }

    // Forget the current target (autopilot reboot).
    void reset() noexcept;

private:
    RejectReason classify(const SetpointMsg& msg, Setpoint& out) const noexcept;

    GateLimits limits_;
    Setpoint last_{};
    TimeNs last_valid_ = kNoSetpoint;
    GateStats stats_{};
};

const char* reject_reason_name(RejectReason reason) noexcept;

}  // namespace fcstub
