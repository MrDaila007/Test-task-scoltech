#pragma once

// Point-mass vehicle with first-order velocity response, in local NED.
//
//   v' = (v_sp - v) / tau,   p' = v,   |v_sp| <= v_max
//   position target: v_sp = sat(pos_gain * (p_sp - p) + v_ff, v_max)
//
// Roll and pitch are the tilt needed for the current horizontal acceleration
// (body frame from yaw); the ground (down = 0) is a hard floor. The model is
// deliberately simple: it only has to make setpoints visible in telemetry.

#include "fcstub/config.hpp"

namespace fcstub {

enum class MotionKind { Disarmed, Stop, Velocity, Position, PositionVelocity };

struct MotionTarget {
    MotionKind kind = MotionKind::Stop;
    Vec3 pos{};
    Vec3 vel{};  // velocity target, or feed-forward for PositionVelocity
    bool yaw_valid = false;
    double yaw = 0.0;
    bool yaw_rate_valid = false;
    double yaw_rate = 0.0;
};

struct VehicleState {
    Vec3 pos{};    // m, NED
    Vec3 vel{};    // m/s, NED
    Vec3 accel{};  // m/s^2, NED, over the last step
    double roll = 0.0, pitch = 0.0, yaw = 0.0;                  // rad
    double roll_rate = 0.0, pitch_rate = 0.0, yaw_rate = 0.0;  // rad/s
};

struct DynamicsParams {
    double tau_s;
    double v_max_mps;
    double pos_gain;
};

class Dynamics {
public:
    explicit Dynamics(DynamicsParams params) noexcept;

    void step(double dt, const MotionTarget& target) noexcept;

    const VehicleState& state() const noexcept { return state_; }

    // Keep position, stop all motion (used when the autopilot restarts).
    void halt() noexcept;

private:
    Vec3 velocity_command(const MotionTarget& target) const noexcept;
    double yaw_rate_command(const MotionTarget& target) const noexcept;
    void update_attitude(double dt, double yaw_rate) noexcept;

    DynamicsParams params_;
    VehicleState state_{};
};

double wrap_pi(double angle) noexcept;

}  // namespace fcstub
