#include "fcstub/dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace fcstub {

namespace {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxYawRate = 1.5;  // rad/s

Vec3 saturate_norm(const Vec3& v, double limit) noexcept {
    const double n = std::hypot(v[0], v[1], v[2]);
    if (n <= limit || n == 0.0) {
        return v;
    }
    const double k = limit / n;
    return {v[0] * k, v[1] * k, v[2] * k};
}

}  // namespace

double wrap_pi(double angle) noexcept {
    double a = std::fmod(angle + kPi, 2.0 * kPi);
    if (a < 0.0) {
        a += 2.0 * kPi;
    }
    return a - kPi;
}

Dynamics::Dynamics(const DynamicsParams& params) noexcept : params_(params) {}

void Dynamics::halt() noexcept {
    state_.vel = {};
    state_.accel = {};
    state_.roll = state_.pitch = 0.0;
    state_.roll_rate = state_.pitch_rate = state_.yaw_rate = 0.0;
}

Vec3 Dynamics::velocity_command(const MotionTarget& t) const noexcept {
    Vec3 v_sp{};
    switch (t.kind) {
        case MotionKind::Velocity:
            v_sp = t.vel;
            break;
        case MotionKind::Position:
        case MotionKind::PositionVelocity: {
            const double ff = t.kind == MotionKind::PositionVelocity ? 1.0 : 0.0;
            for (std::size_t i = 0; i < 3; ++i) {
                v_sp[i] = params_.pos_gain * (t.pos[i] - state_.pos[i]) + ff * t.vel[i];
            }
            break;
        }
        case MotionKind::Disarmed:
        case MotionKind::Stop:
            break;
    }
    return saturate_norm(v_sp, params_.v_max_mps);
}

double Dynamics::yaw_rate_command(const MotionTarget& t) const noexcept {
    double rate = 0.0;
    if (t.yaw_valid) {
        rate = wrap_pi(t.yaw - state_.yaw) / params_.tau_s;
    } else if (t.yaw_rate_valid) {
        rate = t.yaw_rate;
    }
    return std::clamp(rate, -kMaxYawRate, kMaxYawRate);
}

void Dynamics::update_attitude(double dt, double yaw_rate) noexcept {
    const double c = std::cos(state_.yaw);
    const double s = std::sin(state_.yaw);
    const double a_forward = state_.accel[0] * c + state_.accel[1] * s;
    const double a_right = -state_.accel[0] * s + state_.accel[1] * c;
    const double pitch = -std::atan(a_forward / kGravity);
    const double roll = std::atan(a_right / kGravity);
    state_.pitch_rate = (pitch - state_.pitch) / dt;
    state_.roll_rate = (roll - state_.roll) / dt;
    state_.pitch = pitch;
    state_.roll = roll;
    state_.yaw_rate = yaw_rate;
    state_.yaw = wrap_pi(state_.yaw + yaw_rate * dt);
}

void Dynamics::step(double dt, const MotionTarget& target) noexcept {
    if (target.kind == MotionKind::Disarmed) {
        halt();
        return;
    }
    const Vec3 v_sp = velocity_command(target);
    const double k = dt / params_.tau_s;
    for (std::size_t i = 0; i < 3; ++i) {
        const double v_new = state_.vel[i] + (v_sp[i] - state_.vel[i]) * k;
        state_.accel[i] = (v_new - state_.vel[i]) / dt;
        state_.vel[i] = v_new;
        state_.pos[i] += v_new * dt;
    }
    if (state_.pos[2] > 0.0) {  // ground: down is positive in NED
        state_.pos[2] = 0.0;
        state_.vel[2] = std::min(state_.vel[2], 0.0);
        state_.accel[2] = 0.0;
    }
    update_attitude(dt, yaw_rate_command(target));
}

}  // namespace fcstub
