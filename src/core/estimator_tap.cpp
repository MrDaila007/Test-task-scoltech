#include "fcstub/estimator_tap.hpp"

#include <cmath>

namespace fcstub {

namespace {

constexpr std::uint64_t kNoiseStream = 0x4E4F495345ULL;  // apart from the per-fault streams

}  // namespace

EstimatorTap::EstimatorTap(const FaultSchedule& schedule, const EstimatorNoise& noise, double dt_s,
                           std::uint64_t seed)
    : schedule_(&schedule),
      noise_(noise),
      decay_(std::exp(-dt_s / noise.noise_tau_s)),
      rng_(seed, kNoiseStream) {}

// Advances one Gauss-Markov error and adds it. With sigma 0 the value is left
// untouched (not even -0.0 + 0.0), so noise-free runs stay bit-identical.
void EstimatorTap::add_error(double& value, double& state, double sigma) noexcept {
    if (sigma > 0.0) {
        state = decay_ * state + sigma * std::sqrt(1.0 - decay_ * decay_) * rng_.normal();
        value += state;
    }
}

void EstimatorTap::apply_noise(VehicleState& est) noexcept {
    for (std::size_t i = 0; i < 3; ++i) {
        add_error(est.pos[i], pos_err_[i], noise_.pos_noise_m);
        add_error(est.vel[i], vel_err_[i], noise_.vel_noise_mps);
    }
    add_error(est.roll, att_err_[0], noise_.att_noise_rad);
    add_error(est.pitch, att_err_[1], noise_.att_noise_rad);
    add_error(est.yaw, att_err_[2], noise_.att_noise_rad);
    if (noise_.att_noise_rad > 0.0) {
        est.yaw = wrap_pi(est.yaw);  // ATTITUDE.yaw is defined on [-pi, pi]
    }
}

VehicleState EstimatorTap::apply_gnss(const VehicleState& truth, TimeNs now) const noexcept {
    VehicleState est = truth;
    const FaultWindow* w = schedule_->active(FaultType::Gnss, now);
    if (w == nullptr) {
        return est;
    }
    const auto& p = params_of<GnssFaultParams>(*w);
    if (p.kind == GnssKind::Jump) {
        for (std::size_t i = 0; i < 3; ++i) {
            est.pos[i] += p.offset_m[i];
        }
    } else {
        const double elapsed_s = ns_to_seconds(now - w->start);
        for (std::size_t i = 0; i < 3; ++i) {
            est.pos[i] += p.drift_mps[i] * elapsed_s;
            est.vel[i] += p.drift_mps[i];
        }
    }
    return est;
}

const VehicleState& EstimatorTap::update(const VehicleState& truth, TimeNs now) noexcept {
    VehicleState live = apply_gnss(truth, now);
    apply_noise(live);
    const FaultWindow* freeze = schedule_->active(FaultType::EstimatorFreeze, now);
    if (freeze == nullptr) {
        frozen_ = false;
        estimate_ = live;
        return estimate_;
    }
    if (!frozen_ || frozen_window_ != freeze->index) {
        frozen_ = true;
        frozen_window_ = freeze->index;
        snapshot_ = live;
    }
    estimate_ = snapshot_;
    if (!params_of<EstimatorFreezeParams>(*freeze).freeze_velocity) {
        estimate_.vel = live.vel;
    }
    return estimate_;
}

}  // namespace fcstub
