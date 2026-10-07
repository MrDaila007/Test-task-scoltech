#include "fcstub/estimator_tap.hpp"

namespace fcstub {

EstimatorTap::EstimatorTap(const FaultSchedule& schedule) noexcept : schedule_(&schedule) {}

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
    const VehicleState live = apply_gnss(truth, now);
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
