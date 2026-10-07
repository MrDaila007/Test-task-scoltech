#pragma once

// What the autopilot *reports* about its state, as opposed to the true state of
// the model. Telemetry is built from this estimate, so estimator-level faults
// stay consistent across ATTITUDE and GLOBAL_POSITION_INT:
//
//   F5 GNSS jump  : position offset by offset_m; velocity unaffected.
//   F5 GNSS drift : position walks at drift_mps and velocity includes drift_mps
//                   (consistent, hence invisible without an independent sensor).
//   F1 freeze     : the estimate is captured when the window opens; position and
//                   attitude stay frozen, velocity too if freeze_velocity is set.
// After a window closes the estimate snaps back to truth.

#include "fcstub/dynamics.hpp"
#include "fcstub/fault_schedule.hpp"
#include "fcstub/time.hpp"

#include <cstddef>

namespace fcstub {

class EstimatorTap {
public:
    explicit EstimatorTap(const FaultSchedule& schedule) noexcept;

    // Call once per model step; returns the estimate for `now`.
    const VehicleState& update(const VehicleState& truth, TimeNs now) noexcept;

    const VehicleState& estimate() const noexcept { return estimate_; }

private:
    VehicleState apply_gnss(const VehicleState& truth, TimeNs now) const noexcept;

    const FaultSchedule* schedule_;
    VehicleState estimate_{};
    VehicleState snapshot_{};
    bool frozen_ = false;
    std::size_t frozen_window_ = 0;
};

}  // namespace fcstub
