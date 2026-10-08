#pragma once

// Li-ion pack by coulomb counting.
//   I = i_idle (disarmed) or i_hover + k_speed * |v|^2 (armed)
//   soc -= I * dt / C,   U = cells * (3.5 + 0.7 * soc) - I * R_int
//
// soc() is what the autopilot reports: coulomb counting against the configured
// capacity. With a battery fault (F6) the real pack drains against
// C * capacity_factor and sags with R_int * r_int_factor; the voltage follows
// the real pack, so only a voltage-versus-remaining cross-check can notice.

#include "fcstub/config.hpp"

namespace fcstub {

class Battery {
public:
    explicit Battery(const BatteryConfig& cfg) noexcept;

    void step(double dt, bool armed, double speed_mps,
              const BatteryFaultParams* fault = nullptr) noexcept;

    double soc() const noexcept { return soc_; }
    double true_soc() const noexcept { return true_soc_; }
    double voltage_v() const noexcept { return voltage_; }
    double current_a() const noexcept { return current_; }
    double consumed_mah() const noexcept { return consumed_mah_; }
    int cells() const noexcept { return cfg_.cells; }

private:
    BatteryConfig cfg_;
    double soc_;
    double true_soc_;
    double current_ = 0.0;
    double voltage_ = 0.0;
    double consumed_mah_ = 0.0;
};

}  // namespace fcstub
