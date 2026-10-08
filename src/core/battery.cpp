#include "fcstub/battery.hpp"

#include <algorithm>

namespace fcstub {

namespace {

constexpr double kCellEmptyV = 3.5;
constexpr double kCellSpanV = 0.7;  // empty 3.5 V .. full 4.2 V, linear
constexpr double kMahPerAs = 1.0 / 3.6;

}  // namespace

Battery::Battery(const BatteryConfig& cfg) noexcept
    : cfg_(cfg), soc_(cfg.soc_initial), true_soc_(cfg.soc_initial) {
    step(0.0, false, 0.0);
}

void Battery::step(double dt, bool armed, double speed_mps,
                   const BatteryFaultParams* fault) noexcept {
    current_ = armed ? cfg_.i_hover_a + cfg_.k_speed * speed_mps * speed_mps : cfg_.i_idle_a;
    const double used_mah = current_ * dt * kMahPerAs;
    consumed_mah_ += used_mah;
    soc_ = std::max(0.0, soc_ - used_mah / cfg_.capacity_mah);
    const double real_capacity = cfg_.capacity_mah * (fault ? fault->capacity_factor : 1.0);
    true_soc_ = std::max(0.0, true_soc_ - used_mah / real_capacity);
    const double r_int = cfg_.r_int_ohm * (fault ? fault->r_int_factor : 1.0);
    voltage_ = cfg_.cells * (kCellEmptyV + kCellSpanV * true_soc_) - current_ * r_int;
}

}  // namespace fcstub
