#include "fcstub/battery.hpp"

#include <algorithm>

namespace fcstub {

namespace {

constexpr double kCellEmptyV = 3.5;
constexpr double kCellSpanV = 0.7;  // empty 3.5 V .. full 4.2 V, linear
constexpr double kMahPerAs = 1.0 / 3.6;

}  // namespace

Battery::Battery(const BatteryConfig& cfg) noexcept : cfg_(cfg), soc_(cfg.soc_initial) {
    step(0.0, false, 0.0);
}

void Battery::step(double dt, bool armed, double speed_mps) noexcept {
    current_ = armed ? cfg_.i_hover_a + cfg_.k_speed * speed_mps * speed_mps : cfg_.i_idle_a;
    const double used_mah = current_ * dt * kMahPerAs;
    consumed_mah_ += used_mah;
    soc_ = std::max(0.0, soc_ - used_mah / cfg_.capacity_mah);
    voltage_ = cfg_.cells * (kCellEmptyV + kCellSpanV * soc_) - current_ * cfg_.r_int_ohm;
}

}  // namespace fcstub
