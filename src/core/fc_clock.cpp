#include "fcstub/fc_clock.hpp"

#include <algorithm>
#include <cmath>

namespace fcstub {

FcClock::FcClock(const FaultSchedule& schedule) noexcept : schedule_(&schedule) {}

TimeNs FcClock::fc_ns(TimeNs now) const noexcept {
    double offset_ns = 0.0;
    for (const FaultWindow& w : schedule_->windows()) {
        if (w.type != FaultType::ClockFault || w.start < boot_ || w.start > now) {
            continue;
        }
        const auto& p = params_of<ClockFaultParams>(w);
        const TimeNs elapsed = std::min(now, w.end) - w.start;
        offset_ns += p.step_ms * static_cast<double>(kNsPerMs) +
                     p.drift_ppm * 1e-6 * static_cast<double>(elapsed);
    }
    return now - boot_ + static_cast<TimeNs>(std::llround(offset_ns));
}

std::uint32_t FcClock::time_boot_ms(TimeNs now) const noexcept {
    const TimeNs ns = std::max<TimeNs>(0, fc_ns(now));
    return static_cast<std::uint32_t>(static_cast<std::uint64_t>(ns / kNsPerMs) & 0xFFFFFFFFU);
}

}  // namespace fcstub
