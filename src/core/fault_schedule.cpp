#include "fcstub/fault_schedule.hpp"

#include <algorithm>

namespace fcstub {

namespace {

TimeNs window_end(const FaultSpec& f, TimeNs start) noexcept {
    if (f.type == FaultType::FcReboot) {
        return start + ms_to_ns(std::get<FcRebootParams>(f.params).boot_ms);
    }
    return f.duration_s > 0.0 ? start + seconds_to_ns(f.duration_s) : FaultSchedule::kNever;
}

}  // namespace

FaultSchedule::FaultSchedule(const std::vector<FaultSpec>& faults, std::uint64_t seed) {
    windows_.reserve(faults.size());
    rngs_.reserve(faults.size());
    for (std::size_t i = 0; i < faults.size(); ++i) {
        const FaultSpec& f = faults[i];
        const TimeNs start = seconds_to_ns(f.start_s);
        windows_.push_back(FaultWindow{f.type, start, window_end(f, start), i, f.params});
        rngs_.emplace_back(seed, static_cast<std::uint64_t>(i) + 1);
    }
}

const FaultWindow* FaultSchedule::active(FaultType type, TimeNs now) const noexcept {
    for (const FaultWindow& w : windows_) {
        if (w.type == type && w.start <= now && now < w.end) {
            return &w;
        }
    }
    return nullptr;
}

TimeNs FaultSchedule::next_boundary(TimeNs now) const noexcept {
    TimeNs next = kNever;
    for (const FaultWindow& w : windows_) {
        if (w.start > now) {
            next = std::min(next, w.start);
        }
        if (w.end > now) {
            next = std::min(next, w.end);
        }
    }
    return next;
}

}  // namespace fcstub
