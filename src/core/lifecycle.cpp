#include "fcstub/lifecycle.hpp"

#include <algorithm>

namespace fcstub {

Lifecycle::Lifecycle(const FaultSchedule& schedule) {
    for (const FaultWindow& w : schedule.windows()) {
        if (w.type == FaultType::FcReboot) {
            windows_.push_back(&w);
        }
    }
    std::sort(windows_.begin(), windows_.end(),
              [](const FaultWindow* a, const FaultWindow* b) { return a->start < b->start; });
}

LifeEvent Lifecycle::update(TimeNs now) noexcept {
    if (completed_ >= windows_.size()) {
        return LifeEvent::None;
    }
    const FaultWindow& w = *windows_[completed_];
    if (!rebooting_ && now >= w.start) {
        rebooting_ = true;
        return LifeEvent::RebootStarted;
    }
    if (rebooting_ && now >= w.end) {
        rebooting_ = false;
        ++completed_;
        return LifeEvent::Rebooted;
    }
    return LifeEvent::None;
}

}  // namespace fcstub
