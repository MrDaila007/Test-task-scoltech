#pragma once

// Autopilot process lifecycle and the reboot fault (F2). While a reboot window
// is open the autopilot is silent: no output, input ignored. When it closes the
// caller restarts everything a real reboot restarts (modes, arming, clock,
// MAVLink sequence numbers).

#include "fcstub/fault_schedule.hpp"
#include "fcstub/time.hpp"

#include <cstddef>
#include <vector>

namespace fcstub {

enum class LifeEvent { None, RebootStarted, Rebooted };

class Lifecycle {
public:
    // Keeps pointers into `schedule`, which must outlive this object.
    explicit Lifecycle(const FaultSchedule& schedule);

    // Returns at most one event per call; call until it returns None.
    LifeEvent update(TimeNs now) noexcept;

    bool silent() const noexcept { return rebooting_; }

    std::uint64_t reboots() const noexcept { return completed_; }

private:
    std::vector<const FaultWindow*> windows_;  // reboot windows in start order
    bool rebooting_ = false;
    std::size_t completed_ = 0;
};

}  // namespace fcstub
