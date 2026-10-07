#pragma once

// The autopilot's own clock: nanoseconds since its last boot, plus the offset
// injected by clock faults (F3). It stamps time_boot_ms in telemetry and answers
// TIMESYNC, so an onboard computer that runs TIMESYNC can notice the drift and
// one that does not cannot.
//
// The offset is a pure function of time: for every clock-fault window that began
// after the last boot, a step at its start plus drift_ppm over the part of the
// window already elapsed. Past windows keep their accumulated offset, as a real
// clock that drifted does.

#include "fcstub/fault_schedule.hpp"
#include "fcstub/time.hpp"

namespace fcstub {

class FcClock {
public:
    explicit FcClock(const FaultSchedule& schedule) noexcept;

    void boot(TimeNs now) noexcept { boot_ = now; }

    TimeNs fc_ns(TimeNs now) const noexcept;

    std::uint32_t time_boot_ms(TimeNs now) const noexcept;

private:
    const FaultSchedule* schedule_;
    TimeNs boot_ = 0;
};

}  // namespace fcstub
