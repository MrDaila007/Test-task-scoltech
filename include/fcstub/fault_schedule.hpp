#pragma once

// Fault windows from the configuration, in model time, plus one independent
// random stream per fault (seeded from the run seed and the fault's index).
// Windows are half-open: [start, end); end is kNever for "until the end".

#include "fcstub/config.hpp"
#include "fcstub/rng.hpp"
#include "fcstub/time.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace fcstub {

struct FaultWindow {
    FaultType type;
    TimeNs start;
    TimeNs end;
    std::size_t index;  // position in the configuration, also the random stream id
    FaultParams params;
};

class FaultSchedule {
public:
    static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::max();

    FaultSchedule(const std::vector<FaultSpec>& faults, std::uint64_t seed);

    // Components keep pointers to the schedule and its windows; it stays put.
    FaultSchedule(const FaultSchedule&) = delete;
    FaultSchedule& operator=(const FaultSchedule&) = delete;

    // The window of `type` containing `now`, or nullptr (same-type windows never overlap).
    const FaultWindow* active(FaultType type, TimeNs now) const noexcept;

    // Earliest window start or end strictly after `now`.
    TimeNs next_boundary(TimeNs now) const noexcept;

    const std::vector<FaultWindow>& windows() const noexcept { return windows_; }
    Rng& rng(std::size_t index) noexcept { return rngs_[index]; }

private:
    std::vector<FaultWindow> windows_;
    std::vector<Rng> rngs_;
};

template <typename Params>
const Params& params_of(const FaultWindow& w) noexcept {
    return *std::get_if<Params>(&w.params);
}

}  // namespace fcstub
