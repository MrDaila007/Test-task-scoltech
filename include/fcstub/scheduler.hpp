#pragma once

// Fixed-capacity periodic scheduler on an absolute time grid.
//
// Deadlines are first_due + k * period; they never drift with call timing. If
// the caller falls behind by more than a period, the task fires once at the
// latest deadline that has passed (no catch-up burst) and the skipped deadlines
// are counted as missed.

#include "fcstub/time.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace fcstub {

class Scheduler {
public:
    using TaskId = std::uint8_t;

    static constexpr std::size_t kMaxTasks = 8;
    static constexpr TaskId kInvalidTask = 0xFF;
    static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::max();

    struct Due {
        TaskId id;
        TimeNs deadline;  // the grid point this firing stands for
    };
    using DueList = std::array<Due, kMaxTasks>;

    // Registration order is the tie-break when deadlines coincide.
    // Precondition: period > 0. Returns kInvalidTask when full.
    TaskId add(TimeNs period, TimeNs first_due) noexcept;

    // Fills `out` with every task due at `now`, ordered by (deadline, id);
    // each task appears at most once. Returns the number of entries.
    std::size_t due(TimeNs now, DueList& out) noexcept;

    TimeNs next_deadline() const noexcept;
    TimeNs period(TaskId id) const noexcept { return tasks_[id].period; }
    std::uint64_t missed(TaskId id) const noexcept { return tasks_[id].missed; }

private:
    struct Task {
        TimeNs period = 0;
        TimeNs next = kNever;
        std::uint64_t missed = 0;
    };

    std::array<Task, kMaxTasks> tasks_{};
    std::size_t count_ = 0;
};

}  // namespace fcstub
