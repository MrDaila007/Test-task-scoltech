#include "fcstub/scheduler.hpp"

#include <algorithm>

namespace fcstub {

Scheduler::TaskId Scheduler::add(TimeNs period, TimeNs first_due) noexcept {
    if (count_ >= kMaxTasks || period <= 0) {
        return kInvalidTask;
    }
    tasks_[count_] = Task{period, first_due, 0};
    return static_cast<TaskId>(count_++);
}

std::size_t Scheduler::due(TimeNs now, DueList& out) noexcept {
    std::size_t n = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        Task& t = tasks_[i];
        if (t.next > now) {
            continue;
        }
        const TimeNs behind = (now - t.next) / t.period;  // whole periods already passed
        const TimeNs deadline = t.next + behind * t.period;
        t.missed += static_cast<std::uint64_t>(behind);
        t.next = deadline + t.period;
        out[n++] = Due{static_cast<TaskId>(i), deadline};
    }
    std::sort(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n),
              [](const Due& a, const Due& b) {
                  return a.deadline != b.deadline ? a.deadline < b.deadline : a.id < b.id;
              });
    return n;
}

TimeNs Scheduler::next_deadline() const noexcept {
    TimeNs next = kNever;
    for (std::size_t i = 0; i < count_; ++i) {
        next = std::min(next, tasks_[i].next);
    }
    return next;
}

}  // namespace fcstub
