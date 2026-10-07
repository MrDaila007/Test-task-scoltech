#include "fcstub/scheduler.hpp"
#include "fcstub/time.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace {

using fcstub::kNsPerMs;
using fcstub::kNsPerS;
using fcstub::Scheduler;
using fcstub::TimeNs;

struct Fired {
    Scheduler::TaskId id;
    TimeNs deadline;
};

std::vector<Fired> run(Scheduler& s, TimeNs now) {
    Scheduler::DueList due{};
    const std::size_t n = s.due(now, due);
    std::vector<Fired> out;
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back({due[i].id, due[i].deadline});
    }
    return out;
}

}  // namespace

TEST(Scheduler, ExactCountsOverSixtySecondsOfModelTime) {
    Scheduler s;
    const auto hb = s.add(fcstub::period_from_hz(1.0), 0);
    const auto att = s.add(fcstub::period_from_hz(50.0), 0);
    const auto pos = s.add(fcstub::period_from_hz(10.0), 0);
    const auto bat = s.add(fcstub::period_from_hz(2.0), 0);
    std::array<int, Scheduler::kMaxTasks> count{};
    std::array<TimeNs, Scheduler::kMaxTasks> last{};
    for (TimeNs t = 0; t < 60 * kNsPerS; t += kNsPerMs) {
        for (const Fired& f : run(s, t)) {
            if (count[f.id] > 0) {
                ASSERT_EQ(f.deadline - last[f.id], s.period(f.id)) << "task " << int(f.id);
            }
            EXPECT_EQ(f.deadline, t);  // stepping 1 ms never lags a deadline
            last[f.id] = f.deadline;
            ++count[f.id];
        }
    }
    EXPECT_EQ(count[hb], 60);
    EXPECT_EQ(count[att], 3000);
    EXPECT_EQ(count[pos], 600);
    EXPECT_EQ(count[bat], 120);
    EXPECT_EQ(s.missed(att), 0U);
}

TEST(Scheduler, NextDeadlineIsTheEarliestPendingOne) {
    Scheduler s;
    EXPECT_EQ(s.next_deadline(), Scheduler::kNever);
    s.add(100 * kNsPerMs, 100 * kNsPerMs);
    s.add(20 * kNsPerMs, 20 * kNsPerMs);
    EXPECT_EQ(s.next_deadline(), 20 * kNsPerMs);
    (void)run(s, 20 * kNsPerMs);
    EXPECT_EQ(s.next_deadline(), 40 * kNsPerMs);
    EXPECT_TRUE(run(s, 39 * kNsPerMs).empty());
}

TEST(Scheduler, OverrunFiresOnceAtTheLatestDeadlineAndCountsMisses) {
    Scheduler s;
    const auto att = s.add(20 * kNsPerMs, 0);
    (void)run(s, 0);
    const auto fired = run(s, kNsPerS);  // the loop stalled for a whole second
    ASSERT_EQ(fired.size(), 1U);
    EXPECT_EQ(fired[0].deadline, kNsPerS);
    EXPECT_EQ(s.missed(att), 49U);
    EXPECT_EQ(s.next_deadline(), kNsPerS + 20 * kNsPerMs);
}

TEST(Scheduler, OverrunBetweenGridPointsKeepsTheGrid) {
    Scheduler s;
    const auto att = s.add(20 * kNsPerMs, 0);
    (void)run(s, 0);
    const auto fired = run(s, 75 * kNsPerMs);
    ASSERT_EQ(fired.size(), 1U);
    EXPECT_EQ(fired[0].deadline, 60 * kNsPerMs);
    EXPECT_EQ(s.missed(att), 2U);
    EXPECT_EQ(s.next_deadline(), 80 * kNsPerMs);
}

TEST(Scheduler, SimultaneousDeadlinesFireInRegistrationOrder) {
    Scheduler s;
    const auto hb = s.add(kNsPerS, 0);
    const auto att = s.add(20 * kNsPerMs, 0);
    const auto pos = s.add(100 * kNsPerMs, 0);
    const auto bat = s.add(500 * kNsPerMs, 0);
    const auto fired = run(s, 0);
    ASSERT_EQ(fired.size(), 4U);
    EXPECT_EQ(fired[0].id, hb);
    EXPECT_EQ(fired[1].id, att);
    EXPECT_EQ(fired[2].id, pos);
    EXPECT_EQ(fired[3].id, bat);
}

TEST(Scheduler, EarlierDeadlineFiresFirstWhenSeveralAreLate) {
    Scheduler s;
    const auto slow = s.add(100 * kNsPerMs, 30 * kNsPerMs);
    const auto fast = s.add(100 * kNsPerMs, 10 * kNsPerMs);
    const auto fired = run(s, 50 * kNsPerMs);
    ASSERT_EQ(fired.size(), 2U);
    EXPECT_EQ(fired[0].id, fast);
    EXPECT_EQ(fired[1].id, slow);
}

TEST(Scheduler, RejectsMoreThanCapacity) {
    Scheduler s;
    for (std::size_t i = 0; i < Scheduler::kMaxTasks; ++i) {
        EXPECT_NE(s.add(kNsPerS, 0), Scheduler::kInvalidTask);
    }
    EXPECT_EQ(s.add(kNsPerS, 0), Scheduler::kInvalidTask);
}
