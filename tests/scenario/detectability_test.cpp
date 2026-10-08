// For each fault scenario: does the reference onboard detector notice the fault,
// how fast, and - for the faults it cannot notice - proof that it stays silent
// while the vehicle state is actually wrong.

#include "../support/naive_detector.hpp"

#include "fcstub/config_loader.hpp"
#include "fcstub/sim_driver.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

namespace fs = std::filesystem;
using namespace fcstub;
using fcstub::test::Alarm;
using fcstub::test::AlarmEvent;

struct ScenarioRun {
    std::vector<AlarmEvent> alarms;
    fs::path dir;
};

std::string describe(const std::vector<AlarmEvent>& alarms) {
    std::ostringstream os;
    for (const auto& a : alarms) {
        os << test::alarm_name(a.kind) << "@" << a.t_s << " ";
    }
    return os.str();
}

ScenarioRun run_scenario(const std::string& name) {
    Config cfg =
        load_config(std::string(FCSTUB_SOURCE_DIR) + "/config/scenarios/" + name + ".yaml");
    cfg.sim.out_dir = (fs::temp_directory_path() / ("fcstub_scn_" + name)).string();
    fs::remove_all(cfg.sim.out_dir);
    (void)SimDriver(cfg).run();
    ScenarioRun r;
    r.dir = cfg.sim.out_dir;
    r.alarms = test::detect(test::read_journal((r.dir / "frames.bin").string()));
    ::testing::Test::RecordProperty("alarms", describe(r.alarms));
    return r;
}

// First alarm of `kind`, or a negative time if none.
double first(const std::vector<AlarmEvent>& alarms, Alarm kind) {
    for (const auto& a : alarms) {
        if (a.kind == kind) {
            return a.t_s;
        }
    }
    return -1.0;
}

// Last value of the err_h_m column in truth.csv.
double final_position_error(const fs::path& dir) {
    std::ifstream in(dir / "truth.csv");
    std::string line;
    std::string last;
    while (std::getline(in, line)) {
        last = line;
    }
    const auto end = last.rfind(',');
    const auto start = last.rfind(',', end - 1);
    return std::stod(last.substr(start + 1, end - start - 1));
}

// Value of `column` in the first truth.csv row at or after `t_s`.
double value_at(const fs::path& dir, double t_s, const std::string& column) {
    std::ifstream in(dir / "truth.csv");
    std::string header;
    std::getline(in, header);
    std::string row;
    while (std::getline(in, row) && std::stod(row.substr(0, row.find(','))) < t_s) {
    }
    std::stringstream hs(header);
    std::stringstream ls(row);
    std::string name;
    std::string value;
    while (std::getline(hs, name, ',') && std::getline(ls, value, ',')) {
        if (name == column) {
            return std::stod(value);
        }
    }
    ADD_FAILURE() << "no column " << column;
    return 0.0;
}

}  // namespace

TEST(Detectability, CleanRunRaisesNoAlarm) {
    const ScenarioRun r = run_scenario("clean");
    EXPECT_TRUE(r.alarms.empty()) << describe(r.alarms);
}

TEST(Detectability, F1FreezeIsCaughtWithinOneAndAHalfSeconds) {
    const ScenarioRun r = run_scenario("f1_freeze");
    const double t = first(r.alarms, Alarm::FrozenPosition);
    EXPECT_GE(t, 20.0) << describe(r.alarms);
    EXPECT_LE(t, 21.5) << describe(r.alarms);
}

TEST(Detectability, F1FrozenHoverIsNotDetectable) {
    const ScenarioRun r = run_scenario("f1_freeze_all_hover");
    EXPECT_TRUE(r.alarms.empty()) << describe(r.alarms);
}

TEST(Detectability, F1FrozenHoverIsCaughtByAnActiveProbe) {
    const ScenarioRun r = run_scenario("f1_freeze_all_hover_probe");
    const double t = first(r.alarms, Alarm::TrackingError);
    EXPECT_GE(t, 26.0) << describe(r.alarms);  // probe at 24 s, 2 s settle + 1 s mismatch
    EXPECT_LE(t, 27.5) << describe(r.alarms);
}

TEST(Detectability, F2RebootIsCaughtByHeartbeatLossAndTimeRegression) {
    const ScenarioRun r = run_scenario("f2_reboot");
    const double hb = first(r.alarms, Alarm::HeartbeatTimeout);
    EXPECT_GE(hb, 30.0) << describe(r.alarms);
    EXPECT_LE(hb, 32.0) << describe(r.alarms);
    const double back = first(r.alarms, Alarm::TimeRegression);
    EXPECT_GE(back, 33.0) << describe(r.alarms);
    EXPECT_LE(back, 33.1) << describe(r.alarms);
}

TEST(Detectability, F3ClockDriftIsInvisibleWithoutTimesync) {
    const ScenarioRun r = run_scenario("f3_clock_drift");
    EXPECT_TRUE(r.alarms.empty()) << describe(r.alarms);
}

TEST(Detectability, F3ClockDriftIsCaughtWithTimesync) {
    const ScenarioRun r = run_scenario("f3_clock_drift_timesync");
    const double t = first(r.alarms, Alarm::ClockDrift);
    // 100 ppm needs 50 s to reach the 5 ms threshold: drift starts at 10 s.
    EXPECT_GE(t, 59.0) << describe(r.alarms);
    EXPECT_LE(t, 62.0) << describe(r.alarms);
}

TEST(Detectability, F4LinkDegradationIsCaughtBySequenceGaps) {
    const ScenarioRun r = run_scenario("f4_link_burst");
    const double t = first(r.alarms, Alarm::SequenceGap);
    EXPECT_GE(t, 20.0) << describe(r.alarms);
    EXPECT_LE(t, 22.0) << describe(r.alarms);
}

TEST(Detectability, F5GnssJumpIsCaughtImmediately) {
    const ScenarioRun r = run_scenario("f5_gnss_jump");
    const double t = first(r.alarms, Alarm::PositionJump);
    EXPECT_GE(t, 25.0) << describe(r.alarms);
    EXPECT_LE(t, 25.2) << describe(r.alarms);
}

TEST(Detectability, F5GnssDriftIsNotDetectableYetTheErrorGrows) {
    const ScenarioRun r = run_scenario("f5_gnss_drift");
    EXPECT_TRUE(r.alarms.empty()) << describe(r.alarms);
    EXPECT_GE(final_position_error(r.dir), 10.0);  // 0.22 m/s for 50 s
    // The autopilot flies its estimate: reported velocity follows the setpoint
    // (2 m/s north), the real vehicle moves 0.2 m/s slower and 0.1 m/s west.
    EXPECT_NEAR(value_at(r.dir, 50.0, "rep_vn"), 2.0, 0.01);
    EXPECT_NEAR(value_at(r.dir, 50.0, "rep_ve"), 0.0, 0.01);
    EXPECT_NEAR(value_at(r.dir, 50.0, "true_vn"), 1.8, 0.01);
    EXPECT_NEAR(value_at(r.dir, 50.0, "true_ve"), -0.1, 0.01);
}
