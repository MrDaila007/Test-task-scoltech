// Real-time timing on this machine: every telemetry stream keeps its rate and
// the send jitter against the absolute grid stays within 1 ms. Measured by the
// stub itself (CLOCK_MONOTONIC right before sendto) and read from its report.
// Label "timing": exclude with `ctest -LE timing` on a loaded or emulated host.

#include "process.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <regex>
#include <string>

namespace {

using namespace fcstub::itest;

struct StreamReport {
    long n = -1;
    double max_us = -1.0;
    long missed = -1;
};

StreamReport stream(const std::string& json, const std::string& name) {
    const std::regex re(
        "\"" + name +
        "\": \\{\"period_us\": [0-9.]+, \"n\": ([0-9]+), \"err_us\": \\{\"p50\": "
        "[-0-9.]+, \"p99\": [-0-9.]+, \"max\": ([-0-9.]+)\\}, \"missed\": ([0-9]+)\\}");
    std::smatch m;
    StreamReport r;
    if (std::regex_search(json, m, re)) {
        r.n = std::stol(m[1]);
        r.max_us = std::stod(m[2]);
        r.missed = std::stol(m[3]);
    }
    return r;
}

}  // namespace

TEST(Timing, EveryStreamKeepsItsRateWithJitterUnderOneMillisecond) {
    constexpr double kDurationS = 5.0;
    const auto report = std::filesystem::temp_directory_path() / "fcstub_it_timing.json";
    std::filesystem::remove(report);
    const auto cfg = temp_file(
        "timing.yaml",
        "schema_version: 1\nrun: {duration_s: 5}\nlink: {bind_addr: 127.0.0.1, "
        "bind_port: " +
            std::to_string(free_udp_port()) + ", remote_port: " + std::to_string(free_udp_port()) +
            "}\nrealtime: {sched_fifo: false, report_path: \"" + report.string() + "\"}\n");
    ASSERT_EQ(run("--config " + cfg.string()).exit_code, 0);
    std::ifstream in(report);
    const std::string json((std::istreambuf_iterator<char>(in)), {});

    const struct {
        const char* name;
        double hz;
    } streams[] = {{"heartbeat", 1}, {"attitude", 50},  {"global_position", 10},
                   {"battery", 2},   {"sys_status", 1}, {"extended_sys_state", 1},
                   {"timesync", 1}};
    for (const auto& s : streams) {
        const StreamReport r = stream(json, s.name);
        // Grid points 0, T, 2T ... up to and including the end of the run.
        EXPECT_NEAR(static_cast<double>(r.n), s.hz * kDurationS + 1, 1) << s.name << "\n" << json;
        EXPECT_EQ(r.missed, 0) << s.name;
        EXPECT_GE(r.max_us, 0.0) << s.name;
        EXPECT_LE(r.max_us, 1000.0) << s.name;
    }
    EXPECT_NE(json.find("\"jitter_ok\": true"), std::string::npos) << json;
}
