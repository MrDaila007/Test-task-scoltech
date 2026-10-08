#include "process.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace {

using namespace fcstub::itest;

const std::string kFixtures = std::string(FCSTUB_SOURCE_DIR) + "/tests/fixtures/";

std::string realtime_config(std::uint16_t bind_port, std::uint16_t remote_port, double duration,
                            const std::string& report) {
    return "schema_version: 1\nrun: {duration_s: " + std::to_string(duration) +
           "}\nlink: {bind_addr: 127.0.0.1, bind_port: " + std::to_string(bind_port) +
           ", remote_port: " + std::to_string(remote_port) +
           "}\nrealtime: {sched_fifo: false, report_path: \"" + report + "\"}\n";
}

}  // namespace

TEST(Cli, HelpAndVersion) {
    EXPECT_EQ(run("--help").exit_code, 0);
    const RunResult v = run("--version");
    EXPECT_EQ(v.exit_code, 0);
    EXPECT_EQ(v.out, "fc_stub 0.1.0\n");
}

TEST(Cli, BadCommandLineIs64) {
    EXPECT_EQ(run("--no-such-flag").exit_code, 64);
    EXPECT_EQ(run("").exit_code, 64);          // --config missing
    EXPECT_EQ(run("--config").exit_code, 64);  // value missing
}

TEST(Cli, BadConfigurationIs2) {
    const auto bad = temp_file("bad.yaml", "schema_version: 1\nlink: {bind_port: 99999}\n");
    EXPECT_EQ(run("--config " + bad.string() + " --validate").exit_code, 2);
    EXPECT_EQ(run("--config /nonexistent.yaml --validate").exit_code, 2);
    EXPECT_EQ(run("--config " + kFixtures + "golden_all_faults.yaml --seed x").exit_code, 2);
}

// Command-line overrides obey the same ranges as the YAML keys they replace.
TEST(Cli, OverridesAreRangeChecked) {
    const std::string cfg = std::string(FCSTUB_SOURCE_DIR) + "/config/default.yaml";
    for (const char* bad : {"--duration -5", "--duration 5abc", "--duration nan", "--duration 1e9",
                            "--seed -1", "--seed 12x", "--seed \"\""}) {
        const RunResult r = run("--config " + cfg + " --validate " + bad);
        EXPECT_EQ(r.exit_code, 2) << bad;
    }
    EXPECT_EQ(run("--config " + cfg + " --validate --duration 2.5 --seed 42").exit_code, 0);
}

TEST(Cli, ValidateAndParamCount) {
    const std::string cfg = std::string(FCSTUB_SOURCE_DIR) + "/config/default.yaml";
    const RunResult v = run("--config " + cfg + " --validate");
    EXPECT_EQ(v.exit_code, 0);
    EXPECT_EQ(v.out, "config OK (48 parameters)\n");
    EXPECT_EQ(run("--config " + cfg + " --print-param-count").out, "48\n");
}

TEST(Cli, SimPrintsTheGoldenHash) {
    const auto out = std::filesystem::temp_directory_path() / "fcstub_it_sim";
    const RunResult r =
        run("--config " + kFixtures + "golden_all_faults.yaml --sim --out " + out.string());
    EXPECT_EQ(r.exit_code, 0);
    EXPECT_EQ(r.out, "sha256 433dd363ecf6826aca4a3227351ee54b4364a505cce18b7795baa9f979d05e7c\n");
}

TEST(Cli, PortInUseIs3) {
    const std::uint16_t port = free_udp_port();
    const int blocker = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(::bind(blocker, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
    const auto cfg = temp_file("busy.yaml", realtime_config(port, free_udp_port(), 1, ""));
    EXPECT_EQ(run("--config " + cfg.string()).exit_code, 3);
    ::close(blocker);
}

TEST(Cli, SigtermStopsCleanlyAndWritesTheReport) {
    const auto report = std::filesystem::temp_directory_path() / "fcstub_it_report.json";
    std::filesystem::remove(report);
    const auto cfg = temp_file(
        "term.yaml", realtime_config(free_udp_port(), free_udp_port(), 0, report.string()));
    Child child({"--config", cfg.string()});
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    EXPECT_EQ(child.terminate(), 0);
    std::ifstream in(report);
    const std::string json((std::istreambuf_iterator<char>(in)), {});
    EXPECT_NE(json.find("\"jitter_ok\""), std::string::npos);
    EXPECT_NE(json.find("\"attitude\""), std::string::npos);
}

// Regression: with a memlock limit large enough for mlockall(MCL_CURRENT | MCL_FUTURE)
// to succeed but too small for later allocations (8 MB is a common container
// default), the stub must still run. It used to die with std::bad_alloc because
// buffers were allocated after locking.
TEST(Cli, RunsUnderAnEightMegabyteMemlockLimit) {
    if (std::system("sh -c 'ulimit -S -l 8192' 2>/dev/null") != 0) {
        GTEST_SKIP() << "hard memlock limit below 8 MB here";
    }
    const auto report = std::filesystem::temp_directory_path() / "fcstub_it_memlock.json";
    std::filesystem::remove(report);
    const auto cfg = temp_file(
        "memlock.yaml", realtime_config(free_udp_port(), free_udp_port(), 1, report.string()));
    const int status = std::system(("sh -c 'ulimit -S -l 8192 && exec " + kBinary + " --config " +
                                    cfg.string() + "' 2>/dev/null")
                                       .c_str());
    EXPECT_EQ(WEXITSTATUS(status), 0);
    EXPECT_TRUE(std::filesystem::exists(report));
}
