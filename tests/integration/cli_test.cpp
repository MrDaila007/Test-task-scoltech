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

TEST(Cli, ValidateAndParamCount) {
    const std::string cfg = std::string(FCSTUB_SOURCE_DIR) + "/config/default.yaml";
    const RunResult v = run("--config " + cfg + " --validate");
    EXPECT_EQ(v.exit_code, 0);
    EXPECT_EQ(v.out, "config OK (42 parameters)\n");
    EXPECT_EQ(run("--config " + cfg + " --print-param-count").out, "42\n");
}

TEST(Cli, SimPrintsTheGoldenHash) {
    const auto out = std::filesystem::temp_directory_path() / "fcstub_it_sim";
    const RunResult r =
        run("--config " + kFixtures + "golden_all_faults.yaml --sim --out " + out.string());
    EXPECT_EQ(r.exit_code, 0);
    EXPECT_EQ(r.out, "sha256 4705f2d9a96716391717d09b1e2bc4787af82e7a393670aa4a51656f1044ab2d\n");
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
