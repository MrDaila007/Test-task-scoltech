#include "fcstub/config_loader.hpp"

#include <gtest/gtest.h>

#include <string>
#include <variant>

namespace {

using fcstub::ConfigError;
using fcstub::load_config_from_string;

// Expects a ConfigError whose key path equals `path`.
void expect_error_at(const std::string& yaml, const std::string& path) {
    try {
        (void)load_config_from_string(yaml);
        FAIL() << "expected ConfigError at " << path;
    } catch (const ConfigError& e) {
        EXPECT_EQ(e.key_path(), path) << e.what();
    }
}

const std::string kMinimal = "schema_version: 1\n";

}  // namespace

TEST(Config, MinimalDocumentGetsContractDefaults) {
    const fcstub::Config cfg = load_config_from_string(kMinimal);
    EXPECT_EQ(cfg.run.mode, fcstub::RunMode::Realtime);
    EXPECT_EQ(cfg.run.seed, 1U);
    EXPECT_EQ(cfg.link.bind_port, 14580);
    EXPECT_EQ(cfg.link.remote_addr, "127.0.0.1");
    EXPECT_EQ(cfg.link.remote_port, 14540);
    EXPECT_EQ(cfg.identity.system_id, 1);
    EXPECT_DOUBLE_EQ(cfg.telemetry_hz.attitude, 50.0);
    EXPECT_EQ(cfg.modes.offboard_timeout_ms, 500);
    EXPECT_DOUBLE_EQ(cfg.vehicle.origin.lat_deg, 55.7558);
    EXPECT_EQ(cfg.vehicle.dt_ms, 4);
    EXPECT_EQ(cfg.battery.cells, 6);
    EXPECT_TRUE(cfg.realtime.sched_fifo);
    EXPECT_EQ(cfg.realtime.spin_us, 0);
    EXPECT_EQ(cfg.sim.out_dir, "out");
    EXPECT_TRUE(cfg.faults.empty());
    EXPECT_TRUE(cfg.client.empty());
}

TEST(Config, SchemaVersionIsMandatoryAndPinned) {
    expect_error_at("run: {seed: 3}\n", "schema_version");
    expect_error_at("schema_version: 2\n", "schema_version");
}

TEST(Config, UnknownKeyIsRejectedWithItsPath) {
    expect_error_at(kMinimal + "link: {bogus: 1}\n", "link.bogus");
    expect_error_at(kMinimal + "extra: 1\n", "extra");
    expect_error_at(kMinimal + "faults:\n  - {type: link, start_s: 1, delay: 5}\n",
                    "faults[0].delay");
}

TEST(Config, WrongTypeIsRejected) {
    expect_error_at(kMinimal + "telemetry_hz: {attitude: fast}\n", "telemetry_hz.attitude");
    expect_error_at(kMinimal + "link: {bind_port: 1.5}\n", "link.bind_port");
    expect_error_at(kMinimal + "realtime: {lock_memory: maybe}\n", "realtime.lock_memory");
    expect_error_at(kMinimal + "link: {remote_addr: not-an-ip}\n", "link.remote_addr");
    expect_error_at(kMinimal + "run: {mode: turbo}\n", "run.mode");
}

TEST(Config, OutOfRangeIsRejected) {
    expect_error_at(kMinimal + "modes: {offboard_timeout_ms: 20}\n", "modes.offboard_timeout_ms");
    expect_error_at(kMinimal + "link: {bind_port: 70000}\n", "link.bind_port");
    expect_error_at(kMinimal + "battery: {soc_initial: 1.5}\n", "battery.soc_initial");
    expect_error_at(kMinimal + "vehicle: {origin: {lat_deg: 95}}\n", "vehicle.origin.lat_deg");
}

TEST(Config, SimModeNeedsPositiveDuration) {
    expect_error_at(kMinimal + "run: {mode: sim}\n", "run.duration_s");
    const auto cfg = load_config_from_string(kMinimal + "run: {mode: sim, duration_s: 10}\n");
    EXPECT_EQ(cfg.run.mode, fcstub::RunMode::Sim);
}

TEST(Config, AllFaultTypesParse) {
    const auto cfg = load_config_from_string(kMinimal + R"(
faults:
  - {type: estimator_freeze, start_s: 10, duration_s: 5, freeze_velocity: true}
  - {type: fc_reboot, start_s: 20, boot_ms: 2500}
  - {type: clock_fault, start_s: 30, duration_s: 10, drift_ppm: 100, step_ms: 50}
  - {type: link, start_s: 40, duration_s: 5, direction: both, p_good_to_bad: 0.01,
     p_bad_to_good: 0.1, loss_good: 0.0, loss_bad: 1.0, delay_ms: 100, jitter_ms: 50,
     byte_error_rate: 0.001}
  - {type: gnss, start_s: 50, duration_s: 5, kind: drift, drift_mps: [0.2, 0, 0]}
)");
    ASSERT_EQ(cfg.faults.size(), 5U);
    EXPECT_TRUE(std::get<fcstub::EstimatorFreezeParams>(cfg.faults[0].params).freeze_velocity);
    EXPECT_EQ(std::get<fcstub::FcRebootParams>(cfg.faults[1].params).boot_ms, 2500);
    EXPECT_DOUBLE_EQ(std::get<fcstub::ClockFaultParams>(cfg.faults[2].params).drift_ppm, 100.0);
    const auto& link = std::get<fcstub::LinkFaultParams>(cfg.faults[3].params);
    EXPECT_EQ(link.direction, fcstub::LinkDirection::Both);
    EXPECT_DOUBLE_EQ(link.jitter_ms, 50.0);
    const auto& gnss = std::get<fcstub::GnssFaultParams>(cfg.faults[4].params);
    EXPECT_EQ(gnss.kind, fcstub::GnssKind::Drift);
    EXPECT_DOUBLE_EQ(gnss.drift_mps[0], 0.2);
}

TEST(Config, FaultFieldsAreTypeSpecific) {
    expect_error_at(kMinimal + "faults:\n  - {type: fc_reboot, start_s: 1, duration_s: 2}\n",
                    "faults[0].duration_s");
    expect_error_at(
        kMinimal + "faults:\n  - {type: gnss, start_s: 1, kind: jump, drift_mps: [1, 0, 0]}\n",
        "faults[0].drift_mps");
    expect_error_at(kMinimal + "faults:\n  - {type: gnss, start_s: 1, offset_m: [1, 0]}\n",
                    "faults[0].offset_m");
    expect_error_at(kMinimal + "faults:\n  - {start_s: 1}\n", "faults[0].type");
    expect_error_at(kMinimal + "faults:\n  - {type: meteor, start_s: 1}\n", "faults[0].type");
}

TEST(Config, OverlappingWindowsOfSameTypeAreRejected) {
    expect_error_at(kMinimal + R"(
faults:
  - {type: link, start_s: 10, duration_s: 10}
  - {type: link, start_s: 15, duration_s: 10}
)",
                    "faults[1].start_s");
    // "until the end" (duration 0) overlaps everything after it.
    expect_error_at(kMinimal + R"(
faults:
  - {type: gnss, start_s: 10}
  - {type: gnss, start_s: 100, duration_s: 1}
)",
                    "faults[1].start_s");
    // Different types may overlap; same type back-to-back is fine.
    EXPECT_NO_THROW(load_config_from_string(kMinimal + R"(
faults:
  - {type: link, start_s: 10, duration_s: 10}
  - {type: link, start_s: 20, duration_s: 10}
  - {type: gnss, start_s: 12, duration_s: 10}
)"));
}

TEST(Config, ClientActionsParse) {
    const auto cfg = load_config_from_string(kMinimal + R"(
run: {mode: sim, duration_s: 60}
client:
  - {kind: command, at_s: 3, cmd: arm}
  - {kind: setpoints, start_s: 4, end_s: 55, rate_hz: 20, frame: velocity, value: [2, 0, 0]}
  - {kind: timesync, start_s: 0, end_s: 60, rate_hz: 1}
)");
    ASSERT_EQ(cfg.client.size(), 3U);
    EXPECT_EQ(cfg.client[0].kind, fcstub::ClientKind::Command);
    EXPECT_EQ(cfg.client[0].cmd, fcstub::ClientCommand::Arm);
    EXPECT_EQ(cfg.client[1].frame, fcstub::SetpointFrame::Velocity);
    EXPECT_DOUBLE_EQ(cfg.client[1].value[0], 2.0);
    EXPECT_DOUBLE_EQ(cfg.client[2].rate_hz, 1.0);
}

TEST(Config, ClientActionsValidated) {
    expect_error_at(kMinimal +
                        "client:\n  - {kind: setpoints, start_s: 5, end_s: 4, rate_hz: 20, "
                        "frame: velocity, value: [0, 0, 0]}\n",
                    "client[0].end_s");
    expect_error_at(kMinimal + "client:\n  - {kind: command, at_s: 1, cmd: fly}\n",
                    "client[0].cmd");
    expect_error_at(kMinimal + "client:\n  - {kind: command, at_s: 1, cmd: arm, rate_hz: 2}\n",
                    "client[0].rate_hz");
}

TEST(Config, ListLengthLimits) {
    std::string yaml = kMinimal + "faults:\n";
    for (int i = 0; i < 17; ++i) {
        yaml += "  - {type: estimator_freeze, start_s: " + std::to_string(i * 10) +
                ", duration_s: 1}\n";
    }
    expect_error_at(yaml, "faults");
}

TEST(Config, ParamCountCountsEveryLeafIncludingVectorElements) {
    EXPECT_EQ(fcstub::count_leaf_params_in_string(kMinimal), 1U);
    EXPECT_EQ(fcstub::count_leaf_params_in_string(
                  kMinimal + "faults:\n  - {type: gnss, start_s: 1, offset_m: [1, 2, 3]}\n"),
              1U + 2U + 3U);
}

TEST(Config, MoreThan200ParamsIsRejected) {
    std::string yaml = kMinimal + "client:\n";
    // 1 + 32 actions * 7 leaves = 225 > 200, while each list stays within its own length limit.
    for (int i = 0; i < 32; ++i) {
        yaml +=
            "  - {kind: setpoints, start_s: 0, end_s: 1, rate_hz: 1, frame: velocity, "
            "value: [0, 0, 0]}\n";
    }
    expect_error_at(yaml, "");
}

TEST(Config, DefaultYamlIsValidAndWithinBudget) {
    const auto cfg = fcstub::load_config(std::string(FCSTUB_SOURCE_DIR) + "/config/default.yaml");
    EXPECT_EQ(cfg.modes.offboard_timeout_ms, 500);
    EXPECT_LE(fcstub::count_leaf_params(std::string(FCSTUB_SOURCE_DIR) + "/config/default.yaml"),
              200U);
}

TEST(Config, MissingFileIsAConfigError) {
    EXPECT_THROW((void)fcstub::load_config("/nonexistent/fcstub.yaml"), ConfigError);
}

TEST(Config, EdgeCasesAreRejected) {
    expect_error_at(kMinimal + "run: {seed: -1}\n", "run.seed");
    expect_error_at(kMinimal + "vehicle: {tau_s: .nan}\n", "vehicle.tau_s");
    expect_error_at(kMinimal + "vehicle: {tau_s: .inf}\n", "vehicle.tau_s");
    expect_error_at(kMinimal + "link: [1, 2]\n", "link");
    expect_error_at(kMinimal + "faults: {type: link}\n", "faults");
    expect_error_at("schema_version: 1\nrun: {mode: [sim]}\n", "run.mode");
    expect_error_at("- just\n- a list\n", "");
}

TEST(Config, ErrorMessageNamesThePath) {
    try {
        (void)load_config_from_string(kMinimal + "telemetry_hz: {attitude: 900}\n");
        FAIL();
    } catch (const ConfigError& e) {
        EXPECT_EQ(std::string(e.what()), "config: telemetry_hz.attitude: out of range [1, 250]");
    }
}

TEST(Config, EstimatorNoiseParsesAndDefaultsToZero) {
    const auto plain = load_config_from_string(kMinimal);
    EXPECT_DOUBLE_EQ(plain.estimator.pos_noise_m, 0.0);
    EXPECT_DOUBLE_EQ(plain.estimator.vel_noise_mps, 0.0);
    EXPECT_DOUBLE_EQ(plain.estimator.att_noise_rad, 0.0);
    const auto cfg = load_config_from_string(
        kMinimal +
        "estimator: {pos_noise_m: 0.3, vel_noise_mps: 0.05, att_noise_rad: 0.005, "
        "noise_tau_s: 2}\n");
    EXPECT_DOUBLE_EQ(cfg.estimator.pos_noise_m, 0.3);
    EXPECT_DOUBLE_EQ(cfg.estimator.vel_noise_mps, 0.05);
    EXPECT_DOUBLE_EQ(cfg.estimator.att_noise_rad, 0.005);
    EXPECT_DOUBLE_EQ(cfg.estimator.noise_tau_s, 2.0);
    expect_error_at(kMinimal + "estimator: {pos_noise_m: -1}\n", "estimator.pos_noise_m");
    expect_error_at(kMinimal + "estimator: {noise_tau_s: 0}\n", "estimator.noise_tau_s");
}
