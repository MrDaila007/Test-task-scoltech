#pragma once

// Run configuration, schema version 1. Built once by the loader at start-up and
// treated as read-only afterwards. Defaults here are the documented defaults of
// config/default.yaml.

#include <array>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace fcstub {

using Vec3 = std::array<double, 3>;  // NED: north, east, down

enum class RunMode { Realtime, Sim };

struct RunConfig {
    RunMode mode = RunMode::Realtime;
    std::uint64_t seed = 1;
    double duration_s = 0.0;  // 0 = until signal (realtime only)
};

struct LinkConfig {
    std::string bind_addr = "0.0.0.0";
    std::uint16_t bind_port = 14580;
    std::string remote_addr = "127.0.0.1";
    std::uint16_t remote_port = 14540;
    int rx_max_frames_per_iter = 64;
    int queue_capacity = 512;  // per direction, for the link fault model
};

struct IdentityConfig {
    std::uint8_t system_id = 1;
    std::uint8_t component_id = 1;
};

struct TelemetryRates {
    double heartbeat = 1.0;
    double attitude = 50.0;
    double global_position = 10.0;
    double battery = 2.0;
};

struct ModesConfig {
    int ready_after_ms = 2000;
    int offboard_timeout_ms = 500;
    int invalid_text_interval_ms = 1000;
};

struct GeoOrigin {
    double lat_deg = 55.7558;
    double lon_deg = 37.6173;
    double alt_msl_m = 150.0;
};

struct VehicleConfig {
    GeoOrigin origin;
    int dt_ms = 4;
    double tau_s = 0.3;
    double v_max_mps = 12.0;
    double pos_gain = 1.0;
    double geofence_m = 1000.0;
    double geofence_alt_m = 500.0;
};

// Estimator noise: first-order Gauss-Markov error on the reported state, the way
// an EKF output wanders around the truth. Zero = perfect estimate.
struct EstimatorNoise {
    double pos_noise_m = 0.0;    // 1-sigma per axis
    double vel_noise_mps = 0.0;  // 1-sigma per axis
    double att_noise_rad = 0.0;  // 1-sigma per angle
    double noise_tau_s = 1.0;    // correlation time
};

struct BatteryConfig {
    int cells = 6;
    double capacity_mah = 10000.0;
    double r_int_ohm = 0.02;
    double i_idle_a = 0.5;
    double i_hover_a = 18.0;
    double k_speed = 0.05;  // A per (m/s)^2
    double soc_initial = 1.0;
};

struct RealtimeConfig {
    bool lock_memory = true;
    bool sched_fifo = true;
    int priority = 80;
    int spin_us = 0;
    std::string report_path;  // empty = stderr only
};

struct SimConfig {
    std::string out_dir = "out";
    double truth_csv_hz = 10.0;  // 0 = do not write truth.csv
};

// --- faults ------------------------------------------------------------------

enum class FaultType { EstimatorFreeze, FcReboot, ClockFault, Link, Gnss };
enum class LinkDirection { Down, Up, Both };
enum class GnssKind { Jump, Drift };

struct EstimatorFreezeParams {
    bool freeze_velocity = false;
};

struct FcRebootParams {
    int boot_ms = 3000;
};

struct ClockFaultParams {
    double drift_ppm = 0.0;
    double step_ms = 0.0;
};

struct LinkFaultParams {
    LinkDirection direction = LinkDirection::Down;
    double p_good_to_bad = 0.0;
    double p_bad_to_good = 1.0;
    double loss_good = 0.0;
    double loss_bad = 1.0;
    double delay_ms = 0.0;
    double jitter_ms = 0.0;
    double byte_error_rate = 0.0;
};

struct GnssFaultParams {
    GnssKind kind = GnssKind::Jump;
    Vec3 offset_m{};
    Vec3 drift_mps{};
};

using FaultParams = std::variant<EstimatorFreezeParams, FcRebootParams, ClockFaultParams,
                                 LinkFaultParams, GnssFaultParams>;

struct FaultSpec {
    FaultType type = FaultType::EstimatorFreeze;
    double start_s = 0.0;
    double duration_s = 0.0;  // 0 = until the end of the run; fc_reboot uses boot_ms instead
    FaultParams params;
};

// --- virtual client (sim mode) ---------------------------------------------------

enum class ClientKind { Command, Setpoints, Timesync };
enum class ClientCommand { Arm, Disarm, Manual, Offboard, Hold };
enum class SetpointFrame { Velocity, Position };

struct ClientAction {
    ClientKind kind = ClientKind::Command;
    double at_s = 0.0;  // command
    ClientCommand cmd = ClientCommand::Arm;
    double start_s = 0.0;  // setpoints, timesync
    double end_s = 0.0;
    double rate_hz = 1.0;
    SetpointFrame frame = SetpointFrame::Velocity;  // setpoints
    Vec3 value{};
};

// --- whole document --------------------------------------------------------------

inline constexpr int kSchemaVersion = 1;
inline constexpr std::size_t kMaxLeafParams = 200;
inline constexpr std::size_t kMaxFaults = 16;
inline constexpr std::size_t kMaxClientActions = 32;

struct Config {
    RunConfig run;
    LinkConfig link;
    IdentityConfig identity;
    TelemetryRates telemetry_hz;
    ModesConfig modes;
    VehicleConfig vehicle;
    EstimatorNoise estimator;
    BatteryConfig battery;
    RealtimeConfig realtime;
    SimConfig sim;
    std::vector<FaultSpec> faults;
    std::vector<ClientAction> client;
};

}  // namespace fcstub
