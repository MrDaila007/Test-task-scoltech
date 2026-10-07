#include "fcstub/config_loader.hpp"

#include "config_parts.hpp"
#include "yaml_section.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>

namespace fcstub {

ConfigError::ConfigError(std::string key_path, const std::string& reason)
    : std::runtime_error("config: " + (key_path.empty() ? std::string("<document>") : key_path) +
                         ": " + reason),
      key_path_(std::move(key_path)) {}

namespace {

using detail::EnumNames;
using detail::Section;

void parse_run(Section s, RunConfig& run) {
    s.enumeration("mode", run.mode,
                  EnumNames<RunMode>{{"realtime", RunMode::Realtime}, {"sim", RunMode::Sim}});
    s.number("seed", run.seed, std::uint64_t{0}, std::numeric_limits<std::uint64_t>::max());
    s.number("duration_s", run.duration_s, 0.0, 86400.0);
    s.finish();
}

void parse_link(Section s, LinkConfig& link) {
    s.ipv4("bind_addr", link.bind_addr);
    s.number("bind_port", link.bind_port, std::uint16_t{1}, std::uint16_t{65535});
    s.ipv4("remote_addr", link.remote_addr);
    s.number("remote_port", link.remote_port, std::uint16_t{1}, std::uint16_t{65535});
    s.number("rx_max_frames_per_iter", link.rx_max_frames_per_iter, 1, 1024);
    s.number("queue_capacity", link.queue_capacity, 16, 4096);
    s.finish();
}

void parse_identity_and_rates(Section& root, Config& cfg) {
    Section id = root.child("identity");
    id.number("system_id", cfg.identity.system_id, std::uint8_t{1}, std::uint8_t{255});
    id.number("component_id", cfg.identity.component_id, std::uint8_t{1}, std::uint8_t{255});
    id.finish();

    Section hz = root.child("telemetry_hz");
    hz.number("heartbeat", cfg.telemetry_hz.heartbeat, 0.1, 10.0);
    hz.number("attitude", cfg.telemetry_hz.attitude, 1.0, 250.0);
    hz.number("global_position", cfg.telemetry_hz.global_position, 1.0, 50.0);
    hz.number("battery", cfg.telemetry_hz.battery, 0.1, 10.0);
    hz.finish();

    Section modes = root.child("modes");
    modes.number("ready_after_ms", cfg.modes.ready_after_ms, 0, 60000);
    modes.number("offboard_timeout_ms", cfg.modes.offboard_timeout_ms, 50, 10000);
    modes.number("invalid_text_interval_ms", cfg.modes.invalid_text_interval_ms, 100, 60000);
    modes.finish();
}

void parse_vehicle(Section s, VehicleConfig& v) {
    Section origin = s.child("origin");
    origin.number("lat_deg", v.origin.lat_deg, -89.0, 89.0);
    origin.number("lon_deg", v.origin.lon_deg, -180.0, 180.0);
    origin.number("alt_msl_m", v.origin.alt_msl_m, -500.0, 9000.0);
    origin.finish();
    s.number("dt_ms", v.dt_ms, 1, 20);
    s.number("tau_s", v.tau_s, 0.01, 5.0);
    s.number("v_max_mps", v.v_max_mps, 0.1, 50.0);
    s.number("pos_gain", v.pos_gain, 0.01, 10.0);
    s.number("geofence_m", v.geofence_m, 1.0, 100000.0);
    s.number("geofence_alt_m", v.geofence_alt_m, 1.0, 10000.0);
    s.finish();
}

void parse_battery(Section s, BatteryConfig& b) {
    s.number("cells", b.cells, 1, 14);
    s.number("capacity_mah", b.capacity_mah, 100.0, 100000.0);
    s.number("r_int_ohm", b.r_int_ohm, 0.0, 1.0);
    s.number("i_idle_a", b.i_idle_a, 0.0, 10.0);
    s.number("i_hover_a", b.i_hover_a, 0.0, 200.0);
    s.number("k_speed", b.k_speed, 0.0, 10.0);
    s.number("soc_initial", b.soc_initial, 0.0, 1.0);
    s.finish();
}

void parse_runtime_sections(Section& root, Config& cfg) {
    Section rt = root.child("realtime");
    rt.boolean("lock_memory", cfg.realtime.lock_memory);
    rt.boolean("sched_fifo", cfg.realtime.sched_fifo);
    rt.number("priority", cfg.realtime.priority, 1, 99);
    rt.number("spin_us", cfg.realtime.spin_us, 0, 2000);
    rt.text("report_path", cfg.realtime.report_path);
    rt.finish();

    Section sim = root.child("sim");
    sim.text("out_dir", cfg.sim.out_dir);
    sim.number("truth_csv_hz", cfg.sim.truth_csv_hz, 0.0, 250.0);
    sim.finish();
}

std::size_t count_leaves(const YAML::Node& node) {
    if (node.IsMap()) {
        std::size_t n = 0;
        for (const auto& kv : node) {
            n += count_leaves(kv.second);
        }
        return n;
    }
    if (node.IsSequence()) {
        std::size_t n = 0;
        for (const auto& item : node) {
            n += count_leaves(item);
        }
        return n;
    }
    return 1;
}

void check_schema_version(Section& root) {
    root.require("schema_version");
    int version = 0;
    root.number("schema_version", version, 0, 1000);
    if (version != kSchemaVersion) {
        throw ConfigError("schema_version", "unsupported version " + std::to_string(version) +
                                                ", expected " + std::to_string(kSchemaVersion));
    }
}

Config parse_document(const YAML::Node& doc) {
    const std::size_t leaves = count_leaves(doc);
    if (leaves > kMaxLeafParams) {
        throw ConfigError("", std::to_string(leaves) + " parameters, at most " +
                                  std::to_string(kMaxLeafParams) + " allowed");
    }
    Section root(doc, "");
    check_schema_version(root);
    Config cfg;
    parse_run(root.child("run"), cfg.run);
    parse_link(root.child("link"), cfg.link);
    parse_identity_and_rates(root, cfg);
    parse_vehicle(root.child("vehicle"), cfg.vehicle);
    parse_battery(root.child("battery"), cfg.battery);
    parse_runtime_sections(root, cfg);
    cfg.faults = detail::parse_faults(root);
    cfg.client = detail::parse_client(root);
    root.finish();
    validate_config(cfg);
    return cfg;
}

YAML::Node parse_yaml(const std::string& yaml) {
    try {
        return YAML::Load(yaml);
    } catch (const YAML::Exception& e) {
        throw ConfigError("", std::string("YAML syntax: ") + e.what());
    }
}

YAML::Node parse_yaml_file(const std::string& path) {
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::BadFile&) {
        throw ConfigError("", "cannot read file '" + path + "'");
    } catch (const YAML::Exception& e) {
        throw ConfigError("", std::string("YAML syntax in '") + path + "': " + e.what());
    }
}

// End of a fault window in seconds; +inf for "until the end of the run".
double window_end(const FaultSpec& f) {
    if (f.type == FaultType::FcReboot) {
        return f.start_s + std::get<FcRebootParams>(f.params).boot_ms / 1000.0;
    }
    return f.duration_s > 0.0 ? f.start_s + f.duration_s : std::numeric_limits<double>::infinity();
}

}  // namespace

void validate_config(const Config& cfg) {
    if (cfg.run.mode == RunMode::Sim && cfg.run.duration_s <= 0.0) {
        throw ConfigError("run.duration_s", "sim mode needs a positive duration");
    }
    for (std::size_t j = 0; j < cfg.faults.size(); ++j) {
        for (std::size_t i = 0; i < j; ++i) {
            const FaultSpec& a = cfg.faults[i];
            const FaultSpec& b = cfg.faults[j];
            if (a.type == b.type && b.start_s < window_end(a) && a.start_s < window_end(b)) {
                throw ConfigError(detail::index_path("faults", j) + ".start_s",
                                  "overlaps faults[" + std::to_string(i) + "] of the same type");
            }
        }
    }
}

Config load_config(const std::string& path) { return parse_document(parse_yaml_file(path)); }

Config load_config_from_string(const std::string& yaml) { return parse_document(parse_yaml(yaml)); }

std::size_t count_leaf_params(const std::string& path) {
    return count_leaves(parse_yaml_file(path));
}

std::size_t count_leaf_params_in_string(const std::string& yaml) {
    return count_leaves(parse_yaml(yaml));
}

}  // namespace fcstub
