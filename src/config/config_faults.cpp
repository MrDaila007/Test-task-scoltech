// faults[] and client[] sections. Each entry is a tagged mapping: the tag
// (`type` / `kind`) decides which keys are valid, and Section::finish() rejects
// the rest, so a key that belongs to another variant is an error, not ignored.

#include "config_parts.hpp"

namespace fcstub::detail {

namespace {

constexpr double kMaxTimeS = 1.0e6;

const EnumNames<FaultType> kFaultTypes = {{"estimator_freeze", FaultType::EstimatorFreeze},
                                          {"fc_reboot", FaultType::FcReboot},
                                          {"clock_fault", FaultType::ClockFault},
                                          {"link", FaultType::Link},
                                          {"gnss", FaultType::Gnss},
                                          {"battery", FaultType::Battery},
                                          {"mode_override", FaultType::ModeOverride}};
const EnumNames<OverrideTarget> kOverrideTargets = {{"hold", OverrideTarget::Hold},
                                                    {"manual", OverrideTarget::Manual}};
const EnumNames<OverrideCause> kOverrideCauses = {{"rc_override", OverrideCause::RcOverride},
                                                  {"low_battery", OverrideCause::LowBattery},
                                                  {"geofence", OverrideCause::Geofence}};
const EnumNames<LinkDirection> kDirections = {
    {"down", LinkDirection::Down}, {"up", LinkDirection::Up}, {"both", LinkDirection::Both}};
const EnumNames<GnssKind> kGnssKinds = {{"jump", GnssKind::Jump}, {"drift", GnssKind::Drift}};
const EnumNames<ClientKind> kClientKinds = {{"command", ClientKind::Command},
                                            {"setpoints", ClientKind::Setpoints},
                                            {"timesync", ClientKind::Timesync}};
const EnumNames<ClientCommand> kCommands = {{"arm", ClientCommand::Arm},
                                            {"disarm", ClientCommand::Disarm},
                                            {"manual", ClientCommand::Manual},
                                            {"offboard", ClientCommand::Offboard},
                                            {"hold", ClientCommand::Hold}};
const EnumNames<SetpointFrame> kFrames = {{"velocity", SetpointFrame::Velocity},
                                          {"position", SetpointFrame::Position}};

LinkFaultParams parse_link_fault(Section& s) {
    LinkFaultParams p;
    s.enumeration("direction", p.direction, kDirections);
    s.number("p_good_to_bad", p.p_good_to_bad, 0.0, 1.0);
    s.number("p_bad_to_good", p.p_bad_to_good, 0.0, 1.0);
    s.number("loss_good", p.loss_good, 0.0, 1.0);
    s.number("loss_bad", p.loss_bad, 0.0, 1.0);
    s.number("delay_ms", p.delay_ms, 0.0, 10000.0);
    s.number("jitter_ms", p.jitter_ms, 0.0, 10000.0);
    s.number("byte_error_rate", p.byte_error_rate, 0.0, 0.1);
    return p;
}

GnssFaultParams parse_gnss_fault(Section& s) {
    GnssFaultParams p;
    s.enumeration("kind", p.kind, kGnssKinds);
    if (p.kind == GnssKind::Jump) {
        s.vec3("offset_m", p.offset_m, -1.0e5, 1.0e5);
    } else {
        s.vec3("drift_mps", p.drift_mps, -100.0, 100.0);
    }
    return p;
}

FaultParams parse_fault_params(FaultType type, Section& s, FaultSpec& spec) {
    if (type != FaultType::FcReboot) {
        s.number("duration_s", spec.duration_s, 0.0, kMaxTimeS);
    }
    switch (type) {
        case FaultType::EstimatorFreeze: {
            EstimatorFreezeParams p;
            s.boolean("freeze_velocity", p.freeze_velocity);
            return p;
        }
        case FaultType::FcReboot: {
            FcRebootParams p;
            s.number("boot_ms", p.boot_ms, 0, 60000);
            return p;
        }
        case FaultType::ClockFault: {
            ClockFaultParams p;
            s.number("drift_ppm", p.drift_ppm, -10000.0, 10000.0);
            s.number("step_ms", p.step_ms, -1.0e6, 1.0e6);
            return p;
        }
        case FaultType::Link:
            return parse_link_fault(s);
        case FaultType::Gnss:
            return parse_gnss_fault(s);
        case FaultType::Battery: {
            BatteryFaultParams p;
            s.number("capacity_factor", p.capacity_factor, 0.05, 1.0);
            s.number("r_int_factor", p.r_int_factor, 1.0, 50.0);
            return p;
        }
        case FaultType::ModeOverride: {
            ModeOverrideParams p;
            s.enumeration("to", p.to, kOverrideTargets);
            s.enumeration("cause", p.cause, kOverrideCauses);
            return p;
        }
    }
    return EstimatorFreezeParams{};
}

FaultSpec parse_fault(const YAML::Node& node, const std::string& path) {
    Section s(node, path);
    s.require("type");
    FaultSpec spec;
    s.enumeration("type", spec.type, kFaultTypes);
    s.number("start_s", spec.start_s, 0.0, kMaxTimeS);
    spec.params = parse_fault_params(spec.type, s, spec);
    s.finish();
    return spec;
}

void parse_interval(Section& s, ClientAction& a) {
    s.number("start_s", a.start_s, 0.0, kMaxTimeS);
    s.number("end_s", a.end_s, 0.0, kMaxTimeS);
    s.number("rate_hz", a.rate_hz, 0.1, 100.0);
    if (a.end_s <= a.start_s) {
        throw ConfigError(join_path(s.path(), "end_s"), "must be greater than start_s");
    }
}

ClientAction parse_action(const YAML::Node& node, const std::string& path) {
    Section s(node, path);
    s.require("kind");
    ClientAction a;
    s.enumeration("kind", a.kind, kClientKinds);
    switch (a.kind) {
        case ClientKind::Command:
            s.require("cmd");
            s.number("at_s", a.at_s, 0.0, kMaxTimeS);
            s.enumeration("cmd", a.cmd, kCommands);
            break;
        case ClientKind::Setpoints:
            s.require("frame");
            s.require("value");
            parse_interval(s, a);
            s.enumeration("frame", a.frame, kFrames);
            s.vec3("value", a.value, -1.0e5, 1.0e5);
            break;
        case ClientKind::Timesync:
            parse_interval(s, a);
            break;
    }
    s.finish();
    return a;
}

}  // namespace

std::vector<FaultSpec> parse_faults(Section& root) {
    std::vector<FaultSpec> faults;
    const auto items = root.sequence("faults", kMaxFaults);
    for (std::size_t i = 0; i < items.size(); ++i) {
        faults.push_back(parse_fault(items[i], index_path("faults", i)));
    }
    return faults;
}

std::vector<ClientAction> parse_client(Section& root) {
    std::vector<ClientAction> actions;
    const auto items = root.sequence("client", kMaxClientActions);
    for (std::size_t i = 0; i < items.size(); ++i) {
        actions.push_back(parse_action(items[i], index_path("client", i)));
    }
    return actions;
}

}  // namespace fcstub::detail
