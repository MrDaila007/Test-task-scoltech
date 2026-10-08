// FcCore: handling of decoded incoming messages (setpoints, commands, TIMESYNC).

#include "fcstub/fc_core.hpp"

#include <cmath>
#include <cstdio>
#include <type_traits>

namespace fcstub {

namespace {

const char* deny_text(DenyReason reason) noexcept {
    switch (reason) {
        case DenyReason::NotReady:
            return "Denied: not ready";
        case DenyReason::Disarmed:
            return "Denied: arm first";
        case DenyReason::NoSetpointStream:
            return "Denied: OFFBOARD needs setpoints";
        case DenyReason::NotRequestable:
            return "Denied: mode not supported";
        case DenyReason::Overridden:
            return "Denied: autopilot override";
        case DenyReason::InAir:
            return "Denied: in air (force 21196)";
        case DenyReason::None:
            break;
    }
    return "Denied";
}

// PX4 (main, sub) custom mode -> stub mode; false if not one of ours.
bool mode_from_px4(float main_f, float sub_f, Mode& mode) noexcept {
    const long main_mode = std::lround(main_f);
    const long sub_mode = std::lround(sub_f);
    if (main_mode == kPx4MainManual && sub_mode == 0) {
        mode = Mode::Manual;
    } else if (main_mode == kPx4MainOffboard && sub_mode == 0) {
        mode = Mode::Offboard;
    } else if (main_mode == kPx4MainAuto && sub_mode == kPx4SubAutoLoiter) {
        mode = Mode::Hold;
    } else {
        return false;
    }
    return true;
}

}  // namespace

bool FcCore::addressed_to_us(std::uint8_t target_system,
                             std::uint8_t target_component) const noexcept {
    return (target_system == 0 || target_system == cfg_.identity.system_id) &&
           (target_component == 0 || target_component == cfg_.identity.component_id);
}

void FcCore::on_message(const RxMessage& msg) noexcept {
    std::visit(
        [this](const auto& m) {
            using T = std::decay_t<decltype(m)>;
            if constexpr (std::is_same_v<T, SetpointMsg>) {
                handle_setpoint(m);
            } else if constexpr (std::is_same_v<T, CommandLongMsg>) {
                handle_command(m);
            } else if constexpr (std::is_same_v<T, TimesyncMsg>) {
                handle_timesync(m);
            } else if constexpr (std::is_same_v<T, ParamRequestListMsg>) {
                if (addressed_to_us(m.target_system, m.target_component)) {
                    for (std::size_t i = 0; i < kParamCount; ++i) {
                        send_param(static_cast<Param>(i));
                    }
                }
            } else if constexpr (std::is_same_v<T, ParamRequestReadMsg>) {
                const auto p = m.index >= 0 ? param_at(m.index) : find_param(m.id);
                if (p && addressed_to_us(m.target_system, m.target_component)) {
                    send_param(*p);  // unknown names get no answer, as on PX4
                }
            } else if constexpr (std::is_same_v<T, ParamSetMsg>) {
                handle_param_set(m);
            } else if constexpr (std::is_same_v<T, MissionRequestListMsg> ||
                                 std::is_same_v<T, MissionCountMsg>) {
                handle_mission(m);
            }
        },
        msg);
}

void FcCore::handle_setpoint(const SetpointMsg& msg) noexcept {
    const GateOutcome out = gate_.submit(rx_now_, msg);
    if (out.reason == RejectReason::None || out.reason == RejectReason::NotForUs) {
        return;
    }
    const TimeNs interval = ms_to_ns(cfg_.modes.invalid_text_interval_ms);
    if (!reject_text_sent_ || rx_now_ - last_reject_text_ >= interval) {
        reject_text_sent_ = true;
        last_reject_text_ = rx_now_;
        char text[51];
        std::snprintf(text, sizeof(text), "Setpoint rejected: %s", reject_reason_name(out.reason));
        send_text(kSeverityWarning, text);
    }
}

ModeChange FcCore::execute_command(const CommandLongMsg& msg) noexcept {
    if (msg.command == kCmdComponentArmDisarm) {
        const long arg = std::lround(msg.param[0]);
        if (arg != 0 && arg != 1) {
            return {AckResult::Denied, DenyReason::None};
        }
        // PX4 refuses a disarm in the air unless it is forced with the magic number.
        constexpr long kForceDisarm = 21196;
        if (arg == 0 && modes_.armed() && in_air() && std::lround(msg.param[1]) != kForceDisarm) {
            return {AckResult::Denied, DenyReason::InAir};
        }
        return modes_.request_arm(arg == 1, rx_now_);
    }
    if (msg.command == kCmdDoSetMode) {
        Mode target{};
        const auto base_mode = static_cast<long>(std::lround(msg.param[0]));
        if ((base_mode & kModeFlagCustomModeEnabled) == 0 ||
            !mode_from_px4(msg.param[1], msg.param[2], target)) {
            return {AckResult::Unsupported, DenyReason::NotRequestable};
        }
        const FaultWindow* override = schedule_.active(FaultType::ModeOverride, rx_now_);
        if (target == Mode::Offboard && override != nullptr) {
            return {AckResult::Denied, DenyReason::Overridden};
        }
        return modes_.request_mode(target, rx_now_, gate_.last_valid_time());
    }
    return {AckResult::Unsupported, DenyReason::None};
}

void FcCore::handle_command(const CommandLongMsg& msg) noexcept {
    if (!addressed_to_us(msg.target_system, msg.target_component)) {
        return;
    }
    const CommandKey key{msg.source_system, msg.source_component, msg.command};
    AckResult result{};
    if (const auto cached = dedup_.lookup(key, msg.confirmation, rx_now_)) {
        result = *cached;
    } else {
        const Mode before = modes_.mode();
        const ModeChange change = execute_command(msg);
        result = change.result;
        dedup_.remember(key, rx_now_, result);
        if (change.result == AckResult::Accepted) {
            announce_mode(before);
        } else if (change.reason == DenyReason::Overridden) {
            const FaultWindow* w = schedule_.active(FaultType::ModeOverride, rx_now_);
            const bool pilot = w != nullptr &&
                               params_of<ModeOverrideParams>(*w).cause == OverrideCause::RcOverride;
            send_text(kSeverityWarning,
                      pilot ? "Denied: pilot has control" : "Denied: failsafe active");
        } else if (change.reason != DenyReason::None) {
            send_text(kSeverityWarning, deny_text(change.reason));
        }
    }
    send(encoder_.command_ack(
        {msg.command, static_cast<std::uint8_t>(result), msg.source_system, msg.source_component}));
}

void FcCore::send_param(Param p) noexcept {
    send(encoder_.param_value({param_name(p), param_wire_value(p, cfg_), param_type(p),
                               static_cast<std::uint16_t>(kParamCount),
                               static_cast<std::uint16_t>(p)}));
}

void FcCore::handle_param_set(const ParamSetMsg& msg) noexcept {
    const auto p = find_param(msg.id);
    if (!p || !addressed_to_us(msg.target_system, msg.target_component)) {
        return;
    }
    if (param_set(*p, msg.value, msg.type, cfg_)) {
        modes_.set_offboard_timeout(ms_to_ns(cfg_.modes.offboard_timeout_ms));
    }
    send_param(*p);  // the current value either way, as PX4 answers a PARAM_SET
}

// The stub flies no missions: the plan is always empty and uploads are refused,
// so ground stations and SDKs that ask do not wait for a timeout.
void FcCore::handle_mission(const MissionRequestListMsg& msg) noexcept {
    if (addressed_to_us(msg.target_system, msg.target_component)) {
        send(encoder_.mission_count_empty(
            {msg.mission_type, msg.source_system, msg.source_component}));
    }
}

void FcCore::handle_mission(const MissionCountMsg& msg) noexcept {
    if (addressed_to_us(msg.target_system, msg.target_component)) {
        send(encoder_.mission_ack({msg.mission_type, msg.source_system, msg.source_component},
                                  kMissionUnsupported));
    }
}

void FcCore::handle_timesync(const TimesyncMsg& msg) noexcept {
    if (!addressed_to_us(msg.target_system, msg.target_component)) {
        return;
    }
    if (msg.tc1 != 0) {  // a reply: ours only if it echoes the open request
        if (!timesync_open_ || msg.ts1 != timesync_ts1_) {
            return;
        }
        const std::int64_t now = clock_.fc_ns(rx_now_);
        const std::int64_t rtt = now - msg.ts1;
        // The onboard clock read tc1 halfway through the round trip.
        const std::int64_t offset = msg.tc1 - (msg.ts1 + rtt / 2);
        stats_.timesync_offset_ns =
            stats_.timesync_samples == 0
                ? offset
                : stats_.timesync_offset_ns + (offset - stats_.timesync_offset_ns) / 8;
        stats_.timesync_rtt_ns = rtt;
        ++stats_.timesync_samples;
        timesync_open_ = false;
        return;
    }
    send(encoder_.timesync(
        {clock_.fc_ns(rx_now_), msg.ts1, msg.source_system, msg.source_component}));
}

}  // namespace fcstub
