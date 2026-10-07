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
        return modes_.request_arm(arg == 1, rx_now_);
    }
    if (msg.command == kCmdDoSetMode) {
        Mode target{};
        const auto base_mode = static_cast<long>(std::lround(msg.param[0]));
        if ((base_mode & kModeFlagCustomModeEnabled) == 0 ||
            !mode_from_px4(msg.param[1], msg.param[2], target)) {
            return {AckResult::Unsupported, DenyReason::NotRequestable};
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
        } else if (change.reason != DenyReason::None) {
            send_text(kSeverityWarning, deny_text(change.reason));
        }
    }
    send(encoder_.command_ack({msg.command, static_cast<std::uint8_t>(result), msg.source_system,
                               msg.source_component}));
}

void FcCore::handle_timesync(const TimesyncMsg& msg) noexcept {
    if (msg.tc1 != 0 || !addressed_to_us(msg.target_system, msg.target_component)) {
        return;  // a response from someone else, or not for us
    }
    send(encoder_.timesync(
        {clock_.fc_ns(rx_now_), msg.ts1, msg.source_system, msg.source_component}));
}

}  // namespace fcstub
