#include "fcstub/virtual_client.hpp"

#include <algorithm>

namespace fcstub {

namespace {

constexpr std::uint16_t kMaskVelocity = 3527;
constexpr std::uint16_t kMaskPosition = 3576;

std::array<float, 7> command_params(ClientCommand cmd) noexcept {
    const auto custom = static_cast<float>(kModeFlagCustomModeEnabled);
    switch (cmd) {
        case ClientCommand::Arm:
            return {1.0F, 0, 0, 0, 0, 0, 0};
        case ClientCommand::Disarm:
            return {0.0F, 0, 0, 0, 0, 0, 0};
        case ClientCommand::Manual:
            return {custom, kPx4MainManual, 0, 0, 0, 0, 0};
        case ClientCommand::Offboard:
            return {custom, kPx4MainOffboard, 0, 0, 0, 0, 0};
        case ClientCommand::Hold:
            return {custom, kPx4MainAuto, kPx4SubAutoLoiter, 0, 0, 0, 0};
    }
    return {};
}

std::uint16_t command_id(ClientCommand cmd) noexcept {
    return cmd == ClientCommand::Arm || cmd == ClientCommand::Disarm ? kCmdComponentArmDisarm
                                                                     : kCmdDoSetMode;
}

std::array<float, 3> to_float3(const Vec3& v) noexcept {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

}  // namespace

VirtualClient::VirtualClient(const std::vector<ClientAction>& actions, std::uint8_t target_system,
                             std::uint8_t target_component)
    : encoder_(kSystemId, kComponentId, target_system, target_component) {
    tracks_.reserve(actions.size());
    for (const ClientAction& a : actions) {
        if (a.kind == ClientKind::Command) {
            const TimeNs at = seconds_to_ns(a.at_s);
            tracks_.push_back(Track{a, at, at + 1, 0});
        } else {
            tracks_.push_back(Track{a, seconds_to_ns(a.start_s), seconds_to_ns(a.end_s),
                                    period_from_hz(a.rate_hz)});
        }
    }
}

TimeNs VirtualClient::next_event() const noexcept {
    TimeNs next = kNever;
    for (const Track& t : tracks_) {
        if (t.next < t.end) {
            next = std::min(next, t.next);
        }
    }
    return next;
}

FrameBuf VirtualClient::encode(const ClientAction& a, TimeNs t) noexcept {
    switch (a.kind) {
        case ClientKind::Command:
            return encoder_.command_long(command_id(a.cmd), command_params(a.cmd));
        case ClientKind::Setpoints: {
            const auto ms = static_cast<std::uint32_t>(t / kNsPerMs);
            if (a.frame == SetpointFrame::Velocity) {
                return encoder_.setpoint_local_ned(ms, kMaskVelocity, {}, to_float3(a.value));
            }
            return encoder_.setpoint_local_ned(ms, kMaskPosition, to_float3(a.value), {});
        }
        case ClientKind::Timesync:
            return encoder_.timesync_request(t);
    }
    return {};
}

void VirtualClient::emit(TimeNs t, ClientSink& sink) noexcept {
    for (Track& track : tracks_) {
        if (track.next != t || track.next >= track.end) {
            continue;
        }
        const FrameBuf frame = encode(track.action, t);
        sink.send(t, frame.data.data(), frame.len);
        ++track.count;
        // Grid from the start time, so rounding of the period never accumulates.
        track.next = track.period == 0 ? track.end
                                       : seconds_to_ns(track.action.start_s) +
                                             static_cast<TimeNs>(track.count) * track.period;
    }
}

}  // namespace fcstub
