#include "fcstub/mode_machine.hpp"

namespace fcstub {

namespace {

constexpr ModeChange kAccepted{AckResult::Accepted, DenyReason::None};

constexpr ModeChange denied(DenyReason reason) noexcept { return {AckResult::Denied, reason}; }

}  // namespace

ModeMachine::ModeMachine(ModeTimings timings) noexcept : timings_(timings) {}

void ModeMachine::boot(TimeNs now) noexcept {
    mode_ = Mode::NotReady;
    ready_at_ = now + timings_.ready_after;
}

bool ModeMachine::armed() const noexcept {
    return mode_ == Mode::Manual || mode_ == Mode::Offboard || mode_ == Mode::Hold;
}

bool ModeMachine::stream_alive(TimeNs now, TimeNs last_valid_setpoint) const noexcept {
    return last_valid_setpoint != kNoSetpoint &&
           now - last_valid_setpoint <= timings_.offboard_timeout;
}

ModeEvent ModeMachine::tick(TimeNs now, TimeNs last_valid_setpoint) noexcept {
    if (mode_ == Mode::NotReady && now >= ready_at_) {
        mode_ = Mode::Ready;
        return ModeEvent::BecameReady;
    }
    if (mode_ == Mode::Offboard && !stream_alive(now, last_valid_setpoint)) {
        mode_ = Mode::Hold;
        return ModeEvent::OffboardLost;
    }
    return ModeEvent::None;
}

ModeChange ModeMachine::request_arm(bool arm, TimeNs /*now*/) noexcept {
    if (!arm) {
        if (armed()) {
            mode_ = Mode::Ready;
        }
        return kAccepted;  // disarming a disarmed vehicle is a no-op
    }
    if (mode_ == Mode::NotReady) {
        return {AckResult::TemporarilyRejected, DenyReason::NotReady};
    }
    if (mode_ == Mode::Ready) {
        mode_ = Mode::Manual;
    }
    return kAccepted;
}

ModeChange ModeMachine::request_mode(Mode target, TimeNs now, TimeNs last_valid_setpoint) noexcept {
    if (target == Mode::NotReady || target == Mode::Ready) {
        return {AckResult::Unsupported, DenyReason::NotRequestable};
    }
    if (mode_ == Mode::NotReady) {
        return denied(DenyReason::NotReady);
    }
    if (!armed()) {
        return denied(DenyReason::Disarmed);
    }
    if (target == Mode::Offboard && mode_ != Mode::Offboard &&
        !stream_alive(now, last_valid_setpoint)) {
        return denied(DenyReason::NoSetpointStream);
    }
    mode_ = target;
    return kAccepted;
}

bool ModeMachine::force(Mode target) noexcept {
    if (!armed() || (target != Mode::Hold && target != Mode::Manual)) {
        return false;
    }
    mode_ = target;
    return true;
}

TimeNs ModeMachine::next_deadline(TimeNs last_valid_setpoint) const noexcept {
    if (mode_ == Mode::NotReady) {
        return ready_at_;
    }
    if (mode_ == Mode::Offboard && last_valid_setpoint != kNoSetpoint) {
        return last_valid_setpoint + timings_.offboard_timeout + 1;  // loss is strictly ">"
    }
    return kNever;
}

const char* mode_name(Mode mode) noexcept {
    switch (mode) {
        case Mode::NotReady:
            return "NOT_READY";
        case Mode::Ready:
            return "READY";
        case Mode::Manual:
            return "MANUAL";
        case Mode::Offboard:
            return "OFFBOARD";
        case Mode::Hold:
            return "HOLD";
    }
    return "?";
}

}  // namespace fcstub
