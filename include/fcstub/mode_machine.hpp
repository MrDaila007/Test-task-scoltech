#pragma once

// PX4-like flight mode state machine with the five modes of the stub:
// not ready, ready, manual, offboard (external control) and hold.
//
// Transitions (anything else is refused with a reason):
//   NotReady --ready_after elapsed--> Ready
//   Ready --arm--> Manual;   any armed mode --disarm--> Ready
//   Manual/Hold --OFFBOARD (setpoint stream alive)--> Offboard
//   Manual/Offboard --HOLD--> Hold;   Offboard/Hold --MANUAL--> Manual
//   Offboard --no valid setpoint for more than offboard_timeout--> Hold
// Hold never returns to Offboard by itself, even if setpoints resume.

#include "fcstub/time.hpp"

#include <cstdint>
#include <limits>

namespace fcstub {

enum class Mode : std::uint8_t { NotReady, Ready, Manual, Offboard, Hold };

// Values match MAV_RESULT.
enum class AckResult : std::uint8_t {
    Accepted = 0,
    TemporarilyRejected = 1,
    Denied = 2,
    Unsupported = 3,
};

enum class DenyReason : std::uint8_t {
    None,
    NotReady,
    Disarmed,
    NoSetpointStream,
    NotRequestable,
    Overridden,
    InAir
};

struct ModeChange {
    AckResult result;
    DenyReason reason;
};

enum class ModeEvent : std::uint8_t { None, BecameReady, OffboardLost };

struct ModeTimings {
    TimeNs ready_after;
    TimeNs offboard_timeout;
};

class ModeMachine {
public:
    static constexpr TimeNs kNoSetpoint = std::numeric_limits<TimeNs>::min();
    static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::max();

    explicit ModeMachine(ModeTimings timings) noexcept;

    // Power-up or reboot: NotReady, disarmed; Ready after ready_after.
    void boot(TimeNs now) noexcept;

    // Time-driven transitions. `last_valid_setpoint` is the receive time of the
    // newest accepted setpoint, or kNoSetpoint.
    ModeEvent tick(TimeNs now, TimeNs last_valid_setpoint) noexcept;

    ModeChange request_arm(bool arm, TimeNs now) noexcept;
    ModeChange request_mode(Mode target, TimeNs now, TimeNs last_valid_setpoint) noexcept;

    // The autopilot's own decision (failsafe, pilot takeover): switches an armed
    // vehicle to Hold or Manual. Returns false when disarmed or not applicable.
    bool force(Mode target) noexcept;

    // PARAM_SET COM_OF_LOSS_T: takes effect for the next timeout check.
    void set_offboard_timeout(TimeNs timeout) noexcept { timings_.offboard_timeout = timeout; }

    Mode mode() const noexcept { return mode_; }
    bool armed() const noexcept;

    // Earliest time at which tick() could change the mode.
    TimeNs next_deadline(TimeNs last_valid_setpoint) const noexcept;

private:
    bool stream_alive(TimeNs now, TimeNs last_valid_setpoint) const noexcept;

    ModeTimings timings_;
    Mode mode_ = Mode::NotReady;
    TimeNs ready_at_ = 0;
};

const char* mode_name(Mode mode) noexcept;

}  // namespace fcstub
