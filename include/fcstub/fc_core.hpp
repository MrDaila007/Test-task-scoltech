#pragma once

// FcCore: the whole PX4-like autopilot model, independent of the OS.
//
// Time and incoming bytes are passed in; outgoing frames leave through a
// FrameSink. The core never reads a clock, never sleeps and never allocates
// after construction, so the same code runs in real time behind a UDP socket
// and in model time for reproducible tests.
//
// Threading: single-threaded and not reentrant. All calls must come from one
// thread; FrameSink::emit is called synchronously from advance_to().
//
// Order of work in advance_to(now):
//   1. reboot fault events (F2)        2. incoming packets released by the uplink
//   3. dynamics up to `now`            4. mode machine time events (ready, offboard loss)
//   5. periodic telemetry              6. frames released by the downlink -> sink

#include "fcstub/battery.hpp"
#include "fcstub/command_dedup.hpp"
#include "fcstub/config.hpp"
#include "fcstub/dynamics.hpp"
#include "fcstub/estimator_tap.hpp"
#include "fcstub/fault_schedule.hpp"
#include "fcstub/fc_clock.hpp"
#include "fcstub/lifecycle.hpp"
#include "fcstub/link_model.hpp"
#include "fcstub/mav_codec.hpp"
#include "fcstub/mode_machine.hpp"
#include "fcstub/scheduler.hpp"
#include "fcstub/setpoint_gate.hpp"
#include "fcstub/time.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace fcstub {

enum class Stream : std::uint8_t { Heartbeat = 0, Attitude = 1, GlobalPosition = 2, Battery = 3 };
inline constexpr std::size_t kStreamCount = 4;

class FrameSink {
public:
    virtual void emit(TimeNs t, const std::uint8_t* data, std::size_t len,
                      const PacketTag& tag) noexcept = 0;

protected:
    ~FrameSink() = default;
};

struct CoreStats {
    std::uint64_t rx_datagrams = 0;
    std::uint64_t rx_oversize = 0;
    std::uint64_t rx_time_regressions = 0;
    std::uint64_t rx_dropped_silent = 0;   // arrived while rebooting
    std::uint64_t tx_frames = 0;
    std::uint64_t tx_suppressed = 0;       // produced while rebooting
    std::uint64_t statustexts = 0;
    std::uint64_t missed_deadlines = 0;
    DecoderStats decoder;
    GateStats gate;
    LinkStats uplink;
    LinkStats downlink;
};

// Bit i set = fault type i active (EstimatorFreeze=0 ... Gnss=4).
using FaultMask = std::uint32_t;

class FcCore : private RxHandler {
public:
    explicit FcCore(const Config& cfg);
    FcCore(const FcCore&) = delete;
    FcCore& operator=(const FcCore&) = delete;

    // Accepts one datagram; the bytes are copied. Datagrams longer than
    // kMaxLinkPacket are dropped and counted. `now` going backwards is clamped.
    void on_rx_bytes(TimeNs now, const std::uint8_t* data, std::size_t len) noexcept;

    void advance_to(TimeNs now, FrameSink& sink) noexcept;

    // Earliest time at which the core has work: 0 before the first advance_to(),
    // afterwards always later than the last `now`.
    TimeNs next_deadline() const noexcept;

    const CoreStats& stats() const noexcept { return stats_; }
    Mode mode() const noexcept { return modes_.mode(); }
    bool armed() const noexcept { return modes_.armed(); }
    const VehicleState& truth() const noexcept { return dyn_.state(); }
    const VehicleState& estimate() const noexcept { return tap_.estimate(); }
    TimeNs last_setpoint_time() const noexcept { return gate_.last_valid_time(); }
    TimeNs fc_time_ns(TimeNs now) const noexcept { return clock_.fc_ns(now); }
    FaultMask active_faults(TimeNs now) const noexcept;
    const GeoOrigin& origin() const noexcept { return cfg_.vehicle.origin; }

private:
    // RxHandler
    void on_message(const RxMessage& msg) noexcept override;

    // fc_core.cpp
    TimeNs clamp_time(TimeNs now) noexcept;
    void handle_lifecycle(TimeNs now) noexcept;
    void reboot(TimeNs now) noexcept;
    void process_uplink(TimeNs now) noexcept;
    void integrate_to(TimeNs now) noexcept;
    void handle_mode_events(TimeNs now) noexcept;
    void flush_downlink(TimeNs now, FrameSink& sink) noexcept;
    MotionTarget motion_target() const noexcept;
    const LinkFaultParams* link_fault(TimeNs now, bool uplink, Rng** rng) noexcept;

    // fc_core_rx.cpp
    void handle_setpoint(const SetpointMsg& msg) noexcept;
    void handle_command(const CommandLongMsg& msg) noexcept;
    void handle_timesync(const TimesyncMsg& msg) noexcept;
    ModeChange execute_command(const CommandLongMsg& msg) noexcept;
    bool addressed_to_us(std::uint8_t target_system, std::uint8_t target_component) const noexcept;

    // fc_core_tx.cpp
    void send(const FrameBuf& frame, PacketTag tag = {}) noexcept;
    void send_text(std::uint8_t severity, std::string_view text) noexcept;
    void announce_mode(Mode before) noexcept;
    void emit_telemetry(Stream stream, TimeNs deadline) noexcept;
    HeartbeatData heartbeat_data() const noexcept;

    Config cfg_;
    FaultSchedule schedule_;
    FcClock clock_;
    Lifecycle life_;
    EstimatorTap tap_;
    ModeMachine modes_;
    SetpointGate gate_;
    CommandDedup dedup_;
    Dynamics dyn_;
    Battery battery_;
    Scheduler scheduler_;
    MavEncoder encoder_;
    MavDecoder decoder_;
    LinkModel uplink_;
    LinkModel downlink_;
    LinkPacket scratch_{};  // reused for every packet released by a link

    TimeNs dt_;
    TimeNs model_time_ = 0;
    TimeNs now_ = 0;
    TimeNs rx_now_ = 0;  // time of the packet being decoded
    TimeNs last_reject_text_ = 0;
    bool reject_text_sent_ = false;  // rate limit for "Setpoint rejected" texts
    bool started_ = false;
    bool rebooted_once_ = false;
    CoreStats stats_{};
};

}  // namespace fcstub
