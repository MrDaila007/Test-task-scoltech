#include "fcstub/fc_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fcstub {

namespace {

constexpr std::size_t kDownlinkMaxLen = kMaxFrameLen;

DynamicsParams dynamics_params(const VehicleConfig& v) noexcept {
    return {v.tau_s, v.v_max_mps, v.pos_gain};
}

GateLimits gate_limits(const Config& c) noexcept {
    return {c.identity.system_id, c.identity.component_id, c.vehicle.v_max_mps,
            c.vehicle.geofence_m, c.vehicle.geofence_alt_m};
}

bool direction_matches(LinkDirection d, bool uplink) noexcept {
    return d == LinkDirection::Both || (uplink ? d == LinkDirection::Up : d == LinkDirection::Down);
}

}  // namespace

FcCore::FcCore(const Config& cfg)
    : cfg_(cfg),
      schedule_(cfg_.faults, cfg_.run.seed),
      clock_(schedule_),
      life_(schedule_),
      tap_(schedule_),
      modes_({ms_to_ns(cfg_.modes.ready_after_ms), ms_to_ns(cfg_.modes.offboard_timeout_ms)}),
      gate_(gate_limits(cfg_)),
      dyn_(dynamics_params(cfg_.vehicle)),
      battery_(cfg_.battery),
      encoder_(cfg_.identity.system_id, cfg_.identity.component_id),
      uplink_(static_cast<std::size_t>(cfg_.link.queue_capacity), kMaxLinkPacket),
      downlink_(static_cast<std::size_t>(cfg_.link.queue_capacity), kDownlinkMaxLen),
      dt_(ms_to_ns(cfg_.vehicle.dt_ms)) {
    // Registration order is the tie-break order for equal deadlines.
    scheduler_.add(period_from_hz(cfg_.telemetry_hz.heartbeat), 0);
    scheduler_.add(period_from_hz(cfg_.telemetry_hz.attitude), 0);
    scheduler_.add(period_from_hz(cfg_.telemetry_hz.global_position), 0);
    scheduler_.add(period_from_hz(cfg_.telemetry_hz.battery), 0);
    modes_.boot(0);
    clock_.boot(0);
}

TimeNs FcCore::clamp_time(TimeNs now) noexcept {
    if (now < now_) {
        ++stats_.rx_time_regressions;
        return now_;
    }
    return now;
}

const LinkFaultParams* FcCore::link_fault(TimeNs now, bool uplink, Rng** rng) noexcept {
    const FaultWindow* w = schedule_.active(FaultType::Link, now);
    if (w == nullptr) {
        *rng = nullptr;
        return nullptr;
    }
    const auto& p = params_of<LinkFaultParams>(*w);
    if (!direction_matches(p.direction, uplink)) {
        *rng = nullptr;
        return nullptr;
    }
    *rng = &schedule_.rng(w->index);
    return &p;
}

void FcCore::on_rx_bytes(TimeNs now, const std::uint8_t* data, std::size_t len) noexcept {
    now = clamp_time(now);
    now_ = now;
    if (len > kMaxLinkPacket) {
        ++stats_.rx_oversize;
        return;
    }
    ++stats_.rx_datagrams;
    Rng* rng = nullptr;
    const LinkFaultParams* fault = link_fault(now, true, &rng);
    uplink_.push(now, data, len, fault, rng);
}

void FcCore::reboot(TimeNs now) noexcept {
    modes_.boot(now);
    clock_.boot(now);
    encoder_.reset_sequence();
    gate_.reset();
    dyn_.halt();
    rebooted_once_ = true;
    send_text(kSeverityInfo, "FC stub boot");
}

void FcCore::handle_lifecycle(TimeNs now) noexcept {
    for (LifeEvent e = life_.update(now); e != LifeEvent::None; e = life_.update(now)) {
        if (e == LifeEvent::Rebooted) {
            reboot(now);
        }
    }
}

void FcCore::process_uplink(TimeNs now) noexcept {
    while (uplink_.pop_due(now, scratch_)) {
        if (life_.silent()) {
            ++stats_.rx_dropped_silent;
            continue;
        }
        rx_now_ = scratch_.release;
        decoder_.feed(scratch_.data.data(), scratch_.len, *this);
    }
}

MotionTarget FcCore::motion_target() const noexcept {
    MotionTarget t;
    if (!modes_.armed()) {
        t.kind = MotionKind::Disarmed;
        return t;
    }
    if (modes_.mode() != Mode::Offboard || !gate_.has_setpoint()) {
        t.kind = MotionKind::Stop;
        return t;
    }
    const Setpoint& sp = gate_.last();
    switch (sp.kind) {
        case SetpointKind::Velocity:
            t.kind = MotionKind::Velocity;
            break;
        case SetpointKind::Position:
            t.kind = MotionKind::Position;
            break;
        case SetpointKind::PositionVelocity:
            t.kind = MotionKind::PositionVelocity;
            break;
    }
    t.pos = sp.pos;
    t.vel = sp.vel;
    t.yaw_valid = sp.yaw_valid;
    t.yaw = sp.yaw;
    t.yaw_rate_valid = sp.yaw_rate_valid;
    t.yaw_rate = sp.yaw_rate;
    return t;
}

void FcCore::integrate_to(TimeNs now) noexcept {
    const double dt_s = ns_to_seconds(dt_);
    while (model_time_ + dt_ <= now) {
        model_time_ += dt_;
        dyn_.step(dt_s, motion_target());
        const Vec3& v = dyn_.state().vel;
        battery_.step(dt_s, modes_.armed(), std::hypot(v[0], v[1], v[2]));
        tap_.update(dyn_.state(), model_time_);
    }
}

void FcCore::handle_mode_events(TimeNs now) noexcept {
    const Mode before = modes_.mode();
    const ModeEvent e = modes_.tick(now, gate_.last_valid_time());
    if (e == ModeEvent::BecameReady) {
        send_text(kSeverityInfo, "Ready to arm");
    } else if (e == ModeEvent::OffboardLost) {
        char text[51];
        std::snprintf(text, sizeof(text), "Offboard lost >%dms: HOLD",
                      cfg_.modes.offboard_timeout_ms);
        send_text(kSeverityCritical, text);
        announce_mode(before);
    }
}

void FcCore::flush_downlink(TimeNs now, FrameSink& sink) noexcept {
    while (downlink_.pop_due(now, scratch_)) {
        ++stats_.tx_frames;
        sink.emit(scratch_.release, scratch_.data.data(), scratch_.len, scratch_.tag);
    }
}

void FcCore::advance_to(TimeNs now, FrameSink& sink) noexcept {
    now = clamp_time(now);
    now_ = now;
    if (!started_) {
        started_ = true;
        send_text(kSeverityInfo, "FC stub boot");
    }
    handle_lifecycle(now);
    process_uplink(now);
    integrate_to(now);
    handle_mode_events(now);
    Scheduler::DueList due{};
    const std::size_t n = scheduler_.due(now, due);
    for (std::size_t i = 0; i < n; ++i) {
        emit_telemetry(static_cast<Stream>(due[i].id), due[i].deadline);
    }
    flush_downlink(now, sink);
    stats_.decoder = decoder_.stats();
    stats_.gate = gate_.stats();
    stats_.uplink = uplink_.stats();
    stats_.downlink = downlink_.stats();
    stats_.missed_deadlines = 0;
    for (std::size_t i = 0; i < kStreamCount; ++i) {
        stats_.missed_deadlines += scheduler_.missed(static_cast<Scheduler::TaskId>(i));
    }
}

TimeNs FcCore::next_deadline() const noexcept {
    if (!started_) {
        return 0;  // the first step happens at time zero
    }
    TimeNs next = scheduler_.next_deadline();
    next = std::min(next, modes_.next_deadline(gate_.last_valid_time()));
    next = std::min(next, uplink_.next_release());
    next = std::min(next, downlink_.next_release());
    next = std::min(next, schedule_.next_boundary(now_));
    return std::max(next, now_ + 1);
}

FaultMask FcCore::active_faults(TimeNs now) const noexcept {
    FaultMask mask = 0;
    for (const FaultWindow& w : schedule_.windows()) {
        if (w.start <= now && now < w.end) {
            mask |= 1U << static_cast<unsigned>(w.type);
        }
    }
    return mask;
}

}  // namespace fcstub
