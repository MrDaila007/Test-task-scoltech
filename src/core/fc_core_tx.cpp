// FcCore: everything that goes out — periodic telemetry, status texts, responses.

#include "fcstub/fc_core.hpp"

#include "fcstub/geo.hpp"

#include <cstdio>

namespace fcstub {

void FcCore::send(const FrameBuf& frame, PacketTag tag) noexcept {
    if (life_.silent()) {
        ++stats_.tx_suppressed;
        return;
    }
    Rng* rng = nullptr;
    const LinkFaultParams* fault = link_fault(now_, false, &rng);
    downlink_.push(now_, frame.data.data(), frame.len, fault, rng, tag);
}

void FcCore::send_text(std::uint8_t severity, std::string_view text) noexcept {
    ++stats_.statustexts;
    send(encoder_.statustext({severity, text}));
}

void FcCore::announce_mode(Mode before) noexcept {
    if (modes_.mode() == before) {
        return;
    }
    char text[51];
    std::snprintf(text, sizeof(text), "Mode: %s", mode_name(modes_.mode()));
    send_text(kSeverityInfo, text);
}

HeartbeatData FcCore::heartbeat_data() const noexcept {
    HeartbeatData hb{kModeFlagCustomModeEnabled, px4_custom_mode(kPx4MainPosctl, 0), kStateActive};
    switch (modes_.mode()) {
        case Mode::NotReady:
            hb.system_status = rebooted_once_ ? kStateBoot : kStateUninit;
            break;
        case Mode::Ready:
            hb.system_status = kStateStandby;
            break;
        case Mode::Manual:
            hb.custom_mode = px4_custom_mode(kPx4MainManual, 0);
            break;
        case Mode::Offboard:
            hb.custom_mode = px4_custom_mode(kPx4MainOffboard, 0);
            break;
        case Mode::Hold:
            hb.custom_mode = px4_custom_mode(kPx4MainAuto, kPx4SubAutoLoiter);
            break;
    }
    if (modes_.armed()) {
        hb.base_mode = static_cast<std::uint8_t>(hb.base_mode | kModeFlagSafetyArmed);
    }
    return hb;
}

void FcCore::emit_telemetry(Stream stream, TimeNs deadline) noexcept {
    const PacketTag tag{static_cast<std::uint8_t>(stream), deadline};
    const VehicleState& est = tap_.estimate();
    const std::uint32_t boot_ms = clock_.time_boot_ms(now_);
    switch (stream) {
        case Stream::Heartbeat:
            send(encoder_.heartbeat(heartbeat_data()), tag);
            break;
        case Stream::Attitude:
            send(encoder_.attitude(
                     {boot_ms, static_cast<float>(est.roll), static_cast<float>(est.pitch),
                      static_cast<float>(est.yaw), static_cast<float>(est.roll_rate),
                      static_cast<float>(est.pitch_rate), static_cast<float>(est.yaw_rate)}),
                 tag);
            break;
        case Stream::GlobalPosition: {
            const GeoPoint geo =
                ned_to_wgs84(cfg_.vehicle.origin, est.pos[0], est.pos[1], est.pos[2]);
            send(encoder_.global_position_int({boot_ms, geo.lat_deg, geo.lon_deg, geo.alt_msl_m,
                                               -est.pos[2], est.vel[0], est.vel[1], est.vel[2],
                                               est.yaw}),
                 tag);
            break;
        }
        case Stream::Battery:
            send(encoder_.battery_status({battery_.cells(), battery_.voltage_v(),
                                          battery_.current_a(), battery_.consumed_mah(),
                                          battery_.soc()}),
                 tag);
            break;
        case Stream::SysStatus:
            send(encoder_.sys_status({battery_.voltage_v(), battery_.current_a(), battery_.soc()}),
                 tag);
            break;
        case Stream::ExtendedSysState:
            send(encoder_.extended_sys_state(in_air() ? kLandedInAir : kLandedOnGround), tag);
            break;
    }
}

}  // namespace fcstub
