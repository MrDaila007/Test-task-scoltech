#pragma once

// Test harness around FcCore: a client that encodes commands like an onboard
// computer would, and a station that decodes everything the core emits.
// Header-only; allocation happens only when frames are recorded (record=true).

#include "fcstub/config.hpp"
#include "fcstub/fc_core.hpp"
#include "fcstub/time.hpp"

#include <common/mavlink.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace fcstub::test {

struct Received {
    TimeNs t;
    PacketTag tag;
    mavlink_message_t msg;
};

class Station : public FrameSink {
public:
    bool record = true;
    std::uint64_t frames = 0;
    std::vector<Received> log;

    void emit(TimeNs t, const std::uint8_t* data, std::size_t len,
              const PacketTag& tag) noexcept override {
        mavlink_message_t out{};
        mavlink_status_t out_status{};
        for (std::size_t i = 0; i < len; ++i) {
            if (mavlink_frame_char_buffer(&rx_, &status_, data[i], &out, &out_status) ==
                MAVLINK_FRAMING_OK) {
                ++frames;
                if (record) {
                    log.push_back({t, tag, out});
                }
            }
        }
    }

    std::vector<Received> of(std::uint32_t msgid) const {
        std::vector<Received> out;
        for (const auto& r : log) {
            if (r.msg.msgid == msgid) {
                out.push_back(r);
            }
        }
        return out;
    }

    std::vector<std::string> texts() const {
        std::vector<std::string> out;
        for (const auto& r : of(MAVLINK_MSG_ID_STATUSTEXT)) {
            char text[51] = {};
            mavlink_msg_statustext_get_text(&r.msg, text);
            out.emplace_back(text);
        }
        return out;
    }

private:
    mavlink_message_t rx_{};
    mavlink_status_t status_{};
};

class Client {
public:
    struct Bytes {
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> data{};
        std::uint16_t len = 0;
    };

    Bytes command(std::uint16_t cmd, float p1, float p2 = 0, float p3 = 0,
                  std::uint8_t confirmation = 0) {
        mavlink_msg_command_long_pack_status(255, 190, &status_, &msg_, 1, 1, cmd, confirmation, p1,
                                             p2, p3, 0, 0, 0, 0);
        return finish();
    }
    Bytes arm(bool on = true) { return command(MAV_CMD_COMPONENT_ARM_DISARM, on ? 1.0F : 0.0F); }
    Bytes set_mode(std::uint8_t main, std::uint8_t sub = 0) {
        return command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub);
    }
    Bytes velocity(float vn, float ve, float vd) {
        mavlink_msg_set_position_target_local_ned_pack_status(255, 190, &status_, &msg_, 0, 1, 1,
                                                              MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0,
                                                              vn, ve, vd, 0, 0, 0, 0, 0);
        return finish();
    }
    Bytes param_request_list() {
        mavlink_msg_param_request_list_pack_status(255, 190, &status_, &msg_, 1, 1);
        return finish();
    }
    // MAVLink copies all 16 id bytes: pad the name.
    static std::array<char, 16> param_id(const char* id) {
        std::array<char, 16> out{};
        std::memcpy(out.data(), id, strnlen(id, out.size()));
        return out;
    }
    Bytes param_request_read(const char* id, std::int16_t index = -1) {
        mavlink_msg_param_request_read_pack_status(255, 190, &status_, &msg_, 1, 1,
                                                   param_id(id).data(), index);
        return finish();
    }
    Bytes param_set(const char* id, float value, std::uint8_t type = MAV_PARAM_TYPE_REAL32) {
        mavlink_msg_param_set_pack_status(255, 190, &status_, &msg_, 1, 1, param_id(id).data(),
                                          value, type);
        return finish();
    }
    Bytes mission_request_list() {
        mavlink_msg_mission_request_list_pack_status(255, 190, &status_, &msg_, 1, 1,
                                                     MAV_MISSION_TYPE_MISSION);
        return finish();
    }
    Bytes mission_count(std::uint16_t count) {
        mavlink_msg_mission_count_pack_status(255, 190, &status_, &msg_, 1, 1, count,
                                              MAV_MISSION_TYPE_MISSION, 0);
        return finish();
    }
    Bytes mission_clear_all() {
        mavlink_msg_mission_clear_all_pack_status(255, 190, &status_, &msg_, 1, 1,
                                                  MAV_MISSION_TYPE_MISSION);
        return finish();
    }
    Bytes timesync_reply(std::int64_t tc1, std::int64_t ts1) {
        mavlink_msg_timesync_pack_status(255, 190, &status_, &msg_, tc1, ts1, 1, 1);
        return finish();
    }
    Bytes timesync(std::int64_t ts1) {
        mavlink_msg_timesync_pack_status(255, 190, &status_, &msg_, 0, ts1, 1, 1);
        return finish();
    }

private:
    Bytes finish() {
        Bytes b;
        b.len = mavlink_msg_to_send_buffer(b.data.data(), &msg_);
        return b;
    }

    mavlink_status_t status_{};
    mavlink_message_t msg_{};
};

// Drives the core through every deadline up to `until`, like the sim driver.
class Harness {
public:
    explicit Harness(const Config& cfg) : core(cfg) {}

    void run_until(TimeNs until) {
        while (core.next_deadline() <= until) {
            now = core.next_deadline();
            core.advance_to(now, station);
        }
        now = until;
        core.advance_to(now, station);
    }

    void send(const Client::Bytes& b) {
        core.on_rx_bytes(now, b.data.data(), b.len);
        core.advance_to(now, station);
    }

    // Streams velocity setpoints at `hz` from now until `until`.
    void stream_velocity(TimeNs until, double hz, float vn, float ve = 0, float vd = 0) {
        const TimeNs period = period_from_hz(hz);
        for (TimeNs t = now; t < until; t += period) {
            run_until(t);
            send(client.velocity(vn, ve, vd));
        }
        run_until(until);
    }

    FcCore core;
    Station station;
    Client client;
    TimeNs now = 0;
};

inline Config base_config() {
    Config cfg;
    cfg.run.mode = RunMode::Sim;
    cfg.run.duration_s = 60;
    return cfg;
}

}  // namespace fcstub::test
