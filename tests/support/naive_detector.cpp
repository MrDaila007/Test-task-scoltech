#include "naive_detector.hpp"

#include <common/mavlink.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
#include <optional>

namespace fcstub::test {

namespace {

constexpr double kHeartbeatTimeoutS = 2.5;
constexpr double kFrozenWindowS = 1.0;
constexpr double kFrozenSpeed = 0.5;    // m/s
constexpr double kJumpResidual = 2.0;   // m
constexpr double kDriftLimitNs = 5e6;   // 5 ms
constexpr int kGapsPerSecond = 3;
constexpr double kMetresPerDegLat = 111195.0;

struct PositionFix {
    double t;
    std::int32_t lat, lon;
    double vn, ve;  // m/s
};

class Detector {
public:
    void on_downlink(double t, const mavlink_message_t& m) {
        check_sequence(t, m.seq);
        if (last_heartbeat_ && t - *last_heartbeat_ > kHeartbeatTimeoutS) {
            // An onboard watchdog fires 2.5 s after the last heartbeat, not when the
            // next frame happens to arrive.
            raise(*last_heartbeat_ + kHeartbeatTimeoutS, Alarm::HeartbeatTimeout);
            last_heartbeat_.reset();
        }
        switch (m.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                last_heartbeat_ = t;
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                check_time(t, mavlink_msg_attitude_get_time_boot_ms(&m));
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                check_position(t, m);
                break;
            case MAVLINK_MSG_ID_TIMESYNC:
                check_timesync(t, m);
                break;
            default:
                break;
        }
    }

    void on_crc_error(double t) { note_gap(t); }

    std::vector<AlarmEvent> alarms;

private:
    void raise(double t, Alarm a) {
        // One alarm per kind per second is enough to read the outcome.
        for (auto it = alarms.rbegin(); it != alarms.rend(); ++it) {
            if (it->kind == a && t - it->t_s < 1.0) {
                return;
            }
        }
        alarms.push_back({t, a});
    }

    void note_gap(double t) {
        gaps_.push_back(t);
        while (!gaps_.empty() && t - gaps_.front() > 1.0) {
            gaps_.pop_front();
        }
        if (static_cast<int>(gaps_.size()) >= kGapsPerSecond) {
            raise(t, Alarm::SequenceGap);
        }
    }

    void check_sequence(double t, std::uint8_t seq) {
        if (last_seq_ && static_cast<std::uint8_t>(*last_seq_ + 1) != seq) {
            note_gap(t);
        }
        last_seq_ = seq;
    }

    void check_time(double t, std::uint32_t boot_ms) {
        if (last_boot_ms_ && boot_ms < *last_boot_ms_) {
            raise(t, Alarm::TimeRegression);
        }
        last_boot_ms_ = boot_ms;
    }

    void check_position(double t, const mavlink_message_t& m) {
        mavlink_global_position_int_t p{};
        mavlink_msg_global_position_int_decode(&m, &p);
        const PositionFix fix{t, p.lat, p.lon, p.vx / 100.0, p.vy / 100.0};
        if (last_fix_) {
            check_jump(*last_fix_, fix);
        }
        check_frozen(fix);
        last_fix_ = fix;
    }

    void check_jump(const PositionFix& a, const PositionFix& b) {
        const double dn = (b.lat - a.lat) * 1e-7 * kMetresPerDegLat;
        const double de = (b.lon - a.lon) * 1e-7 * kMetresPerDegLat * std::cos(b.lat * 1e-7 * M_PI / 180);
        const double dt = b.t - a.t;
        const double rn = dn - 0.5 * (a.vn + b.vn) * dt;
        const double re = de - 0.5 * (a.ve + b.ve) * dt;
        if (std::hypot(rn, re) > kJumpResidual) {
            raise(b.t, Alarm::PositionJump);
        }
    }

    void check_frozen(const PositionFix& f) {
        const bool moving = std::hypot(f.vn, f.ve) > kFrozenSpeed;
        if (!moving || !frozen_since_ || f.lat != frozen_since_->lat || f.lon != frozen_since_->lon) {
            frozen_since_ = moving ? std::optional<PositionFix>(f) : std::nullopt;
            return;
        }
        if (f.t - frozen_since_->t >= kFrozenWindowS) {
            raise(f.t, Alarm::FrozenPosition);
        }
    }

    void check_timesync(double t, const mavlink_message_t& m) {
        mavlink_timesync_t ts{};
        mavlink_msg_timesync_decode(&m, &ts);
        if (ts.tc1 == 0) {
            return;
        }
        // Offset of the autopilot clock against ours at the midpoint of the round trip.
        const double rtt = t * 1e9 - static_cast<double>(ts.ts1);
        const double offset = static_cast<double>(ts.tc1) - (static_cast<double>(ts.ts1) + rtt / 2);
        if (!first_offset_) {
            first_offset_ = offset;
        } else if (std::fabs(offset - *first_offset_) > kDriftLimitNs) {
            raise(t, Alarm::ClockDrift);
        }
    }

    std::optional<double> last_heartbeat_;
    std::optional<std::uint8_t> last_seq_;
    std::optional<std::uint32_t> last_boot_ms_;
    std::optional<PositionFix> last_fix_;
    std::optional<PositionFix> frozen_since_;
    std::optional<double> first_offset_;
    std::deque<double> gaps_;
};

}  // namespace

std::vector<JournalRecord> read_journal(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(in)), {});
    std::vector<JournalRecord> out;
    for (std::size_t off = 8; off + 11 <= data.size();) {
        JournalRecord r{};
        std::uint16_t len = 0;
        std::memcpy(&r.t_ns, data.data() + off, 8);
        r.dir = static_cast<std::uint8_t>(data[off + 8]);
        std::memcpy(&len, data.data() + off + 9, 2);
        r.bytes.assign(data.begin() + static_cast<std::ptrdiff_t>(off + 11),
                       data.begin() + static_cast<std::ptrdiff_t>(off + 11 + len));
        out.push_back(std::move(r));
        off += 11U + len;
    }
    return out;
}

std::vector<AlarmEvent> detect(const std::vector<JournalRecord>& journal) {
    Detector d;
    mavlink_message_t rx{};
    mavlink_status_t st{};
    for (const JournalRecord& r : journal) {
        if (r.dir != 0) {
            continue;
        }
        const double t = static_cast<double>(r.t_ns) * 1e-9;
        for (const std::uint8_t b : r.bytes) {
            mavlink_message_t msg{};
            mavlink_status_t msg_st{};
            const std::uint8_t res = mavlink_frame_char_buffer(&rx, &st, b, &msg, &msg_st);
            if (res == MAVLINK_FRAMING_OK) {
                d.on_downlink(t, msg);
            } else if (res == MAVLINK_FRAMING_BAD_CRC) {
                d.on_crc_error(t);
            }
        }
    }
    return d.alarms;
}

const char* alarm_name(Alarm a) {
    switch (a) {
        case Alarm::HeartbeatTimeout:
            return "heartbeat_timeout";
        case Alarm::TimeRegression:
            return "time_regression";
        case Alarm::SequenceGap:
            return "sequence_gap";
        case Alarm::FrozenPosition:
            return "frozen_position";
        case Alarm::PositionJump:
            return "position_jump";
        case Alarm::ClockDrift:
            return "clock_drift";
    }
    return "?";
}

}  // namespace fcstub::test
