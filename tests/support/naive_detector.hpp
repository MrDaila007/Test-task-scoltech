#pragma once

// Reference fault detector: the checks a reasonable onboard computer runs on
// autopilot telemetry. It reads the sim journal (frames.bin) - the same bytes the
// onboard software would receive - and reports alarms. It is deliberately simple:
// the point is to show which injected faults such checks catch, how fast, and
// which ones they cannot catch at all.
//
//   HeartbeatTimeout  no HEARTBEAT for more than 2.5 s
//   TimeRegression    ATTITUDE time_boot_ms went backwards (autopilot restarted)
//   SequenceGap       >= 3 MAVLink sequence gaps or CRC errors within 1 s
//   FrozenPosition    position unchanged for 1 s while reported speed > 0.5 m/s
//   PositionJump      position step not explained by velocity (> 2 m residual)
//   ClockDrift        TIMESYNC offset moved by more than 5 ms from its first value

#include <cstdint>
#include <string>
#include <vector>

namespace fcstub::test {

enum class Alarm {
    HeartbeatTimeout,
    TimeRegression,
    SequenceGap,
    FrozenPosition,
    PositionJump,
    ClockDrift
};

struct AlarmEvent {
    double t_s;
    Alarm kind;
};

struct JournalRecord {
    std::int64_t t_ns;
    std::uint8_t dir;  // 0 = from the autopilot, 1 = sent by the client
    std::vector<std::uint8_t> bytes;
};

std::vector<JournalRecord> read_journal(const std::string& path);

std::vector<AlarmEvent> detect(const std::vector<JournalRecord>& journal);

const char* alarm_name(Alarm a);

}  // namespace fcstub::test
