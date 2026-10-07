#pragma once

// Wall-clock driver: runs the core against CLOCK_MONOTONIC behind a UDP socket.
//
// One thread. Each iteration waits in ppoll() until the core's next deadline or
// an incoming datagram (optionally busy-waiting the last spin_us), drains at most
// rx_max_frames_per_iter datagrams, then advances the core to "now". Deadlines
// are absolute, so waiting late never shifts the schedule. SIGINT/SIGTERM are
// blocked except inside ppoll, so a stop request cannot be lost between the flag
// check and the wait.
//
// Send jitter is measured per telemetry stream as |t_send - deadline| right
// before sendto(); frames sent while a link fault is active are excluded (their
// delay is the injected fault, not jitter).

#include "fcstub/config.hpp"

#include <string>

namespace fcstub {

class RealtimeDriver {
public:
    explicit RealtimeDriver(const Config& cfg);

    // Returns the process exit code: 0 on a normal stop, 3 if the socket cannot
    // be opened (`error` explains), 1 if the report cannot be written.
    int run(std::string& error);

private:
    Config cfg_;
};

}  // namespace fcstub
