#pragma once

// Model-time driver: runs the core and the virtual client from t = 0 to
// run.duration_s by jumping from one deadline to the next (no sleeping), and
// records the run:
//   frames.bin  - journal of every frame (see below)
//   truth.csv   - true vs reported state at sim.truth_csv_hz
//   sha256.txt  - SHA-256 over the downlink journal records
//
// Journal format, version 1, little-endian: "FCSJ", uint32 version, then records
//   int64 t_ns, uint8 direction (0 = from the autopilot, 1 = sent by the client),
//   uint16 length, bytes.
// The hash covers exactly the direction-0 records, so the same configuration and
// seed give the same hash on every run and platform with the same build flags.

#include "fcstub/config.hpp"

#include <cstdint>
#include <string>

namespace fcstub {

struct SimResult {
    std::string sha256;
    std::uint64_t frames_down = 0;
    std::uint64_t frames_up = 0;
};

class SimDriver {
public:
    explicit SimDriver(const Config& cfg);

    // Throws std::runtime_error if the output files cannot be written.
    SimResult run();

private:
    Config cfg_;
};

}  // namespace fcstub
