#pragma once

// Time base of the whole core: signed 64-bit nanoseconds since driver start.
// The core never reads a clock; drivers pass `now` in.

#include <cmath>
#include <cstdint>

namespace fcstub {

using TimeNs = std::int64_t;

inline constexpr TimeNs kNsPerUs = 1'000;
inline constexpr TimeNs kNsPerMs = 1'000'000;
inline constexpr TimeNs kNsPerS = 1'000'000'000;

// Nominal period of a stream; rounding to the nearest nanosecond.
// Precondition: hz > 0 (validated by the config loader).
inline TimeNs period_from_hz(double hz) noexcept {
    return static_cast<TimeNs>(std::llround(static_cast<double>(kNsPerS) / hz));
}

inline constexpr TimeNs ms_to_ns(std::int64_t ms) noexcept { return ms * kNsPerMs; }

inline TimeNs seconds_to_ns(double s) noexcept {
    return static_cast<TimeNs>(std::llround(s * static_cast<double>(kNsPerS)));
}

inline constexpr double ns_to_seconds(TimeNs t) noexcept {
    return static_cast<double>(t) / static_cast<double>(kNsPerS);
}

}  // namespace fcstub
