#pragma once

// Send-time error per telemetry stream: |t_send - deadline|. A fixed histogram
// with 1 us bins up to 20 ms (exact maximum kept separately) gives percentiles
// without storing samples or allocating while the loop runs.

#include "fcstub/time.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace fcstub {

class JitterStats {
public:
    static constexpr std::size_t kBins = 20000;  // 1 us each

    JitterStats();

    void add(TimeNs error_ns) noexcept;

    std::uint64_t count() const noexcept { return count_; }
    TimeNs max_ns() const noexcept { return max_; }
    // Upper edge of the bin holding the q-quantile, in microseconds.
    double percentile_us(double q) const noexcept;

private:
    std::vector<std::uint32_t> bins_;
    std::uint64_t count_ = 0;
    std::uint64_t overflow_ = 0;
    TimeNs max_ = 0;
};

}  // namespace fcstub
