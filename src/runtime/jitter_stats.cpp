#include "fcstub/jitter_stats.hpp"

#include <algorithm>
#include <cmath>

namespace fcstub {

JitterStats::JitterStats() : bins_(kBins, 0) {}

void JitterStats::add(TimeNs error_ns) noexcept {
    const TimeNs e = error_ns < 0 ? -error_ns : error_ns;
    const auto bin = static_cast<std::size_t>(e / kNsPerUs);
    if (bin < kBins) {
        ++bins_[bin];
    } else {
        ++overflow_;
    }
    max_ = std::max(max_, e);
    ++count_;
}

double JitterStats::percentile_us(double q) const noexcept {
    if (count_ == 0) {
        return 0.0;
    }
    const auto target = static_cast<std::uint64_t>(std::ceil(q * static_cast<double>(count_)));
    std::uint64_t seen = 0;
    for (std::size_t i = 0; i < kBins; ++i) {
        seen += bins_[i];
        if (seen >= target && target > 0) {
            return static_cast<double>(i + 1);
        }
    }
    return static_cast<double>(max_) / static_cast<double>(kNsPerUs);
}

}  // namespace fcstub
