#pragma once

// Portable, reproducible random streams.
//
// std::mt19937_64 output is fully specified by the standard, but the std::
// distributions are not (libstdc++ and libc++ differ), so the distributions
// below are hand-written. Each fault owns its own stream, seeded from the
// global seed and a stream id, so enabling one fault never perturbs another.

#include <cstdint>
#include <random>

namespace fcstub {

std::uint64_t splitmix64(std::uint64_t x) noexcept;

class Rng {
public:
    Rng(std::uint64_t seed, std::uint64_t stream_id);

    std::uint64_t next_u64() noexcept;

    // Uniform in [0, 1) with 53 bits of resolution.
    double uniform01() noexcept;

    // Standard normal (Box-Muller, cosine branch only; one sample per call
    // keeps the stream position independent of call history).
    double normal() noexcept;

    // True with probability p; always consumes exactly one draw.
    bool bernoulli(double p) noexcept;

private:
    std::mt19937_64 engine_;
};

}  // namespace fcstub
