#include "fcstub/rng.hpp"

#include <cmath>

namespace fcstub {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kTwoPowMinus53 = 1.0 / 9007199254740992.0;  // 2^-53
constexpr std::uint64_t kGoldenGamma = UINT64_C(0x9E3779B97F4A7C15);

}  // namespace

std::uint64_t splitmix64(std::uint64_t x) noexcept {
    x += kGoldenGamma;
    x = (x ^ (x >> 30U)) * UINT64_C(0xBF58476D1CE4E5B9);
    x = (x ^ (x >> 27U)) * UINT64_C(0x94D049BB133111EB);
    return x ^ (x >> 31U);
}

Rng::Rng(std::uint64_t seed, std::uint64_t stream_id)
    : engine_(splitmix64(seed ^ splitmix64(stream_id))) {}

std::uint64_t Rng::next_u64() noexcept { return engine_(); }

double Rng::uniform01() noexcept {
    return static_cast<double>(next_u64() >> 11U) * kTwoPowMinus53;
}

double Rng::normal() noexcept {
    const double u1 = 1.0 - uniform01();  // (0, 1]: log() stays finite
    const double u2 = uniform01();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

bool Rng::bernoulli(double p) noexcept { return uniform01() < p; }

}  // namespace fcstub
