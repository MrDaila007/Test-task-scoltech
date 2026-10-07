#include "fcstub/rng.hpp"
#include "fcstub/time.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

TEST(Time, HzToPeriod) {
    EXPECT_EQ(fcstub::period_from_hz(50.0), 20 * fcstub::kNsPerMs);
    EXPECT_EQ(fcstub::period_from_hz(1.0), fcstub::kNsPerS);
    EXPECT_EQ(fcstub::period_from_hz(3.0), 333'333'333);
}

TEST(Rng, SameSeedAndStreamGiveSameSequence) {
    fcstub::Rng a(42, 7);
    fcstub::Rng b(42, 7);
    for (int i = 0; i < 1000; ++i) {
        ASSERT_EQ(a.next_u64(), b.next_u64());
    }
}

TEST(Rng, DifferentStreamsDiverge) {
    fcstub::Rng a(42, 1);
    fcstub::Rng b(42, 2);
    int equal = 0;
    for (int i = 0; i < 100; ++i) {
        equal += a.next_u64() == b.next_u64() ? 1 : 0;
    }
    EXPECT_EQ(equal, 0);
}

// Golden values pin the sequence: a change here silently changes every
// reproducibility hash, so it must be deliberate.
// Values recorded on x86_64 g++ 11.4 with -ffp-contract=off (2026-10-07).
TEST(Rng, GoldenSequence) {
    fcstub::Rng raw(42, 0);
    EXPECT_EQ(raw.next_u64(), UINT64_C(0xd87787ea79651735));
    EXPECT_EQ(raw.next_u64(), UINT64_C(0xc2e03cee97d0cec5));
    EXPECT_EQ(raw.next_u64(), UINT64_C(0x1c84d963f660482d));

    fcstub::Rng uniform(42, 0);
    EXPECT_EQ(uniform.uniform01(), 0x1.b0ef0fd4f2ca2p-1);
    EXPECT_EQ(uniform.uniform01(), 0x1.85c079dd2fa19p-1);
    EXPECT_EQ(uniform.uniform01(), 0x1.c84d963f66048p-4);

    fcstub::Rng normal(42, 0);
    EXPECT_EQ(normal.normal(), 0x1.172feae539d94p-3);
    EXPECT_EQ(normal.normal(), -0x1.120ef88a0de07p-2);
    EXPECT_EQ(normal.normal(), -0x1.c90b52ce928cfp-1);
}

TEST(Rng, UniformInUnitInterval) {
    fcstub::Rng rng(1, 0);
    for (int i = 0; i < 100000; ++i) {
        const double u = rng.uniform01();
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
    }
}

TEST(Rng, NormalHasUnitMomentsApproximately) {
    fcstub::Rng rng(3, 0);
    constexpr int kN = 100000;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < kN; ++i) {
        const double x = rng.normal();
        ASSERT_TRUE(std::isfinite(x));
        sum += x;
        sum_sq += x * x;
    }
    const double mean = sum / kN;
    const double var = sum_sq / kN - mean * mean;
    EXPECT_NEAR(mean, 0.0, 0.02);
    EXPECT_NEAR(var, 1.0, 0.02);
}

TEST(Rng, BernoulliRate) {
    fcstub::Rng rng(5, 0);
    constexpr int kN = 100000;
    int hits = 0;
    for (int i = 0; i < kN; ++i) {
        hits += rng.bernoulli(0.1) ? 1 : 0;
    }
    EXPECT_NEAR(static_cast<double>(hits) / kN, 0.1, 0.005);
    EXPECT_FALSE(fcstub::Rng(5, 0).bernoulli(0.0));
    EXPECT_TRUE(fcstub::Rng(5, 0).bernoulli(1.0));
}
