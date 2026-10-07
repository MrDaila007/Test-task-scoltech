#include "fcstub/command_dedup.hpp"

#include <gtest/gtest.h>

namespace {

using fcstub::AckResult;
using fcstub::CommandDedup;
using fcstub::CommandKey;
using fcstub::kNsPerMs;

constexpr CommandKey kArm{255, 190, 400};

}  // namespace

TEST(CommandDedup, FirstTransmissionIsNotADuplicate) {
    CommandDedup d;
    EXPECT_FALSE(d.lookup(kArm, 0, 0).has_value());
}

TEST(CommandDedup, RetransmissionWithinOneSecondReturnsTheCachedResult) {
    CommandDedup d;
    d.remember(kArm, 0, AckResult::Accepted);
    const auto r = d.lookup(kArm, 1, 900 * kNsPerMs);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, AckResult::Accepted);
}

TEST(CommandDedup, ConfirmationZeroIsAlwaysANewCommand) {
    CommandDedup d;
    d.remember(kArm, 0, AckResult::Accepted);
    EXPECT_FALSE(d.lookup(kArm, 0, 10 * kNsPerMs).has_value());
}

TEST(CommandDedup, ExpiresAfterOneSecondAndDistinguishesCommandsAndSenders) {
    CommandDedup d;
    d.remember(kArm, 0, AckResult::Denied);
    EXPECT_FALSE(d.lookup(kArm, 1, 1001 * kNsPerMs).has_value());
    EXPECT_FALSE(d.lookup(CommandKey{255, 190, 176}, 1, 10 * kNsPerMs).has_value());
    EXPECT_FALSE(d.lookup(CommandKey{254, 190, 400}, 1, 10 * kNsPerMs).has_value());
}
