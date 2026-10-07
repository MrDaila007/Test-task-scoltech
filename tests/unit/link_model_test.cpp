#include "fcstub/link_model.hpp"
#include "fcstub/mav_codec.hpp"
#include "fcstub/rng.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

using namespace fcstub;

std::vector<std::uint8_t> payload(std::uint32_t id) {
    return {static_cast<std::uint8_t>(id), static_cast<std::uint8_t>(id >> 8U),
            static_cast<std::uint8_t>(id >> 16U), 0xAB};
}

std::uint32_t id_of(const LinkPacket& p) {
    return static_cast<std::uint32_t>(p.data[0]) | (static_cast<std::uint32_t>(p.data[1]) << 8U) |
           (static_cast<std::uint32_t>(p.data[2]) << 16U);
}

// Pushes `n` packets 20 ms apart through the link and drains everything.
std::vector<LinkPacket> send_all(LinkModel& link, int n, const LinkFaultParams* fault, Rng* rng) {
    std::vector<LinkPacket> out;
    LinkPacket p;
    for (int i = 0; i < n; ++i) {
        const TimeNs now = i * 20 * kNsPerMs;
        const auto bytes = payload(static_cast<std::uint32_t>(i));
        link.push(now, bytes.data(), bytes.size(), fault, rng);
        while (link.pop_due(now, p)) {
            out.push_back(p);
        }
    }
    while (link.pop_due(LinkModel::kNever - 1, p)) {
        out.push_back(p);
    }
    return out;
}

}  // namespace

TEST(LinkModel, CleanLinkDeliversImmediatelyInOrderUntouched) {
    LinkModel link(64, 64);
    const auto out = send_all(link, 100, nullptr, nullptr);
    ASSERT_EQ(out.size(), 100U);
    for (std::uint32_t i = 0; i < 100; ++i) {
        EXPECT_EQ(id_of(out[i]), i);
        EXPECT_EQ(out[i].release, static_cast<TimeNs>(i) * 20 * kNsPerMs);
        EXPECT_EQ(out[i].data[3], 0xAB);
    }
    EXPECT_EQ(link.stats().lost, 0U);
}

TEST(LinkModel, IndependentLossRate) {
    LinkModel link(64, 64);
    Rng rng(1, 1);
    LinkFaultParams f;
    f.p_good_to_bad = 0.0;
    f.loss_good = 0.1;
    const auto out = send_all(link, 100000, &f, &rng);
    const double loss = 1.0 - static_cast<double>(out.size()) / 100000.0;
    EXPECT_NEAR(loss, 0.1, 0.005);
    EXPECT_EQ(link.stats().lost, 100000U - out.size());
}

TEST(LinkModel, GilbertElliottLossesComeInBursts) {
    LinkModel link(64, 64);
    Rng rng(2, 1);
    LinkFaultParams f;
    f.p_good_to_bad = 0.01;
    f.p_bad_to_good = 0.1;
    f.loss_good = 0.0;
    f.loss_bad = 1.0;
    const auto out = send_all(link, 200000, &f, &rng);
    std::uint64_t bursts = 0;
    std::uint64_t lost = 0;
    std::uint32_t expected = 0;
    for (const auto& p : out) {
        const std::uint32_t id = id_of(p);
        if (id != expected) {
            ++bursts;
            lost += id - expected;
        }
        expected = id + 1;
    }
    ASSERT_GT(bursts, 500U);
    EXPECT_NEAR(static_cast<double>(lost) / static_cast<double>(bursts), 10.0, 1.0);
}

TEST(LinkModel, DelayAndJitterReorderButNeverReleaseEarly) {
    LinkModel link(256, 64);
    Rng rng(3, 1);
    LinkFaultParams f;
    f.delay_ms = 100.0;
    f.jitter_ms = 50.0;
    std::vector<TimeNs> sent(500);
    std::vector<LinkPacket> out;
    LinkPacket p;
    for (int i = 0; i < 500; ++i) {
        const TimeNs now = i * 20 * kNsPerMs;
        sent[static_cast<std::size_t>(i)] = now;
        const auto bytes = payload(static_cast<std::uint32_t>(i));
        link.push(now, bytes.data(), bytes.size(), &f, &rng);
        while (link.pop_due(now, p)) {
            out.push_back(p);
        }
    }
    while (link.pop_due(LinkModel::kNever - 1, p)) {
        out.push_back(p);
    }
    ASSERT_EQ(out.size(), 500U);
    int reordered = 0;
    for (std::size_t i = 0; i < out.size(); ++i) {
        const TimeNs delay = out[i].release - sent[id_of(out[i])];
        EXPECT_GE(delay, 50 * kNsPerMs);
        EXPECT_LE(delay, 150 * kNsPerMs);
        reordered += (i > 0 && id_of(out[i]) < id_of(out[i - 1])) ? 1 : 0;
    }
    EXPECT_GT(reordered, 0);
}

TEST(LinkModel, ByteErrorsBreakMavlinkCrc) {
    LinkModel link(64, kMaxFrameLen);
    Rng rng(4, 1);
    LinkFaultParams f;
    f.byte_error_rate = 0.01;
    MavEncoder enc(1, 1);
    MavDecoder dec;
    struct Sink : RxHandler {
        int n = 0;
        void on_message(const RxMessage&) noexcept override { ++n; }
    } sink;
    LinkPacket p;
    for (int i = 0; i < 1000; ++i) {
        const FrameBuf frame = enc.attitude({0, 0.1F, 0.2F, 0.3F, 0, 0, 0});
        link.push(0, frame.data.data(), frame.len, &f, &rng);
        while (link.pop_due(0, p)) {
            dec.feed(p.data.data(), p.len, sink);
        }
    }
    EXPECT_GT(dec.stats().crc_errors, 100U);
    EXPECT_GT(sink.n, 500);
    EXPECT_GT(link.stats().corrupted, 0U);
}

TEST(LinkModel, OverflowDropsTheOldestAndNeverGrows) {
    LinkModel link(16, 64);
    Rng rng(5, 1);
    LinkFaultParams f;
    f.delay_ms = 10000.0;  // everything stays queued
    for (int i = 0; i < 100; ++i) {
        const auto bytes = payload(static_cast<std::uint32_t>(i));
        link.push(i * kNsPerMs, bytes.data(), bytes.size(), &f, &rng);
    }
    EXPECT_EQ(link.queued(), 16U);
    EXPECT_EQ(link.stats().overflow, 84U);
    LinkPacket p;
    std::vector<std::uint32_t> ids;
    while (link.pop_due(LinkModel::kNever - 1, p)) {
        ids.push_back(id_of(p));
    }
    ASSERT_EQ(ids.size(), 16U);
    EXPECT_EQ(ids.front(), 84U);  // the newest 16 survived
}

TEST(LinkModel, OversizedPacketIsRejected) {
    LinkModel link(16, 8);
    const std::vector<std::uint8_t> big(9, 0);
    link.push(0, big.data(), big.size(), nullptr, nullptr);
    EXPECT_EQ(link.stats().oversize, 1U);
    EXPECT_EQ(link.queued(), 0U);
}

TEST(LinkModel, SameSeedSameDecisions) {
    LinkFaultParams f;
    f.p_good_to_bad = 0.05;
    f.p_bad_to_good = 0.3;
    f.loss_good = 0.02;
    f.delay_ms = 30.0;
    f.jitter_ms = 25.0;
    f.byte_error_rate = 0.001;
    LinkModel a(256, 64);
    LinkModel b(256, 64);
    Rng ra(9, 3);
    Rng rb(9, 3);
    const auto oa = send_all(a, 5000, &f, &ra);
    const auto ob = send_all(b, 5000, &f, &rb);
    ASSERT_EQ(oa.size(), ob.size());
    for (std::size_t i = 0; i < oa.size(); ++i) {
        ASSERT_EQ(oa[i].release, ob[i].release);
        ASSERT_EQ(oa[i].data, ob[i].data);
    }
}
