// Proves the core does not allocate after construction: the global allocation
// functions are replaced by counting versions (in this executable only), the
// core runs 60 s of model time with every fault active and a client streaming
// setpoints, and the count must stay at zero.

#include "../support/fc_harness.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

namespace {

std::atomic<bool> g_counting{false};
std::atomic<std::uint64_t> g_allocations{0};

void* counted_alloc(std::size_t size) {
    if (g_counting.load(std::memory_order_relaxed)) {
        g_allocations.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}

void* counted_aligned_alloc(std::size_t size, std::align_val_t align) {
    if (g_counting.load(std::memory_order_relaxed)) {
        g_allocations.fetch_add(1, std::memory_order_relaxed);
    }
    const auto a = static_cast<std::size_t>(align);
    if (void* p = std::aligned_alloc(a, (size + a - 1) / a * a)) {
        return p;
    }
    throw std::bad_alloc();
}

}  // namespace

void* operator new(std::size_t size) { return counted_alloc(size); }
void* operator new[](std::size_t size) { return counted_alloc(size); }
void* operator new(std::size_t size, std::align_val_t a) { return counted_aligned_alloc(size, a); }
void* operator new[](std::size_t size, std::align_val_t a) {
    return counted_aligned_alloc(size, a);
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {

using namespace fcstub;

FaultSpec make(FaultType type, double start, double duration, FaultParams params) {
    FaultSpec f;
    f.type = type;
    f.start_s = start;
    f.duration_s = duration;
    f.params = params;
    return f;
}

Config all_faults_config() {
    Config cfg = test::base_config();
    LinkFaultParams link;
    link.direction = LinkDirection::Both;
    link.p_good_to_bad = 0.05;
    link.p_bad_to_good = 0.3;
    link.loss_good = 0.01;
    link.delay_ms = 80;
    link.jitter_ms = 60;
    link.byte_error_rate = 0.002;
    GnssFaultParams gnss;
    gnss.kind = GnssKind::Drift;
    gnss.drift_mps = {0.3, 0.1, 0.0};
    ClockFaultParams clock;
    clock.drift_ppm = 500;
    clock.step_ms = 20;
    cfg.faults = {make(FaultType::Link, 8, 10, link),
                  make(FaultType::Gnss, 12, 20, gnss),
                  make(FaultType::EstimatorFreeze, 20, 5, EstimatorFreezeParams{false}),
                  make(FaultType::ClockFault, 25, 10, clock),
                  make(FaultType::FcReboot, 40, 0, FcRebootParams{3000}),
                  make(FaultType::Battery, 5, 0, BatteryFaultParams{0.5, 4.0}),
                  make(FaultType::ModeOverride, 15, 2,
                       ModeOverrideParams{OverrideTarget::Hold, OverrideCause::Geofence})};
    cfg.estimator = {0.5, 0.05, 0.01, 1.0};
    return cfg;
}

}  // namespace

TEST(NoAlloc, CoreRunsSixtySecondsWithAllFaultsWithoutAllocating) {
    test::Harness h(all_faults_config());
    h.station.record = false;  // recording would allocate in the test, not in the core
    h.core.advance_to(0, h.station);

    g_allocations = 0;
    g_counting = true;
    h.run_until(3 * kNsPerS);
    h.send(h.client.arm());
    h.send(h.client.velocity(1, 0, -1));
    h.send(h.client.set_mode(kPx4MainOffboard));
    for (TimeNs t = h.now; t < 60 * kNsPerS; t += 50 * kNsPerMs) {
        h.run_until(t);
        h.send(h.client.velocity(1, 0.5F, -1));
        if (t % (5 * kNsPerS) == 0) {
            h.send(h.client.set_mode(kPx4MainOffboard));  // back after the override
        }
        if (t % kNsPerS == 0) {
            h.send(h.client.timesync(t));
        }
    }
    h.send(h.client.velocity(1.0F / 0.0F, 0, 0));  // rejected setpoint path (status text)
    h.run_until(61 * kNsPerS);
    g_counting = false;

    EXPECT_EQ(g_allocations.load(), 0U);
    EXPECT_GT(h.station.frames, 3000U);
    EXPECT_GT(h.core.stats().statustexts, 3U);  // override, denials, reboot
    EXPECT_GT(h.core.stats().downlink.lost, 0U);
    EXPECT_GT(h.core.stats().decoder.frames_ok, 1000U);
}

// Guards against a vacuous pass: the counter must see an allocation.
TEST(NoAlloc, CounterDetectsAnAllocation) {
    g_allocations = 0;
    g_counting = true;
    // cppcheck-suppress unusedAllocatedMemory ; the allocation itself is the point
    auto* volatile p = new int(42);
    g_counting = false;
    delete p;
    EXPECT_EQ(g_allocations.load(), 1U);
}
