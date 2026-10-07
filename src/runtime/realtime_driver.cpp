#include "fcstub/realtime_driver.hpp"

#include "fcstub/fc_core.hpp"
#include "fcstub/jitter_stats.hpp"
#include "fcstub/udp_link.hpp"

#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>

namespace fcstub {

namespace {

volatile sig_atomic_t g_stop = 0;

void on_signal(int /*signo*/) { g_stop = 1; }

TimeNs monotonic_ns() noexcept {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<TimeNs>(ts.tv_sec) * kNsPerS + ts.tv_nsec;
}

timespec to_timespec(TimeNs ns) noexcept {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(ns / kNsPerS);
    ts.tv_nsec = static_cast<long>(ns % kNsPerS);
    return ts;
}

struct HostInfo {
    std::string kernel = "unknown";
    std::string arch = "unknown";
    std::string governor = "unknown";
    bool sched_fifo = false;
    bool mlockall = false;
};

HostInfo host_info() {
    HostInfo h;
    utsname u{};
    if (uname(&u) == 0) {
        h.kernel = u.release;
        h.arch = u.machine;
    }
    std::ifstream gov("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    std::getline(gov, h.governor);
    if (h.governor.empty()) {
        h.governor = "unknown";
    }
    return h;
}

class SocketSink final : public FrameSink {
public:
    SocketSink(UdpLink& link, const FcCore& core, TimeNs t0) : link_(link), core_(core), t0_(t0) {}

    void emit(TimeNs /*t*/, const std::uint8_t* data, std::size_t len,
              const PacketTag& tag) noexcept override {
        const TimeNs t_send = monotonic_ns() - t0_;
        if (tag.stream < kStreamCount && (core_.active_faults(tag.deadline) &
                                          (1U << static_cast<unsigned>(FaultType::Link))) == 0) {
            jitter[tag.stream].add(t_send - tag.deadline);
        }
        if (link_.send(data, len)) {
            ++sent;
        } else {
            ++send_errors;
        }
    }

    std::array<JitterStats, kStreamCount> jitter;
    std::uint64_t sent = 0;
    std::uint64_t send_errors = 0;

private:
    UdpLink& link_;
    const FcCore& core_;
    TimeNs t0_;
};

void enable_realtime(const RealtimeConfig& rt, HostInfo& host) {
    if (rt.lock_memory) {
        host.mlockall = mlockall(MCL_CURRENT | MCL_FUTURE) == 0 || mlockall(MCL_CURRENT) == 0;
        if (!host.mlockall) {
            std::fprintf(stderr, "warning: mlockall failed (%s); page faults may add jitter\n",
                         std::strerror(errno));
        }
    }
    if (rt.sched_fifo) {
        sched_param sp{};
        sp.sched_priority = rt.priority;
        host.sched_fifo = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
        if (!host.sched_fifo) {
            std::fprintf(stderr,
                         "warning: SCHED_FIFO not permitted (%s); running with normal priority. "
                         "Grant CAP_SYS_NICE or an rtprio limit for loaded systems.\n",
                         std::strerror(errno));
        }
    }
}

void install_signals(sigset_t& wait_mask) {
    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigset_t block;
    sigemptyset(&block);
    sigaddset(&block, SIGINT);
    sigaddset(&block, SIGTERM);
    sigprocmask(SIG_BLOCK, &block, &wait_mask);  // wait_mask = mask to use inside ppoll
    sigdelset(&wait_mask, SIGINT);
    sigdelset(&wait_mask, SIGTERM);
}

std::uint64_t rejected_setpoints(const GateStats& g) noexcept {
    std::uint64_t n = 0;
    for (std::size_t i = 0; i < g.rejected.size(); ++i) {
        n += i == static_cast<std::size_t>(RejectReason::NotForUs) ? 0 : g.rejected[i];
    }
    return n;
}

std::string build_report(const HostInfo& host, double duration_s, const SocketSink& sink,
                         const CoreStats& st, const Config& cfg) {
    const double rates[kStreamCount] = {cfg.telemetry_hz.heartbeat, cfg.telemetry_hz.attitude,
                                        cfg.telemetry_hz.global_position, cfg.telemetry_hz.battery};
    std::string out;
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "{\n  \"host\": {\"kernel\": \"%s\", \"arch\": \"%s\", \"governor\": \"%s\", "
        "\"sched_fifo\": %s, \"mlockall\": %s},\n  \"duration_s\": %.3f,\n  \"streams\": {",
        host.kernel.c_str(), host.arch.c_str(), host.governor.c_str(),
        host.sched_fifo ? "true" : "false", host.mlockall ? "true" : "false", duration_s);
    out += buf;
    bool ok = true;
    for (std::size_t i = 0; i < kStreamCount; ++i) {
        const JitterStats& j = sink.jitter[i];
        const double max_us = static_cast<double>(j.max_ns()) / 1000.0;
        ok = ok && max_us <= 1000.0;
        std::snprintf(
            buf, sizeof(buf),
            "%s\n    \"%s\": {\"period_us\": %.0f, \"n\": %llu, \"err_us\": {\"p50\": %.0f, "
            "\"p99\": %.0f, \"max\": %.1f}, \"missed\": %llu}",
            i == 0 ? "" : ",", stream_name(static_cast<Stream>(i)), 1e6 / rates[i],
            static_cast<unsigned long long>(j.count()), j.percentile_us(0.5), j.percentile_us(0.99),
            max_us, static_cast<unsigned long long>(st.missed_per_stream[i]));
        out += buf;
    }
    std::snprintf(
        buf, sizeof(buf),
        "\n  },\n  \"rx\": {\"datagrams\": %llu, \"frames\": %llu, \"crc_errors\": %llu, "
        "\"setpoints_rejected\": %llu},\n  \"tx\": {\"frames\": %llu, \"send_errors\": %llu},"
        "\n  \"jitter_ok\": %s\n}\n",
        static_cast<unsigned long long>(st.rx_datagrams),
        static_cast<unsigned long long>(st.decoder.frames_ok),
        static_cast<unsigned long long>(st.decoder.crc_errors),
        static_cast<unsigned long long>(rejected_setpoints(st.gate)),
        static_cast<unsigned long long>(sink.sent),
        static_cast<unsigned long long>(sink.send_errors), ok ? "true" : "false");
    out += buf;
    return out;
}

}  // namespace

RealtimeDriver::RealtimeDriver(const Config& cfg) : cfg_(cfg) {}

int RealtimeDriver::run(std::string& error) {
    UdpLink link = UdpLink::open(cfg_.link, error);
    if (!link.valid()) {
        return 3;
    }
    // Everything the loop needs is allocated before memory is locked: with
    // MCL_FUTURE every later allocation counts against RLIMIT_MEMLOCK and can fail.
    auto core = std::make_unique<FcCore>(cfg_);
    std::vector<std::uint8_t> rx(kMaxLinkPacket + 1);
    HostInfo host = host_info();
    const TimeNs t0 = monotonic_ns();
    auto sink = std::make_unique<SocketSink>(link, *core, t0);
    sigset_t wait_mask;
    install_signals(wait_mask);
    enable_realtime(cfg_.realtime, host);
    const TimeNs end = cfg_.run.duration_s > 0 ? seconds_to_ns(cfg_.run.duration_s)
                                               : std::numeric_limits<TimeNs>::max();
    const TimeNs spin = cfg_.realtime.spin_us * kNsPerUs;
    pollfd pfd{link.fd(), POLLIN, 0};
    TimeNs now = 0;
    core->advance_to(0, *sink);
    while (g_stop == 0 && now < end) {
        const TimeNs deadline = std::min(core->next_deadline(), end);
        const TimeNs wake = deadline - spin;
        now = monotonic_ns() - t0;
        pfd.revents = 0;
        if (wake > now) {
            const timespec timeout = to_timespec(wake - now);
            ppoll(&pfd, 1, &timeout, &wait_mask);
        }
        while (spin > 0 && pfd.revents == 0 && monotonic_ns() - t0 < deadline) {
        }
        now = monotonic_ns() - t0;
        for (int i = 0; i < cfg_.link.rx_max_frames_per_iter; ++i) {
            const long n = link.receive(rx.data(), kMaxLinkPacket);
            if (n <= 0) {
                break;
            }
            core->on_rx_bytes(now, rx.data(), static_cast<std::size_t>(n));
        }
        core->advance_to(now, *sink);
    }

    if (host.mlockall) {
        munlockall();  // the report below allocates; the real-time part is over
    }
    const std::string report = build_report(host, ns_to_seconds(now), *sink, core->stats(), cfg_);
    if (cfg_.realtime.report_path.empty()) {
        std::fputs(report.c_str(), stderr);
        return 0;
    }
    std::ofstream out(cfg_.realtime.report_path, std::ios::trunc);
    out << report;
    if (!out) {
        error = "cannot write report '" + cfg_.realtime.report_path + "'";
        return 1;
    }
    return 0;
}

}  // namespace fcstub
