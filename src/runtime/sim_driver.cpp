#include "fcstub/sim_driver.hpp"

#include "fcstub/fc_core.hpp"
#include "fcstub/geo.hpp"
#include "fcstub/sha256.hpp"
#include "fcstub/virtual_client.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace fcstub {

namespace {

constexpr std::uint32_t kJournalVersion = 1;

template <typename T>
void put_le(std::string& out, T value) {
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        out.push_back(static_cast<char>((static_cast<std::uint64_t>(value) >> (8U * i)) & 0xFFU));
    }
}

class Recorder : public FrameSink, public ClientSink {
public:
    Recorder(FcCore& core, std::ofstream& journal) : core_(core), journal_(journal) {}

    void emit(TimeNs t, const std::uint8_t* data, std::size_t len,
              const PacketTag& /*tag*/) noexcept override {
        const std::string record = encode(t, 0, data, len);
        sha_.update(reinterpret_cast<const std::uint8_t*>(record.data()), record.size());
        journal_.write(record.data(), static_cast<std::streamsize>(record.size()));
        ++frames_down;
    }

    void send(TimeNs t, const std::uint8_t* data, std::size_t len) noexcept override {
        const std::string record = encode(t, 1, data, len);
        journal_.write(record.data(), static_cast<std::streamsize>(record.size()));
        ++frames_up;
        core_.on_rx_bytes(t, data, len);
    }

    std::string digest() { return Sha256::to_hex(sha_.finish()); }

    std::uint64_t frames_down = 0;
    std::uint64_t frames_up = 0;

private:
    static std::string encode(TimeNs t, std::uint8_t dir, const std::uint8_t* data,
                              std::size_t len) {
        std::string r;
        r.reserve(11 + len);
        put_le<std::int64_t>(r, t);
        put_le<std::uint8_t>(r, dir);
        put_le<std::uint16_t>(r, static_cast<std::uint16_t>(len));
        r.append(reinterpret_cast<const char*>(data), len);
        return r;
    }

    FcCore& core_;
    std::ofstream& journal_;
    Sha256 sha_;
};

void write_csv_header(std::ofstream& csv) {
    csv << "t_s,mode,armed,faults,true_n,true_e,true_d,true_vn,true_ve,true_vd,rep_lat_e7,"
           "rep_lon_e7,rep_alt_mm,rep_vn,rep_ve,rep_vd,err_h_m,t_fc_ms\n";
}

void write_csv_row(std::ofstream& csv, const FcCore& core, TimeNs t) {
    const VehicleState& tr = core.truth();
    const VehicleState& est = core.estimate();
    const GeoPoint geo = ned_to_wgs84(core.origin(), est.pos[0], est.pos[1], est.pos[2]);
    const double err_h = std::hypot(est.pos[0] - tr.pos[0], est.pos[1] - tr.pos[1]);
    char line[512];
    std::snprintf(line, sizeof(line),
                  "%.3f,%s,%d,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lld,%lld,%lld,%.3f,%.3f,%.3f,%.3f,"
                  "%lld\n",
                  ns_to_seconds(t), mode_name(core.mode()), core.armed() ? 1 : 0,
                  core.active_faults(t), tr.pos[0], tr.pos[1], tr.pos[2], tr.vel[0], tr.vel[1],
                  tr.vel[2], std::llround(geo.lat_deg * 1e7), std::llround(geo.lon_deg * 1e7),
                  std::llround(geo.alt_msl_m * 1000.0), est.vel[0], est.vel[1], est.vel[2], err_h,
                  static_cast<long long>(core.fc_time_ns(t) / kNsPerMs));
    csv << line;
}

std::ofstream open_out(const std::filesystem::path& p, std::ios::openmode mode) {
    std::ofstream f(p, mode);
    if (!f) {
        throw std::runtime_error("cannot write '" + p.string() + "'");
    }
    return f;
}

}  // namespace

SimDriver::SimDriver(const Config& cfg) : cfg_(cfg) {}

SimResult SimDriver::run() {
    const std::filesystem::path dir(cfg_.sim.out_dir);
    std::filesystem::create_directories(dir);
    std::ofstream journal = open_out(dir / "frames.bin", std::ios::binary | std::ios::trunc);
    journal.write("FCSJ", 4);
    std::string version;
    put_le<std::uint32_t>(version, kJournalVersion);
    journal.write(version.data(), static_cast<std::streamsize>(version.size()));

    const bool with_csv = cfg_.sim.truth_csv_hz > 0.0;
    std::ofstream csv;
    if (with_csv) {
        csv = open_out(dir / "truth.csv", std::ios::trunc);
        write_csv_header(csv);
    }

    // Heap-allocated: the core and client are large and live for the whole run.
    auto core = std::make_unique<FcCore>(cfg_);
    VirtualClient client(cfg_.client, cfg_.identity.system_id, cfg_.identity.component_id);
    Recorder rec(*core, journal);

    const TimeNs end = seconds_to_ns(cfg_.run.duration_s);
    const TimeNs csv_period = with_csv ? period_from_hz(cfg_.sim.truth_csv_hz) : 0;
    std::uint64_t csv_rows = 0;
    TimeNs next_csv = with_csv ? 0 : std::numeric_limits<TimeNs>::max();
    for (;;) {
        const TimeNs t = std::min({core->next_deadline(), client.next_event(), next_csv});
        if (t > end) {
            break;
        }
        if (client.next_event() == t) {
            client.emit(t, rec);
        }
        core->advance_to(t, rec);
        if (t == next_csv) {
            write_csv_row(csv, *core, t);
            next_csv = static_cast<TimeNs>(++csv_rows) * csv_period;
        }
    }

    SimResult result{rec.digest(), rec.frames_down, rec.frames_up};
    open_out(dir / "sha256.txt", std::ios::trunc) << result.sha256 << "\n";
    if (!journal || (with_csv && !csv)) {
        throw std::runtime_error("write error in '" + dir.string() + "'");
    }
    return result;
}

}  // namespace fcstub
