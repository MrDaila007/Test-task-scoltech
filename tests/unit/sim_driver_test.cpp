#include "fcstub/sim_driver.hpp"
#include "fcstub/config_loader.hpp"
#include "fcstub/virtual_client.hpp"

#include <common/mavlink.h>
#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using namespace fcstub;

const std::string kScenario = R"(
schema_version: 1
run: {mode: sim, seed: 42, duration_s: 30}
client:
  - {kind: command, at_s: 3, cmd: arm}
  - {kind: setpoints, start_s: 3.5, end_s: 25, rate_hz: 20, frame: velocity, value: [2, 0, -1]}
  - {kind: command, at_s: 4, cmd: offboard}
  - {kind: timesync, start_s: 1, end_s: 29, rate_hz: 1}
)";

fs::path fresh_dir(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / ("fcstub_sim_" + name);
    fs::remove_all(dir);
    return dir;
}

SimResult run_sim(const std::string& yaml, const std::string& name) {
    Config cfg = load_config_from_string(yaml);
    cfg.sim.out_dir = fresh_dir(name).string();
    return SimDriver(cfg).run();
}

std::string read_file(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), {});
}

struct JournalRecord {
    std::int64_t t;
    std::uint8_t dir;
    std::vector<std::uint8_t> bytes;
};

std::vector<JournalRecord> read_journal(const fs::path& p, std::uint32_t& version) {
    const std::string data = read_file(p);
    EXPECT_EQ(data.substr(0, 4), "FCSJ");
    std::memcpy(&version, data.data() + 4, 4);
    std::vector<JournalRecord> out;
    std::size_t off = 8;
    while (off + 11 <= data.size()) {
        JournalRecord r{};
        std::memcpy(&r.t, data.data() + off, 8);
        r.dir = static_cast<std::uint8_t>(data[off + 8]);
        std::uint16_t len = 0;
        std::memcpy(&len, data.data() + off + 9, 2);
        r.bytes.assign(data.begin() + static_cast<std::ptrdiff_t>(off + 11),
                       data.begin() + static_cast<std::ptrdiff_t>(off + 11 + len));
        out.push_back(r);
        off += 11U + len;
    }
    EXPECT_EQ(off, data.size());
    return out;
}

std::string with_faults(const std::string& faults) { return kScenario + "faults:\n" + faults; }

}  // namespace

TEST(VirtualClient, ScheduleFollowsActions) {
    Config cfg = load_config_from_string(kScenario);
    VirtualClient client(cfg.client);
    EXPECT_EQ(client.next_event(), seconds_to_ns(1.0));  // first timesync
    int setpoints = 0;
    struct Counter : ClientSink {
        int* n;
        void send(TimeNs, const std::uint8_t* data, std::size_t len) noexcept override {
            mavlink_message_t msg{};
            mavlink_status_t st{};
            mavlink_message_t out{};
            mavlink_status_t out_st{};
            for (std::size_t i = 0; i < len; ++i) {
                if (mavlink_frame_char_buffer(&msg, &st, data[i], &out, &out_st) == 1 &&
                    out.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
                    ++*n;
                }
            }
        }
    } sink;
    sink.n = &setpoints;
    while (client.next_event() <= seconds_to_ns(30.0)) {
        client.emit(client.next_event(), sink);
    }
    EXPECT_EQ(setpoints, 430);  // [3.5, 25) at 20 Hz
    EXPECT_EQ(client.next_event(), VirtualClient::kNever);
}

TEST(SimDriver, JournalHasTheExpectedTelemetry) {
    Config cfg = load_config_from_string(kScenario);
    cfg.sim.out_dir = fresh_dir("journal").string();
    const SimResult res = SimDriver(cfg).run();
    std::uint32_t version = 0;
    const auto records = read_journal(fs::path(cfg.sim.out_dir) / "frames.bin", version);
    EXPECT_EQ(version, 1U);
    std::size_t att = 0;
    std::size_t up = 0;
    for (const auto& r : records) {
        if (r.dir == 1) {
            ++up;
        } else if (r.bytes.size() > 7 && r.bytes[7] == MAVLINK_MSG_ID_ATTITUDE) {
            ++att;
        }
    }
    EXPECT_NEAR(static_cast<double>(att), 30 * 50, 1);
    EXPECT_EQ(up, res.frames_up);
    EXPECT_GT(res.frames_down, 1800U);
    EXPECT_EQ(read_file(fs::path(cfg.sim.out_dir) / "sha256.txt"), res.sha256 + "\n");
    EXPECT_EQ(res.sha256.size(), 64U);
}

TEST(SimDriver, TruthCsvRowsAndHeader) {
    Config cfg = load_config_from_string(kScenario);
    cfg.sim.out_dir = fresh_dir("csv").string();
    (void)SimDriver(cfg).run();
    std::ifstream in(fs::path(cfg.sim.out_dir) / "truth.csv");
    std::string header;
    std::getline(in, header);
    EXPECT_EQ(header,
              "t_s,mode,armed,faults,true_n,true_e,true_d,true_vn,true_ve,true_vd,rep_lat_e7,"
              "rep_lon_e7,rep_alt_mm,rep_vn,rep_ve,rep_vd,err_h_m,t_fc_ms");
    int rows = 0;
    std::string line;
    std::string last;
    while (std::getline(in, line)) {
        ++rows;
        last = line;
    }
    EXPECT_EQ(rows, 301);  // 0.0 .. 30.0 s at 10 Hz
    EXPECT_EQ(last.substr(0, 7), "30.000,");
}

TEST(SimDriver, SameSeedSameHash) {
    const std::string yaml = with_faults(
        "  - {type: link, start_s: 5, duration_s: 10, direction: both, p_good_to_bad: 0.05,"
        " p_bad_to_good: 0.3, loss_good: 0.01, delay_ms: 50, jitter_ms: 40, byte_error_rate: "
        "0.001}\n");
    EXPECT_EQ(run_sim(yaml, "a").sha256, run_sim(yaml, "b").sha256);
}

TEST(SimDriver, DifferentSeedDifferentHashWhenRandomnessIsUsed) {
    const std::string faults =
        "  - {type: link, start_s: 5, duration_s: 10, direction: down, loss_good: 0.2}\n";
    std::string other = with_faults(faults);
    other.replace(other.find("seed: 42"), 8, "seed: 43");
    EXPECT_NE(run_sim(with_faults(faults), "s42").sha256, run_sim(other, "s43").sha256);
}

TEST(SimDriver, FaultStreamsAreIndependent) {
    // A downlink fault must not change what the vehicle does or what the GNSS drift
    // fault produces: compare truth.csv with and without it.
    const std::string gnss =
        "  - {type: gnss, kind: drift, start_s: 8, duration_s: 15, drift_mps: [0.2, 0, 0]}\n";
    const std::string link =
        "  - {type: link, start_s: 6, duration_s: 20, direction: down, loss_good: 0.3,"
        " delay_ms: 30, jitter_ms: 20}\n";
    Config a = load_config_from_string(with_faults(gnss));
    Config b = load_config_from_string(with_faults(gnss + link));
    a.sim.out_dir = fresh_dir("ind_a").string();
    b.sim.out_dir = fresh_dir("ind_b").string();
    (void)SimDriver(a).run();
    (void)SimDriver(b).run();
    std::ifstream fa(fs::path(a.sim.out_dir) / "truth.csv");
    std::ifstream fb(fs::path(b.sim.out_dir) / "truth.csv");
    std::string la;
    std::string lb;
    int compared = 0;
    while (std::getline(fa, la) && std::getline(fb, lb)) {
        // Drop the fault-mask column (index 3), which legitimately differs.
        auto strip = [](std::string s) {
            std::size_t pos = 0;
            for (int i = 0; i < 3; ++i) {
                pos = s.find(',', pos) + 1;
            }
            return s.erase(pos, s.find(',', pos) - pos);
        };
        ASSERT_EQ(strip(la), strip(lb)) << "row " << compared;
        ++compared;
    }
    EXPECT_EQ(compared, 302);
}

// Pinned on x86_64 (g++ 11.4, -ffp-contract=off). The same value is expected on
// aarch64; a deliberate change to the output stream must update it.
TEST(SimDriver, GoldenScenarioHash) {
    Config cfg =
        load_config(std::string(FCSTUB_SOURCE_DIR) + "/tests/fixtures/golden_all_faults.yaml");
    cfg.sim.out_dir = fresh_dir("golden").string();
    const SimResult res = SimDriver(cfg).run();
    EXPECT_EQ(res.sha256, "20517a384ed92b5c30f8fc70147d3d7c94148c0e6eee0ced69703e91dcc310b8");
}
