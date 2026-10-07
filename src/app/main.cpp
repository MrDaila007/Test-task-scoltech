// fc_stub: PX4-like flight controller stub with fault injection.
//
// Exit codes: 0 ok, 1 runtime error, 2 configuration error, 3 network error at
// start-up, 64 bad command line.

#include "fcstub/config_loader.hpp"
#include "fcstub/realtime_driver.hpp"
#include "fcstub/sim_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>

namespace {

constexpr int kExitOk = 0;
constexpr int kExitRuntime = 1;
constexpr int kExitConfig = 2;
constexpr int kExitUsage = 64;

constexpr const char* kUsage =
    "usage: fc_stub --config FILE [--sim] [--seed N] [--duration S] [--out DIR]\n"
    "               [--report FILE] [--validate] [--print-param-count] [--version] [--help]\n"
    "\n"
    "  --config FILE          scenario/configuration (YAML, schema_version 1)\n"
    "  --sim                  model time: deterministic run, writes frames.bin, truth.csv,\n"
    "                         sha256.txt to the output directory and prints the hash\n"
    "  --seed N               override run.seed\n"
    "  --duration S           override run.duration_s (0 = until SIGINT/SIGTERM, realtime)\n"
    "  --out DIR              override sim.out_dir\n"
    "  --report FILE          override realtime.report_path (JSON jitter/traffic report)\n"
    "  --validate             check the configuration and exit\n"
    "  --print-param-count    print the number of configuration parameters and exit\n";

struct Options {
    std::optional<std::string> config, seed, duration, out, report;
    bool sim = false;
    bool validate = false;
    bool print_count = false;
    bool help = false;
    bool version = false;
};

// Option that takes a value -> where to store it; nullptr if not such an option.
std::optional<std::string>* value_slot(Options& o, const std::string& name) {
    if (name == "--config") {
        return &o.config;
    }
    if (name == "--seed") {
        return &o.seed;
    }
    if (name == "--duration") {
        return &o.duration;
    }
    if (name == "--out") {
        return &o.out;
    }
    return name == "--report" ? &o.report : nullptr;
}

// Option without a value -> its flag; nullptr if not such an option.
bool* flag_slot(Options& o, const std::string& name) {
    if (name == "--sim") {
        return &o.sim;
    }
    if (name == "--validate") {
        return &o.validate;
    }
    if (name == "--print-param-count") {
        return &o.print_count;
    }
    if (name == "--help" || name == "-h") {
        return &o.help;
    }
    return name == "--version" ? &o.version : nullptr;
}

// Returns false (after printing why) on a malformed command line.
bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string name = argv[i];
        if (bool* flag = flag_slot(o, name)) {
            *flag = true;
        } else if (std::optional<std::string>* slot = value_slot(o, name)) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "fc_stub: %s needs a value\n", name.c_str());
                return false;
            }
            *slot = argv[++i];
        } else {
            std::fprintf(stderr, "fc_stub: unknown option '%s'\n", name.c_str());
            return false;
        }
    }
    if (!o.help && !o.version && !o.config) {
        std::fprintf(stderr, "fc_stub: --config is required\n");
        return false;
    }
    return true;
}

void apply_overrides(const Options& o, fcstub::Config& cfg) {
    try {
        if (o.sim) {
            cfg.run.mode = fcstub::RunMode::Sim;
        }
        if (o.seed) {
            cfg.run.seed = std::stoull(*o.seed);
        }
        if (o.duration) {
            cfg.run.duration_s = std::stod(*o.duration);
        }
    } catch (const std::exception&) {
        throw fcstub::ConfigError("<command line>", "--seed/--duration expect a number");
    }
    if (o.out) {
        cfg.sim.out_dir = *o.out;
    }
    if (o.report) {
        cfg.realtime.report_path = *o.report;
    }
    fcstub::validate_config(cfg);
}

int run(const Options& o) {
    if (o.print_count) {
        std::printf("%zu\n", fcstub::count_leaf_params(*o.config));
        return kExitOk;
    }
    fcstub::Config cfg = fcstub::load_config(*o.config);
    apply_overrides(o, cfg);
    if (o.validate) {
        std::printf("config OK (%zu parameters)\n", fcstub::count_leaf_params(*o.config));
        return kExitOk;
    }
    if (cfg.run.mode == fcstub::RunMode::Sim) {
        const fcstub::SimResult res = fcstub::SimDriver(cfg).run();
        std::printf("sha256 %s\n", res.sha256.c_str());
        return kExitOk;
    }
    std::string error;
    const int code = fcstub::RealtimeDriver(cfg).run(error);
    if (code != kExitOk) {
        std::fprintf(stderr, "fc_stub: %s\n", error.c_str());
    }
    return code;
}

}  // namespace

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        std::fputs(kUsage, stderr);
        return kExitUsage;
    }
    if (opts.help) {
        std::fputs(kUsage, stdout);
        return kExitOk;
    }
    if (opts.version) {
        std::puts("fc_stub 0.1.0");
        return kExitOk;
    }
    try {
        return run(opts);
    } catch (const fcstub::ConfigError& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return kExitConfig;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fc_stub: %s\n", e.what());
        return kExitRuntime;
    }
}
