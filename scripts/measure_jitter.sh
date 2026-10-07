#!/usr/bin/env bash
# Measures telemetry send jitter of the real-time stub on this machine.
#
# Usage: scripts/measure_jitter.sh [idle|load] [duration_s] [spin_us]
#   idle  - nothing else running (default)
#   load  - one busy-loop per CPU core during the run (stress-ng if installed)
# Writes out/jitter_<arch>_<mode>_spin<us>.json and prints a summary.
#
# For best results on a loaded or embedded target (e.g. Jetson): allow SCHED_FIFO
# (sudo setcap cap_sys_nice+ep build/release/src/fc_stub, or an rtprio limit),
# fix the CPU frequency (performance governor; on Jetson: nvpmodel + jetson_clocks)
# and prefer a PREEMPT_RT kernel.
set -euo pipefail

mode="${1:-idle}"
duration="${2:-60}"
spin="${3:-0}"
case "${mode}" in idle|load) ;; *) echo "usage: $0 [idle|load] [duration_s] [spin_us]" >&2; exit 64 ;; esac

cd "$(dirname "$0")/.."
bin=build/release/src/fc_stub
[[ -x "${bin}" ]] || { cmake --preset release >/dev/null && cmake --build --preset release >/dev/null; }

mkdir -p out
report="out/jitter_$(uname -m)_${mode}_spin${spin}.json"
config="$(mktemp)"
trap 'rm -f "${config}"; [[ -n "${load_pids:-}" ]] && kill ${load_pids} 2>/dev/null || true' EXIT
cat > "${config}" <<EOF
schema_version: 1
run: {duration_s: ${duration}}
link: {bind_addr: 127.0.0.1, bind_port: 0, remote_port: 9}
realtime: {spin_us: ${spin}, report_path: "${report}"}
EOF
# bind_port 0 is out of range for the config schema; pick a free port instead.
port=$(python3 -c 'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')
sed -i "s/bind_port: 0/bind_port: ${port}/" "${config}"

load_pids=""
if [[ "${mode}" == load ]]; then
    if command -v stress-ng >/dev/null; then
        stress-ng --cpu "$(nproc)" --timeout "$((duration + 2))s" >/dev/null 2>&1 &
        load_pids=$!
    else
        for _ in $(seq "$(nproc)"); do
            ( end=$((SECONDS + duration + 2)); while (( SECONDS < end )); do :; done ) &
            load_pids="${load_pids} $!"
        done
    fi
fi

"${bin}" --config "${config}"
echo "report: ${report}"
python3 - "${report}" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
h = r["host"]
print(f"kernel {h['kernel']} {h['arch']}, governor {h['governor']}, "
      f"SCHED_FIFO {h['sched_fifo']}, mlockall {h['mlockall']}")
for name, s in r["streams"].items():
    e = s["err_us"]
    print(f"  {name:16} n={s['n']:6}  p50={e['p50']:6.0f} us  p99={e['p99']:6.0f} us  "
          f"max={e['max']:8.1f} us  missed={s['missed']}")
print("jitter <= 1 ms:", "PASS" if r["jitter_ok"] else "FAIL")
PY
