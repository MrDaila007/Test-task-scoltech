#!/usr/bin/env bash
# Builds and tests the committed tree (git archive HEAD) in fresh containers:
#   ubuntu:22.04 g++ | ubuntu:24.04 g++ | ubuntu:22.04 clang++ | coverage | arm64 (qemu)
# Usage: scripts/verify_clean_machine.sh [all|x86|clang|coverage|arm64]
set -euo pipefail
cd "$(dirname "$0")/.."
what="${1:-all}"

build() {  # name, docker build args...
    local name="$1"; shift
    echo "=== ${name}"
    if git archive --format=tar HEAD | docker build -q -f docker/Dockerfile "$@" - >/dev/null; then
        echo "PASS ${name}"
    else
        echo "FAIL ${name}"; return 1
    fi
}

rc=0
if [[ "${what}" == all || "${what}" == x86 ]]; then
    build "ubuntu:22.04 g++" --build-arg BASE=ubuntu:22.04 || rc=1
    build "ubuntu:24.04 g++" --build-arg BASE=ubuntu:24.04 || rc=1
fi
if [[ "${what}" == all || "${what}" == clang ]]; then
    build "ubuntu:22.04 clang++" --build-arg EXTRA_PKGS=clang --build-arg CXX_COMPILER=clang++ || rc=1
fi
if [[ "${what}" == all || "${what}" == coverage ]]; then
    build "coverage >= 80 %" --build-arg EXTRA_PKGS=gcovr --build-arg STEP=./scripts/coverage.sh || rc=1
fi
if [[ "${what}" == all || "${what}" == arm64 ]]; then
    # Emulated: functional tests and the golden hash only; real-time tests are excluded.
    build "ubuntu:22.04 g++ arm64 (qemu)" --platform linux/arm64 \
        --build-arg "CTEST_EXTRA_ARGS=-LE integration" || rc=1
fi
exit "${rc}"
