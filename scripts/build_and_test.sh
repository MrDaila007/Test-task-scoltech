#!/usr/bin/env bash
# One-command build and test on a clean machine.
# Usage: scripts/build_and_test.sh [release|debug|asan|coverage]   (default: release)
set -euo pipefail

preset="${1:-release}"
case "${preset}" in
    release|debug|asan|coverage) ;;
    *) echo "usage: $0 [release|debug|asan|coverage]" >&2; exit 64 ;;
esac

cd "$(dirname "$0")/.."

cmake --preset "${preset}"
cmake --build --preset "${preset}" -j "$(nproc)"
ctest --preset "${preset}" -j "$(nproc)"
