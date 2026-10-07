#!/usr/bin/env bash
# Line coverage of the project's own code (src/, without third-party), gcov-based.
# Needs gcovr (`apt install gcovr` or `pip install gcovr`); fails under 80 %.
set -euo pipefail
cd "$(dirname "$0")/.."
./scripts/build_and_test.sh coverage
gcovr --root . --filter 'src/' --exclude 'src/app/' build/coverage \
      --print-summary --fail-under-line 80
