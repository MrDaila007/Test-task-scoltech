#!/usr/bin/env bash
# Static checks: cppcheck over the project's own sources (third-party excluded) and
# clang-format conformance. Sanitizers run via `scripts/build_and_test.sh asan`.
set -euo pipefail
cd "$(dirname "$0")/.."

[[ -f build/debug/compile_commands.json ]] || cmake --preset debug >/dev/null

echo "== cppcheck"
cppcheck --project=build/debug/compile_commands.json \
    --enable=warning,performance,portability,style --std=c++17 --inline-suppr \
    --suppress='*:*/_deps/*' -i build/debug/_deps \
    --suppress=useStlAlgorithm \
    --error-exitcode=1 --quiet
echo "cppcheck: no findings"

echo "== clang-format"
mapfile -t files < <(git ls-files '*.cpp' '*.hpp')
clang-format --dry-run --Werror "${files[@]}"
echo "clang-format: ${#files[@]} files conform"
