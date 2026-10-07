# Clean-machine build and test: only the repository content (fed in via
# `git archive`), a stock Ubuntu image and the packages listed here.
#   BASE         ubuntu:22.04 (default) or ubuntu:24.04; any --platform
#   EXTRA_PKGS   additional packages, e.g. "clang" or "gcovr"
#   CXX_COMPILER g++ (default) or clang++
#   PRESET       release (default), debug, asan or coverage
#   STEP         command to run, default: the one-command build and test
#   CTEST_EXTRA_ARGS  e.g. "-LE integration" under emulation, where timing is not real
ARG BASE=ubuntu:22.04
FROM ${BASE}
ARG EXTRA_PKGS=""
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git ca-certificates ${EXTRA_PKGS} \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . /src
ARG CXX_COMPILER=g++
ARG PRESET=release
ARG CTEST_EXTRA_ARGS=""
ARG STEP="./scripts/build_and_test.sh ${PRESET}"
ENV CXX=${CXX_COMPILER} CTEST_EXTRA_ARGS=${CTEST_EXTRA_ARGS}
RUN ${STEP}
