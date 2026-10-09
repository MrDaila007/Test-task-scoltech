# Shortcuts for building, testing and running fc_stub. Every target wraps a script
# or a CMake preset, so the commands in README.md stay the source of truth.
# `make` alone prints the list.

SHELL := /bin/bash
.DEFAULT_GOAL := help

BIN      := build/release/src/fc_stub
CONFIG   ?= config/default.yaml
SCENARIO ?= f5_gnss_drift
OUT      ?= out/$(SCENARIO)
SEED     ?=
DURATION ?=
JOBS     ?= $(shell nproc)
VENV     ?= .venv

# Optional overrides passed through to fc_stub.
RUN_ARGS := $(if $(SEED),--seed $(SEED)) $(if $(DURATION),--duration $(DURATION))

.PHONY: help build test test-fast test-timing asan coverage quality check verify \
        run sim scenarios validate jitter mavsdk clean

help: ## list the targets
	@awk 'BEGIN {FS = ":.*## "} /^[a-z0-9-]+:.*## / {printf "  %-12s %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo
	@echo "  Variables: CONFIG=$(CONFIG) SCENARIO=$(SCENARIO) SEED= DURATION= JOBS=$(JOBS)"
	@echo "  Examples:  make sim SCENARIO=f2_reboot   make run DURATION=30   make scenarios"

$(BIN):
	cmake --preset release
	cmake --build --preset release -j $(JOBS)

build: ## configure and build the release binary
	cmake --preset release
	cmake --build --preset release -j $(JOBS)

test: ## build and run every test (release), including the real-time jitter test
	./scripts/build_and_test.sh release

test-fast: ## every test except the 5 s real-time jitter test (for a loaded machine)
	CTEST_EXTRA_ARGS="-LE timing" ./scripts/build_and_test.sh release

test-timing: build ## only the real-time rate and jitter test (needs an idle machine)
	ctest --preset release -L timing --output-on-failure

asan: ## tests under AddressSanitizer + UndefinedBehaviorSanitizer
	./scripts/build_and_test.sh asan

coverage: ## line coverage of src/ with the 80 % gate (needs gcovr)
	./scripts/coverage.sh

quality: ## cppcheck and clang-format
	./scripts/quality.sh

check: quality test asan ## everything that runs locally: quality, tests, sanitizers

verify: ## clean containers: Ubuntu 22.04/24.04 g++, clang, coverage, arm64 (needs Docker)
	./scripts/verify_clean_machine.sh all

run: $(BIN) ## real time over UDP (listens 127.0.0.1:14580, sends to :14540); CONFIG=...
	$(BIN) --config $(CONFIG) $(RUN_ARGS)

sim: $(BIN) ## one scenario in model time: SCENARIO=f5_gnss_drift -> out/<scenario>/
	$(BIN) --config config/scenarios/$(SCENARIO).yaml --sim --out $(OUT) $(RUN_ARGS)

scenarios: $(BIN) ## every scenario in config/scenarios/ in model time, with its hash
	@for f in config/scenarios/*.yaml; do \
	    name=$$(basename "$$f" .yaml); \
	    printf '%-30s ' "$$name"; \
	    $(BIN) --config "$$f" --sim --out "out/$$name" | tail -n1 || exit 1; \
	done

validate: $(BIN) ## check CONFIG and every scenario against the schema
	@for f in $(CONFIG) config/scenarios/*.yaml; do \
	    printf '%-45s ' "$$f"; $(BIN) --config "$$f" --validate || exit 1; \
	done

jitter: ## 60 s jitter measurement on this machine (idle)
	./scripts/measure_jitter.sh idle 60

$(VENV)/bin/python:
	python3 -m venv $(VENV)
	$(VENV)/bin/pip install --quiet "mavsdk>=2.8,<3"

mavsdk: $(BIN) $(VENV)/bin/python ## drive the stub with the real MAVSDK (creates .venv on first use)
	$(VENV)/bin/python scripts/mavsdk_check.py $(BIN)

clean: ## remove build/ and out/
	rm -rf build out
