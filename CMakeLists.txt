cmake_minimum_required(VERSION 3.22)

project(fcstub
    VERSION 0.1.0
    DESCRIPTION "PX4-like flight controller stub with fault injection"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FCSTUB_BUILD_TESTS "Build the test suite" ON)

# Project warning policy: applied to our own targets only, never to third-party code.
add_library(fcstub_warnings INTERFACE)
target_compile_options(fcstub_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow)

# Floating-point determinism: no FMA contraction. GCC on aarch64 contracts a*b+c into
# FMA by default, which changes results in the last bit; with contraction off the
# reproducibility hash matches between x86_64 and aarch64 (Jetson). Never add -march=native / -ffast-math.
add_library(fcstub_fp INTERFACE)
target_compile_options(fcstub_fp INTERFACE -ffp-contract=off)

include(cmake/Dependencies.cmake)

add_subdirectory(src)

if(FCSTUB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
