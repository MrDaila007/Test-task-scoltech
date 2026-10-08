include(GoogleTest)

add_executable(fcstub_tests
    unit/smoke_test.cpp
    unit/rng_test.cpp
    unit/sha256_test.cpp
    unit/config_test.cpp
    unit/mav_codec_test.cpp
    unit/scheduler_test.cpp
    unit/mode_machine_test.cpp
    unit/setpoint_gate_test.cpp
    unit/command_dedup_test.cpp
    unit/dynamics_test.cpp
    unit/faults_test.cpp
    unit/link_model_test.cpp
    unit/fc_core_test.cpp
    unit/sim_driver_test.cpp
    scenario/detectability_test.cpp
    scenario/naive_detector_test.cpp
    support/naive_detector.cpp
)
target_link_libraries(fcstub_tests PRIVATE
    fcstub_runtime fcstub_core fcstub_config fcstub_warnings fcstub_fp fcstub::mavlink GTest::gtest_main)

target_compile_definitions(fcstub_tests PRIVATE FCSTUB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

gtest_discover_tests(fcstub_tests DISCOVERY_TIMEOUT 30)

# Separate executable: it replaces the global allocation functions to prove the
# core does not allocate after construction. Sanitizers intercept the same
# functions, so this check only runs in non-sanitizer builds.
if(NOT CMAKE_CXX_FLAGS MATCHES "-fsanitize")
    add_executable(fcstub_noalloc_tests noalloc/no_alloc_test.cpp)
    target_link_libraries(fcstub_noalloc_tests PRIVATE
        fcstub_core fcstub_warnings fcstub_fp fcstub::mavlink GTest::gtest_main)
    gtest_discover_tests(fcstub_noalloc_tests DISCOVERY_TIMEOUT 30)
endif()

# Integration tests run the real fc_stub binary (real time, UDP on loopback).
# Label "integration": skip with `ctest -LE integration` where timing is unreliable.
add_executable(fcstub_integration_tests
    integration/cli_test.cpp
    integration/udp_e2e_test.cpp
)
target_link_libraries(fcstub_integration_tests PRIVATE
    fcstub_warnings fcstub::mavlink GTest::gtest_main)
target_compile_definitions(fcstub_integration_tests PRIVATE
    FCSTUB_BIN="$<TARGET_FILE:fc_stub>" FCSTUB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
add_dependencies(fcstub_integration_tests fc_stub)
gtest_discover_tests(fcstub_integration_tests DISCOVERY_TIMEOUT 30 PROPERTIES LABELS integration)
