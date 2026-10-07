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
)
target_link_libraries(fcstub_tests PRIVATE
    fcstub_core fcstub_config fcstub_warnings fcstub_fp fcstub::mavlink GTest::gtest_main)

target_compile_definitions(fcstub_tests PRIVATE FCSTUB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

gtest_discover_tests(fcstub_tests DISCOVERY_TIMEOUT 30)
