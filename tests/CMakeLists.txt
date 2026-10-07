include(GoogleTest)

add_executable(fcstub_tests
    unit/smoke_test.cpp
)
target_link_libraries(fcstub_tests PRIVATE
    fcstub_warnings fcstub_fp fcstub::mavlink GTest::gtest_main)

gtest_discover_tests(fcstub_tests DISCOVERY_TIMEOUT 30)
