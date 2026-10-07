# fcstub_core: OS-free deterministic core. No sockets, files or clocks here:
# time and bytes are passed in by the drivers, frames are passed out through a sink.
add_library(fcstub_core STATIC
    core/battery.cpp
    core/command_dedup.cpp
    core/dynamics.cpp
    core/estimator_tap.cpp
    core/fault_schedule.cpp
    core/fc_clock.cpp
    core/lifecycle.cpp
    core/mav_codec.cpp
    core/mode_machine.cpp
    core/rng.cpp
    core/scheduler.cpp
    core/setpoint_gate.cpp
    core/sha256.cpp
)
target_include_directories(fcstub_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(fcstub_core
    PUBLIC fcstub::mavlink
    PRIVATE fcstub_warnings fcstub_fp)

# fcstub_config: YAML loader (start-up only; the one place allowed to throw).
add_library(fcstub_config STATIC
    config/config_loader.cpp
    config/config_faults.cpp
    config/yaml_section.cpp
)
target_include_directories(fcstub_config PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(fcstub_config
    PUBLIC yaml-cpp::yaml-cpp
    PRIVATE fcstub_warnings)
