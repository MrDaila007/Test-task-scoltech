# fcstub_core: OS-free deterministic core. No sockets, files or clocks here:
# time and bytes are passed in by the drivers, frames are passed out through a sink.
add_library(fcstub_core STATIC
    core/rng.cpp
    core/sha256.cpp
)
target_include_directories(fcstub_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(fcstub_core
    PUBLIC fcstub::mavlink
    PRIVATE fcstub_warnings fcstub_fp)
