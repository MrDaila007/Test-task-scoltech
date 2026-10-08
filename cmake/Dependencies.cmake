# Third-party dependencies, pinned to exact revisions.
# FCSTUB_USE_SYSTEM_DEPS=ON switches yaml-cpp and GoogleTest to system packages for offline builds;
# MAVLink is header-only and always fetched at the pinned commit.

include(FetchContent)

option(FCSTUB_USE_SYSTEM_DEPS "Use system yaml-cpp and GoogleTest instead of fetching them" OFF)

# --- MAVLink C library (generated headers, MAVLink 2) ------------------------
FetchContent_Declare(
    mavlink_c
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG        28eae47457249ab6c8c1cadd104fcbe25eacd7a3
    GIT_SHALLOW    FALSE
    # Headers only: point MakeAvailable at a directory without a CMakeLists.txt so it
    # populates the sources but adds no subproject (single-argument
    # FetchContent_Populate is deprecated since CMake 3.30).
    SOURCE_SUBDIR  no-cmake-project
)
FetchContent_MakeAvailable(mavlink_c)
add_library(fcstub_mavlink INTERFACE)
# SYSTEM: the generated headers trigger -Waddress-of-packed-member / -Wpedantic,
# which -Werror would turn into errors in our own translation units.
target_include_directories(fcstub_mavlink SYSTEM INTERFACE ${mavlink_c_SOURCE_DIR})
add_library(fcstub::mavlink ALIAS fcstub_mavlink)

# --- yaml-cpp ----------------------------------------------------------------
if(FCSTUB_USE_SYSTEM_DEPS)
    find_package(yaml-cpp REQUIRED)
else()
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG        f7320141120f720aecc4c32be25586e7da9eb978 # 0.8.0
    )
    # yaml-cpp 0.8.0 declares cmake_minimum_required(VERSION 3.4); CMake 4 refuses
    # anything below 3.5 unless this floor is set. Older CMake ignores the variable.
    if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
        set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    endif()
    FetchContent_MakeAvailable(yaml-cpp)
endif()
if(NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
if(NOT FCSTUB_USE_SYSTEM_DEPS)
    # Consumers see yaml-cpp's headers as system headers, so our -Werror warning set
    # (e.g. clang's -Wshadow) does not fire inside third-party code.
    set_target_properties(yaml-cpp PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
        "$<TARGET_PROPERTY:yaml-cpp,INTERFACE_INCLUDE_DIRECTORIES>")
endif()

# --- GoogleTest --------------------------------------------------------------
if(FCSTUB_BUILD_TESTS)
    if(FCSTUB_USE_SYSTEM_DEPS)
        find_package(GTest REQUIRED)
    else()
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        f8d7d77c06936315286eb55f8de22cd23c188571 # v1.14.0
        )
        FetchContent_MakeAvailable(googletest)
    endif()
endif()
