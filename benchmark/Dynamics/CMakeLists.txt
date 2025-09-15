cmake_minimum_required(VERSION 3.16.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Ofast -march=native -flto=auto")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../../)

include_directories(../../src/dsf/headers)
include_directories(../../src/dsf/utility/)

file(GLOB SOURCES "../../src/dsf/sources/*.cpp" "../../src/dsf/utility/*.cpp")

# Compile
add_executable(bench_dynamics.out BenchDynamics.cpp ${SOURCES})
target_link_libraries(bench_dynamics.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog)
