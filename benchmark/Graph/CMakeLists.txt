cmake_minimum_required(VERSION 3.16.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -O3")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../../)

include_directories(../../src/dsm/headers/)
include_directories(../../src/dsm/utility/)

file(GLOB SOURCES "../../src/dsm/sources/*.cpp", "../../src/dsm/utility/*.cpp")

# Compile
add_executable(bench_graph.out BenchGraph.cpp ${SOURCES})
target_link_libraries(bench_graph.out PRIVATE TBB::tbb)
