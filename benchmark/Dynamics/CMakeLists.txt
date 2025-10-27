cmake_minimum_required(VERSION 3.16.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)
find_package(spdlog REQUIRED)
find_package(simdjson REQUIRED)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Ofast -march=native -flto=auto")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../../)

include(FetchContent)
# Get rapidcsv
FetchContent_Declare(rapidcsv
  GIT_REPOSITORY https://github.com/d99kris/rapidcsv
  GIT_TAG v8.89
)
FetchContent_GetProperties(rapidcsv)
if(NOT rapidcsv_POPULATED)
  FetchContent_MakeAvailable(rapidcsv)
endif()

include_directories(../../src/dsf/headers)
include_directories(../../src/dsf/utility/)

file(GLOB SOURCES "../../src/dsf/sources/*.cpp" "../../src/dsf/utility/*.cpp" "../../src/dsf/geometry/*.cpp")

# Compile
add_executable(bench_dynamics.out BenchDynamics.cpp ${SOURCES})
target_include_directories(bench_dynamics.out PRIVATE ${rapidcsv_SOURCE_DIR}/src)
target_link_libraries(bench_dynamics.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)
