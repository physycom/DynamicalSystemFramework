cmake_minimum_required(VERSION 3.16.0)

project(prof VERSION 1.0.1)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)
find_package(spdlog REQUIRED)
find_package(simdjson REQUIRED)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Os")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

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

file(GLOB SOURCES "../src/dsf/base/*.cpp src/dsf/mobility/*.cpp" "../src/dsf/utility/*.cpp" "../src/dsf/geometry/*.cpp")

# Define the executable
add_executable(prof.out main.cpp ${SOURCES})
target_include_directories(prof.out PRIVATE ../src/ ${rapidcsv_SOURCE_DIR}/src)
target_link_libraries(prof.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)
target_compile_options(prof.out PRIVATE -pg)
target_link_options(prof.out PRIVATE -pg)
add_executable(mem.out main.cpp ${SOURCES})
target_include_directories(mem.out PRIVATE ../src/ ${rapidcsv_SOURCE_DIR}/src)
target_link_libraries(mem.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)
add_executable(parse_massif.out parse_massif.cpp)
