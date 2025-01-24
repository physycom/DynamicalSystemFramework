cmake_minimum_required(VERSION 3.16.0)

project(prof VERSION 1.0.1)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Os")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

file(GLOB SOURCES "../src/dsm/sources/*.cpp" "../src/dsm/utility/*.cpp")

# Define the executable
add_executable(prof.out main.cpp ${SOURCES})
target_include_directories(prof.out PRIVATE ../src/)
target_compile_options(prof.out PRIVATE -pg)
target_link_options(prof.out PRIVATE -pg)
add_executable(mem.out main.cpp ${SOURCES})
target_include_directories(mem.out PRIVATE ../src/)
add_executable(parse_massif.out parse_massif.cpp)
