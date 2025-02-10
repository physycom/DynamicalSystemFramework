cmake_minimum_required(VERSION 3.16.0)

project(Benchmark VERSION 1.0.1 LANGUAGES CXX)

# set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(../extern/benchmark/)

# add subdirectories
add_subdirectory(Graph)
add_subdirectory(Street)
add_subdirectory(Dynamics)
