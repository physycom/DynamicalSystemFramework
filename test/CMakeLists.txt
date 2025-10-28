cmake_minimum_required(VERSION 3.16.0)

project(dsf_tests VERSION 3.0.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add code coverage options
if(CMAKE_BUILD_TYPE STREQUAL "COVERAGE")
    message(STATUS "Enable code coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g --coverage -fprofile-update=atomic")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g -fsanitize=address")
endif()

find_package(TBB REQUIRED CONFIG)
find_package(spdlog REQUIRED)
find_package(simdjson REQUIRED)

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

include(FetchContent)
# Get doctest
FetchContent_Declare(doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.12
)
FetchContent_GetProperties(doctest)
if(NOT doctest_POPULATED)
  FetchContent_MakeAvailable(doctest)
endif()

# Get rapidcsv
FetchContent_Declare(rapidcsv
  GIT_REPOSITORY https://github.com/d99kris/rapidcsv
  GIT_TAG v8.89
)
FetchContent_GetProperties(rapidcsv)
if(NOT rapidcsv_POPULATED)
  FetchContent_MakeAvailable(rapidcsv)
endif()

# add as executable all cpp files into '.' folder and subdirectories
file(GLOB TEST_SOURCES "*.cpp" "*/*.cpp")
file(GLOB SRC_SOURCES "../src/dsf/base/*.cpp" "../src/dsf/mobility/*.cpp" "../src/dsf/utility/*.cpp" "../src/dsf/geometry/*.cpp")
list(APPEND SOURCES ${TEST_SOURCES} ${SRC_SOURCES})

# Define the executable
add_executable(dsf_tests.out ${SOURCES})
target_include_directories(dsf_tests.out PRIVATE ../src/ ../src/dsf/utility/TypeTraits/)
target_include_directories(dsf_tests.out SYSTEM PRIVATE ${doctest_SOURCE_DIR}/doctest)
target_include_directories(dsf_tests.out PRIVATE ${rapidcsv_SOURCE_DIR}/src)
target_link_libraries(dsf_tests.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)