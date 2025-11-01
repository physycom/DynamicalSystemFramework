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
# Get simdjson
FetchContent_Declare(simdjson
  GIT_REPOSITORY https://github.com/simdjson/simdjson
  GIT_TAG v4.1.0
)
FetchContent_GetProperties(simdjson)
if(NOT simdjson_POPULATED)
  FetchContent_MakeAvailable(simdjson)
endif()
# Get spdlog
FetchContent_Declare(spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog
  GIT_TAG v1.16.0
)
FetchContent_GetProperties(spdlog)
if(NOT spdlog_POPULATED)
  FetchContent_MakeAvailable(spdlog)
endif()
# Get TBB
set(TBB_TEST OFF CACHE BOOL "Disable TBB tests")
FetchContent_Declare(tbb
  GIT_REPOSITORY https://github.com/uxlfoundation/oneTBB
  GIT_TAG v2022.3.0
)
FetchContent_GetProperties(tbb)
if(NOT tbb_POPULATED)
  FetchContent_MakeAvailable(tbb)
endif()

add_library(dsf STATIC ${SOURCES})
target_include_directories(dsf PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

file(GLOB SOURCES "../src/dsf/base/*.cpp" "../src/dsf/mobility/*.cpp" "../src/dsf/utility/*.cpp" "../src/dsf/geometry/*.cpp")

# Define the executable
add_executable(prof.out main.cpp ${SOURCES})
target_include_directories(prof.out PRIVATE "${rapidcsv_SOURCE_DIR}/src" "${simdjson_SOURCE_DIR}/include" "${spdlog_SOURCE_DIR}/include" "${tbb_SOURCE_DIR}/include")
target_link_libraries(prof.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)
target_compile_options(prof.out PRIVATE -pg)
target_link_options(prof.out PRIVATE -pg)
add_executable(mem.out main.cpp ${SOURCES})
target_include_directories(mem.out PRIVATE "${rapidcsv_SOURCE_DIR}/src" "${simdjson_SOURCE_DIR}/include" "${spdlog_SOURCE_DIR}/include" "${tbb_SOURCE_DIR}/include")
target_link_libraries(mem.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)
add_executable(parse_massif.out parse_massif.cpp)
