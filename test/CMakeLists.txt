cmake_minimum_required(VERSION 3.16.0)

project(dsm_tests VERSION 1.3.10 LANGUAGES CXX)

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

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

include(FetchContent)
# Get doctest
FetchContent_Declare(doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.11
)
FetchContent_GetProperties(doctest)
if(NOT doctest_POPULATED)
  FetchContent_MakeAvailable(doctest)
endif()

# add as executable all cpp files into '.' folder
file(GLOB TEST_SOURCES "*.cpp")
file(GLOB SRC_SOURCES "../src/dsm/sources/*.cpp" "../src/dsm/utility/*.cpp")
list(APPEND SOURCES ${TEST_SOURCES} ${SRC_SOURCES})

# Define the executable
add_executable(dsm_tests.out ${SOURCES})
target_include_directories(dsm_tests.out PRIVATE ../src/dsm/headers/ ../src/dsm/utility/TypeTraits/)
target_include_directories(dsm_tests.out SYSTEM PRIVATE ${doctest_SOURCE_DIR}/doctest)
target_link_libraries(dsm_tests.out PRIVATE TBB::tbb)