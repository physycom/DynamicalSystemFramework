cmake_minimum_required(VERSION 3.16.0)

# Read version from header file
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/dsf/dsf.hpp" DSF_HPP_CONTENT)
string(REGEX MATCH "DSF_VERSION_MAJOR = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "DSF_VERSION_MINOR = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "DSF_VERSION_PATCH = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_PATCH ${CMAKE_MATCH_1})

set(DSF_VERSION "${DSF_VERSION_MAJOR}.${DSF_VERSION_MINOR}.${DSF_VERSION_PATCH}")

project(dsf VERSION ${DSF_VERSION} LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure optimization flags are applied only in Release mode
if (CMAKE_BUILD_TYPE MATCHES "Release")
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native -flto=auto")
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2")
    endif()
elseif(CMAKE_BUILD_TYPE MATCHES "Coverage")
    message(STATUS "Enable code coverage")
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g --coverage -fprofile-update=atomic")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    endif()
else() # Default to Debug settings
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -fsanitize=address")
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
endif()

find_package(TBB REQUIRED CONFIG)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(simdjson REQUIRED)

file(GLOB SOURCES "src/dsf/base/*.cpp" "src/dsf/mobility/*.cpp" "src/dsf/utility/*.cpp" "src/dsf/geometry/*.cpp")

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

add_library(dsf STATIC ${SOURCES})
target_include_directories(dsf PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Include csv headers privately
target_include_directories(dsf PRIVATE "${rapidcsv_SOURCE_DIR}/src")

# Link other libraries - no csv dependency needed now
target_link_libraries(dsf PRIVATE TBB::tbb fmt::fmt spdlog::spdlog simdjson::simdjson)

# Install dsf library
install(TARGETS dsf
        EXPORT dsfConfig
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

install(EXPORT dsfConfig
        FILE dsfConfig.cmake
        NAMESPACE dsf::
        DESTINATION lib/cmake/dsf)

# Optional Python bindings - only build if requested
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

if(BUILD_PYTHON_BINDINGS)
    include(FetchContent)
    
    # Get pybind11
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v3.0.1
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_MakeAvailable(pybind11)
    endif()

    # Add the Python binding module
    add_library(dsf_python_module MODULE src/dsf/binding_mobility.cpp)

    # Ensure the Python module name has no 'lib' prefix on Unix systems
    set_target_properties(dsf_python_module PROPERTIES
        PREFIX ""
        OUTPUT_NAME "dsf_cpp"
    )

    # Link the pybind11 module with your static library and pybind11
    target_link_libraries(dsf_python_module PRIVATE dsf pybind11::module pybind11::headers)

    # Set include directories (if binding.cpp needs headers from your project)
    target_include_directories(dsf_python_module PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
endif()

# Tests
option(DSF_BUILD_TESTS "Build DSF tests" OFF)
message(STATUS "Build DSF tests: ${DSF_BUILD_TESTS}")
if (DSF_BUILD_TESTS)
    include(CTest)
    enable_testing()

    # Get Doctest library
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.12
    )
    FetchContent_GetProperties(doctest)
    if(NOT doctest_POPULATED)
    FetchContent_MakeAvailable(doctest)
    endif()

    macro(add_unit_test testname)
        add_executable(${testname} ${ARGN})
        target_include_directories(${testname} PRIVATE ../src/ ../src/dsf/utility/TypeTraits/)
        target_include_directories(${testname} SYSTEM PRIVATE ${doctest_SOURCE_DIR}/doctest)
        target_include_directories(${testname} PRIVATE ${rapidcsv_SOURCE_DIR}/src)
        target_link_libraries(${testname} PRIVATE dsf fmt::fmt spdlog::spdlog simdjson::simdjson TBB::tbb)
        # Put test binaries under build/tests so they are easy to find.
        # Use per-config output dirs to play nicely with multi-config generators.
        set_target_properties(${testname} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/tests
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/tests
            RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/tests
            RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/tests
        )

        # Register the test using the full target file so CTest invokes the
        # correct executable regardless of output directory or generator.
        add_test(NAME ${testname} COMMAND $<TARGET_FILE:${testname}>)
    endmacro()

    file(GLOB TEST_SOURCES "test/*/*.cpp")
    foreach(testsource ${TEST_SOURCES})
        get_filename_component(testname ${testsource} NAME_WE)
        add_unit_test(${testname} ${testsource})
    endforeach()
endif()
