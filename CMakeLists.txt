cmake_minimum_required(VERSION 3.16.0)

# Read version from header file
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/dsf/dsf.hpp" DSF_HPP_CONTENT)
string(REGEX MATCH "DSF_VERSION_MAJOR = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "DSF_VERSION_MINOR = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "DSF_VERSION_PATCH = ([0-9]+)" _ ${DSF_HPP_CONTENT})
set(DSF_VERSION_PATCH ${CMAKE_MATCH_1})

set(DSF_VERSION
    "${DSF_VERSION_MAJOR}.${DSF_VERSION_MINOR}.${DSF_VERSION_PATCH}")

project(
  dsf
  VERSION ${DSF_VERSION}
  LANGUAGES CXX)

option(DSF_TESTS "Build DSF tests" OFF)
option(DSF_EXAMPLES "Build DSF examples" OFF)
option(DSF_BENCHMARKS "Build DSF benchmarks" OFF)
option(DSF_BUILD_PIC "Build DSF with position-independent code" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

# If CMAKE_BUILD_TYPE not set, default to Debug
if(NOT CMAKE_BUILD_TYPE)
  if(BUILD_PYTHON_BINDINGS OR DSF_BENCHMARKS)
    set(CMAKE_BUILD_TYPE
        "Release"
        CACHE STRING
              "Build type (default: Release when building Python bindings or benchmarks)"
              FORCE)
  else()
    set(CMAKE_BUILD_TYPE
        "Debug"
        CACHE STRING "Build type (default: Debug)" FORCE)
  endif()
endif()
if(BUILD_PYTHON_BINDINGS)
  set(DSF_BUILD_PIC ON)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure optimization flags are applied only in Release mode
if(CMAKE_BUILD_TYPE MATCHES "Release")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native -flto=auto")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2")
  endif()
elseif(CMAKE_BUILD_TYPE MATCHES "Profile")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native -flto=auto -pg")
  endif()
elseif(CMAKE_BUILD_TYPE MATCHES "Coverage")
  set(DSF_TESTS ON)
  message(STATUS "Enable code coverage")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -pg --coverage -fprofile-update=atomic"
    )
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  endif()
else() # Default to Debug settings
  set(DSF_TESTS ON)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -pg -fsanitize=address")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -pg")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
endif()

file(
  GLOB
  SOURCES
  "src/dsf/base/*.cpp"
  "src/dsf/mobility/*.cpp"
  "src/dsf/utility/*.cpp"
  "src/dsf/geometry/*.cpp"
  "src/dsf/mdt/*.cpp")

include(FetchContent)
# Get rapidcsv
FetchContent_Declare(
  rapidcsv
  GIT_REPOSITORY https://github.com/d99kris/rapidcsv
  GIT_TAG v8.89)
FetchContent_GetProperties(rapidcsv)
if(NOT rapidcsv_POPULATED)
  FetchContent_MakeAvailable(rapidcsv)
endif()
# Get spdlog
set(SPDLOG_USE_STD_FORMAT
    ON
    CACHE BOOL "Use std::format instead of fmt" FORCE)
set(SPDLOG_FMT_EXTERNAL
    OFF
    CACHE BOOL "Use bundled fmt library" FORCE)
set(SPDLOG_INSTALL
    ON
    CACHE BOOL "Install spdlog targets" FORCE)
if(BUILD_PYTHON_BINDINGS)
  set(SPDLOG_BUILD_PIC
      ON
      CACHE BOOL "Build position-independent code for spdlog" FORCE)
endif()
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog
  GIT_TAG v1.16.0)
FetchContent_GetProperties(spdlog)
if(NOT spdlog_POPULATED)
  FetchContent_MakeAvailable(spdlog)
endif()
# Get simdjson
FetchContent_Declare(
  simdjson
  GIT_REPOSITORY https://github.com/simdjson/simdjson
  GIT_TAG v4.2.1)
FetchContent_GetProperties(simdjson)
if(NOT simdjson_POPULATED)
  FetchContent_MakeAvailable(simdjson)
endif()
# Get TBB
set(TBB_TEST
    OFF
    CACHE BOOL "Disable TBB tests" FORCE)
FetchContent_Declare(
  tbb
  GIT_REPOSITORY https://github.com/uxlfoundation/oneTBB
  GIT_TAG v2022.3.0)
FetchContent_GetProperties(tbb)
if(NOT tbb_POPULATED)
  FetchContent_MakeAvailable(tbb)
endif()

add_library(dsf STATIC ${SOURCES})
target_compile_definitions(dsf PRIVATE SPDLOG_USE_STD_FORMAT)
if(DSF_BUILD_PIC)
  set_target_properties(dsf PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
target_include_directories(
  dsf PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
             $<INSTALL_INTERFACE:include>)

target_include_directories(dsf PRIVATE ${rapidcsv_SOURCE_DIR}/src)

# Link other libraries - no csv dependency needed now
target_link_libraries(dsf PRIVATE TBB::tbb simdjson::simdjson spdlog::spdlog)

# Install dsf library
install(
  TARGETS dsf
  EXPORT dsfConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

install(
  EXPORT dsfConfig
  FILE dsfConfig.cmake
  NAMESPACE dsf::
  DESTINATION lib/cmake/dsf)

# Optional Python bindings - only build if requested
if(BUILD_PYTHON_BINDINGS)
  include(FetchContent)

  # Get pybind11
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v3.0.1)
  FetchContent_GetProperties(pybind11)
  if(NOT pybind11_POPULATED)
    FetchContent_MakeAvailable(pybind11)
  endif()

  # Add the Python binding module
  add_library(dsf_python_module MODULE src/dsf/bindings.cpp)

  # Ensure the Python module name has no 'lib' prefix on Unix systems
  set_target_properties(dsf_python_module PROPERTIES PREFIX "" OUTPUT_NAME
                                                               "dsf_cpp")

  # Link the pybind11 module with your static library and pybind11
  target_link_libraries(
    dsf_python_module PRIVATE dsf pybind11::module pybind11::headers TBB::tbb
                              spdlog::spdlog)
  target_compile_definitions(dsf_python_module PRIVATE SPDLOG_USE_STD_FORMAT)

  target_include_directories(dsf_python_module
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  if(APPLE)
    # Try to detect Homebrew prefix dynamically
    execute_process(
      COMMAND brew --prefix
      OUTPUT_VARIABLE HOMEBREW_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(HOMEBREW_PREFIX)
      set(HOMEBREW_LIB "${HOMEBREW_PREFIX}/lib")
    else()
      # Fallback to default Homebrew locations
      set(HOMEBREW_LIB "/opt/homebrew/lib;/usr/local/lib")
    endif()
    set_target_properties(
      dsf_python_module
      PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
                 INSTALL_RPATH "${HOMEBREW_LIB}"
                 INSTALL_RPATH_USE_LINK_PATH TRUE)
  endif()
endif()

# Tests
message(STATUS "Build DSF tests: ${DSF_TESTS}")
if(DSF_TESTS)
  include(CTest)
  enable_testing()

  # Get Doctest library
  FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.12)
  FetchContent_GetProperties(doctest)
  if(NOT doctest_POPULATED)
    FetchContent_MakeAvailable(doctest)
  endif()

  macro(add_unit_test testname)
    add_executable(${testname} ${ARGN})
    target_include_directories(${testname}
                               PRIVATE ../src/ ../src/dsf/utility/TypeTraits/)
    target_include_directories(
      ${testname} SYSTEM PRIVATE ${doctest_SOURCE_DIR}/doctest
                                 ${rapidcsv_SOURCE_DIR}/src)
    target_compile_definitions(${testname} PRIVATE SPDLOG_USE_STD_FORMAT)
    target_link_libraries(${testname} PRIVATE dsf TBB::tbb simdjson::simdjson spdlog::spdlog)
    # Put test binaries under build/tests so they are easy to find. Use
    # per-config output dirs to play nicely with multi-config generators.
    set_target_properties(
      ${testname}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
                 RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/tests
                 RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/tests
                 RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/tests
                 RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO
                 ${CMAKE_BINARY_DIR}/tests)

    # Register the test using the full target file so CTest invokes the correct
    # executable regardless of output directory or generator.
    add_test(NAME ${testname} COMMAND $<TARGET_FILE:${testname}>)
  endmacro()

  file(GLOB TEST_SOURCES "test/*/*.cpp")
  foreach(testsource ${TEST_SOURCES})
    get_filename_component(testname ${testsource} NAME_WE)
    add_unit_test(${testname} ${testsource})
  endforeach()
endif()

# Examples
message(STATUS "Build DSF examples: ${DSF_EXAMPLES}")
if(DSF_EXAMPLES)
  set(CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH}")
  add_subdirectory(examples)
endif()

# Benchmarks
message(STATUS "Build DSF benchmarks: ${DSF_BENCHMARKS}")
if(DSF_BENCHMARKS)
  set(CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH}")
  add_subdirectory(benchmark)
endif()
