cmake_minimum_required(VERSION 3.16.0)

project(dsf_examples VERSION 3.0.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)

# Set the C++ flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -O0 -g -fsanitize=address")
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
  string(APPEND CMAKE_CXX_FLAGS "-Ofast -march=native -flto=auto")
endif()

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

# add as executable all cpp files into '.' folder
file(GLOB SOURCES "*.cpp")
file(GLOB SRC_SOURCES "../src/dsf/sources/*.cpp" "../src/dsf/utility/*.cpp")

# Loop through each source file and create an executable
foreach(SOURCE ${SOURCES})
    get_filename_component(EXE_NAME ${SOURCE} NAME_WE)
	add_executable(${EXE_NAME}.out ${SOURCE} ${SRC_SOURCES})
    target_include_directories(${EXE_NAME}.out PRIVATE ../src/dsf/headers/ ../src/dsf/utility/TypeTraits/)
    target_link_libraries(${EXE_NAME}.out PRIVATE TBB::tbb)
endforeach()
