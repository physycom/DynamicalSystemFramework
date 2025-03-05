cmake_minimum_required(VERSION 3.16.0)

project(dms VERSION 2.5.8 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure optimization flags are applied only in Release mode
if (CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native -flto=auto")
endif()

find_package(TBB REQUIRED CONFIG)

file(GLOB SOURCES "src/dsm/sources/*.cpp" "src/dsm/utility/*.cpp")

add_library(dsm STATIC ${SOURCES})
target_include_directories(dsm PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/headers>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dsm PRIVATE TBB::tbb)

install(TARGETS dsm
        EXPORT dsmConfig
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

install(EXPORT dsmConfig
        FILE dsmConfig.cmake
        NAMESPACE dsm::
        DESTINATION lib/cmake/dsm)
