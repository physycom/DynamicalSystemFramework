cmake_minimum_required(VERSION 3.16.0)

project(dms VERSION 2.0.0 LANGUAGES CXX)

# set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOURCES "src/dsm/headers/*.cpp")

add_library(dsm STATIC ${SOURCES})
target_include_directories(dsm PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/headers>
    $<INSTALL_INTERFACE:include>
)

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
