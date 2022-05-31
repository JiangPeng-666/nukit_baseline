cmake_minimum_required(VERSION 3.13)
project(phy_simulator)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc
    src/phy_simulator/phy_simulator.cc
)
set_property(TARGET phy_simulator_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(phy_simulator_lib
    vehicle_model
    common
)
target_include_directories(phy_simulator_lib
    PUBLIC inc
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
