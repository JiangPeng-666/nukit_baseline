cmake_minimum_required(VERSION 2.8)
project(forward_simulator)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



include_directories(
    inc
    ${common_INCLUDE_DIRS}
    /home/jp/ego_solver/core/common/inc
    ${vehicle_model_INCLUDE_DIRS}
    /home/jp/ego_solver/core/vehicle_model/inc

)

add_library(forward_simulator INTERFACE)

# set_property(TARGET forward_simulator PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(forward_simulator
    INTERFACE inc
)
target_link_libraries(forward_simulator
    INTERFACE common
    INTERFACE vehicle_model
)


