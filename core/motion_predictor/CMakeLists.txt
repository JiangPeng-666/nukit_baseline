cmake_minimum_required(VERSION 2.8)
project(motion_predictor)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fPIC")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
    inc

)

add_library(motion_predictor INTERFACE)

target_include_directories(motion_predictor
    INTERFACE inc
)

target_link_libraries(motion_predictor
    INTERFACE common
    INTERFACE vehicle_model
    INTERFACE forward_simulator
)
