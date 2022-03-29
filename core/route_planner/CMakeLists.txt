cmake_minimum_required(VERSION 2.8)
project(route_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${semantic_map_manager_INCLUDE_DIRS}
)

add_library(route_planner
    src/route_planner/route_planner.cc
)
set_property(TARGET route_planner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(route_planner
    common
    semantic_map_manager
)

target_include_directories(route_planner
    PUBLIC inc
)
