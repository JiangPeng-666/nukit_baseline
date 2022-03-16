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

add_library(hkust_pl_rp
    src/route_planner/route_planner.cc
)
target_link_libraries(hkust_pl_rp
    hkust_pl_common
    hkust_pl_smm
)

target_include_directories(hkust_pl_rp
    PUBLIC inc
)
