cmake_minimum_required(VERSION 3.13)
project(eudm_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/eudm_config.proto
)

find_package(Eigen3 REQUIRED)

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(eudm
    src/eudm_planner/eudm_planner.cc
    src/eudm_planner/map_adapter.cc
    src/eudm_planner/eudm_manager.cc
    src/eudm_planner/dcp_tree.cc
    ${PROTO_SRCS}
)
set_property(TARGET eudm PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(eudm
    common
    semantic_map_manager
    route_planner
    ssc_planner
    vehicle_model
    motion_predictor
    forward_simulator
)

target_include_directories(eudm
    PUBLIC inc
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
