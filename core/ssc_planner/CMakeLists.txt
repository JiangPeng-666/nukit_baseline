cmake_minimum_required(VERSION 3.13)
project(ssc_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/ssc_config.proto
)


include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(ssc_planner
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
    ${PROTO_SRCS}
)
set_property(TARGET ssc_planner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ssc_planner 
    common
    semantic_map_manager
    vehicle_model
    "${OpenMP_CXX_FLAGS}"
    forward_simulator
    motion_predictor
)

target_compile_options(ssc_planner PRIVATE "${OpenMP_CXX_FLAGS}")

target_include_directories(ssc_planner
    PUBLIC inc
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)


