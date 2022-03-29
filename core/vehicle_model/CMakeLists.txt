cmake_minimum_required(VERSION 2.8)
project(vehicle_model)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


find_package(Eigen3 REQUIRED)


set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

message(STATUS "project dir ${PROJECT_SOURCE_DIR}/../common/inc")

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    /usr/include
    /usr/include/opencv4
    ${EIGEN3_INCLUDE_DIR}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

add_library(vehicle_model
    src/vehicle_model/vehicle_model.cc
    src/vehicle_model/ideal_steer_model.cc
    src/vehicle_model/idm_model.cc
    src/vehicle_model/ctx_idm_model.cc
    src/vehicle_model/controllers/pure_pursuit_controller.cc
    src/vehicle_model/controllers/pid_controller.cc
    src/vehicle_model/controllers/idm_velocity_controller.cc
    src/vehicle_model/controllers/ctx_idm_velocity_controller.cc
)
set_property(TARGET vehicle_model PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(vehicle_model
    common
)
target_include_directories(vehicle_model
    PUBLIC inc
    PUBLIC thirdparty
)

