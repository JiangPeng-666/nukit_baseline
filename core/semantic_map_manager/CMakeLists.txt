cmake_minimum_required(VERSION 2.8)
project(semantic_map_manager)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

include_directories(
    inc
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(hkust_pl_smm
    src/semantic_map_manager.cc
    src/data_renderer.cc
    src/config_loader.cc
    src/traffic_signal_manager.cc
)
target_link_libraries(hkust_pl_smm
    hkust_pl_common
    ${vehicle_model_LIBRARIES}
    hkust_pl_vehicle_model
    ${common_LIBRARIES}
    # ${motion_predictor_LIBRARIES}
    ${OpenCV_LIBS}
    motion_predictor
    forward_simulator
)

target_include_directories(hkust_pl_smm
    PUBLIC inc
    PUBLIC thirdparty
)
