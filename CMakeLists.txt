cmake_minimum_required(VERSION 3.13)
project(ego_solver)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED)

add_subdirectory(core/common)
add_subdirectory(core/vehicle_model)
add_subdirectory(core/forward_simulator)
add_subdirectory(core/motion_predictor)
add_subdirectory(core/semantic_map_manager)
add_subdirectory(core/route_planner)
add_subdirectory(core/ssc_planner)
add_subdirectory(core/eudm_planner)
add_subdirectory(core/phy_simulator)

message(STATUS "project build dir ${CMAKE_CURRENT_BINARY_DIR}")

include_directories(
    inc
    ${PROJECT_SOURCE_DIR}/include
    /usr/include
    /usr/include/opencv4
    /home/lain/anaconda3/envs/nuplan_modified/include/python3.9
    /home/lain/anaconda3/envs/nuplan_modified/include
    ${EIGEN3_INCLUDE_DIR}

)

# add_executable(ego_solver
#     src/solver.cc
# )

add_library(${PROJECT_NAME} SHARED 
    src/solver.cc)
set_property(TARGET ego_solver PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
    common
    eudm
    semantic_map_manager
    route_planner
    ssc_planner
    vehicle_model
    forward_simulator
    motion_predictor
    phy_simulator_lib
)

add_library(py_solver_util SHARED
    # src/solver.cc
    src/wrapper.cc
)

set_property(TARGET py_solver_util PROPERTY POSITION_INDEPENDENT_CODE ON)

# SET_TARGET_PROPERTIES(py_solver_util PROPERTIES PREFIX "")


target_link_libraries(py_solver_util
    ${Boost_LIBRARIES}
    ego_solver
    common
    eudm
    semantic_map_manager
    route_planner
    ssc_planner
    vehicle_model
    forward_simulator
    motion_predictor
    phy_simulator_lib
    boost_python39
    boost_numpy39)