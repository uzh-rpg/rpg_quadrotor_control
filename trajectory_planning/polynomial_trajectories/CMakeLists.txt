cmake_minimum_required(VERSION 2.8.3)
project(polynomial_trajectories)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/polynomial_trajectory.cpp
    src/polynomial_trajectories_common.cpp 
    src/minimum_snap_trajectories.cpp 
    src/constrained_polynomial_trajectories.cpp)

cs_install()
cs_export()
