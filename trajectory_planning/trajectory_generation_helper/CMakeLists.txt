cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generation_helper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/polynomial_trajectory_helper.cpp
	src/heading_trajectory_helper.cpp src/circle_trajectory_helper.cpp)

cs_install()
cs_export()
