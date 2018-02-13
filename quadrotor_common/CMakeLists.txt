cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/control_command.cpp
  src/geometry_eigen_conversions.cpp
  src/math_common.cpp
  src/parameter_helper.cpp
  src/quad_state_estimate.cpp
  src/quaternion_functions.cpp
  src/trajectory_point.cpp)
  
cs_install()
cs_export()
