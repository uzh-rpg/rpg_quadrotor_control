cmake_minimum_required(VERSION 2.8.3)
project(quad_common)

find_package(catkin_simple REQUIRED)
catkin_simple()
quad_cmake_setup()

cs_add_library(${PROJECT_NAME} 
  src/control_command.cpp
  src/geometry_eigen_conversions.cpp
  src/math_common.cpp
  src/parameter_helper.cpp
  src/quad_desired_state.cpp
  src/quad_state.cpp
  src/quaternion_functions.cpp
  src/torques_and_thrust.cpp)


catkin_python_setup()

cs_install()
cs_export()
