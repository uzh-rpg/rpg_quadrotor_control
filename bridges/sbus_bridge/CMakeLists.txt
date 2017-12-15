cmake_minimum_required(VERSION 2.8.3)
project(sbus_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_executable(sbus_bridge src/sbus_bridge.cpp src/serial_port.cpp src/sbus_msg.cpp)

cs_export()
