cmake_minimum_required(VERSION 2.8.3)
project(sbus_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(sbus_bridge src/sbus_bridge_node.cpp src/sbus_bridge.cpp 
    src/sbus_serial_port.cpp src/sbus_msg.cpp src/thrust_mapping.cpp)

cs_install()
cs_export()
