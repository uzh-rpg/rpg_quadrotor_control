cmake_minimum_required(VERSION 2.8.3)
project(ctrl_bridge)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

cs_add_library(ctrl_bridge
  src/serial.cpp
  src/msg_encoding.cpp
  src/ctrl_bridge.cpp
)

cs_add_executable(ctrl_bridge_node src/ctrl_bridge_node.cpp)
target_link_libraries(ctrl_bridge_node ctrl_bridge)

cs_install()

cs_export()
