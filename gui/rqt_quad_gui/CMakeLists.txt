cmake_minimum_required(VERSION 2.8.3)
project(rqt_quad_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs quadrotor_msgs rospy rqt_gui rqt_gui_py std_msgs
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 

install(PROGRAMS scripts/rqt_quad_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
