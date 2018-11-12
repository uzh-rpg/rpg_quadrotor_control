cmake_minimum_required(VERSION 2.8.3)
project(laird_bridge)

find_package(catkin_simple REQUIRED)
catkin_simple()
#quad_cmake_setup()

cs_add_executable(laird_bridge src/laird_bridge_node.cpp src/laird_bridge.cpp src/serial_port.cpp src/parameter_handler.cpp)

cs_add_library( laird_bridge_nodelet src/laird_bridge_nodelet.cpp src/laird_bridge.cpp src/serial_port.cpp src/parameter_handler.cpp)

cs_add_executable(laird_timesync src/laird_timesync_node.cpp)


cs_install()
install(DIRECTORY parameters
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cs_export()
