cmake_minimum_required(VERSION 2.8.3)
project(quad_cmake)

find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED)
catkin_simple()

cs_install()
cs_export(CFG_EXTRAS quad_cmake.cmake)

