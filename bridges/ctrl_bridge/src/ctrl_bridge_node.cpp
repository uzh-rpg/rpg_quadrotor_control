#include "ros/ros.h"
#include <ctrl_bridge/ctrl_bridge.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctrl_bridge");
  ctrl_bridge::CtrlBridge ctrl_bridge;
  ros::spin();
  return 0;
}
