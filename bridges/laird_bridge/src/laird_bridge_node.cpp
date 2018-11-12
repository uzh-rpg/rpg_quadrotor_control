#include "ros/ros.h"
#include <laird_bridge/laird_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laird_bridge");
  laird_bridge::LairdBridge laird_bridge;
  ros::spin();
  return 0;
}
