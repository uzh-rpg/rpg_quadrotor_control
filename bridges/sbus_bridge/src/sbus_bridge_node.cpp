#include "sbus_bridge/sbus_bridge.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sbus_bridge");
  sbus_bridge::SBusBridge sbus_bridge;

  ros::spin();

  return 0;
}
