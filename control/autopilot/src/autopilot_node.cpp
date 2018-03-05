#include "autopilot/autopilot.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot");
  autopilot::AutoPilot autopilot;

  ros::spin();

  return 0;
}
