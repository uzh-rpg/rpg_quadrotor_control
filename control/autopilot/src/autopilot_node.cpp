#include "autopilot/autopilot.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot");
  autopilot::AutoPilot<position_controller::PositionController,
                       position_controller::PositionControllerParams> autopilot;

  ros::spin();

  return 0;
}
