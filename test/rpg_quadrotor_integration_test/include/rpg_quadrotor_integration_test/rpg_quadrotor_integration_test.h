#pragma once

#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>

namespace rpg_quadrotor_integration_test
{

class QuadrotorIntegrationTest
{
public:
  QuadrotorIntegrationTest();
  virtual ~QuadrotorIntegrationTest();

  void run();

private:

  void autopilotFeedbackCallback(
      const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);
  bool waitForAutopilotState(const uint state, const double timeout);

  ros::Publisher arm_pub_;
  ros::Publisher start_pub_;
  ros::Publisher land_pub_;
  ros::Publisher off_pub_;

  ros::Publisher pose_command_pub_;
  ros::Publisher velocity_command_pub_;
  ros::Publisher reference_state_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher control_command_pub_;
  ros::Publisher force_hover_pub_;

  ros::Subscriber autopilot_feedback_sub_;


  quadrotor_msgs::AutopilotFeedback autopilot_feedback_;

  bool autopilot_feedback_received_;
  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;

  // Constants
  static constexpr double kExecLoopRate_ = 50.0;
};

} // namespace rpg_quadrotor_integration_test
