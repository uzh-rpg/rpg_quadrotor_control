#pragma once

#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>

namespace rpg_quadrotor_integration_test {

class QuadrotorIntegrationTest {
 public:
  QuadrotorIntegrationTest();
  virtual ~QuadrotorIntegrationTest();

  void run();

 private:
  void measureTracking(const ros::TimerEvent& time);

  ros::NodeHandle nh_;

  ros::Publisher arm_pub_;

  ros::Subscriber autopilot_feedback_sub_;

  ros::Timer measure_tracking_timer_;

  autopilot_helper::AutoPilotHelper autopilot_helper_;
  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;

  // Constants
  static constexpr double kExecLoopRate_ = 50.0;
};

}  // namespace rpg_quadrotor_integration_test
