/*
 * large_angle_controller_node.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: ffontana
 */
#pragma once

// c includes
#include "iostream"
#include "Eigen/Dense"

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"

#include "controller_common/flight_controller_base.h"
#include "copilot/copilot_parameters.h"
#include "copilot/states.h"
#include "large_angle_controller/large_angle_controller.h"
#include "quad_msgs/ControllerFeedback.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_msgs/QuadDesiredState.h"
#include "sensor_fusion_util/sensor_fusion_util_helper.h"

namespace copilot
{

using namespace quad_common;
using namespace controller_base;

class ControllerWithPrimitives : public FlightControllerBase
{
public:
  ControllerWithPrimitives(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
  ControllerWithPrimitives() :
      ControllerWithPrimitives(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }
  virtual ~ControllerWithPrimitives();

  bool loadParameters();

  void mainloop(const ros::TimerEvent& time);

  ControlCommand optitrack_start(const QuadState & predicted_state);
  ControlCommand vision_start();
  ControlCommand hover(const QuadState & predicted_state);
  ControlCommand optitrack_land(const QuadState & predicted_state);
  ControlCommand vision_land(const QuadState & predicted_state);
  ControlCommand emergencyLand();
  ControlCommand waitForUserControllerInput(const QuadState & predicted_state);
  ControlCommand breakVelocity(const QuadState & predicted_state);

  void handleManualDesiredVelocityCommand();

  void manualDesiredVelocityCallback(const geometry_msgs::TwistStampedConstPtr msg);
  void startCallback(const std_msgs::EmptyConstPtr msg);
  void landCallback(const std_msgs::EmptyConstPtr msg);
  void offCallback(const std_msgs::EmptyConstPtr msg);
  void switchRCManualModeCallback(const std_msgs::BoolConstPtr msg);
  void heightEstimateCallback(const quad_msgs::QuadStateEstimateConstPtr msg);
  void reloadParamCallback(const std_msgs::EmptyConstPtr msg);

  bool isHeightEstimateAvailable();

  double timeInCurrentState() const;
  bool setNewState(const int desired_new_state);

protected:
  void flightcontrollerFdbCallback(const quad_msgs::ControllerFeedbackConstPtr& msg);

  ros::Subscriber manual_desired_velocity_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber switch_rc_manual_mode_sub_;
  ros::Subscriber height_estimate_sub_;
  ros::Subscriber reload_param_;
  ros::Subscriber flightcontroller_fdb_sub_;
  ros::Publisher desired_state_pub_;
  CopilotSettings settings_;

  large_angle_controller::LargeAngleController controller_;
  large_angle_controller::ControllerParameters controller_parameters_;

  sensor_fusion_util::SensorFusionUtilHelper vision_pipeline_helper_;

  quad_msgs::QuadStateEstimate height_estimate_;
  geometry_msgs::TwistStamped manual_desired_velocity_command_;

  int state_machine_before_emg_landing_;
  int desired_state_after_breaking_;
  int state_machine_before_waiting_for_user_controller_;
  int state_before_rc_manual_flight_;

  bool first_time_in_new_mode_;
  ros::Time time_of_switch_to_current_state_;

  QuadDesiredState optitrack_start_desired_state_;
  QuadDesiredState optitrack_land_desired_state_;
  QuadDesiredState vision_land_desired_state_;
  double drop_thrust_;

  ros::Time most_recent_flight_controller_command_stamp_;
  ros::Time time_first_height_measurement_during_start_;
  bool received_first_height_measurement_during_start_;

  double initialization_height_;
  bool land_using_vision_;
  bool time_to_ramp_down_;

  bool allow_open_loop_start_;
  bool feedthrough_safety_checks_;

  ros::Time time_last_manual_input_handled_;
  
  bool received_flight_controller_feedback_ = false;
  quad_msgs::ControllerFeedback flightcontroller_feedback_msg_;
};

} //namespace copilot
