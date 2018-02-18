#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/LowLevelFeedback.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <ros/ros.h>
#include <state_predictor/state_predictor.h>
#include <std_msgs/Empty.h>

namespace autopilot
{

enum class States
{
  OFF, START, HOVER, LAND, EMERGENCY_LAND, BREAKING, GO_TO_POSE,
  VELOCITY_CONTROL, REFERENCE_CONTROL, TRAJECTORY_CONTROL, COMMAND_FEEDTHROUGH,
  RC_MANUAL
};

class AutoPilot
{
public:

  AutoPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  AutoPilot() :
      AutoPilot(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }

  virtual ~AutoPilot();

private:

  void stateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void lowLevelFeedbackCallback(
      const quadrotor_msgs::LowLevelFeedback::ConstPtr& msg);

  void poseCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void velocityCommandCallback(
      const geometry_msgs::TwistStamped::ConstPtr& msg);
  void referenceStateCallback(
      const quadrotor_msgs::TrajectoryPoint::ConstPtr& msg);
  void trajectoryCallback(const quadrotor_msgs::Trajectory::ConstPtr& msg);
  void controlCommandInputCallback(
      const quadrotor_msgs::ControlCommand::ConstPtr& msg);
  void startCallback(const std_msgs::Empty::ConstPtr& msg);
  void landCallback(const std_msgs::Empty::ConstPtr& msg);
  void offCallback(const std_msgs::Empty::ConstPtr& msg);

  void setAutoPilotState(const States& new_state);
  double timeInCurrentState() const;

  quadrotor_common::QuadStateEstimate getPredictedStateEstimate(
      const ros::Time& time) const;
  void publishControlCommand(const quadrotor_common::ControlCommand& command);

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher control_command_pub_;

  ros::Subscriber state_estimate_sub_;
  ros::Subscriber low_level_feedback_sub_;

  ros::Subscriber pose_command_sub_;
  ros::Subscriber velocity_command_sub_;
  ros::Subscriber reference_state_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber control_command_input_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;

  States autopilot_state_;

  state_predictor::StatePredictor state_predictor_;

  ros::Time time_of_switch_to_current_state_;
  bool first_time_in_new_state_;

  // Parameters
  bool velocity_estimate_in_world_frame_;
  double control_command_delay_;
};

} // namespace autopilot
