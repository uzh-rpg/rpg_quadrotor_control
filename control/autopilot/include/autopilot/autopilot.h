#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <position_controller/position_controller.h>
#include <position_controller/position_controller_params.h>
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

  quadrotor_common::ControlCommand start(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand hover(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand land(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand breakVelocity(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand velocityControl(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand followReference(
      const quadrotor_common::QuadStateEstimate& state_estimate);

  void setAutoPilotState(const States& new_state);
  void setAutoPilotStateAfterExecutingBreakManeuver(const States& new_state);
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
  States state_before_rc_manual_flight_;

  state_predictor::StatePredictor state_predictor_;

  position_controller::PositionController base_controller_;
  position_controller::PositionControllerParams base_controller_params_;

  quadrotor_common::TrajectoryPoint reference_state_;

  quadrotor_common::QuadStateEstimate received_state_est_;
  bool state_estimate_available_;

  ros::Time time_of_switch_to_current_state_;
  bool first_time_in_new_state_;

  bool time_to_ramp_down_;

  Eigen::Vector3d initial_start_position_;
  Eigen::Vector3d initial_land_position_;
  double initial_drop_thrust_;
  ros::Time time_started_ramping_down_;

  geometry_msgs::TwistStamped desired_velocity_command_;
  ros::Time time_last_velocity_command_handled_;

  quadrotor_msgs::TrajectoryPoint reference_state_input_;
  ros::Time time_last_reference_state_input_received_;
  States desired_state_after_breaking_;

  // Parameters
  bool velocity_estimate_in_world_frame_;
  double control_command_delay_;
  double optitrack_land_drop_height_;
  double optitrack_start_land_timeout_;
  double optitrack_start_height_;
  double start_idle_duration_;
  double idle_thrust_;
  double start_land_velocity_;
  double propeller_ramp_down_timeout_;
  double velocity_command_input_timeout_;
  double tau_velocity_command_;
  double reference_state_input_timeout_;
  double breaking_velocity_threshold_;
  double breaking_timeout_;

  // Constants
  static constexpr double kVelocityCommandZeroThreshold_ = 0.03;
  static constexpr double kPositionJumpTolerance_ = 0.5;
};

} // namespace autopilot
