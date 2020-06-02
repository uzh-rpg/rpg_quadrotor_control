#pragma once

#include <atomic>
#include <list>
#include <mutex>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <position_controller/position_controller.h>
#include <position_controller/position_controller_params.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/LowLevelFeedback.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <ros/ros.h>
#include <state_predictor/state_predictor.h>
#include <std_msgs/Empty.h>

#include "autopilot/autopilot_states.h"

namespace autopilot {

template <typename Tcontroller, typename Tparams>
class AutoPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AutoPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  AutoPilot() : AutoPilot(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~AutoPilot();

 private:
  void watchdogThread();
  void goToPoseThread();

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
  void forceHoverCallback(const std_msgs::Empty::ConstPtr& msg);
  void landCallback(const std_msgs::Empty::ConstPtr& msg);
  void offCallback(const std_msgs::Empty::ConstPtr& msg);
  void reloadParamsCallback(const std_msgs::Empty::ConstPtr& msg);

  quadrotor_common::ControlCommand start(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand hover(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand land(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand breakVelocity(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand waitForGoToPoseAction(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand velocityControl(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand followReference(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::ControlCommand executeTrajectory(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      ros::Duration* trajectory_execution_left_duration,
      int* trajectories_left_in_queue);

  void setAutoPilotState(const States& new_state);
  void setAutoPilotStateForced(const States& new_state);
  double timeInCurrentState() const;
  quadrotor_common::QuadStateEstimate getPredictedStateEstimate(
      const ros::Time& time) const;

  void publishControlCommand(const quadrotor_common::ControlCommand& command);
  void publishAutopilotFeedback(
      const States& autopilot_state, const ros::Duration& control_command_delay,
      const ros::Duration& control_computation_time,
      const ros::Duration& trajectory_execution_left_duration,
      const int trajectories_left_in_queue,
      const quadrotor_msgs::LowLevelFeedback& low_level_feedback,
      const quadrotor_common::TrajectoryPoint& reference_state,
      const quadrotor_common::QuadStateEstimate& state_estimate);

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Main mutex:
  // This mutex is locked in the watchdogThread and all the Callback functions
  // All other functions should only be called from one of those and therefore
  // do not need to lock the mutex themselves
  // The functions that lock the mutex do so at their start of execution and
  // keep it locked until they are finished
  mutable std::mutex main_mutex_;

  // Go to pose mutex:
  // This mutex is locked in the goToPoseThread and the poseCommandCallback and
  // should be called according to the order in goToPoseThread before locking
  // the main thread
  // It specifically protects
  // - requested_go_to_pose_
  // - received_go_to_pose_command_
  mutable std::mutex go_to_pose_mutex_;

  ros::Publisher control_command_pub_;
  ros::Publisher autopilot_feedback_pub_;

  ros::Subscriber state_estimate_sub_;
  ros::Subscriber low_level_feedback_sub_;
  ros::Subscriber pose_command_sub_;
  ros::Subscriber velocity_command_sub_;
  ros::Subscriber reference_state_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber control_command_input_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber force_hover_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber reload_param_sub_;

  state_predictor::StatePredictor state_predictor_;

  Tcontroller base_controller_;
  Tparams base_controller_params_;

  quadrotor_common::TrajectoryPoint reference_state_;
  quadrotor_common::Trajectory reference_trajectory_;

  // Values received from callbacks
  quadrotor_common::QuadStateEstimate received_state_est_;
  geometry_msgs::TwistStamped desired_velocity_command_;
  quadrotor_msgs::TrajectoryPoint reference_state_input_;
  quadrotor_msgs::LowLevelFeedback received_low_level_feedback_;

  States autopilot_state_;
  States state_before_rc_manual_flight_;

  // State switching variables
  bool state_estimate_available_;
  ros::Time time_of_switch_to_current_state_;
  bool first_time_in_new_state_;
  Eigen::Vector3d initial_start_position_;
  Eigen::Vector3d initial_land_position_;
  bool time_to_ramp_down_;
  ros::Time time_started_ramping_down_;
  double initial_drop_thrust_;
  ros::Time time_last_velocity_command_handled_;
  ros::Time time_last_reference_state_input_received_;
  States desired_state_after_breaking_;
  States state_before_emergency_landing_;
  bool force_breaking_;

  // Go to pose variables
  std::thread go_to_pose_thread_;
  geometry_msgs::PoseStamped requested_go_to_pose_;
  bool received_go_to_pose_command_;
  std::atomic_bool stop_go_to_pose_thread_;

  // Trajectory execution variables
  std::list<quadrotor_common::Trajectory> trajectory_queue_;
  ros::Time time_start_trajectory_execution_;

  // Control command input variables
  ros::Time time_last_control_command_input_received_;
  bool last_control_command_input_thrust_high_;

  // Watchdog
  std::thread watchdog_thread_;
  std::atomic_bool stop_watchdog_thread_;
  ros::Time time_last_state_estimate_received_;
  ros::Time time_started_emergency_landing_;

  std::atomic_bool destructor_invoked_;

  ros::Time time_last_autopilot_feedback_published_;
  ros::Time time_last_control_command_published_;
  ros::Time time_last_control_command_computed_;

  // Parameters
  double state_estimate_timeout_;
  bool velocity_estimate_in_world_frame_;
  double control_command_delay_;
  double start_land_velocity_;
  double start_land_acceleration_;
  double start_idle_duration_;
  double idle_thrust_;
  double optitrack_start_height_;
  double optitrack_start_land_timeout_;
  double optitrack_land_drop_height_;
  double propeller_ramp_down_timeout_;
  double breaking_velocity_threshold_;
  double breaking_timeout_;
  double go_to_pose_max_velocity_;
  double go_to_pose_max_normalized_thrust_;
  double go_to_pose_max_roll_pitch_rate_;
  double velocity_command_input_timeout_;
  double tau_velocity_command_;
  double reference_state_input_timeout_;
  double emergency_land_duration_;
  double emergency_land_thrust_;
  double control_command_input_timeout_;
  bool enable_command_feedthrough_;
  double predictive_control_lookahead_;
  double min_control_period_pub_;
  double min_control_period_comp_;

  // Constants
  static constexpr double kVelocityCommandZeroThreshold_ = 0.03;
  static constexpr double kPositionJumpTolerance_ = 0.5;
  static constexpr double kGravityAcc_ = 9.81;
  static constexpr double kWatchdogFrequency_ = 50.0;
  static constexpr double kMaxAutopilotFeedbackPublishFrequency_ = 60.0;
  static constexpr double kGoToPoseIdleFrequency_ = 50.0;
  static constexpr double kGoToPoseTrajectorySamplingFrequency_ = 50.0;
  static constexpr int kGoToPosePolynomialOrderOfContinuity_ = 5;
  static constexpr double kGoToPoseNeglectThreshold_ = 0.05;
  static constexpr double kThrustHighThreshold_ = 0.5;
};

}  // namespace autopilot

#include "./autopilot_inl.h"
