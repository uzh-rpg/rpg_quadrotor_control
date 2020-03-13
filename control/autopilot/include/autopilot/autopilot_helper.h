#pragma once

#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include "autopilot/autopilot_states.h"

namespace autopilot_helper {

class AutoPilotHelper {
 public:
  AutoPilotHelper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  AutoPilotHelper()
      : AutoPilotHelper(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~AutoPilotHelper();

  bool feedbackAvailable() const;
  double feedbackMessageAge() const;
  bool stateEstimateAvailable() const;

  // Blocking function. Blocks until either autopilot feedback is
  // available or the timeout has been reached.
  bool waitForAutopilotFeedback(const double timeout_s,
                                const double loop_rate_hz) const;
  bool waitForSpecificAutopilotState(const autopilot::States& state,
                                     const double timeout_s,
                                     const double loop_rate_hz) const;

  // Functions to get specific feedback from autopilot
  autopilot::States getCurrentAutopilotState() const;
  ros::Duration getCurrentControlCommandDelay() const;
  ros::Duration getCurrentControlComputationTime() const;
  ros::Duration getCurrentTrajectoryExecutionLeftDuration() const;
  int getCurrentTrajectoriesLeftInQueue() const;

  quadrotor_common::TrajectoryPoint getCurrentReferenceState() const;
  Eigen::Vector3d getCurrentReferencePosition() const;
  Eigen::Vector3d getCurrentReferenceVelocity() const;
  Eigen::Quaterniond getCurrentReferenceOrientation() const;
  double getCurrentReferenceHeading() const;

  quadrotor_common::QuadStateEstimate getCurrentStateEstimate() const;
  Eigen::Vector3d getCurrentPositionEstimate() const;
  Eigen::Vector3d getCurrentVelocityEstimate() const;
  Eigen::Quaterniond getCurrentOrientationEstimate() const;
  double getCurrentHeadingEstimate() const;

  // Functions to get specific errors from autopilot
  // error = estimate - reference
  Eigen::Vector3d getCurrentPositionError() const;
  Eigen::Vector3d getCurrentVelocityError() const;
  Eigen::Quaterniond getCurrentOrientationError() const;
  double getCurrentHeadingError() const;

  // Functions to send commands to the autopilot
  void sendPoseCommand(const Eigen::Vector3d& position,
                       const double heading) const;
  void sendVelocityCommand(const Eigen::Vector3d& velocity,
                           const double heading_rate) const;
  void sendReferenceState(
      const quadrotor_common::TrajectoryPoint& trajectory_point) const;
  void sendTrajectory(const quadrotor_common::Trajectory& trajectory) const;
  void sendControlCommandInput(
      const quadrotor_common::ControlCommand& control_command) const;

  void sendStart() const;
  void sendForceHover() const;
  void sendLand() const;
  void sendOff() const;

 private:
  void autopilotFeedbackCallback(
      const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher reference_state_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher control_command_input_pub_;

  ros::Publisher start_pub_;
  ros::Publisher force_hover_pub_;
  ros::Publisher land_pub_;
  ros::Publisher off_pub_;

  ros::Subscriber autopilot_feedback_sub_;

  // Variables
  quadrotor_msgs::AutopilotFeedback autopilot_feedback_;
  bool first_autopilot_feedback_received_;
  ros::Time time_last_feedback_received_;

  // Constants
  static constexpr double kFeedbackValidTimeout_ = 2.0;
};

}  // namespace autopilot_helper
