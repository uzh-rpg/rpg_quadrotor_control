#include "autopilot/autopilot_helper.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <std_msgs/Empty.h>

namespace autopilot_helper {

AutoPilotHelper::AutoPilotHelper(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
    : autopilot_feedback_(),
      first_autopilot_feedback_received_(false),
      time_last_feedback_received_() {
  pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
  velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "autopilot/velocity_command", 1);
  reference_state_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>(
      "autopilot/reference_state", 1);
  trajectory_pub_ =
      nh_.advertise<quadrotor_msgs::Trajectory>("autopilot/trajectory", 1);
  control_command_input_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>(
      "autopilot/control_command_input", 1);

  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  force_hover_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/force_hover", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/land", 1);
  off_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);

  autopilot_feedback_sub_ =
      nh_.subscribe("autopilot/feedback", 10,
                    &AutoPilotHelper::autopilotFeedbackCallback, this);
}

AutoPilotHelper::~AutoPilotHelper() {}

bool AutoPilotHelper::feedbackAvailable() const {
  if (!first_autopilot_feedback_received_) {
    return false;
  }

  if (feedbackMessageAge() > kFeedbackValidTimeout_) {
    return false;
  }

  return true;
}

double AutoPilotHelper::feedbackMessageAge() const {
  if (!first_autopilot_feedback_received_) {
    // Just return a "very" large number
    return 100.0 * kFeedbackValidTimeout_;
  }

  return (ros::Time::now() - time_last_feedback_received_).toSec();
}

bool AutoPilotHelper::stateEstimateAvailable() const {
  if (getCurrentStateEstimate().coordinate_frame ==
      quadrotor_common::QuadStateEstimate::CoordinateFrame::INVALID) {
    return false;
  }

  return true;
}

bool AutoPilotHelper::waitForAutopilotFeedback(
    const double timeout_s, const double loop_rate_hz) const {
  const ros::Duration timeout(timeout_s);
  ros::Rate loop_rate(loop_rate_hz);
  const ros::Time start_wait = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_wait) <= timeout) {
    ros::spinOnce();
    if (feedbackAvailable()) {
      return true;
    }
    loop_rate.sleep();
  }

  return false;
}

bool AutoPilotHelper::waitForSpecificAutopilotState(
    const autopilot::States& state, const double timeout_s,
    const double loop_rate_hz) const {
  const ros::Duration timeout(timeout_s);
  ros::Rate loop_rate(loop_rate_hz);
  const ros::Time start_wait = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_wait) <= timeout) {
    ros::spinOnce();
    if (feedbackAvailable() && getCurrentAutopilotState() == state) {
      return true;
    }
    loop_rate.sleep();
  }

  return false;
}

autopilot::States AutoPilotHelper::getCurrentAutopilotState() const {
  switch (autopilot_feedback_.autopilot_state) {
    case autopilot_feedback_.OFF:
      return autopilot::States::OFF;
    case autopilot_feedback_.START:
      return autopilot::States::START;
    case autopilot_feedback_.HOVER:
      return autopilot::States::HOVER;
    case autopilot_feedback_.LAND:
      return autopilot::States::LAND;
    case autopilot_feedback_.EMERGENCY_LAND:
      return autopilot::States::EMERGENCY_LAND;
    case autopilot_feedback_.BREAKING:
      return autopilot::States::BREAKING;
    case autopilot_feedback_.GO_TO_POSE:
      return autopilot::States::GO_TO_POSE;
    case autopilot_feedback_.VELOCITY_CONTROL:
      return autopilot::States::VELOCITY_CONTROL;
    case autopilot_feedback_.REFERENCE_CONTROL:
      return autopilot::States::REFERENCE_CONTROL;
    case autopilot_feedback_.TRAJECTORY_CONTROL:
      return autopilot::States::TRAJECTORY_CONTROL;
    case autopilot_feedback_.COMMAND_FEEDTHROUGH:
      return autopilot::States::COMMAND_FEEDTHROUGH;
    case autopilot_feedback_.RC_MANUAL:
      return autopilot::States::RC_MANUAL;
    default:
      return autopilot::States::OFF;
  }
}

ros::Duration AutoPilotHelper::getCurrentControlCommandDelay() const {
  return autopilot_feedback_.control_command_delay;
}

ros::Duration AutoPilotHelper::getCurrentControlComputationTime() const {
  return autopilot_feedback_.control_computation_time;
}

ros::Duration AutoPilotHelper::getCurrentTrajectoryExecutionLeftDuration()
    const {
  return autopilot_feedback_.trajectory_execution_left_duration;
}

int AutoPilotHelper::getCurrentTrajectoriesLeftInQueue() const {
  return autopilot_feedback_.trajectories_left_in_queue;
}

quadrotor_common::TrajectoryPoint AutoPilotHelper::getCurrentReferenceState()
    const {
  return quadrotor_common::TrajectoryPoint(autopilot_feedback_.reference_state);
}

Eigen::Vector3d AutoPilotHelper::getCurrentReferencePosition() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.reference_state.pose.position);
}

Eigen::Vector3d AutoPilotHelper::getCurrentReferenceVelocity() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.reference_state.velocity.linear);
}

Eigen::Quaterniond AutoPilotHelper::getCurrentReferenceOrientation() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.reference_state.pose.orientation);
}

double AutoPilotHelper::getCurrentReferenceHeading() const {
  return quadrotor_common::quaternionToEulerAnglesZYX(
             quadrotor_common::geometryToEigen(
                 autopilot_feedback_.reference_state.pose.orientation))
      .z();
}

quadrotor_common::QuadStateEstimate AutoPilotHelper::getCurrentStateEstimate()
    const {
  return quadrotor_common::QuadStateEstimate(
      autopilot_feedback_.state_estimate);
}

Eigen::Vector3d AutoPilotHelper::getCurrentPositionEstimate() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.state_estimate.pose.pose.position);
}

Eigen::Vector3d AutoPilotHelper::getCurrentVelocityEstimate() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.state_estimate.twist.twist.linear);
}

Eigen::Quaterniond AutoPilotHelper::getCurrentOrientationEstimate() const {
  return quadrotor_common::geometryToEigen(
      autopilot_feedback_.state_estimate.pose.pose.orientation);
}

double AutoPilotHelper::getCurrentHeadingEstimate() const {
  return quadrotor_common::quaternionToEulerAnglesZYX(
             quadrotor_common::geometryToEigen(
                 autopilot_feedback_.state_estimate.pose.pose.orientation))
      .z();
}

Eigen::Vector3d AutoPilotHelper::getCurrentPositionError() const {
  return getCurrentPositionEstimate() - getCurrentReferencePosition();
}

Eigen::Vector3d AutoPilotHelper::getCurrentVelocityError() const {
  return getCurrentVelocityEstimate() - getCurrentReferenceVelocity();
}

Eigen::Quaterniond AutoPilotHelper::getCurrentOrientationError() const {
  return getCurrentReferenceOrientation().inverse() *
         getCurrentOrientationEstimate();
}

double AutoPilotHelper::getCurrentHeadingError() const {
  return quadrotor_common::wrapMinusPiToPi(getCurrentHeadingEstimate() -
                                           getCurrentReferenceHeading());
}

void AutoPilotHelper::sendPoseCommand(const Eigen::Vector3d& position,
                                      const double heading) const {
  geometry_msgs::PoseStamped pose_cmd;
  pose_cmd.pose.position.x = position.x();
  pose_cmd.pose.position.y = position.y();
  pose_cmd.pose.position.z = position.z();
  pose_cmd.pose.orientation =
      quadrotor_common::eigenToGeometry(Eigen::Quaterniond(
          Eigen::AngleAxisd(quadrotor_common::wrapMinusPiToPi(heading),
                            Eigen::Vector3d::UnitZ())));

  pose_pub_.publish(pose_cmd);
}

void AutoPilotHelper::sendVelocityCommand(const Eigen::Vector3d& velocity,
                                          const double heading_rate) const {
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.twist.linear.x = velocity.x();
  vel_cmd.twist.linear.y = velocity.y();
  vel_cmd.twist.linear.z = velocity.z();
  vel_cmd.twist.angular.z = heading_rate;

  velocity_pub_.publish(vel_cmd);
}

void AutoPilotHelper::sendReferenceState(
    const quadrotor_common::TrajectoryPoint& trajectory_point) const {
  reference_state_pub_.publish(trajectory_point.toRosMessage());
}

void AutoPilotHelper::sendTrajectory(
    const quadrotor_common::Trajectory& trajectory) const {
  trajectory_pub_.publish(trajectory.toRosMessage());
}

void AutoPilotHelper::sendControlCommandInput(
    const quadrotor_common::ControlCommand& control_command) const {
  control_command_input_pub_.publish(control_command.toRosMessage());
}

void AutoPilotHelper::sendStart() const {
  start_pub_.publish(std_msgs::Empty());
}

void AutoPilotHelper::sendForceHover() const {
  force_hover_pub_.publish(std_msgs::Empty());
}

void AutoPilotHelper::sendLand() const { land_pub_.publish(std_msgs::Empty()); }

void AutoPilotHelper::sendOff() const { off_pub_.publish(std_msgs::Empty()); }

void AutoPilotHelper::autopilotFeedbackCallback(
    const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg) {
  time_last_feedback_received_ = ros::Time::now();

  autopilot_feedback_ = *msg;

  if (!first_autopilot_feedback_received_) {
    first_autopilot_feedback_received_ = true;
  }
}

}  // namespace autopilot_helper
