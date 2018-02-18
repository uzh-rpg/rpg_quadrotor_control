#include "autopilot/autopilot.h"

#include <quadrotor_common/parameter_helper.h>

namespace autopilot
{

AutoPilot::AutoPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), autopilot_state_(States::OFF), state_predictor_(nh_,
                                                                        pnh_), time_of_switch_to_current_state_(), first_time_in_new_state_(
        true)
{
  if (!loadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  control_command_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>(
      "control_command", 1);

  // Subscribers
  state_estimate_sub_ = nh_.subscribe("state_estimate", 1,
                                      &AutoPilot::stateEstimateCallback, this);
  low_level_feedback_sub_ = nh_.subscribe("low_level_feedback", 1,
                                          &AutoPilot::lowLevelFeedbackCallback,
                                          this);

  pose_command_sub_ = nh_.subscribe("autopilot/pose_command", 1,
                                    &AutoPilot::poseCommandCallback, this);
  velocity_command_sub_ = nh_.subscribe("autopilot/velocity_command", 1,
                                        &AutoPilot::velocityCommandCallback,
                                        this);
  reference_state_sub_ = nh_.subscribe("autopilot/reference_state", 1,
                                       &AutoPilot::referenceStateCallback,
                                       this);
  trajectory_sub_ = nh_.subscribe("autopilot/trajectory", 1,
                                  &AutoPilot::trajectoryCallback, this);
  control_command_input_sub_ = nh_.subscribe(
      "autopilot/control_command_input", 1,
      &AutoPilot::controlCommandInputCallback, this);

  start_sub_ = nh_.subscribe("autopilot/start", 1, &AutoPilot::startCallback,
                             this);
  land_sub_ = nh_.subscribe("autopilot/land", 1, &AutoPilot::landCallback,
                            this);
  off_sub_ = nh_.subscribe("autopilot/off", 1, &AutoPilot::offCallback, this);
}

AutoPilot::~AutoPilot()
{
}

// TODO watchdog thread to check when the last state estimate was received
// -> trigger emergency landing if necessary

void AutoPilot::stateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // This triggers the control loop

  // TODO: Check if state estimate is valid, otherwise go to EMERGENCY_LANDING
  // immediately

  // Push received state estimate into predictor
  state_predictor_.updateWithStateEstimate(
      quadrotor_common::QuadStateEstimate(*msg));

  quadrotor_common::ControlCommand control_cmd;

  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time = wall_time_now
      + ros::Duration(control_command_delay_);

  quadrotor_common::QuadStateEstimate predicted_state;

  // Compute control command depending on autopilot state
  switch (autopilot_state_)
  {
    case States::OFF:
      control_cmd.zero();
      break;
    case States::START:
      break;
    case States::HOVER:
      break;
    case States::LAND:
      break;
    case States::EMERGENCY_LAND:
      break;
    case States::BREAKING:
      break;
    case States::GO_TO_POSE:
      break;
    case States::VELOCITY_CONTROL:
      break;
    case States::REFERENCE_CONTROL:
      break;
    case States::TRAJECTORY_CONTROL:
      break;
    case States::COMMAND_FEEDTHROUGH:
      // Do nothing here, command is being published in the command input
      // callback directly
      break;
    case States::RC_MANUAL:
      // Send an armed command with zero body rates and hover thrust to avoid
      // letting the low level part switch off when the remote control gives
      // the authority back to our autonomous controller
      control_cmd.zero();
      control_cmd.armed = true;
      control_cmd.collective_thrust = 9.81;
      break;
  }

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH)
  {
    control_cmd.timestamp = wall_time_now;
    control_cmd.expected_execution_time = command_execution_time;
    publishControlCommand(control_cmd);
  }
}

void AutoPilot::lowLevelFeedbackCallback(
    const quadrotor_msgs::LowLevelFeedback::ConstPtr& msg)
{
  if (msg->control_mode == msg->RC_MANUAL
      && autopilot_state_ != States::RC_MANUAL)
  {
    autopilot_state_ = States::RC_MANUAL;
  }
  if (msg->control_mode != msg->RC_MANUAL
      && autopilot_state_ == States::RC_MANUAL)
  {
    autopilot_state_ = States::BREAKING;
  }
}

void AutoPilot::poseCommandCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
}

void AutoPilot::velocityCommandCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
}

void AutoPilot::referenceStateCallback(
    const quadrotor_msgs::TrajectoryPoint::ConstPtr& msg)
{

}

void AutoPilot::trajectoryCallback(
    const quadrotor_msgs::Trajectory::ConstPtr& msg)
{

}

void AutoPilot::controlCommandInputCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg)
{
  // TODO: Some checks whether we allow feed forwarding of received control
  // commands

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH)
  {
    autopilot_state_ = States::COMMAND_FEEDTHROUGH;
  }

  control_command_pub_.publish(*msg);
}

void AutoPilot::startCallback(const std_msgs::Empty::ConstPtr& msg)
{

}

void AutoPilot::landCallback(const std_msgs::Empty::ConstPtr& msg)
{

}

void AutoPilot::offCallback(const std_msgs::Empty::ConstPtr& msg)
{

}

void AutoPilot::setAutoPilotState(const States& new_state)
{
  time_of_switch_to_current_state_ = ros::Time::now();
  first_time_in_new_state_ = true;
}

double AutoPilot::timeInCurrentState() const
{
  return (ros::Time::now() - time_of_switch_to_current_state_).toSec();
}

quadrotor_common::QuadStateEstimate AutoPilot::getPredictedStateEstimate(
    const ros::Time& time) const
{
  return state_predictor_.predictState(time);
}

void AutoPilot::publishControlCommand(
    const quadrotor_common::ControlCommand& control_cmd)
{
  if (control_cmd.control_mode == quadrotor_common::ControlMode::NONE)
  {
    ROS_ERROR("[%s] Control mode is NONE, will not publish ControlCommand",
              pnh_.getNamespace().c_str());
  }
  else
  {
    quadrotor_msgs::ControlCommand control_cmd_msg;

    control_cmd_msg = control_cmd.toRosMessage();

    control_command_pub_.publish(control_cmd_msg);
    state_predictor_.pushCommandToQueue(control_cmd);
  }
}

bool AutoPilot::loadParameters()
{
#define GET_PARAM(name) \
if (!quadrotor_common::getParam(#name, name ## _, pnh_)) \
  return false

  GET_PARAM(velocity_estimate_in_world_frame);
  GET_PARAM(control_command_delay);

  return true;

#undef GET_PARAM
}

} // namespace autopilot
