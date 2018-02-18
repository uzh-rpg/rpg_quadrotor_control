/*
 * Copilot.cpp
 *
 *  Created on: Apr 1, 2014
 *      Author: ffontana
 */

#include "copilot/copilot.h"

namespace copilot
{

Copilot::Copilot(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) :
    ControllerWithPrimitives::ControllerWithPrimitives(nh, pnh), check_flight_controller_loop_timer_(), flyingroom_(nh,
                                                                                                                    pnh)
{
  if (!loadParameters())
    ros::shutdown();

  flight_controller_command_sub_ = nh_.subscribe(node_name_ + "/control_command_input", 1,
                                                 &Copilot::flightControllerCommandCallback, this);

  feedthrough_sub_ = nh_.subscribe(node_name_ + "/feedthrough", 1, &Copilot::feedthroughCallback, this);

  check_flight_controller_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / kLoopRateHz_),
                                                        &Copilot::checkUserControllerLoop, this);
}

Copilot::~Copilot()
{
  // TODO Auto-generated destructor stub
}

void Copilot::checkUserControllerLoop(const ros::TimerEvent& time)
{
  if (state_machine_ == states::FEEDTHROUGH)
  {
    if (!userControllerIsOk())
    {
      disableUserController();
    }
  }
}

bool Copilot::userControllerIsOk()
{
  if (!PositionIsWithinArena())
    return false;
  if (!userControllerIsAlive())
    return false;
  return true;
}

void Copilot::enableUserController()
{
  ROS_INFO("[%s] Enabling feedthrough mode", node_name_.c_str());

  // Perform some safety checks first
  // Check if copilot is hovering
  if (feedthrough_safety_checks_ && state_machine_ != states::HOVER)
  {
    ROS_ERROR("[%s] Safety check: Copilot is not in hover-mode. Will not "
              "enable feedthrough!",
              node_name_.c_str());
    return;
  }

  if (state_machine_ != states::WAIT_FOR_USER_CONTROLLER)
  {
    setNewState(states::WAIT_FOR_USER_CONTROLLER);
  }
}

void Copilot::disableUserController()
{
  if (state_machine_ == states::FEEDTHROUGH)
  {
    ROS_INFO("[%s] Disabling feedthrough mode", node_name_.c_str());

    // TODO: The following is probably useless, unless it changes the
    // predictor's internal state
    QuadState state_estimate = getStateEstimate(ros::Time::now()); // use time now since it is only used for updating the desired state

    // State transition
    setNewState(states::HOVER);
  }
}

void Copilot::feedthroughCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    enableUserController();
  }
  if (msg->data == false)
  {
    disableUserController();
  }
}

void Copilot::flightControllerCommandCallback(const quad_msgs::ControlCommand::ConstPtr& cmd)
{
  if (cmd->off)
  {
    // Before the desired state in the flight controller is set this will prevent its commands to be considered
    // If the flight controller does not have a state estimate anymore it will also send an off command.
    // This will cause the copilot to timeout on the flight controller command. Note that it checks the availability
    // of the state estimate separately and starts emergency landing if necessary.
    return;
  }
  most_recent_flight_controller_command_stamp_ = ros::Time::now();
  if (state_machine_ == states::FEEDTHROUGH)
  {
    if (cmd->off || cmd->control_mode == cmd->NONE || std::isnan(cmd->thrust) || std::isnan(cmd->orientation.x)
        || std::isnan(cmd->orientation.y) || std::isnan(cmd->orientation.z) || std::isnan(cmd->bodyrates.x)
        || std::isnan(cmd->bodyrates.y) || std::isnan(cmd->bodyrates.z) || std::isnan(cmd->angular_accelerations.x)
        || std::isnan(cmd->angular_accelerations.y) || std::isnan(cmd->angular_accelerations.z))
    {
      ROS_WARN("[%s] Received NaN value as control input from flight controller.", node_name_.c_str());
      disableUserController();
    }
    else
    {
      control_command_pub_.publish(*cmd);
      predictor_.pushCommandToQueue(ControlCommand(*cmd));
    }
  }
}

bool Copilot::PositionIsWithinArena()
{
  if (flyingroom_.useBounds())
  {
    QuadState state_estimate = getStateEstimate(ros::Time::now()); // use time now since it is only used for checking position bounds
    return flyingroom_.isWithinArena(state_estimate);
  }
  return true;
}

void Copilot::updateDesiredState(const quad_common::QuadDesiredState& desired_state)
{
  // only accept desired state when tracking is ok
  if (stateEstimateAvailable())
  {
    desired_state_ = desired_state;
    if (flyingroom_.useBounds())
    {
      flyingroom_.forceWithinArena(desired_state_);
    }
  }
  return;
}

bool Copilot::userControllerIsAlive()
{
  if (ros::Time::now() - most_recent_flight_controller_command_stamp_ < ros::Duration(settings_.user_command_timeout_))
  {
    return true;
  }
  ROS_INFO_THROTTLE(1, "[%s] User controller did not send a command within the last %.3f s ", node_name_.c_str(),
                    (ros::Time::now() - most_recent_flight_controller_command_stamp_).toSec());
  return false;
}

} /* namespace copilot */
