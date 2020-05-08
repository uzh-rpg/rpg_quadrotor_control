/*
 * control_command.cpp
 *
 *  Created on: Jul 8, 2015
 *      Author: matthias
 */

#include "quad_common/control_command.h"

namespace quad_common
{

ControlCommand::ControlCommand() :
    mode(ControllerMode::NONE), off(true), timestamp(ros::Time::now()), execution_time(timestamp), orientation(
        Eigen::Quaterniond::Identity()), bodyrates(Eigen::Vector3d::Zero()), angular_accelerations(
        Eigen::Vector3d::Zero()), thrust(0.0)
{
}

ControlCommand::ControlCommand(const quad_msgs::ControlCommand& control_command_msg)
{
  mode = ControllerMode::NONE;
  if (control_command_msg.control_mode == control_command_msg.ANGLE)
  {
    mode = ControllerMode::ANGLE;
  }
  else if (control_command_msg.control_mode == control_command_msg.ANGLERATE)
  {
    mode = ControllerMode::ANGLERATE;
  }
  off = control_command_msg.off;
  timestamp = control_command_msg.header.stamp;
  execution_time = control_command_msg.execution_time;
  orientation = geometryToEigen(control_command_msg.orientation);
  bodyrates = geometryToEigen(control_command_msg.bodyrates);
  angular_accelerations = geometryToEigen(control_command_msg.angular_accelerations);
  thrust = control_command_msg.thrust;
}

ControlCommand::~ControlCommand()
{
}

void ControlCommand::zero()
{
  mode = ControllerMode::ANGLERATE;
  off = true;
  timestamp = ros::Time::now();
  execution_time = timestamp;
  orientation = Eigen::Quaterniond::Identity();
  bodyrates = Eigen::Vector3d::Zero();
  angular_accelerations = Eigen::Vector3d::Zero();
  thrust = 0.0;
}

}

