/*
 * control_command.h
 *
 *  Created on: Feb 11, 2014
 *      Author: ffontana
 */

#pragma once

#include "Eigen/Dense"
#include "quad_msgs/ControlCommand.h"
#include "quad_common/geometry_eigen_conversions.h"

namespace quad_common
{

enum class ControllerMode
{
  NONE, ANGLE, ANGLERATE
};

class ControlCommand
{

public:
  ControlCommand();
  ControlCommand(const quad_msgs::ControlCommand& control_command_msg);
  ~ControlCommand();

  void zero();

  ControllerMode mode;

  bool off;

  ros::Time timestamp;
  ros::Time execution_time;

  // Orientation of the body frame with respect to the world frame
  Eigen::Quaterniond orientation; // [-]

  // Body rates in body frame
  // Note that in ANGLE mode the x-y-bodyrates are only feed forward terms computed from a reference trajectory
  // Also in ANGLE mode, the z-bodyrate has to be from feedback control
  Eigen::Vector3d bodyrates; // [rad/s]

  // Angular accelerations in body frame
  Eigen::Vector3d angular_accelerations; // [rad/s^2]

  // Collective mass normalized thrust
  double thrust; // [m/s^2]
};

}
