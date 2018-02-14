#pragma once

#include <Eigen/Dense>
#include <quadrotor_msgs/ControlCommand.h>

namespace quadrotor_common
{

enum class ControlMode
{
  NONE, ATTITUDE, BODY_RATES, ANGULAR_ACCELERATIONS, ROTOR_THRUSTS
};

struct ControlCommand
{
  ControlCommand();
  ControlCommand(const quadrotor_msgs::ControlCommand& control_command_msg);
  virtual ~ControlCommand();

  void zero();
  quadrotor_msgs::ControlCommand toRosMessage();

  ros::Time timestamp;

  // Control mode as defined above
  ControlMode control_mode;

  // Flag whether controller is allowed to arm
  bool armed;

  // Time at which this command should be executed
  ros::Time expected_execution_time;

  // Orientation of the body frame with respect to the world frame
  Eigen::Quaterniond orientation; // [-]

  // Body rates in body frame
  // Note that in ATTITUDE mode the x-y-bodyrates are only feed forward terms 
  // computed from a reference trajectory
  // Also in ATTITUDE mode, the z-bodyrate has to be from feedback control
  Eigen::Vector3d bodyrates; // [rad/s]

  // Angular accelerations in body frame
  Eigen::Vector3d angular_accelerations; // [rad/s^2]

  // Collective mass normalized thrust
  double collective_thrust; // [m/s^2]

  // Single rotor thrusts [N]
  // These are only considered in the ROTOR_THRUSTS control mode
  Eigen::VectorXd rotor_thrusts;
};

} // namespace quadrotor_common
