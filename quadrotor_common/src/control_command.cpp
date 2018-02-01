#include "quadrotor_common/control_command.h"

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

ControlCommand::ControlCommand() :
    control_mode(ControlMode::NONE), armed(false), timestamp(ros::Time::now()), execution_time(timestamp), orientation(
        Eigen::Quaterniond::Identity()), bodyrates(Eigen::Vector3d::Zero()), angular_accelerations(
        Eigen::Vector3d::Zero()), collective_thrust(0.0)
{
}

ControlCommand::ControlCommand(const quadrotor_msgs::ControlCommand& control_command_msg)
{
  control_mode = ControlMode::NONE;
  if (control_command_msg.control_mode == control_command_msg.ATTITUDE)
  {
    control_mode = ControlMode::ATTITUDE;
  }
  else if (control_command_msg.control_mode == control_command_msg.BODY_RATES)
  {
    control_mode = ControlMode::BODY_RATES;
  }
  armed = control_command_msg.armed;
  timestamp = control_command_msg.header.stamp;
  execution_time = control_command_msg.execution_time;
  orientation = geometryToEigen(control_command_msg.orientation);
  bodyrates = geometryToEigen(control_command_msg.bodyrates);
  angular_accelerations = geometryToEigen(control_command_msg.angular_accelerations);
  collective_thrust = control_command_msg.collective_thrust;
  // TODO: rotor_thrusts
}

ControlCommand::~ControlCommand()
{
}

void ControlCommand::zero()
{
  control_mode = ControlMode::BODY_RATES;
  armed = false;
  timestamp = ros::Time::now();
  execution_time = timestamp;
  orientation = Eigen::Quaterniond::Identity();
  bodyrates = Eigen::Vector3d::Zero();
  angular_accelerations = Eigen::Vector3d::Zero();
  collective_thrust = 0.0;
  // TODO: rotor_thrusts
}

} // namespace quadrotor_common
