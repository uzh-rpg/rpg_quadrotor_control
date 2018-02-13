#include "quadrotor_common/control_command.h"

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

ControlCommand::ControlCommand() :
    control_mode(ControlMode::NONE), armed(false), timestamp(ros::Time::now()),
        expected_execution_time(timestamp),
        orientation(Eigen::Quaterniond::Identity()), bodyrates(
        Eigen::Vector3d::Zero()), angular_accelerations(
        Eigen::Vector3d::Zero()), collective_thrust(0.0), rotor_thrusts()
{
}

ControlCommand::ControlCommand(
    const quadrotor_msgs::ControlCommand& control_command_msg)
{
  timestamp = control_command_msg.header.stamp;
  switch (control_command_msg.control_mode)
  {
    case control_command_msg.NONE:
      control_mode = ControlMode::NONE;
      break;
    case control_command_msg.ATTITUDE:
      control_mode = ControlMode::ATTITUDE;
      break;
    case control_command_msg.BODY_RATES:
      control_mode = ControlMode::BODY_RATES;
      break;
    case control_command_msg.ANGULAR_ACCELERATIONS:
      control_mode = ControlMode::ANGULAR_ACCELERATIONS;
      break;
    case control_command_msg.ROTOR_THRUSTS:
      control_mode = ControlMode::ROTOR_THRUSTS;
      break;
  }
  armed = control_command_msg.armed;
  expected_execution_time = control_command_msg.expected_execution_time;
  orientation = geometryToEigen(control_command_msg.orientation);
  bodyrates = geometryToEigen(control_command_msg.bodyrates);
  angular_accelerations = geometryToEigen(
      control_command_msg.angular_accelerations);
  collective_thrust = control_command_msg.collective_thrust;
  rotor_thrusts.resize(control_command_msg.rotor_thrusts.size());
  for (int i = 0; i < control_command_msg.rotor_thrusts.size(); i++)
  {
    rotor_thrusts(i) = control_command_msg.rotor_thrusts[i];
  }
}

ControlCommand::~ControlCommand()
{
}

void ControlCommand::zero()
{
  timestamp = ros::Time::now();
  control_mode = ControlMode::BODY_RATES;
  armed = false;
  expected_execution_time = timestamp;
  orientation = Eigen::Quaterniond::Identity();
  bodyrates = Eigen::Vector3d::Zero();
  angular_accelerations = Eigen::Vector3d::Zero();
  collective_thrust = 0.0;
  rotor_thrusts = Eigen::VectorXd::Zero(rotor_thrusts.size());
}

quadrotor_msgs::ControlCommand ControlCommand::toRosMessage()
{
  quadrotor_msgs::ControlCommand ros_msg;

  ros_msg.header.stamp = timestamp;

  switch (control_mode)
  {
    case ControlMode::NONE:
      ros_msg.control_mode = ros_msg.NONE;
      break;
    case ControlMode::ATTITUDE:
      ros_msg.control_mode = ros_msg.ATTITUDE;
      break;
    case ControlMode::BODY_RATES:
      ros_msg.control_mode = ros_msg.BODY_RATES;
      break;
    case ControlMode::ANGULAR_ACCELERATIONS:
      ros_msg.control_mode = ros_msg.ANGULAR_ACCELERATIONS;
      break;
    case ControlMode::ROTOR_THRUSTS:
      ros_msg.control_mode = ros_msg.ROTOR_THRUSTS;
      break;
  }
  ros_msg.armed = armed;
  ros_msg.expected_execution_time = expected_execution_time;
  ros_msg.orientation = eigenToGeometry(orientation);
  ros_msg.bodyrates = eigenToGeometry(bodyrates);
  ros_msg.angular_accelerations = eigenToGeometry(angular_accelerations);
  ros_msg.collective_thrust = collective_thrust;
  for (int i = 0; i < rotor_thrusts.size(); i++)
  {
    ros_msg.rotor_thrusts.push_back(rotor_thrusts(i));
  }

  return ros_msg;
}

} // namespace quadrotor_common
