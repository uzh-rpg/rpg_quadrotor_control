#pragma once

#include <Eigen/Dense>
#include <quadrotor_msgs/QuadDesiredState.h>
#include <ros/time.h>

#include "quadrotor_common/geometry_eigen_conversions.h"
#include "quadrotor_common/math_common.h"

namespace quadrotor_common
{

class QuadDesiredState
{
public:
  QuadDesiredState();
  QuadDesiredState(const quadrotor_msgs::QuadDesiredState& msg);
  ~QuadDesiredState();

  quadrotor_msgs::QuadDesiredState toRosMessage() const;

  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;
  double heading;
  double heading_rate;
  double heading_acceleration;
};

} // quadrotor_common
