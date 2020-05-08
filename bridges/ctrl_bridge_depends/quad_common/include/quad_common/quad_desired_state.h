#pragma once

#include "Eigen/Dense"
#include "quad_msgs/QuadDesiredState.h"
#include "quad_common/geometry_eigen_conversions.h"
#include "quad_common/math_common.h"  // quaternionToEulerAnglesZYX
#include "quad_common/quad_state.h"
#include "ros/time.h"

namespace quad_common
{

class QuadDesiredState
{
public:
  QuadDesiredState();
  QuadDesiredState(quad_msgs::QuadDesiredState quad_desired_state);
  QuadDesiredState(quad_common::QuadState quad_state);  // Conversion
  ~QuadDesiredState();

  quad_msgs::QuadDesiredState toRosMessage() const;

  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;
  double yaw;
  double yaw_rate;
  double yaw_acceleration;
};

} // quad_common
