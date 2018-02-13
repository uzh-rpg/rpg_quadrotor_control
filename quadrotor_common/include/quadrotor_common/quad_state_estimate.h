#pragma once

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

enum class CoordinateFrame
{
  WORLD, OPTITRACK, VISION, LOCAL
};

struct QuadStateEstimate
{
public:
  QuadStateEstimate();
  QuadStateEstimate(const nav_msgs::Odometry& state_estimate_msg);

  ~QuadStateEstimate();

  nav_msgs::Odometry toRosMessage();
  void transformBodyRatesToBodyFrame();

  ros::Time timestamp;
  CoordinateFrame coordinate_frame;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
};

} // namespace quadrotor_common
