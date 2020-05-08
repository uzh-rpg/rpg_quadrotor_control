/*
 * QuadState.h
 *
 *  Created on: Dec 17, 2013
 *      Author: ffontana
 */

#pragma once

#include "Eigen/Dense"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_common/geometry_eigen_conversions.h"
#include "ros/time.h"

namespace quad_common
{

class QuadState
{
public:
  QuadState();
  QuadState(quad_msgs::QuadStateEstimate quad_state_est);

  ~QuadState();

  quad_msgs::QuadStateEstimate toRosMessage() const;

  ros::Time timestamp;
  int estimator_id;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyrates;
};

} // quad_common
