#pragma once

#include <quadrotor_common/quad_state_estimate.h>

namespace state_predictor
{

struct QuadExtStateEstimate : quadrotor_common::QuadStateEstimate
{
  QuadExtStateEstimate();
  QuadExtStateEstimate(const nav_msgs::Odometry& quad_state_est);
  QuadExtStateEstimate(
      const quadrotor_common::QuadStateEstimate& quad_state_est);
  virtual ~QuadExtStateEstimate();

  quadrotor_common::QuadStateEstimate getQuadStateEstimate();

  double thrust;

  // Constants
  static constexpr double kDefaultThrust = 0.0;
};

} // namespace state_predictor
