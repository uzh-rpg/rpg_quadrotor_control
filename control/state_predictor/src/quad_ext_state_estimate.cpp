#include "state_predictor/quad_ext_state_estimate.h"

namespace state_predictor
{

QuadExtStateEstimate::QuadExtStateEstimate() :
    quadrotor_common::QuadStateEstimate(), thrust(kDefaultThrust)
{
}

QuadExtStateEstimate::QuadExtStateEstimate(
    const nav_msgs::Odometry& quad_state_est) :
    quadrotor_common::QuadStateEstimate(quad_state_est), thrust(kDefaultThrust)
{
}

QuadExtStateEstimate::QuadExtStateEstimate(
    const quadrotor_common::QuadStateEstimate& quad_state_est)
{
  timestamp = quad_state_est.timestamp;
  coordinate_frame = quad_state_est.coordinate_frame;
  position = quad_state_est.position;
  velocity = quad_state_est.velocity;
  orientation = quad_state_est.orientation;
  bodyrates = quad_state_est.bodyrates;
  thrust = kDefaultThrust;
}

QuadExtStateEstimate::~QuadExtStateEstimate()
{
}

quadrotor_common::QuadStateEstimate QuadExtStateEstimate::getQuadStateEstimate()
{
  quadrotor_common::QuadStateEstimate quad_state_est;
  quad_state_est.timestamp = timestamp;
  quad_state_est.coordinate_frame = coordinate_frame;
  quad_state_est.position = position;
  quad_state_est.velocity = velocity;
  quad_state_est.orientation = orientation;
  quad_state_est.bodyrates = bodyrates;

  return quad_state_est;
}

} // namespace state_predictor
