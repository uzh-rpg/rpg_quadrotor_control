#include "quad_common/quad_state.h"

namespace quad_common
{

QuadState::QuadState() :
    timestamp(ros::Time::now()), estimator_id(0), position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()), orientation(
        Eigen::Quaterniond(1, 0, 0, 0)), bodyrates(Eigen::Vector3d::Zero())
{
}

QuadState::QuadState(quad_msgs::QuadStateEstimate quad_state_est)
{
  timestamp = quad_state_est.header.stamp;
  estimator_id = quad_state_est.estimator_id;
  position = geometryToEigen(quad_state_est.position);
  velocity = geometryToEigen(quad_state_est.velocity);
  orientation = geometryToEigen(quad_state_est.orientation);
  bodyrates = geometryToEigen(quad_state_est.bodyrates);
}

QuadState::~QuadState()
{
}

quad_msgs::QuadStateEstimate QuadState::toRosMessage() const
{
  quad_msgs::QuadStateEstimate msg;
  msg.header.stamp = timestamp;
  msg.estimator_id = estimator_id;
  msg.position = eigenToGeometry(position);
  msg.velocity = eigenToGeometry(velocity);
  msg.orientation = eigenToGeometry(orientation);
  msg.bodyrates = eigenToGeometry(bodyrates);
  return msg;
}

} // quad_common
