#include "quad_common/quad_desired_state.h"

namespace quad_common
{

QuadDesiredState::QuadDesiredState() :
    timestamp(ros::Time::now()), position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()), acceleration(
        Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(Eigen::Vector3d::Zero()), yaw(0.0), yaw_rate(0.0), yaw_acceleration(
        0.0)
{
}

QuadDesiredState::QuadDesiredState(quad_msgs::QuadDesiredState quad_desired_state)
{
  timestamp = quad_desired_state.header.stamp;
  position = geometryToEigen(quad_desired_state.position);
  velocity = geometryToEigen(quad_desired_state.velocity);
  acceleration = geometryToEigen(quad_desired_state.acceleration);
  jerk = geometryToEigen(quad_desired_state.jerk);
  snap = geometryToEigen(quad_desired_state.snap);
  yaw = quad_desired_state.yaw;
  yaw_rate = quad_desired_state.yaw_rate;
  yaw_acceleration = quad_desired_state.yaw_acceleration;
}

QuadDesiredState::QuadDesiredState(quad_common::QuadState quad_state)
{
  timestamp = quad_state.timestamp;
  position = quad_state.position;
  velocity = quad_state.velocity;
  acceleration = Eigen::Vector3d::Zero();
  jerk = Eigen::Vector3d::Zero();
  yaw = quad_common::quaternionToEulerAnglesZYX(quad_state.orientation).z();
}

QuadDesiredState::~QuadDesiredState()
{
}

quad_msgs::QuadDesiredState QuadDesiredState::toRosMessage() const
{
  quad_msgs::QuadDesiredState msg;
  msg.header.stamp = timestamp;
  msg.position = eigenToGeometry(position);
  msg.velocity = eigenToGeometry(velocity);
  msg.acceleration = eigenToGeometry(acceleration);
  msg.jerk = eigenToGeometry(jerk);
  msg.snap = eigenToGeometry(snap);
  msg.yaw = yaw;
  msg.yaw_rate = yaw_rate;
  msg.yaw_acceleration = yaw_acceleration;
  return msg;
}

} // quad_common
