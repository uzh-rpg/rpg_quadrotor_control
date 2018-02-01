#include "quadrotor_common/quad_desired_state.h"

namespace quadrotor_common
{

QuadDesiredState::QuadDesiredState() :
    timestamp(ros::Time::now()), position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()), acceleration(
        Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(Eigen::Vector3d::Zero()), heading(0.0), heading_rate(
        0.0), heading_acceleration(0.0)
{
}

QuadDesiredState::QuadDesiredState(const quadrotor_msgs::QuadDesiredState& msg)
{
  timestamp = msg.header.stamp;
  position = geometryToEigen(msg.position);
  velocity = geometryToEigen(msg.velocity);
  acceleration = geometryToEigen(msg.acceleration);
  jerk = geometryToEigen(msg.jerk);
  snap = geometryToEigen(msg.snap);
  heading = msg.heading;
  heading_rate = msg.heading_rate;
  heading_acceleration = msg.heading_acceleration;
}

QuadDesiredState::~QuadDesiredState()
{
}

quadrotor_msgs::QuadDesiredState QuadDesiredState::toRosMessage() const
{
  quadrotor_msgs::QuadDesiredState msg;
  msg.header.stamp = timestamp;
  msg.position = eigenToGeometry(position);
  msg.velocity = eigenToGeometry(velocity);
  msg.acceleration = eigenToGeometry(acceleration);
  msg.jerk = eigenToGeometry(jerk);
  msg.snap = eigenToGeometry(snap);
  msg.heading = heading;
  msg.heading_rate = heading_rate;
  msg.heading_acceleration = heading_acceleration;
  return msg;
}

} // quadrotor_common
