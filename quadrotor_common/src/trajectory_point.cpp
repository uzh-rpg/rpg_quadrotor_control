#include "quadrotor_common/trajectory_point.h"

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

TrajectoryPoint::TrajectoryPoint() :
    time_from_start(ros::Duration(0.0)), position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()), acceleration(
        Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(
        Eigen::Vector3d::Zero()), bodyrates(Eigen::Vector3d::Zero()),
        angular_acceleration(Eigen::Vector3d::Zero()),
        angular_jerk(Eigen::Vector3d::Zero()), angular_snap(
        Eigen::Vector3d::Zero()), heading(0.0), heading_rate(0.0),
        heading_acceleration(0.0)
{
}

TrajectoryPoint::TrajectoryPoint(
    const quadrotor_msgs::TrajectoryPoint& trajectory_point_msg)
{
  time_from_start = trajectory_point_msg.time_from_start;

  position = geometryToEigen(trajectory_point_msg.pose.position);
  orientation = geometryToEigen(trajectory_point_msg.pose.orientation);

  velocity = geometryToEigen(trajectory_point_msg.velocity.linear);
  acceleration = geometryToEigen(trajectory_point_msg.acceleration.linear);
  jerk = geometryToEigen(trajectory_point_msg.jerk.linear);
  snap = geometryToEigen(trajectory_point_msg.snap.linear);

  bodyrates = geometryToEigen(trajectory_point_msg.velocity.angular);
  angular_acceleration = geometryToEigen(
      trajectory_point_msg.acceleration.angular);
  angular_jerk = geometryToEigen(trajectory_point_msg.jerk.angular);
  angular_snap = geometryToEigen(trajectory_point_msg.snap.angular);

  heading = trajectory_point_msg.heading;
  heading_rate = trajectory_point_msg.heading_rate;
  heading_acceleration = trajectory_point_msg.heading_acceleration;
}

TrajectoryPoint::~TrajectoryPoint()
{
}

quadrotor_msgs::TrajectoryPoint TrajectoryPoint::toRosMessage()
{
  quadrotor_msgs::TrajectoryPoint ros_msg;

  ros_msg.time_from_start = time_from_start;

  ros_msg.pose.position = vectorToPoint(eigenToGeometry(position));
  ros_msg.pose.orientation = eigenToGeometry(orientation);

  ros_msg.velocity.linear = eigenToGeometry(velocity);
  ros_msg.acceleration.linear = eigenToGeometry(acceleration);
  ros_msg.jerk.linear = eigenToGeometry(jerk);
  ros_msg.snap.linear = eigenToGeometry(snap);

  ros_msg.velocity.angular = eigenToGeometry(bodyrates);
  ros_msg.acceleration.angular = eigenToGeometry(angular_acceleration);
  ros_msg.jerk.angular = eigenToGeometry(angular_jerk);
  ros_msg.snap.angular = eigenToGeometry(angular_snap);

  ros_msg.heading = heading;
  ros_msg.heading_rate = heading_rate;
  ros_msg.heading_acceleration = heading_acceleration;

  return ros_msg;
}

} // namespace quadrotor_common
