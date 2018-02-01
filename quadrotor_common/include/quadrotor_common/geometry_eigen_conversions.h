#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace quadrotor_common
{

// Quaternions
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& vec_ros);
geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& vec_eigen);

//Vectors and Points
Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3& vec_ros);
Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& vec_ros);
geometry_msgs::Vector3 eigenToGeometry(const Eigen::Vector3d& vec_eigen);
geometry_msgs::Point vectorToPoint(const geometry_msgs::Vector3& vector);

} // quadrotor_common
