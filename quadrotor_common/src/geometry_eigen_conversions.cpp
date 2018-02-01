#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

// Quaternions
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& vec_ros)
{
  Eigen::Quaterniond vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  vec_eigen.w() = vec_ros.w;
  return vec_eigen;
}

geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& vec_eigen)
{
  geometry_msgs::Quaternion vec_ros;
  vec_ros.x = vec_eigen.x();
  vec_ros.y = vec_eigen.y();
  vec_ros.z = vec_eigen.z();
  vec_ros.w = vec_eigen.w();
  return vec_ros;
}

//Vectors and Points
Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3& vec_ros)
{
  Eigen::Vector3d vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  return vec_eigen;
}

Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& vec_ros)
{
  Eigen::Vector3d vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  return vec_eigen;
}

geometry_msgs::Vector3 eigenToGeometry(const Eigen::Vector3d& vec_eigen)
{
  geometry_msgs::Vector3 vec_ros;
  vec_ros.x = vec_eigen.x();
  vec_ros.y = vec_eigen.y();
  vec_ros.z = vec_eigen.z();
  return vec_ros;
}

geometry_msgs::Point vectorToPoint(const geometry_msgs::Vector3& vector)
{
  geometry_msgs::Point point;
  point.x = vector.x;
  point.y = vector.y;
  point.z = vector.z;
  return point;
}

} // quadrotor_common
