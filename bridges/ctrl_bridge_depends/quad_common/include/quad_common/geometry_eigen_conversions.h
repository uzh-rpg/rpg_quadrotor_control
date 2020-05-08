/*
 * geometry_eigen_conversions.h
 *
 *  Created on: Aug 6, 2013
 *      Author: ffontana
 */

#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

namespace quad_common
{

// Quaternions
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion &vec_ros);
geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond &vec_eigen);

//Vectors and Points
Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3 &vec_ros);
Eigen::Vector3d geometryToEigen(const geometry_msgs::Point &vec_ros);
geometry_msgs::Vector3 eigenToGeometry(const Eigen::Vector3d &vec_eigen);
geometry_msgs::Point vectorToPoint(const geometry_msgs::Vector3 & vector);

} // quad_common
