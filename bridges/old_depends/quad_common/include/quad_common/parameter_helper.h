/*
 * parameter_helper.h
 *
 *  Created on: Aug 8, 2013
 *      Author: ffontana
 */

#pragma once

#include "ros/ros.h"
#include "Eigen/Dense"

namespace quad_common
{

template<typename T>
bool getParam(const std::string& name, T& parameter, const ros::NodeHandle & pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
  {
    ROS_INFO_STREAM("[" << pnh.getNamespace() << "] " << name << " = " << parameter);
    return true;
  }
  ROS_ERROR_STREAM("[" << pnh.getNamespace() << "] Could not load parameter " << pnh.getNamespace() << "/"<< name);
  return false;
}

template<typename T>
bool getParam(const std::string& name, T& parameter, const T& defaultValue, const ros::NodeHandle & pnh =
                  ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
    ROS_INFO_STREAM("[" << pnh.getNamespace() << "] " << name << " = " << parameter);
  else
  {
    parameter = defaultValue;
    ROS_WARN_STREAM("[" << pnh.getNamespace() << "] " << name << " = " << parameter << " set to default value");
  }
  return true;
}

bool getParamVector3d(const std::string& name, Eigen::Vector3d& vector,
                      const ros::NodeHandle & pnh = ros::NodeHandle("~"));
bool getParamQuaterniond(const std::string& name, Eigen::Quaterniond& q,
                         const ros::NodeHandle & pnh = ros::NodeHandle("~"));

} // quad_common
