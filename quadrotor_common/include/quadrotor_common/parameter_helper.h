#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>

namespace quadrotor_common
{

template<typename T>
bool getParam(const std::string& name, T& parameter,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
  {
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
    return true;
  }
  ROS_ERROR_STREAM(
      "[" << pnh.getNamespace() << "] Could not load parameter "
      << pnh.getNamespace() << "/"<< name);
  return false;
}

template<typename T>
bool getParam(const std::string& name, T& parameter, const T& defaultValue,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
  else
  {
    parameter = defaultValue;
    ROS_WARN_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter
        << " set to default value");
  }
  return true;
}

bool getParamVector3d(const std::string& name, Eigen::Vector3d& vector,
                      const ros::NodeHandle& pnh = ros::NodeHandle("~"));
bool getParamQuaterniond(const std::string& name, Eigen::Quaterniond& q,
                         const ros::NodeHandle& pnh = ros::NodeHandle("~"));

} // quadrotor_common
