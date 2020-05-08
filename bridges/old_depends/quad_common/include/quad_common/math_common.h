#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>

namespace quad_common
{

double fcToAlpha(const double fc, const double dt);
float lowpass(const float filterin, const float input, const double alpha);
float lowpass(const float filterin, const float input, const double fc, const double dt);
Eigen::Vector3d lowpass(const Eigen::Vector3d& filterin, const Eigen::Vector3d& input, double alpha);
Eigen::Vector3d lowpass(const Eigen::Vector3d& filterin, const Eigen::Vector3d& input, double fc, double dt);
Eigen::Quaterniond lowpass(const Eigen::Quaterniond& filterin, const Eigen::Quaterniond& input, double alpha);
Eigen::Quaterniond lowpass(const Eigen::Quaterniond& filterin, const Eigen::Quaterniond& input, double fc, double dt);
Eigen::Affine3d lowpass(const Eigen::Affine3d& filterin, const Eigen::Affine3d& input, double alpha);
Eigen::Affine3d lowpass(const Eigen::Affine3d& filterin, const Eigen::Affine3d& input, double fc, double dt);

double wrapZeroToTwoPi(const double angle);
double wrapMinusPiToPi(const double angle);
double wrapAngleDifference(const double current_angle, const double desired_angle);

void limit(double &val, const double min, const double max);

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

double degToRad(const double deg);
double radToDeg(const double rad);

Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q);
Eigen::Quaterniond eulerAnglesZYXToQuaternion(const Eigen::Vector3d& euler_angles);
Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(const Eigen::Vector3d& euler_angles);
Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q);
Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R);

} // quad_common

