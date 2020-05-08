/*
 * quaternion_functions.h
 *
 *  Created on: Dec 19, 2013
 *      Author: ffontana
 */

#pragma once

#include <Eigen/Dense>

namespace quad_common
{

const int BODY_FRAME = 0;
const int WORLD_FRAME = 1;

Eigen::Vector3d quaternionRatesToBodyRates(Eigen::Quaterniond q2, Eigen::Quaterniond q1, double dt, int coord_frame);
Eigen::Vector3d quaternionDeltaToBodyRates(Eigen::Quaterniond q2, Eigen::Quaterniond q1, double dt, int coord_frame);
Eigen::Quaterniond integrateQuaternion(Eigen::Quaterniond q_start, Eigen::Vector3d bodyrates, double dt);
double sinc(double a);

} //quad_common
