#pragma once

#include <Eigen/Dense>

namespace quadrotor_common
{

Eigen::Vector3d quaternionRatesToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
Eigen::Vector3d quaternionDeltaToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q_start,
                                       const Eigen::Vector3d& bodyrates,
                                       const double dt);
double sinc(const double a);

} //quadrotor_common
