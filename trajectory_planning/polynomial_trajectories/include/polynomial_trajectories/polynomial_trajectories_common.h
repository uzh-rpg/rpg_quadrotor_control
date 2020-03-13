#pragma once

#include <quadrotor_common/trajectory_point.h>
#include <ros/duration.h>
#include <Eigen/Dense>

#include "polynomial_trajectories/polynomial_trajectory.h"

namespace polynomial_trajectories {

quadrotor_common::TrajectoryPoint getPointFromTrajectory(
    const PolynomialTrajectory& trajectory,
    const ros::Duration& time_from_start);
Eigen::VectorXd computeFactorials(const int length, const int order);
Eigen::VectorXd dVec(const int number_of_coefficients,
                     const int derivative_order);
Eigen::VectorXd tVec(const int number_of_coefficients,
                     const int derivative_order, const double t);
void computeMaxima(const PolynomialTrajectory& trajectory,
                   double* maximal_velocity, double* maximal_acceleration,
                   double* maximal_jerk, double* maximal_snap);
void computeQuadRelevantMaxima(const PolynomialTrajectory& trajectory,
                               double* maximal_velocity,
                               double* maximal_normalized_thrust,
                               double* maximal_roll_pitch_rate);
bool isStartAndEndStateFeasibleUnderConstraints(
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);
double computeRollPitchRateNormFromTrajectoryPoint(
    const quadrotor_common::TrajectoryPoint& desired_state);

}  // namespace polynomial_trajectories
