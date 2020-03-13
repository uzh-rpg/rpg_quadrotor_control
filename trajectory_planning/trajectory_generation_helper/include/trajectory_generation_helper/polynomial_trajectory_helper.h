#pragma once

#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <Eigen/Dense>

namespace trajectory_generation_helper {

namespace polynomials {

// Constrained Polynomials
quadrotor_common::Trajectory computeTimeOptimalTrajectory(
    const quadrotor_common::TrajectoryPoint& s0,
    const quadrotor_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

quadrotor_common::Trajectory computeFixedTimeTrajectory(
    const quadrotor_common::TrajectoryPoint& s0,
    const quadrotor_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time, const double sampling_frequency);

// Minimum Snap Style Polynomials
quadrotor_common::Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
quadrotor_common::Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

quadrotor_common::Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
quadrotor_common::Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

quadrotor_common::Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
quadrotor_common::Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

quadrotor_common::Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
quadrotor_common::Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

// Sampling function
quadrotor_common::Trajectory samplePolynomial(
    const polynomial_trajectories::PolynomialTrajectory& polynomial,
    const double sampling_frequency);

}  // namespace polynomials

}  // namespace trajectory_generation_helper
