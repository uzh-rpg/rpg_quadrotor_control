#pragma once

#include <vector>

#include <Eigen/Dense>
#include <quadrotor_common/trajectory_point.h>

#include "polynomial_trajectories/polynomial_trajectory.h"
#include "polynomial_trajectories/polynomial_trajectory_settings.h"

namespace polynomial_trajectories
{

namespace minimum_snap_trajectories
{

PolynomialTrajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PloynomialTrajectorySettings& trajectory_settings);
PolynomialTrajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PloynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

PolynomialTrajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PloynomialTrajectorySettings& trajectory_settings);
PolynomialTrajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PloynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

PolynomialTrajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const PloynomialTrajectorySettings& trajectory_settings);
PolynomialTrajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const PloynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

PolynomialTrajectory generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const PloynomialTrajectorySettings& trajectory_settings);
PolynomialTrajectory generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const PloynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

// these functions should not be used from the outside
namespace implementation
{
Eigen::MatrixXd generate1DTrajectory(const int num_polynoms,
                                     const int polynomial_order,
                                     const Eigen::MatrixXd& H,
                                     const Eigen::VectorXd& f,
                                     const Eigen::MatrixXd& A,
                                     const Eigen::VectorXd& b,
                                     double* optimization_cost);

Eigen::MatrixXd generateHMatrix(
    const PloynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot);
Eigen::VectorXd generateFVector(
    const PloynomialTrajectorySettings& trajectory_settings,
    const Eigen::VectorXd& way_points_1D, const int num_polynoms);
Eigen::MatrixXd generateEqualityConstraintsAMatrix(
    const PloynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot);
Eigen::VectorXd generateEqualityConstraintsBVector(
    const PloynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& way_points_1D,
    const Eigen::Vector3d& start_conditions,
    const Eigen::Vector3d& end_conditions);

Eigen::MatrixXd generateRingEqualityConstraintsAMatrix(
    const PloynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot);
Eigen::VectorXd generateRingEqualityConstraintsBVector(
    const PloynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& way_points_1D);

Eigen::VectorXd computeCostGradient(
    const PolynomialTrajectory& initial_trajectory,
    const PloynomialTrajectorySettings& trajectory_settings);
Eigen::VectorXd computeSearchDirection(
    const PolynomialTrajectory& initial_trajectory,
    const Eigen::VectorXd& gradient);
Eigen::VectorXd updateSegmentTimes(
    const PolynomialTrajectory& initial_trajectory,
    const Eigen::VectorXd& gradient,
    const PloynomialTrajectorySettings& trajectory_settings);

PolynomialTrajectory enforceMaximumVelocityAndThrust(
    const PolynomialTrajectory& initial_trajectory,
    const PloynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

bool computeMaximaGradient(
    const PolynomialTrajectory& trajectory, const Eigen::Vector3d& maxima,
    const PloynomialTrajectorySettings& trajectory_settings,
    Eigen::Vector3d* gradient);

std::vector<Eigen::MatrixXd> reorganiceCoefficientsSegmentWise(
    const std::vector<Eigen::MatrixXd>& coefficients, const int num_segments,
    const int polynomial_order);
PloynomialTrajectorySettings ensureFeasibleTrajectorySettings(
    const PloynomialTrajectorySettings& original_trajectory_settings,
    const int min_poly_order);
std::vector<Eigen::Vector3d> addStartAndEndToWayPointList(
    const std::vector<Eigen::Vector3d>& intermediate_way_points,
    const Eigen::Vector3d& start_position, const Eigen::Vector3d& end_position);

Eigen::VectorXd solveQuadraticProgram(const Eigen::MatrixXd& H,
                                      const Eigen::VectorXd& f,
                                      const Eigen::MatrixXd& A_eq,
                                      const Eigen::VectorXd& b_eq,
                                      double* objective_value);
} // namespace implementation

} // namespace minimum_snap_trajectories

} //namespace polynomial_trajectories
