#pragma once

#include <vector>

#include <quadrotor_common/trajectory_point.h>
#include <ros/duration.h>
#include <Eigen/Dense>

namespace polynomial_trajectories {

enum class TrajectoryType {
  UNDEFINED,
  FULLY_CONSTRAINED,
  MINIMUM_SNAP,
  MINIMUM_SNAP_RING,
  MINIMUM_SNAP_OPTIMIZED_SEGMENTS,
  MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS
};

struct PolynomialTrajectory {
  PolynomialTrajectory();
  virtual ~PolynomialTrajectory();

  TrajectoryType trajectory_type;
  // Polynomial coefficients
  // Each element of this vector contains the coefficients for one polynomial
  // segment (rows: dimension, columns: order)
  std::vector<Eigen::MatrixXd> coeff;
  ros::Duration T;
  quadrotor_common::TrajectoryPoint start_state;
  quadrotor_common::TrajectoryPoint end_state;
  int number_of_segments;
  Eigen::VectorXd segment_times;
  double optimization_cost;
};

}  // namespace polynomial_trajectories
