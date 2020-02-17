#pragma once

#include <polynomial_trajectories/polynomial_trajectory.h>

#include "Eigen/Dense"

namespace polynomial_trajectories {

struct PolynomialTrajectorySettings {
  PolynomialTrajectorySettings() = default;

  PolynomialTrajectorySettings(const std::vector<Eigen::Vector3d>& way_points,
                               const Eigen::VectorXd& minimization_weights,
                               const int polynomial_order,
                               const int continuity_order)
      : way_points(way_points),
        minimization_weights(minimization_weights),
        polynomial_order(polynomial_order),
        continuity_order(continuity_order) {}

  virtual ~PolynomialTrajectorySettings(){};

  std::vector<Eigen::Vector3d> way_points;
  Eigen::VectorXd minimization_weights;
  int polynomial_order = 0;
  int continuity_order = 0;
};

}  // namespace polynomial_trajectories
