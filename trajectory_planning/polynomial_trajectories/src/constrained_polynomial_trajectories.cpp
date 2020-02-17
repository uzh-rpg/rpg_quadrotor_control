#include "polynomial_trajectories/constrained_polynomial_trajectories.h"

#include <ros/ros.h>

#include "polynomial_trajectories/polynomial_trajectories_common.h"

namespace polynomial_trajectories {
namespace constrained_polynomial_trajectories {

PolynomialTrajectory computeTimeOptimalTrajectory(
    const quadrotor_common::TrajectoryPoint& s0,
    const quadrotor_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate) {
  if (!isStartAndEndStateFeasibleUnderConstraints(
          s0, s1, max_velocity, max_normalized_thrust, max_roll_pitch_rate)) {
    ROS_ERROR(
        "[%s] Desired start or end state of trajectory is not feasible "
        "under given velocity and thrust constraints.",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  // Compute initial trajectory duration from a trapezoidal trajectory
  // considering max velocity and max thrust
  double distance = (s1.position - s0.position).norm();
  double n_z = ((s1.position - s0.position).normalized()).z();
  double acc_towards_s1_max = std::max(
      -n_z * 9.81 + sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                         max_normalized_thrust * max_normalized_thrust),
      -n_z * 9.81 - sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                         max_normalized_thrust * max_normalized_thrust));

  // value according to old implementation, will be overwritten
  double init_trajectory_duration = distance / max_velocity;
  if (max_velocity >= sqrt(acc_towards_s1_max * distance)) {
    init_trajectory_duration = 2 * sqrt(distance / acc_towards_s1_max);
  } else {
    init_trajectory_duration =
        max_velocity / acc_towards_s1_max + distance / max_velocity;
  }

  // Compute initial trajectory such that limits must be violated
  PolynomialTrajectory initial_trajectory;
  initial_trajectory.trajectory_type =
      polynomial_trajectories::TrajectoryType::FULLY_CONSTRAINED;
  initial_trajectory.start_state = s0;
  initial_trajectory.start_state.time_from_start = ros::Duration(0.0);
  initial_trajectory.end_state = s1;
  initial_trajectory.number_of_segments = 1;
  initial_trajectory.T = ros::Duration(init_trajectory_duration);
  initial_trajectory.end_state.time_from_start = initial_trajectory.T;
  initial_trajectory.coeff = implementation::computeTrajectoryCoeff(
      s0, s1, order_of_continuity, initial_trajectory.T.toSec());
  initial_trajectory.segment_times.resize(1);
  initial_trajectory.segment_times(0) = initial_trajectory.T.toSec();

  PolynomialTrajectory trajectory = initial_trajectory;
  Eigen::Vector3d desired_maxima =
      Eigen::Vector3d(max_velocity, max_normalized_thrust, max_roll_pitch_rate);
  Eigen::Vector3d prev_maxima;
  computeQuadRelevantMaxima(trajectory, &prev_maxima.x(), &prev_maxima.y(),
                            &prev_maxima.z());

  while (prev_maxima.x() <= 1.01 * desired_maxima.x() &&
         prev_maxima.y() <= 1.01 * desired_maxima.y() &&
         prev_maxima.z() <= 1.01 * desired_maxima.z()) {
    if ((prev_maxima.x() >= 0.99 * desired_maxima.x() ||
         prev_maxima.y() >= 0.99 * desired_maxima.y() ||
         prev_maxima.z() >= 0.99 * desired_maxima.z()) &&
        (prev_maxima.x() <= desired_maxima.x() &&
         prev_maxima.y() <= desired_maxima.y() &&
         prev_maxima.z() <= desired_maxima.z())) {
      return trajectory;
    }

    trajectory.T = ros::Duration(0.9 * trajectory.T.toSec());
    trajectory.end_state.time_from_start = trajectory.T;
    trajectory.segment_times(0) = trajectory.T.toSec();
    trajectory.coeff = implementation::computeTrajectoryCoeff(
        s0, s1, order_of_continuity, trajectory.T.toSec());

    computeQuadRelevantMaxima(trajectory, &prev_maxima.x(), &prev_maxima.y(),
                              &prev_maxima.z());
  }

  // compute gradient of maxima with respect to T
  Eigen::Vector3d maxima_gradient = implementation::computeMaximaGradient(
      trajectory, prev_maxima, order_of_continuity);

  // Iteratively optimize execution time
  const int max_iterations = 15;
  for (int i = 0; i < max_iterations; i++) {
    Eigen::Vector3d new_intersections =
        (desired_maxima - prev_maxima).cwiseQuotient(maxima_gradient) +
        trajectory.T.toSec() * Eigen::Vector3d::Ones();

    for (int i = 0; i < 3; i++) {
      if (fabs(maxima_gradient(i)) < 5e-2) {
        // make it negative since we will filter this out later
        new_intersections(i) = -1.0;
      }
    }

    Eigen::Vector3d prev_gradient = maxima_gradient;
    PolynomialTrajectory prev_trajectory = trajectory;
    bool exhaustivly_search_minimum = false;
    Eigen::Vector3d maxima;

    if (new_intersections.maxCoeff() > initial_trajectory.T.toSec()) {
      // recompute trajectory with new duration
      trajectory.T = ros::Duration(new_intersections.maxCoeff());
      trajectory.end_state.time_from_start = trajectory.T;
      trajectory.segment_times(0) = trajectory.T.toSec();
      trajectory.coeff = implementation::computeTrajectoryCoeff(
          s0, s1, order_of_continuity, trajectory.T.toSec());

      computeQuadRelevantMaxima(trajectory, &maxima.x(), &maxima.y(),
                                &maxima.z());

      if (maxima.x() <= 1.01 * desired_maxima.x() &&
          maxima.y() <= 1.01 * desired_maxima.y() &&
          maxima.z() <= 1.01 * desired_maxima.z()) {
        // we are close enough at the limits so we stop here
        break;
      }

      // compute gradient of maxima with respect to T
      maxima_gradient = implementation::computeMaximaGradient(
          trajectory, maxima, order_of_continuity);

      // check if gradient flipped sign
      if ((prev_gradient.cwiseProduct(maxima_gradient)).minCoeff() < 0.0) {
        trajectory = prev_trajectory;
        maxima_gradient = prev_gradient;
        exhaustivly_search_minimum = true;
      }
    } else {
      exhaustivly_search_minimum = true;
    }

    if (exhaustivly_search_minimum) {
      for (int j = i; j < max_iterations; j++) {
        trajectory.T = ros::Duration(1.1 * trajectory.T.toSec());
        trajectory.end_state.time_from_start = trajectory.T;
        trajectory.segment_times(0) = trajectory.T.toSec();
        trajectory.coeff = implementation::computeTrajectoryCoeff(
            s0, s1, order_of_continuity, trajectory.T.toSec());

        computeQuadRelevantMaxima(trajectory, &maxima.x(), &maxima.y(),
                                  &maxima.z());
        if ((maxima.x() > desired_maxima.x() &&
             maxima.x() > prev_maxima.x() + 1e-6) ||
            (maxima.y() > desired_maxima.y() &&
             maxima.y() > prev_maxima.y() + 1e-6) ||
            (maxima.z() > desired_maxima.z() &&
             maxima.z() > prev_maxima.z() + 1e-6)) {
          trajectory = prev_trajectory;
          break;
        }
        if (maxima.x() <= 1.01 * desired_maxima.x() &&
            maxima.y() <= 1.01 * desired_maxima.y() &&
            maxima.z() <= 1.01 * desired_maxima.z()) {
          // Reached desired maxima so we take this trajectory
          break;
        }
        prev_maxima = maxima;
        prev_trajectory = trajectory;
      }

      break;
    }
    prev_maxima = maxima;
  }

  return trajectory;
}

PolynomialTrajectory computeFixedTimeTrajectory(
    const quadrotor_common::TrajectoryPoint& s0,
    const quadrotor_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time) {
  PolynomialTrajectory trajectory;
  trajectory.trajectory_type =
      polynomial_trajectories::TrajectoryType::FULLY_CONSTRAINED;
  trajectory.start_state = s0;
  trajectory.start_state.time_from_start = ros::Duration(0.0);
  trajectory.end_state = s1;
  trajectory.number_of_segments = 1;
  trajectory.T = ros::Duration(execution_time);
  trajectory.end_state.time_from_start = trajectory.T;
  trajectory.coeff = implementation::computeTrajectoryCoeff(
      s0, s1, order_of_continuity, execution_time);

  trajectory.segment_times.resize(1);
  trajectory.segment_times(0) = trajectory.T.toSec();

  return trajectory;
}

Eigen::MatrixXd implementation::computeConstraintMatriceA(
    const int order_of_continuity, const double t) {
  int number_of_coefficients = 2 * order_of_continuity;

  Eigen::MatrixXd A =
      Eigen::MatrixXd::Zero(order_of_continuity, number_of_coefficients);

  for (int i = 0; i < order_of_continuity; i++) {
    A.row(i) = (dVec(number_of_coefficients, i).asDiagonal() *
                tVec(number_of_coefficients, i, t))
                   .transpose();
  }
  return A;
}

Eigen::VectorXd implementation::computeConstraintMatriceB(
    const int order_of_continuity,
    const quadrotor_common::TrajectoryPoint& state, const int axis) {
  Eigen::VectorXd b = Eigen::VectorXd::Zero(order_of_continuity);
  for (int i = 0; i < order_of_continuity; i++) {
    if (i == 0) b[i] = state.position(axis);
    if (i == 1) b[i] = state.velocity(axis);
    if (i == 2) b[i] = state.acceleration(axis);
    if (i == 3) b[i] = state.jerk(axis);
  }
  return b;
}

std::vector<Eigen::MatrixXd> implementation::computeTrajectoryCoeff(
    const quadrotor_common::TrajectoryPoint& s0,
    const quadrotor_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double T) {
  int number_of_coefficients = 2 * order_of_continuity;
  Eigen::MatrixXd p_coeff = Eigen::MatrixXd::Zero(3, number_of_coefficients);

  // solve the problem for each axis
  for (int axis = 0; axis < 3; axis++) {
    Eigen::MatrixXd A =
        Eigen::MatrixXd::Zero(number_of_coefficients, number_of_coefficients);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(number_of_coefficients);

    A.topLeftCorner(order_of_continuity, number_of_coefficients) =
        implementation::computeConstraintMatriceA(order_of_continuity, 0.0);
    A.bottomLeftCorner(order_of_continuity, number_of_coefficients) =
        implementation::computeConstraintMatriceA(order_of_continuity, T);

    b.head(order_of_continuity) = implementation::computeConstraintMatriceB(
        order_of_continuity, s0, axis);
    b.tail(order_of_continuity) = implementation::computeConstraintMatriceB(
        order_of_continuity, s1, axis);

    p_coeff.row(axis) = A.inverse() * b;
  }

  std::vector<Eigen::MatrixXd> coeff_vec;
  coeff_vec.push_back(p_coeff);

  return coeff_vec;
}

Eigen::Vector3d implementation::computeMaximaGradient(
    const PolynomialTrajectory& trajectory, const Eigen::Vector3d& maxima,
    const double order_of_continuity) {
  PolynomialTrajectory gradient_trajectory = trajectory;
  gradient_trajectory.T = ros::Duration(1.01 * trajectory.T.toSec());
  gradient_trajectory.segment_times(0) = gradient_trajectory.T.toSec();
  gradient_trajectory.coeff = implementation::computeTrajectoryCoeff(
      trajectory.start_state, trajectory.end_state, order_of_continuity,
      gradient_trajectory.T.toSec());
  Eigen::Vector3d gradient_trajectory_maxima;
  computeQuadRelevantMaxima(
      gradient_trajectory, &gradient_trajectory_maxima.x(),
      &gradient_trajectory_maxima.y(), &gradient_trajectory_maxima.z());

  Eigen::Vector3d gradient = (gradient_trajectory_maxima - maxima) /
                             (gradient_trajectory.T - trajectory.T).toSec();

  return gradient;
}

}  // namespace constrained_polynomial_trajectories

}  // namespace polynomial_trajectories
