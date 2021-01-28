#include "polynomial_trajectories/minimum_snap_trajectories.h"

#include <ros/ros.h>

#include "polynomial_trajectories/polynomial_trajectories_common.h"

namespace polynomial_trajectories {

namespace minimum_snap_trajectories {

PolynomialTrajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PolynomialTrajectorySettings& trajectory_settings) {
  const int num_segments = segment_times.size();

  if (num_segments != trajectory_settings.way_points.size() + 1) {
    ROS_ERROR(
        "[%s] Number of way points and segments are not agreeing. "
        "(Need num_segments == num_waypoints + 1 for open trajectories.)",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  PolynomialTrajectory minimum_snap_trajectory;
  minimum_snap_trajectory.trajectory_type =
      polynomial_trajectories::TrajectoryType::MINIMUM_SNAP;
  minimum_snap_trajectory.number_of_segments = num_segments;
  minimum_snap_trajectory.segment_times = segment_times;

  minimum_snap_trajectory.start_state = start_state;
  minimum_snap_trajectory.start_state.time_from_start = ros::Duration(0.0);
  minimum_snap_trajectory.end_state = end_state;

  minimum_snap_trajectory.optimization_cost = 0.0;  // will be reset later on
  minimum_snap_trajectory.T = ros::Duration(segment_times.sum());
  minimum_snap_trajectory.end_state.time_from_start = minimum_snap_trajectory.T;

  // Ensure trajectory settings that result in feasible optimization problem
  const int min_poly_order = 2 +
                             ceil(trajectory_settings.continuity_order *
                                  (num_segments + 1) / float(num_segments)) -
                             1;
  PolynomialTrajectorySettings new_trajectory_settings =
      implementation::ensureFeasibleTrajectorySettings(trajectory_settings,
                                                       min_poly_order);

  // Add start and end position to way points vector
  new_trajectory_settings.way_points =
      implementation::addStartAndEndToWayPointList(
          trajectory_settings.way_points, start_state.position,
          end_state.position);

  // Compute tau dot
  // Definition: tau(i) = (time - wp_times_zero_start_(i)) /
  //     (wp_times_zero_start_(i+1) - wp_times_zero_start_(i))
  // Therefore tau_dot(i) = 1.0 / (wp_times_zero_start_(i+1) -
  //     wp_times_zero_start_(i))
  Eigen::VectorXd tau_dot(num_segments);
  for (int i = 0; i < num_segments; i++) {
    tau_dot(i) = 1.0 / segment_times(i);
  }

  // Compute common matrices used for optimization later on
  Eigen::MatrixXd H = implementation::generateHMatrix(new_trajectory_settings,
                                                      num_segments, tau_dot);
  Eigen::MatrixXd A_eq = implementation::generateEqualityConstraintsAMatrix(
      new_trajectory_settings, num_segments, tau_dot);

  std::vector<Eigen::MatrixXd> coefficients;
  // Compute trajectory for each spatial dimension
  for (int d = 0; d < 3; d++) {
    Eigen::VectorXd way_points_d = Eigen::VectorXd::Zero(num_segments + 1);
    for (int i = 0; i < num_segments + 1; i++) {
      Eigen::VectorXd way_point_i = new_trajectory_settings.way_points[i];
      way_points_d(i) = way_point_i(d);
    }

    Eigen::Vector3d start_conditions(start_state.velocity(d),
                                     start_state.acceleration(d),
                                     start_state.jerk(d));
    Eigen::Vector3d end_conditions(
        end_state.velocity(d), end_state.acceleration(d), end_state.jerk(d));

    Eigen::MatrixXd coefficients_for_this_dimension;
    Eigen::VectorXd f = implementation::generateFVector(
        new_trajectory_settings, way_points_d, num_segments);
    Eigen::VectorXd b_eq = implementation::generateEqualityConstraintsBVector(
        new_trajectory_settings, num_segments, way_points_d, start_conditions,
        end_conditions);

    double cost_dimension;
    coefficients_for_this_dimension = implementation::generate1DTrajectory(
        num_segments, new_trajectory_settings.polynomial_order, H, f, A_eq,
        b_eq, &cost_dimension);
    if (cost_dimension > 1e20 || std::isnan(cost_dimension)) {
      ROS_ERROR("[%s] Could not solve quadratic program.",
                ros::this_node::getName().c_str());
      minimum_snap_trajectory.trajectory_type =
          polynomial_trajectories::TrajectoryType::UNDEFINED;
      return minimum_snap_trajectory;
    }

    minimum_snap_trajectory.optimization_cost += cost_dimension;
    coefficients.push_back(coefficients_for_this_dimension);
  }

  minimum_snap_trajectory.coeff =
      implementation::reorganiceCoefficientsSegmentWise(
          coefficients, num_segments, new_trajectory_settings.polynomial_order);

  return minimum_snap_trajectory;
}

PolynomialTrajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PolynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate)

{
  if (!isStartAndEndStateFeasibleUnderConstraints(
          start_state, end_state, max_velocity, max_normalized_thrust,
          max_roll_pitch_rate)) {
    ROS_ERROR(
        "[%s] Desired start or end state of trajectory is not feasible "
        "under given velocity and thrust constraints.",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  // Compute straight line distance of trajectory
  double straight_line_distance = 0.0;
  if (trajectory_settings.way_points.size() == 0) {
    straight_line_distance = (end_state.position - start_state.position).norm();
  } else {
    straight_line_distance +=
        (trajectory_settings.way_points.front() - start_state.position).norm();
    straight_line_distance +=
        (end_state.position - trajectory_settings.way_points.back()).norm();
    for (int i = 0; i < trajectory_settings.way_points.size() - 1; i++) {
      straight_line_distance += (trajectory_settings.way_points[i + 1] -
                                 trajectory_settings.way_points[i])
                                    .norm();
    }
  }

  // Compute initial trajectory duration from a trapezoidal trajectory
  // considering max velocity and max thrust
  double init_trajectory_duration = straight_line_distance / max_velocity;
  if ((end_state.position - start_state.position).norm() >= 0.01) {
    double n_z = ((end_state.position - start_state.position).normalized()).z();
    double acc_towards_s1_max = std::max(
        -n_z * 9.81 + sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                           max_normalized_thrust * max_normalized_thrust),
        -n_z * 9.81 - sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                           max_normalized_thrust * max_normalized_thrust));

    if (max_velocity >= sqrt(acc_towards_s1_max * straight_line_distance)) {
      init_trajectory_duration =
          2 * sqrt(straight_line_distance / acc_towards_s1_max);
    } else {
      init_trajectory_duration = max_velocity / acc_towards_s1_max +
                                 straight_line_distance / max_velocity;
    }
  }

  const Eigen::VectorXd& bounds_violating_segment_times =
      initial_segment_times / initial_segment_times.sum() *
      init_trajectory_duration;

  // Compute initial trajectory such that bounds on velocity and thrust must
  // be violated
  PolynomialTrajectory initial_trajectory =
      generateMinimumSnapTrajectory(bounds_violating_segment_times, start_state,
                                    end_state, trajectory_settings);

  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory =
      implementation::enforceMaximumVelocityAndThrust(
          initial_trajectory, trajectory_settings, max_velocity,
          max_normalized_thrust, max_roll_pitch_rate);

  return trajectory;
}

PolynomialTrajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PolynomialTrajectorySettings& trajectory_settings) {
  // Compute trajectory with initial values
  PolynomialTrajectory initial_trajectory = generateMinimumSnapTrajectory(
      initial_segment_times, start_state, end_state, trajectory_settings);
  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }
  initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_OPTIMIZED_SEGMENTS;

  if (trajectory_settings.way_points.empty()) {
    // There is only one segment so no refinement is required
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory = initial_trajectory;

  std::vector<double> costs;
  costs.push_back(initial_trajectory.optimization_cost);

  const int max_refinement_iterations = 20;
  for (int i = 0; i < max_refinement_iterations; i++) {
    Eigen::VectorXd gradient =
        implementation::computeCostGradient(trajectory, trajectory_settings);

    Eigen::VectorXd segment_times = implementation::updateSegmentTimes(
        trajectory, gradient, trajectory_settings);

    trajectory = generateMinimumSnapTrajectory(segment_times, start_state,
                                               end_state, trajectory_settings);

    if (fabs(costs.back() - trajectory.optimization_cost) < 1e-2) {
      costs.push_back(trajectory.optimization_cost);
      break;
    }
    costs.push_back(trajectory.optimization_cost);
  }

  trajectory.trajectory_type = initial_trajectory.trajectory_type;

  return trajectory;
}

PolynomialTrajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const PolynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate)

{
  if (!isStartAndEndStateFeasibleUnderConstraints(
          start_state, end_state, max_velocity, max_normalized_thrust,
          max_roll_pitch_rate)) {
    ROS_ERROR(
        "[%s] Desired start or end state of trajectory is not feasible "
        "under given velocity and thrust constraints.",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  // Compute straight line distance of trajectory
  double straight_line_distance = 0.0;
  if (trajectory_settings.way_points.size() == 0) {
    straight_line_distance = (end_state.position - start_state.position).norm();
  } else {
    straight_line_distance +=
        (trajectory_settings.way_points.front() - start_state.position).norm();
    straight_line_distance +=
        (end_state.position - trajectory_settings.way_points.back()).norm();
    for (int i = 0; i < trajectory_settings.way_points.size() - 1; i++) {
      straight_line_distance += (trajectory_settings.way_points[i + 1] -
                                 trajectory_settings.way_points[i])
                                    .norm();
    }
  }

  // Compute initial trajectory duration from a trapezoidal trajectory
  // considering max velocity and max thrust
  double init_trajectory_duration = straight_line_distance / max_velocity;
  if ((end_state.position - start_state.position).norm() >= 0.01) {
    double n_z = ((end_state.position - start_state.position).normalized()).z();
    double acc_towards_s1_max = std::max(
        -n_z * 9.81 + sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                           max_normalized_thrust * max_normalized_thrust),
        -n_z * 9.81 - sqrt(9.81 * 9.81 * (n_z * n_z - 1.0) +
                           max_normalized_thrust * max_normalized_thrust));

    if (max_velocity >= sqrt(acc_towards_s1_max * straight_line_distance)) {
      init_trajectory_duration =
          2 * sqrt(straight_line_distance / acc_towards_s1_max);
    } else {
      init_trajectory_duration = max_velocity / acc_towards_s1_max +
                                 straight_line_distance / max_velocity;
    }
  }

  const Eigen::VectorXd& bounds_violating_segment_times =
      initial_segment_times / initial_segment_times.sum() *
      init_trajectory_duration;

  // Compute initial trajectory such that bounds on velocity and thrust must
  // be violated
  PolynomialTrajectory initial_trajectory =
      generateMinimumSnapTrajectoryWithSegmentRefinement(
          bounds_violating_segment_times, start_state, end_state,
          trajectory_settings);

  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory =
      implementation::enforceMaximumVelocityAndThrust(
          initial_trajectory, trajectory_settings, max_velocity,
          max_normalized_thrust, max_roll_pitch_rate);

  return trajectory;
}

PolynomialTrajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const PolynomialTrajectorySettings& trajectory_settings) {
  if (trajectory_settings.way_points.size() <= 2) {
    ROS_ERROR(
        "[%s] To create a ring trajectory, at least 2 way points must be "
        "specified!",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  if (int(segment_times.size()) != int(trajectory_settings.way_points.size())) {
    ROS_ERROR(
        "[%s] Number of way points and segments are not agreeing. "
        "(Need num_segments == num_waypoints for ring trajectories.)",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  if (trajectory_settings.way_points.size() <= 2) {
    ROS_ERROR(
        "[%s] To create a ring trajectory, at least 2 DISTINCT way points must "
        "be specified!",
        ros::this_node::getName().c_str());
    return PolynomialTrajectory();
  }

  // Some values from function arguments
  const int num_waypoints = trajectory_settings.way_points.size();
  const int num_segments = segment_times.size();

  PolynomialTrajectory minimum_snap_trajectory;
  minimum_snap_trajectory.trajectory_type =
      polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_RING;
  minimum_snap_trajectory.number_of_segments = num_segments;
  minimum_snap_trajectory.segment_times = segment_times;

  minimum_snap_trajectory.optimization_cost = 0.0;  // will be reset later on
  minimum_snap_trajectory.T = ros::Duration(segment_times.sum());

  // Ensure trajectory settings that result in feasible optimization problem
  PolynomialTrajectorySettings new_trajectory_settings = trajectory_settings;
  const int min_poly_order = trajectory_settings.continuity_order + 1;
  new_trajectory_settings = implementation::ensureFeasibleTrajectorySettings(
      trajectory_settings, min_poly_order);

  // Compute tau dot
  // Definition: tau(i) = (time - wp_times_zero_start_(i)) /
  //     (wp_times_zero_start_(i+1) - wp_times_zero_start_(i))
  // Therefore tau_dot(i) = 1.0 / (wp_times_zero_start_(i+1) -
  //     wp_times_zero_start_(i))
  Eigen::VectorXd tau_dot(num_segments);
  for (int i = 0; i < num_segments; i++) {
    tau_dot(i) = 1.0 / segment_times(i);
  }

  // Compute common matrices used for optimization later on
  Eigen::MatrixXd H = implementation::generateHMatrix(new_trajectory_settings,
                                                      num_segments, tau_dot);
  Eigen::MatrixXd A_eq = implementation::generateRingEqualityConstraintsAMatrix(
      new_trajectory_settings, num_segments, tau_dot);

  std::vector<Eigen::MatrixXd> coefficients;
  // Compute trajectory for each spatial dimension
  for (int d = 0; d < 3; d++) {
    Eigen::VectorXd way_points_d = Eigen::VectorXd::Zero(num_waypoints);
    for (int i = 0; i < num_waypoints; i++) {
      Eigen::VectorXd way_point_i = trajectory_settings.way_points[i];
      way_points_d(i) = way_point_i(d);
    }

    Eigen::MatrixXd coefficients_for_this_dimension;
    Eigen::VectorXd f = implementation::generateFVector(
        new_trajectory_settings, way_points_d, num_segments);
    Eigen::VectorXd b_eq =
        implementation::generateRingEqualityConstraintsBVector(
            new_trajectory_settings, num_segments, way_points_d);

    double cost_dimension;
    coefficients_for_this_dimension = implementation::generate1DTrajectory(
        num_segments, new_trajectory_settings.polynomial_order, H, f, A_eq,
        b_eq, &cost_dimension);
    if (cost_dimension > 1e20 || std::isnan(cost_dimension)) {
      ROS_ERROR("[%s] Could not solve quadratic program.",
                ros::this_node::getName().c_str());
      minimum_snap_trajectory.trajectory_type =
          polynomial_trajectories::TrajectoryType::UNDEFINED;
      return minimum_snap_trajectory;
    }

    minimum_snap_trajectory.optimization_cost += cost_dimension;
    coefficients.push_back(coefficients_for_this_dimension);
  }

  minimum_snap_trajectory.coeff =
      implementation::reorganiceCoefficientsSegmentWise(
          coefficients, num_segments, new_trajectory_settings.polynomial_order);

  // Set start and end state after computing trajectory
  quadrotor_common::TrajectoryPoint quad_state;
  quad_state =
      getPointFromTrajectory(minimum_snap_trajectory, ros::Duration(0.0));
  minimum_snap_trajectory.start_state = quad_state;
  quad_state = getPointFromTrajectory(minimum_snap_trajectory,
                                      minimum_snap_trajectory.T);
  minimum_snap_trajectory.end_state = quad_state;

  return minimum_snap_trajectory;
}

PolynomialTrajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const PolynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate) {
  // Compute initial trajectory such that bounds on velocity and thrust must
  // be violated
  double straight_line_distance = 0.0;
  for (int i = 0; i < trajectory_settings.way_points.size() - 1; i++) {
    straight_line_distance += (trajectory_settings.way_points[i + 1] -
                               trajectory_settings.way_points[i])
                                  .norm();
  }
  straight_line_distance += (trajectory_settings.way_points.front() -
                             trajectory_settings.way_points.back())
                                .norm();

  const Eigen::VectorXd& bounds_violating_segment_times =
      initial_segment_times / initial_segment_times.sum() *
      straight_line_distance / max_velocity;

  PolynomialTrajectory initial_trajectory = generateMinimumSnapRingTrajectory(
      bounds_violating_segment_times, trajectory_settings);

  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory =
      implementation::enforceMaximumVelocityAndThrust(
          initial_trajectory, trajectory_settings, max_velocity,
          max_normalized_thrust, max_roll_pitch_rate);

  return trajectory;
}

PolynomialTrajectory generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const PolynomialTrajectorySettings& trajectory_settings) {
  // Compute trajectory with initial values
  PolynomialTrajectory initial_trajectory = generateMinimumSnapRingTrajectory(
      initial_segment_times, trajectory_settings);

  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }
  initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::
          MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS;

  if (trajectory_settings.way_points.empty()) {
    // There is only one segment so no refinement is required
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory = initial_trajectory;

  std::vector<double> costs;
  costs.push_back(initial_trajectory.optimization_cost);

  const int max_refinement_iterations = 20;
  for (int i = 0; i < max_refinement_iterations; i++) {
    Eigen::VectorXd gradient =
        implementation::computeCostGradient(trajectory, trajectory_settings);

    Eigen::VectorXd segment_times = implementation::updateSegmentTimes(
        trajectory, gradient, trajectory_settings);

    trajectory =
        generateMinimumSnapRingTrajectory(segment_times, trajectory_settings);

    if (fabs(costs.back() - trajectory.optimization_cost) < 1e-2) {
      costs.push_back(trajectory.optimization_cost);
      break;
    }
    costs.push_back(trajectory.optimization_cost);
  }

  trajectory.trajectory_type = initial_trajectory.trajectory_type;

  return trajectory;
}

PolynomialTrajectory generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const PolynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate) {
  // Compute initial trajectory such that bounds on velocity and thrust must
  // be violated
  double straight_line_distance = 0.0;
  for (int i = 0; i < trajectory_settings.way_points.size() - 1; i++) {
    straight_line_distance += (trajectory_settings.way_points[i + 1] -
                               trajectory_settings.way_points[i])
                                  .norm();
  }
  straight_line_distance += (trajectory_settings.way_points.front() -
                             trajectory_settings.way_points.back())
                                .norm();

  const Eigen::VectorXd& bounds_violating_segment_times =
      initial_segment_times / initial_segment_times.sum() *
      straight_line_distance / max_velocity;

  PolynomialTrajectory initial_trajectory =
      generateMinimumSnapRingTrajectoryWithSegmentRefinement(
          bounds_violating_segment_times, trajectory_settings);

  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return initial_trajectory;
  }

  PolynomialTrajectory trajectory =
      implementation::enforceMaximumVelocityAndThrust(
          initial_trajectory, trajectory_settings, max_velocity,
          max_normalized_thrust, max_roll_pitch_rate);

  return trajectory;
}

namespace implementation {

Eigen::MatrixXd generate1DTrajectory(const int num_polynoms,
                                     const int polynomial_order,
                                     const Eigen::MatrixXd& H,
                                     const Eigen::VectorXd& f,
                                     const Eigen::MatrixXd& A,
                                     const Eigen::VectorXd& b,
                                     double* optimization_cost) {
  Eigen::VectorXd solution =
      implementation::solveQuadraticProgram(H, f, A, b, optimization_cost);

  Eigen::MatrixXd coefficients;
  coefficients = Eigen::Map<Eigen::MatrixXd>(
      solution.data(), polynomial_order + 1, num_polynoms);
  coefficients.transposeInPlace();

  return coefficients;
}

Eigen::MatrixXd generateHMatrix(
    const PolynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot) {
  // up to which order derivatives should be minimized
  const int k_r = trajectory_settings.minimization_weights.size() - 1;
  const int poly_order = trajectory_settings.polynomial_order;

  // Some factorials we are going to use multiple times
  Eigen::VectorXd factorials = Eigen::VectorXd::Ones(poly_order + 1);
  for (int i = 2; i < poly_order + 1; i++) {
    factorials(i) = i * factorials(i - 1);
  }

  // Initialize zero H matrix
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero((poly_order + 1) * num_polynoms,
                                            (poly_order + 1) * num_polynoms);

  // Create the H matrix
  for (int hh = 0; hh < std::min(poly_order, k_r + 1); hh++) {
    if (trajectory_settings.minimization_weights(hh) != 0.0) {
      const int num_terms = poly_order - hh + 1;

      // Create a basis matrix which is used for computations later on
      Eigen::MatrixXd H_basis = Eigen::MatrixXd::Zero(num_terms, num_terms);
      for (int i = 0; i < num_terms; i++) {
        for (int j = 0; j < num_terms; j++) {
          double numerator =
              factorials(poly_order - i) / factorials(poly_order - i - hh) *
              factorials(poly_order - j) / factorials(poly_order - j - hh);
          double denominator = 2.0 * (poly_order - hh) + 1 - i - j;
          H_basis(i, j) = numerator / denominator;
        }
      }

      // Create another basis matrix
      Eigen::MatrixXd H_times = Eigen::MatrixXd::Zero(num_terms, num_terms);
      for (int i = 0; i < num_terms; i++) {
        for (int j = 0; j < num_terms; j++) {
          H_times(i, j) = 1.0;
        }
      }

      Eigen::MatrixXd H_hh = Eigen::MatrixXd::Zero(
          (poly_order + 1) * num_polynoms, (poly_order + 1) * num_polynoms);
      for (int k = 0; k < num_polynoms; k++) {
        H_hh.block(k * (poly_order + 1), k * (poly_order + 1), num_terms,
                   num_terms) =
            H_times.cwiseProduct(H_basis) * pow(tau_dot(k), 2.0 * hh);
      }

      Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(
          (poly_order + 1) * num_polynoms, (poly_order + 1) * num_polynoms);
      for (int i = 0; i < num_polynoms; i++) {
        weight_matrix.block(i * (poly_order + 1), i * (poly_order + 1),
                            poly_order + 1, poly_order + 1) =
            trajectory_settings.minimization_weights(hh) *
            Eigen::MatrixXd::Ones(poly_order + 1, poly_order + 1);
      }

      H += H_hh.cwiseProduct(weight_matrix);
    }
  }

  return H;
}

Eigen::VectorXd generateFVector(
    const PolynomialTrajectorySettings& trajectory_settings,
    const Eigen::VectorXd& way_points_1D, const int num_polynoms) {
  const int poly_order = trajectory_settings.polynomial_order;

  Eigen::VectorXd f = Eigen::VectorXd::Zero((poly_order + 1) * num_polynoms);

  if (trajectory_settings.minimization_weights(0) == 0.0) {
    f.setZero();
    return f;
  }

  for (int k = 0; k < num_polynoms; k++) {
    // Create a basis vector which is used for computations later on
    Eigen::VectorXd f_k = Eigen::VectorXd::Zero(poly_order + 1);
    for (int i = 0; i < poly_order + 1; i++) {
      f_k(i) = -2.0 * ((way_points_1D(k + 1) - way_points_1D(k)) /
                           (poly_order + 2 - i) +
                       way_points_1D(k) / (poly_order + 1 - i));
    }
    f.segment(k * (poly_order + 1), poly_order + 1) = f_k;
  }
  f *= trajectory_settings.minimization_weights(0);

  return f;
}

Eigen::MatrixXd generateEqualityConstraintsAMatrix(
    const PolynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot) {
  const int poly_order = trajectory_settings.polynomial_order;
  const int continuity_order = trajectory_settings.continuity_order;
  const int num_constraints =
      2 * num_polynoms + continuity_order * (num_polynoms + 1);

  Eigen::MatrixXd A =
      Eigen::MatrixXd::Zero(num_constraints, (poly_order + 1) * num_polynoms);

  //
  // Create position constraints at waypoints
  //
  for (int i = 0; i < num_polynoms; i++) {
    A(2 * i, (poly_order + 1) * i + poly_order) = 1.0;
    for (int j = 0; j < poly_order + 1; j++) {
      A(2 * i + 1, (poly_order + 1) * i + j) = 1.0;
    }
  }

  //
  // Create constraints for continuity of the derivatives of position
  //
  for (int k = 0; k < continuity_order; k++) {
    Eigen::VectorXd factors = computeFactorials(poly_order - k, k);
    for (int j = 0; j < num_polynoms - 1; j++) {
      for (int i = 0; i < poly_order - k; i++) {
        A(2 * num_polynoms + k * (num_polynoms - 1) + j,
          j * (poly_order + 1) + i) =
            factors(poly_order - k - 1 - i) * pow(tau_dot(j), k + 1);
      }
      A(2 * num_polynoms + k * (num_polynoms - 1) + j,
        (j + 1) * (poly_order + 1) + (poly_order - 1 - k)) =
          -factors(0) * pow(tau_dot(j + 1), k + 1);
    }
  }

  // Create constraints for the derivatives of position at the start and end
  // point
  for (int k = 0; k < continuity_order; k++) {
    Eigen::VectorXd factors = computeFactorials(poly_order - k, k);
    A(2 * num_polynoms + continuity_order * (num_polynoms - 1) + k * 2,
      poly_order - 1 - k) = factors(0) * pow(tau_dot(0), k + 1);
    for (int i = 0; i < poly_order - k; i++) {
      A(2 * num_polynoms + continuity_order * (num_polynoms - 1) + k * 2 + 1,
        (num_polynoms - 1) * (poly_order + 1) + i) =
          factors(poly_order - k - 1 - i) *
          pow(tau_dot(num_polynoms - 1), k + 1);
    }
  }

  return A;
}

Eigen::VectorXd generateEqualityConstraintsBVector(
    const PolynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& way_points_1D,
    const Eigen::Vector3d& start_conditions,
    const Eigen::Vector3d& end_conditions) {
  const int continuity_order = trajectory_settings.continuity_order;
  const int num_constraints =
      2 * num_polynoms + continuity_order * (num_polynoms + 1);

  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);

  // Create position constraints
  b(0) = way_points_1D(0);                // trajectory starting point
  for (int i = 1; i < num_polynoms; i++)  // intermediate waypoints
  {
    b(i * 2 - 1) = way_points_1D(i);
    b(i * 2) = way_points_1D(i);
  }
  b(2 * num_polynoms - 1) =
      way_points_1D(num_polynoms);  // trajectory end point

  // Create constraints for the derivatives of position at the start and end
  // point
  for (int k = 0; k < std::min(continuity_order, 3); k++) {
    b(2 * num_polynoms + continuity_order * (num_polynoms - 1) + k * 2) =
        start_conditions(k);
    b(2 * num_polynoms + continuity_order * (num_polynoms - 1) + k * 2 + 1) =
        end_conditions(k);
  }

  return b;
}

Eigen::MatrixXd generateRingEqualityConstraintsAMatrix(
    const PolynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& tau_dot) {
  const int poly_order = trajectory_settings.polynomial_order;
  const int continuity_order = trajectory_settings.continuity_order;
  const int num_constraints =
      2 * num_polynoms + continuity_order * num_polynoms;

  Eigen::MatrixXd A =
      Eigen::MatrixXd::Zero(num_constraints, (poly_order + 1) * num_polynoms);

  // Create position constraints at waypoints
  for (int i = 0; i < num_polynoms; i++) {
    A(2 * i, (poly_order + 1) * i + poly_order) = 1.0;
    for (int j = 0; j < poly_order + 1; j++) {
      A(2 * i + 1, (poly_order + 1) * i + j) = 1.0;
    }
  }

  // Create constraints for continuity of the derivatives of position
  for (int k = 0; k < continuity_order; k++) {
    Eigen::VectorXd factors = computeFactorials(poly_order - k, k);
    for (int j = 0; j < num_polynoms; j++) {
      for (int i = 0; i < poly_order - k; i++) {
        A(2 * num_polynoms + k * num_polynoms + j, j * (poly_order + 1) + i) =
            factors(poly_order - k - 1 - i) * pow(tau_dot(j), k + 1);
      }
      if (j < num_polynoms - 1) {
        A(2 * num_polynoms + k * num_polynoms + j,
          (j + 1) * (poly_order + 1) + (poly_order - 1 - k)) =
            -factors(0) * pow(tau_dot(j + 1), k + 1);
      } else {
        A(2 * num_polynoms + k * num_polynoms + j, poly_order - k - 1) =
            -factors(0) * pow(tau_dot(0), k + 1);
      }
    }
  }

  return A;
}

Eigen::VectorXd generateRingEqualityConstraintsBVector(
    const PolynomialTrajectorySettings& trajectory_settings,
    const int num_polynoms, const Eigen::VectorXd& way_points_1D) {
  const int continuity_order = trajectory_settings.continuity_order;
  const int num_constraints =
      2 * num_polynoms + continuity_order * num_polynoms;

  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);

  // Create position constraints
  b(0) = way_points_1D(0);                // trajectory starting point
  for (int i = 1; i < num_polynoms; i++)  // intermediate waypoints
  {
    b(i * 2 - 1) = way_points_1D(i);
    b(i * 2) = way_points_1D(i);
  }
  b(2 * num_polynoms - 1) = way_points_1D(0);  // trajectory end point

  return b;
}

Eigen::VectorXd computeCostGradient(
    const PolynomialTrajectory& initial_trajectory,
    const PolynomialTrajectorySettings& trajectory_settings) {
  int num_segments = int(initial_trajectory.segment_times.size());
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_segments);

  for (int segment = 0; segment < num_segments; segment++) {
    // compute delta S
    Eigen::VectorXd delta_s =
        -1.0 / (num_segments - 1) * Eigen::VectorXd::Ones(num_segments);
    delta_s(segment) = 1.0;

    // compute new segment times
    double gradient_computation_step_size = 0.01;
    Eigen::VectorXd segment_times = initial_trajectory.segment_times +
                                    gradient_computation_step_size * delta_s;

    // compute new cost by solving optimization with new segment times
    PolynomialTrajectory trajectory;
    if (initial_trajectory.trajectory_type ==
        polynomial_trajectories::TrajectoryType::
            MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
      trajectory = generateMinimumSnapTrajectory(
          segment_times, initial_trajectory.start_state,
          initial_trajectory.end_state, trajectory_settings);
    } else if (initial_trajectory.trajectory_type ==
               polynomial_trajectories::TrajectoryType::
                   MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
      trajectory =
          generateMinimumSnapRingTrajectory(segment_times, trajectory_settings);
    }

    // add element to gradient
    gradient(segment) =
        (trajectory.optimization_cost - initial_trajectory.optimization_cost) /
        gradient_computation_step_size;
  }

  return gradient;
}

Eigen::VectorXd computeSearchDirection(
    const PolynomialTrajectory& initial_trajectory,
    const Eigen::VectorXd& gradient) {
  // compute search direction starting from -gradient then scale it such that
  // the norm of its elements are maximally
  // (max_segment_update_ratio * segment_times(i))
  Eigen::VectorXd search_direction = -gradient;
  const double max_segment_update_ratio = 0.5;

  for (int i = 0; i < search_direction.rows(); i++) {
    if (fabs(search_direction(i)) >
        max_segment_update_ratio * initial_trajectory.segment_times(i)) {
      search_direction *=
          (max_segment_update_ratio * initial_trajectory.segment_times(i) /
           search_direction(i));
    }
  }

  // adapt search direction such that its vector sum is zero (total time of
  // trajectory stays the same)
  // we want |segment_times + search_direction| /
  //     normalization_factor = trajectory.T
  double normalization_factor =
      (initial_trajectory.segment_times + search_direction).sum() /
      initial_trajectory.T.toSec();

  // and we want (segment_times + search_direction) / normalization_factor =
  //     segment_times + adjusted_search_direction
  search_direction = (initial_trajectory.segment_times + search_direction) /
                         normalization_factor -
                     initial_trajectory.segment_times;

  return search_direction;
}

Eigen::VectorXd updateSegmentTimes(
    const PolynomialTrajectory& initial_trajectory,
    const Eigen::VectorXd& gradient,
    const PolynomialTrajectorySettings& trajectory_settings) {
  Eigen::VectorXd updated_segment_times;

  const Eigen::VectorXd search_direction =
      computeSearchDirection(initial_trajectory, gradient);
  double step_ratio = 1.0;

  const double backtracking_alpha = 0.1;
  const double backtracking_beta = 0.5;

  for (;;) {
    Eigen::VectorXd step = step_ratio * search_direction;

    updated_segment_times = initial_trajectory.segment_times + step;

    // compute new cost by solving optimization with new segment times
    PolynomialTrajectory trajectory;
    if (initial_trajectory.trajectory_type ==
        polynomial_trajectories::TrajectoryType::
            MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
      trajectory = generateMinimumSnapTrajectory(
          updated_segment_times, initial_trajectory.start_state,
          initial_trajectory.end_state, trajectory_settings);
    } else if (initial_trajectory.trajectory_type ==
               polynomial_trajectories::TrajectoryType::
                   MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
      trajectory = generateMinimumSnapRingTrajectory(updated_segment_times,
                                                     trajectory_settings);
    }

    step_ratio *= backtracking_beta;
    if (trajectory.optimization_cost <
        initial_trajectory.optimization_cost +
            backtracking_alpha * step.dot(gradient)) {
      break;
    }
  }

  return updated_segment_times;
}

PolynomialTrajectory enforceMaximumVelocityAndThrust(
    const PolynomialTrajectory& initial_trajectory,
    const PolynomialTrajectorySettings& trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate) {
  if (initial_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    ROS_ERROR(
        "[%s] Cannot enforce maximum velocity/thrust on undefined "
        "trajectory",
        ros::this_node::getName().c_str());
    return initial_trajectory;
  }

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

    if (initial_trajectory.trajectory_type ==
            polynomial_trajectories::TrajectoryType::MINIMUM_SNAP ||
        initial_trajectory.trajectory_type ==
            polynomial_trajectories::TrajectoryType::
                MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
      trajectory = generateMinimumSnapTrajectory(
          0.9 * trajectory.segment_times, initial_trajectory.start_state,
          initial_trajectory.end_state, trajectory_settings);
    } else if (initial_trajectory.trajectory_type ==
                   polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_RING ||
               initial_trajectory.trajectory_type ==
                   polynomial_trajectories::TrajectoryType::
                       MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
      trajectory = generateMinimumSnapRingTrajectory(
          0.9 * trajectory.segment_times, trajectory_settings);
    }

    // Check if generateMinimumSnap<Ring>Trajectory() was successful
    if (trajectory.trajectory_type ==
        polynomial_trajectories::TrajectoryType::UNDEFINED) {
      return trajectory;
    }

    computeQuadRelevantMaxima(trajectory, &prev_maxima.x(), &prev_maxima.y(),
                              &prev_maxima.z());
  }

  // compute gradient of maxima with respect to T
  Eigen::Vector3d maxima_gradient;
  if (!computeMaximaGradient(trajectory, prev_maxima, trajectory_settings,
                             &maxima_gradient)) {
    trajectory.trajectory_type =
        polynomial_trajectories::TrajectoryType::UNDEFINED;
    return trajectory;
  }

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
      // the ratio of segment times remains with scaling the execution time so
      // only compute trajectory without segment refinement
      if (initial_trajectory.trajectory_type ==
              polynomial_trajectories::TrajectoryType::MINIMUM_SNAP ||
          initial_trajectory.trajectory_type ==
              polynomial_trajectories::TrajectoryType::
                  MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
        trajectory = generateMinimumSnapTrajectory(
            trajectory.segment_times / trajectory.T.toSec() *
                new_intersections.maxCoeff(),
            initial_trajectory.start_state, initial_trajectory.end_state,
            trajectory_settings);
      } else if (initial_trajectory.trajectory_type ==
                     polynomial_trajectories::TrajectoryType::
                         MINIMUM_SNAP_RING ||
                 initial_trajectory.trajectory_type ==
                     polynomial_trajectories::TrajectoryType::
                         MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
        trajectory = generateMinimumSnapRingTrajectory(
            trajectory.segment_times / trajectory.T.toSec() *
                new_intersections.maxCoeff(),
            trajectory_settings);
      }

      // Check if generateMinimumSnap<Ring>Trajectory() was successful
      if (trajectory.trajectory_type ==
          polynomial_trajectories::TrajectoryType::UNDEFINED) {
        return trajectory;
      }

      computeQuadRelevantMaxima(trajectory, &maxima.x(), &maxima.y(),
                                &maxima.z());

      if (maxima.x() <= 1.01 * desired_maxima.x() &&
          maxima.y() <= 1.01 * desired_maxima.y() &&
          maxima.z() <= 1.01 * desired_maxima.z()) {
        // we are close enough at the limits so we stop here
        break;
      }

      // compute gradient of maxima with respect to T
      if (!computeMaximaGradient(trajectory, maxima, trajectory_settings,
                                 &maxima_gradient)) {
        trajectory.trajectory_type =
            polynomial_trajectories::TrajectoryType::UNDEFINED;
        return trajectory;
      }

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
        if (initial_trajectory.trajectory_type ==
                polynomial_trajectories::TrajectoryType::MINIMUM_SNAP ||
            initial_trajectory.trajectory_type ==
                polynomial_trajectories::TrajectoryType::
                    MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
          trajectory = generateMinimumSnapTrajectory(
              1.1 * trajectory.segment_times, initial_trajectory.start_state,
              initial_trajectory.end_state, trajectory_settings);
        } else if (initial_trajectory.trajectory_type ==
                       polynomial_trajectories::TrajectoryType::
                           MINIMUM_SNAP_RING ||
                   initial_trajectory.trajectory_type ==
                       polynomial_trajectories::TrajectoryType::
                           MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
          trajectory = generateMinimumSnapRingTrajectory(
              1.1 * trajectory.segment_times, trajectory_settings);
        }

        // Check if generateMinimumSnap<Ring>Trajectory() was successful
        if (trajectory.trajectory_type ==
            polynomial_trajectories::TrajectoryType::UNDEFINED) {
          return trajectory;
        }

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

  trajectory.trajectory_type = initial_trajectory.trajectory_type;

  return trajectory;
}

bool computeMaximaGradient(
    const PolynomialTrajectory& trajectory, const Eigen::Vector3d& maxima,
    const PolynomialTrajectorySettings& trajectory_settings,
    Eigen::Vector3d* gradient) {
  PolynomialTrajectory gradient_trajectory = trajectory;
  if (trajectory.trajectory_type ==
          polynomial_trajectories::TrajectoryType::MINIMUM_SNAP ||
      trajectory.trajectory_type == polynomial_trajectories::TrajectoryType::
                                        MINIMUM_SNAP_OPTIMIZED_SEGMENTS) {
    gradient_trajectory = generateMinimumSnapTrajectory(
        1.01 * trajectory.segment_times, trajectory.start_state,
        trajectory.end_state, trajectory_settings);
  } else if (trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_RING ||
             trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::
                     MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
    gradient_trajectory = generateMinimumSnapRingTrajectory(
        1.01 * trajectory.segment_times, trajectory_settings);
  }

  // Check if generateMinimumSnap<Ring>Trajectory() was successful
  if (gradient_trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return false;
  }

  Eigen::Vector3d gradient_trajectory_maxima;
  computeQuadRelevantMaxima(
      gradient_trajectory, &gradient_trajectory_maxima.x(),
      &gradient_trajectory_maxima.y(), &gradient_trajectory_maxima.z());

  *gradient = (gradient_trajectory_maxima - maxima) /
              (gradient_trajectory.T - trajectory.T).toSec();

  return true;
}

std::vector<Eigen::MatrixXd> reorganiceCoefficientsSegmentWise(
    const std::vector<Eigen::MatrixXd>& coefficients, const int num_segments,
    const int polynomial_order) {
  std::vector<Eigen::MatrixXd> reorganized_coefficients;

  // Reorganize coefficients such that each element of the vector contains the
  // coefficients for one trajectory segment
  for (int segment = 0; segment < num_segments; segment++) {
    Eigen::MatrixXd segment_coeff =
        Eigen::MatrixXd::Zero(3, polynomial_order + 1);

    for (int dimension = 0; dimension < 3; dimension++) {
      segment_coeff.row(dimension) = coefficients[dimension].row(segment);
    }

    reorganized_coefficients.push_back(segment_coeff);
  }

  return reorganized_coefficients;
}

std::vector<Eigen::Vector3d> addStartAndEndToWayPointList(
    const std::vector<Eigen::Vector3d>& intermediate_way_points,
    const Eigen::Vector3d& start_position,
    const Eigen::Vector3d& end_position) {
  // Add start and end position to way points vector
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(start_position);
  for (int i = 0; i < int(intermediate_way_points.size()); i++) {
    way_points.push_back(intermediate_way_points[i]);
  }
  way_points.push_back(end_position);

  return way_points;
}

PolynomialTrajectorySettings ensureFeasibleTrajectorySettings(
    const PolynomialTrajectorySettings& original_trajectory_settings,
    const int min_poly_order) {
  PolynomialTrajectorySettings new_trajectory_settings =
      original_trajectory_settings;

  // enforce minimum necessary polynomial order
  if (original_trajectory_settings.polynomial_order < min_poly_order) {
    new_trajectory_settings.polynomial_order = min_poly_order;
    ROS_WARN_THROTTLE(
        1.0,
        "[%s] Requested polynomial order (%d) is too small to enforce "
        "continuity order of %d for the desired trajectory. Polynomial order "
        "is set to the required minimum (%d).",
        ros::this_node::getName().c_str(),
        original_trajectory_settings.polynomial_order,
        original_trajectory_settings.continuity_order, min_poly_order);
  }

  // Ensure non zero minimization weight on position (otherwise QP solver
  // cannot handle the problem)
  // Also, the weights are normalized such that the largest one is equal to 1
  new_trajectory_settings.minimization_weights =
      original_trajectory_settings.minimization_weights /
      original_trajectory_settings.minimization_weights.maxCoeff();

  return new_trajectory_settings;
}

Eigen::VectorXd solveQuadraticProgram(const Eigen::MatrixXd& H,
                                      const Eigen::VectorXd& f,
                                      const Eigen::MatrixXd& A_eq,
                                      const Eigen::VectorXd& b_eq,
                                      double* objective_value) {
  Eigen::VectorXd solution;

  // Try to solve problem as Lagrange optimization (using lagrange multipliers)
  Eigen::MatrixXd A_lagrange =
      Eigen::MatrixXd::Zero(H.rows() + A_eq.rows(), H.rows() + A_eq.rows());
  A_lagrange.block(0, 0, H.cols(), H.rows()) = 2.0 * H.transpose();
  A_lagrange.block(0, H.rows(), A_eq.cols(), A_eq.rows()) = A_eq.transpose();
  A_lagrange.block(H.cols(), 0, A_eq.rows(), A_eq.cols()) = A_eq;
  Eigen::VectorXd b_lagrange = Eigen::VectorXd::Zero(H.rows() + A_eq.rows());
  b_lagrange.segment(0, H.rows()) = -f;
  b_lagrange.segment(H.rows(), A_eq.rows()) = b_eq;

//  Eigen::VectorXd x = A_lagrange.colPivHouseholderQr().solve(b_lagrange);
  Eigen::VectorXd x = A_lagrange.householderQr().solve(b_lagrange);
  solution = x.segment(0, H.rows());

  *objective_value = solution.transpose() * H * solution + f.dot(solution);

  return solution;
}

}  // namespace implementation

}  // namespace minimum_snap_trajectories

}  // namespace polynomial_trajectories
