#include "polynomial_trajectories/polynomial_trajectories_common.h"

#include <ros/ros.h>

namespace polynomial_trajectories {

quadrotor_common::TrajectoryPoint getPointFromTrajectory(
    const PolynomialTrajectory& trajectory,
    const ros::Duration& time_from_start) {
  quadrotor_common::TrajectoryPoint desired_state;

  if (trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    ROS_ERROR(
        "The type of the trajectory you wanted to evaluate is not defined!");
    return desired_state;
  }

  if (trajectory.coeff.size() == 0) {
    ROS_ERROR("[%s] The passed trajectory contains no polynomial coefficients!",
              ros::this_node::getName().c_str());
    return desired_state;
  }

  const int dimension = trajectory.coeff[0].rows();
  const int poly_order = trajectory.coeff[0].cols() - 1;
  const int num_polynoms = trajectory.number_of_segments;

  // Check if the dimension of the computed trajectory is at least 3
  if (dimension < 3) {
    ROS_ERROR(
        "[%s] The computed trajectory has a dimension less than 3 "
        "(dimension = %d)",
        ros::this_node::getName().c_str(), dimension);
    return desired_state;
  }

  // Check if time is between 0 and trajectory duration
  double time_eval = time_from_start.toSec();
  if (time_eval < 0) {
    ROS_WARN(
        "[%s] Requested desired state from trajectory for a time where the "
        "trajectory is not defined (t = %f). Trajectory is defined for "
        "t = [%f, %f]. Trajectory at time t = %f is returned instead.",
        ros::this_node::getName().c_str(), time_eval, 0.0, trajectory.T.toSec(),
        0.0);
    return trajectory.start_state;
  } else if (time_eval > trajectory.T.toSec()) {
    if (trajectory.trajectory_type ==
            polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_RING ||
        trajectory.trajectory_type ==
            polynomial_trajectories::TrajectoryType::
                MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
      time_eval = fmod(time_eval, trajectory.T.toSec());
    } else if (time_eval > trajectory.T.toSec() + 0.01) {
      ROS_WARN(
          "[%s] Requested desired state from trajectory for a time where the "
          "trajectory is not defined (t = %f). Trajectory is defined for "
          "t = [%f, %f]. Trajectory at time t = %f is returned instead.",
          ros::this_node::getName().c_str(), time_eval, 0.0,
          trajectory.T.toSec(), trajectory.T.toSec());
      return trajectory.end_state;
    }
  }

  if (trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::FULLY_CONSTRAINED) {
    const int number_of_coefficients = trajectory.coeff[0].cols();

    for (int axis = 0; axis < 3; axis++) {
      desired_state.position(axis) =
          trajectory.coeff[0].row(axis) *
          (dVec(number_of_coefficients, 0).asDiagonal() *
           tVec(number_of_coefficients, 0, time_eval));
      desired_state.velocity(axis) =
          trajectory.coeff[0].row(axis) *
          (dVec(number_of_coefficients, 1).asDiagonal() *
           tVec(number_of_coefficients, 1, time_eval));
      desired_state.acceleration(axis) =
          trajectory.coeff[0].row(axis) *
          (dVec(number_of_coefficients, 2).asDiagonal() *
           tVec(number_of_coefficients, 2, time_eval));
      desired_state.jerk(axis) = trajectory.coeff[0].row(axis) *
                                 (dVec(number_of_coefficients, 3).asDiagonal() *
                                  tVec(number_of_coefficients, 3, time_eval));
      desired_state.snap(axis) = trajectory.coeff[0].row(axis) *
                                 (dVec(number_of_coefficients, 4).asDiagonal() *
                                  tVec(number_of_coefficients, 4, time_eval));
    }
  } else if (trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::MINIMUM_SNAP ||
             trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::MINIMUM_SNAP_RING ||
             trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::
                     MINIMUM_SNAP_OPTIMIZED_SEGMENTS ||
             trajectory.trajectory_type ==
                 polynomial_trajectories::TrajectoryType::
                     MINIMUM_SNAP_RING_OPTIMIZED_SEGMENTS) {
    // Figure out which single polynomial we have to evaluate -> m
    int m = 0;
    if (trajectory.number_of_segments > 1) {
      for (int i = 0; i < trajectory.number_of_segments; i++) {
        if (time_eval <= trajectory.segment_times.head(i + 1).sum()) {
          m = i;
          break;
        }
      }
    }

    // Compute desired state from trajectory
    Eigen::Vector3d desired_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_jerk = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_snap = Eigen::Vector3d::Zero();

    Eigen::VectorXd velocity_factorials = computeFactorials(poly_order, 0);
    Eigen::VectorXd acceleration_factorials =
        computeFactorials(poly_order - 1, 1);
    Eigen::VectorXd jerk_factorials = computeFactorials(poly_order - 2, 2);
    Eigen::VectorXd snap_factorials = computeFactorials(poly_order - 3, 3);

    // time with which coefficients are computed
    const double tau = (time_eval - trajectory.segment_times.head(m).sum()) /
                       trajectory.segment_times(m);

    Eigen::VectorXd tau_dot(num_polynoms);
    for (int i = 0; i < num_polynoms; i++) {
      tau_dot(i) = 1.0 / trajectory.segment_times(i);
    }

    for (int d = 0; d < 3; d++) {
      Eigen::MatrixXd segment_coefficients = trajectory.coeff[m];
      for (int i = 0; i < poly_order + 1; i++) {
        desired_position(d) +=
            segment_coefficients(d, i) * pow(tau, poly_order - i);
      }
      for (int i = 0; i < poly_order; i++) {
        desired_velocity(d) += velocity_factorials(poly_order - 1 - i) *
                               segment_coefficients(d, i) *
                               pow(tau, poly_order - 1 - i) * tau_dot(m);
      }
      for (int i = 0; i < poly_order - 1; i++) {
        desired_acceleration(d) += acceleration_factorials(poly_order - 2 - i) *
                                   segment_coefficients(d, i) *
                                   pow(tau, poly_order - 2 - i) *
                                   pow(tau_dot(m), 2.0);
      }
      for (int i = 0; i < poly_order - 2; i++) {
        desired_jerk(d) += jerk_factorials(poly_order - 3 - i) *
                           segment_coefficients(d, i) *
                           pow(tau, poly_order - 3 - i) * pow(tau_dot(m), 3.0);
      }
      for (int i = 0; i < poly_order - 3; i++) {
        desired_snap(d) += snap_factorials(poly_order - 4 - i) *
                           segment_coefficients(d, i) *
                           pow(tau, poly_order - 4 - i) * pow(tau_dot(m), 4.0);
      }
    }

    desired_state.position = desired_position;
    desired_state.velocity = desired_velocity;
    desired_state.acceleration = desired_acceleration;
    desired_state.jerk = desired_jerk;
    desired_state.snap = desired_snap;
    desired_state.heading = 0.0;
    desired_state.heading_rate = 0.0;
    desired_state.heading_acceleration = 0.0;
  }

  desired_state.time_from_start = ros::Duration(time_eval);

  return desired_state;
}

Eigen::VectorXd computeFactorials(const int length, const int order) {
  Eigen::VectorXd factorials = Eigen::VectorXd::Zero(length);

  for (int i = 0; i < length; i++) {
    int temp = 1;
    for (int k = 0; k < order + 1; k++) {
      temp *= (i + 1 + k);
    }
    factorials(i) = temp;
  }

  return factorials;
}

// TODO: These two functions (computeFactorials and dVec) are very similar,
// they should be mergable
Eigen::VectorXd dVec(const int number_of_coefficients,
                     const int derivative_order) {
  // computes the coefficient that comes with the derivative
  // derivative_order = 0: [ 1 1 1 1 1]
  // derivative_order = 1: [ 0 1 2 3 4]
  // derivative_order = 2: [ 0 0 2 6 12.]

  Eigen::VectorXd dvec = Eigen::VectorXd::Ones(number_of_coefficients);

  if (derivative_order == 0) return dvec;

  for (int i = 0; i < number_of_coefficients; i++) {
    double tmp = std::max(0, i);
    for (int k = 1; k <= derivative_order - 1; k++) {
      tmp *= std::max(0, i - k);
    }
    dvec(i) = tmp;
  }
  return dvec;
}

Eigen::VectorXd tVec(const int number_of_coefficients,
                     const int derivative_order, const double t) {
  // def t_vec(polynomial_order, derivative_order, time):
  //    # fill a time vector like
  //    # derivative_order = 0: [1 t t^2 t^3 ...]
  //    # derivative_order = 1: [0 1 t   t^2 ...]
  //    # derivative_order = 2: [0 0 1   t^1 ...]
  //    # .......

  Eigen::VectorXd tvec = Eigen::VectorXd::Zero(number_of_coefficients);
  for (int i = derivative_order; i < number_of_coefficients; i++) {
    double power = std::max(0, i - derivative_order);
    tvec(i) = std::pow(t, power);
  }
  return tvec;
}

void computeMaxima(const PolynomialTrajectory& trajectory,
                   double* maximal_velocity, double* maximal_acceleration,
                   double* maximal_jerk, double* maximal_snap) {
  *maximal_velocity = 0.0;
  *maximal_acceleration = 0.0;
  *maximal_jerk = 0.0;
  *maximal_snap = 0.0;
  double dt = 0.05;
  if (trajectory.T.toSec() < 0.5) {
    dt = 0.005;
  }
  double t = 0.0;
  while (t + dt <= trajectory.T.toSec()) {
    t += dt;
    quadrotor_common::TrajectoryPoint state =
        getPointFromTrajectory(trajectory, ros::Duration(t));
    double velocity = state.velocity.norm();
    double acceleration = state.acceleration.norm();
    double jerk = state.jerk.norm();

    if (velocity > *maximal_velocity) {
      *maximal_velocity = velocity;
    }
    if (acceleration > *maximal_acceleration) {
      *maximal_acceleration = acceleration;
    }
    if (jerk > *maximal_jerk) {
      *maximal_jerk = jerk;
    }
  }
}

void computeQuadRelevantMaxima(const PolynomialTrajectory& trajectory,
                               double* maximal_velocity,
                               double* maximal_normalized_thrust,
                               double* maximal_roll_pitch_rate) {
  if (trajectory.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    ROS_ERROR("Could not compute maxima since trajectory type is UNDEFINED");
    return;
  }

  *maximal_velocity = 0.0;
  *maximal_normalized_thrust = 0.0;
  *maximal_roll_pitch_rate = 0.0;

  const Eigen::Vector3d gravity(0.0, 0.0, 9.81);

  double dt = 0.01;

  double t = 0.0;
  while (t <= trajectory.T.toSec()) {
    quadrotor_common::TrajectoryPoint state =
        getPointFromTrajectory(trajectory, ros::Duration(t));

    const double velocity = state.velocity.norm();

    const double thrust = (state.acceleration + gravity).norm();

    const double roll_pitch_rate =
        computeRollPitchRateNormFromTrajectoryPoint(state);

    if (velocity > *maximal_velocity) {
      *maximal_velocity = velocity;
    }
    if (thrust > *maximal_normalized_thrust) {
      *maximal_normalized_thrust = thrust;
    }

    if (roll_pitch_rate > *maximal_roll_pitch_rate) {
      *maximal_roll_pitch_rate = roll_pitch_rate;
    }

    t += dt;
  }
}

bool isStartAndEndStateFeasibleUnderConstraints(
    const quadrotor_common::TrajectoryPoint& start_state,
    const quadrotor_common::TrajectoryPoint& end_state,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate) {
  if (max_normalized_thrust <= 9.81) {
    return false;
  }
  if (start_state.velocity.norm() > max_velocity) {
    return false;
  }
  if (end_state.velocity.norm() > max_velocity) {
    return false;
  }
  if ((start_state.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81)).norm() >
      max_normalized_thrust) {
    return false;
  }
  if ((end_state.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81)).norm() >
      max_normalized_thrust) {
    return false;
  }
  if (computeRollPitchRateNormFromTrajectoryPoint(start_state) >
      max_roll_pitch_rate) {
    return false;
  }
  if (computeRollPitchRateNormFromTrajectoryPoint(end_state) >
      max_roll_pitch_rate) {
    return false;
  }

  return true;
}

double computeRollPitchRateNormFromTrajectoryPoint(
    const quadrotor_common::TrajectoryPoint& desired_state) {
  const Eigen::Vector3d gravity(0.0, 0.0, 9.81);

  const Eigen::Vector3d des_acceleration = desired_state.acceleration + gravity;
  const double thrust = des_acceleration.norm();

  const Eigen::Vector3d e_z_body_desired = des_acceleration.normalized();

  const Eigen::Vector3d thrust_normalized_jerk = desired_state.jerk / thrust;

  const double roll_pitch_rate =
      sqrt(pow(thrust_normalized_jerk.norm(), 2.0) -
           pow(e_z_body_desired.dot(thrust_normalized_jerk), 2.0));

  return roll_pitch_rate;
}

}  // namespace polynomial_trajectories
