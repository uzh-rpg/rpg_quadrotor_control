#include "trajectory_generation_helper/acrobatic_sequence.h"

#include <minimum_jerk_trajectories/RapidTrajectoryGenerator.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_common/trajectory_point.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/flip_trajectory_helper.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <fstream>

namespace fpv_aggressive_trajectories {
AcrobaticSequence::AcrobaticSequence(
    const quadrotor_common::TrajectoryPoint& start_state) {
  printf("Initiated acrobatic sequence\n");
  quadrotor_common::Trajectory init_trajectory;
  quadrotor_common::TrajectoryPoint init_point;
  //  init_point.position = start_pos;
  init_point = start_state;
  init_trajectory.points.push_back(init_point);
  maneuver_list_.push_back(init_trajectory);
}

AcrobaticSequence::~AcrobaticSequence() {}

bool AcrobaticSequence::appendStraight(const Eigen::Vector3d& end_position,
                                       const Eigen::Vector3d& end_velocity,
                                       const double& end_yaw,
                                       const double& max_velocity,
                                       const double& traj_sampling_freq,
                                       const bool& minimum_snap) {
  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  const double exec_loop_rate = traj_sampling_freq;

  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = end_position;
  end_state.velocity = end_velocity;
  double execution_time =
      (end_position - init_state.position).norm() / max_velocity;

  const double max_thrust = 20.0;
  const double max_roll_pitch_rate = 3.0;

  quadrotor_common::Trajectory trajectory;
  if (!minimum_snap) {
    trajectory =
        trajectory_generation_helper::polynomials::computeFixedTimeTrajectory(
            init_state, end_state, 4, execution_time, exec_loop_rate);
  } else {
    Eigen::VectorXd segment_times = Eigen::VectorXd::Ones(1);
    segment_times[0] = execution_time;
    Eigen::VectorXd minimization_weights(4);
    minimization_weights << 0.0, 0.0, 100.0, 100.0;
    std::vector<Eigen::Vector3d> waypoints;

    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
    trajectory_settings.way_points = waypoints;
    trajectory_settings.minimization_weights = minimization_weights;
    trajectory_settings.polynomial_order = 11;
    trajectory_settings.continuity_order = 4;
    trajectory = trajectory_generation_helper::polynomials::
        generateMinimumSnapTrajectory(
            segment_times, init_state, end_state, trajectory_settings,
            max_velocity, max_thrust, max_roll_pitch_rate, exec_loop_rate);
  }

  trajectory_generation_helper::heading::addConstantHeading(end_yaw,
                                                            &trajectory);

  maneuver_list_.push_back(trajectory);

  return !(trajectory.trajectory_type ==
           quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

std::list<quadrotor_common::Trajectory> AcrobaticSequence::getManeuverList() {
  return maneuver_list_;
}

}  // namespace fpv_aggressive_trajectories
