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

bool AcrobaticSequence::appendMinJerk(
    const quadrotor_common::TrajectoryPoint& end_state, const double& duration,
    const double& traj_sampling_freq) {
  bool success = false;
  quadrotor_common::TrajectoryPoint start_state =
      maneuver_list_.back().points.back();
  Vec3 pos0 = Vec3(start_state.position(0), start_state.position(1),
                   start_state.position(2));
  Vec3 vel0 = Vec3(start_state.velocity(0), start_state.velocity(1),
                   start_state.velocity(2));
  Vec3 acc0 = Vec3(start_state.acceleration(0), start_state.acceleration(1),
                   start_state.acceleration(2));

  // Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0, 0, -9.81);  //[m/s**2]

  minimum_jerk_trajectories::RapidTrajectoryGenerator traj =
      minimum_jerk_trajectories::RapidTrajectoryGenerator(pos0, vel0, acc0,
                                                          gravity);

  Vec3 posf =
      Vec3(end_state.position(0), end_state.position(1), end_state.position(2));
  Vec3 velf =
      Vec3(end_state.velocity(0), end_state.velocity(1), end_state.velocity(2));
  Vec3 accf = Vec3(end_state.acceleration(0), end_state.acceleration(1),
                   end_state.acceleration(2));

  traj.SetGoalPosition(posf);
  traj.SetGoalVelocity(velf);
  traj.SetGoalAcceleration(accf);

  traj.Generate(duration);

  double min_normalized_thrust = 2.0;
  double max_normalized_thrust = 20.0;
  double max_roll_pitch_rate = 6.9;
  double min_traj_sampling_time = 1.0 / traj_sampling_freq;

  minimum_jerk_trajectories::RapidTrajectoryGenerator::InputFeasibilityResult
      res = traj.CheckInputFeasibility(
          min_normalized_thrust, max_normalized_thrust, max_roll_pitch_rate,
          min_traj_sampling_time);
  if (res ==
      minimum_jerk_trajectories::RapidTrajectoryGenerator::InputFeasible) {
    // check also for floor/ceiling feasibility
    Vec3 boundary_point = Vec3(0.0, 0.0, 0.0);  // a point on the floor
    Vec3 boundary_normal =
        Vec3(0.0, 0.0,
             1.0);  // we want to be in this direction of the point (upwards)
    minimum_jerk_trajectories::RapidTrajectoryGenerator::StateFeasibilityResult
        positionFeasible_floor =
            traj.CheckPositionFeasibility(boundary_point, boundary_normal);
    boundary_point = Vec3(0.0, 0.0, 100);  // a point on the floor
    boundary_normal =
        Vec3(0.0, 0.0,
             -1.0);  // we want to be in this direction of the point (upwards)
    minimum_jerk_trajectories::RapidTrajectoryGenerator::StateFeasibilityResult
        positionFeasible_ceiling =
            traj.CheckPositionFeasibility(boundary_point, boundary_normal);
    if (positionFeasible_ceiling ==
            minimum_jerk_trajectories::RapidTrajectoryGenerator::
                StateFeasible &&
        positionFeasible_floor == minimum_jerk_trajectories::
                                      RapidTrajectoryGenerator::StateFeasible) {
      success = true;
    }
  } else {
    ROS_WARN("[%s] Trajectory not input feasible.",
             ros::this_node::getName().c_str());
  }

  if (!success) {
    ROS_WARN("[%s] Failed to calculate trajectory.",
             ros::this_node::getName().c_str());
    ROS_WARN("[%s] Start: [%f, %f, %f].", ros::this_node::getName().c_str(),
             start_state.position.x(), start_state.position.y(),
             start_state.position.z());
    ROS_WARN("[%s] End: [%f, %f, %f].", ros::this_node::getName().c_str(),
             end_state.position.x(), end_state.position.y(),
             end_state.position.z());
  }

  // sample trajectory
  quadrotor_common::Trajectory min_jerk_trajectory;

  double t_temp = 0.0;

  while (t_temp <= duration) {
    Vec3 position = traj.GetPosition(t_temp);
    Vec3 velocity = traj.GetVelocity(t_temp);
    Vec3 acceleration = traj.GetAcceleration(t_temp);

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(t_temp);
    point.position = Eigen::Vector3d(position.x, position.y, position.z);
    point.velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
    point.acceleration =
        Eigen::Vector3d(acceleration.x, acceleration.y, acceleration.z);

    min_jerk_trajectory.points.push_back(point);
    t_temp += 1.0 / traj_sampling_freq;
  }
  trajectory_generation_helper::heading::addConstantHeading(
      0.0, &min_jerk_trajectory);

  maneuver_list_.push_back(min_jerk_trajectory);
  return success;
}

bool AcrobaticSequence::appendLoopli(const int n_loops,
                                     const double& circle_velocity,
                                     const double& radius,
                                     const double& traj_sampling_freq) {
  printf("appending flying room loop\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  printf(
      "Enter state of loop maneuver: Pos: %.2f, %.2f, %.2f | Vel: %.2f, %.2f, "
      "%.2f\n",
      init_state.position.x(), init_state.position.y(), init_state.position.z(),
      init_state.velocity.x(), init_state.velocity.y(),
      init_state.velocity.z());

  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = 0.0;

  const double figure_z_rotation_angle = 0.0;
  //  const double figure_z_rotation_angle = 0.785398163;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      init_state.position + q_W_P.inverse() * Eigen::Vector3d(1.0, 0.0, 0.4);

  printf("circle center: %.2f, %.2f, %.2f\n", circle_center.x(),
         circle_center.y(), circle_center.z());

  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity, 2.0) / radius;
  const double max_roll_pitch_rate = 3.0;

  // Compute Circle trajectory
  printf("compute circle trajectory\n");
  printf("max thrust: %.2f", max_thrust);

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, radius, circle_velocity,
          M_PI / 2.0, -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();

  // Start position relative to circle center
  quadrotor_common::TrajectoryPoint start_state;
  start_state = init_state;

  // enter trajectory
  printf("compute enter trajectory\n");
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeFixedTimeTrajectory(
          start_state, circle_enter_state, 4, 3.0, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &enter_trajectory);

  // End position RELATIVE to circle center
  printf("compute exit trajectory\n");

  const Eigen::Vector3d end_pos_P =
      Eigen::Vector3d(1.0, 0.0, 0.0);  // nice breaking forward flip
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = q_W_P * end_pos_P + circle_center;
  end_state.velocity = q_W_P * Eigen::Vector3d(circle_velocity, 0.0, 0.0);

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeFixedTimeTrajectory(
          circle_enter_state, end_state, 4, 3.0, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &exit_trajectory);

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(exit_trajectory);

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendHorizontalCircle(
    const int n_loops, const double& circle_velocity, const double& radius,
    const Eigen::Vector3d& circle_center_offset,
    const Eigen::Vector3d& circle_center_offset_end, const bool break_at_end,
    const double& traj_sampling_freq) {
  printf("appending horizontal circle\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  printf(
      "Enter state of maneuver: Pos: %.2f, %.2f, %.2f | Vel: %.2f, %.2f, "
      "%.2f\n",
      init_state.position.x(), init_state.position.y(), init_state.position.z(),
      init_state.velocity.x(), init_state.velocity.y(),
      init_state.velocity.z());

  const double exec_loop_rate = traj_sampling_freq;

  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      init_state.position + circle_center_offset +
      Eigen::Vector3d::UnitX() * circle_velocity;

  printf("circle center: %.2f, %.2f, %.2f\n", circle_center.x(),
         circle_center.y(), circle_center.z());

  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity, 2.0) / radius;
  const double max_roll_pitch_rate = 3.0;

  // Compute Circle trajectory
  printf("compute circle trajectory\n");

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          circle_center, radius, circle_velocity, M_PI_2,
          -(0.5 + 2 * n_loops) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addForwardHeading(&circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();
  quadrotor_common::TrajectoryPoint circle_exit_state =
      circle_trajectory.points.back();

  // Start position relative to circle center
  quadrotor_common::TrajectoryPoint start_state;
  start_state = init_state;

  quadrotor_common::TrajectoryPoint speedup_state;
  speedup_state = start_state;
  speedup_state.position += Eigen::Vector3d::UnitX() * circle_velocity / 2.0;
  speedup_state.velocity = Eigen::Vector3d::UnitX() * circle_velocity / 2.0;

  // enter trajectory
  printf("compute enter trajectory\n");
  quadrotor_common::Trajectory enter_trajectory_1 =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, speedup_state, 4,
          1.1 * std::max(start_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addForwardHeading(&enter_trajectory_1);
  quadrotor_common::Trajectory enter_trajectory_2 =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          speedup_state, circle_enter_state, 4,
          1.1 * std::max(start_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addForwardHeading(&enter_trajectory_2);

  // End position RELATIVE to circle center
  printf("compute exit trajectory\n");
  const Eigen::Vector3d end_pos_P = circle_center_offset_end;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = circle_exit_state.position + circle_exit_state.velocity;
  end_state.velocity = circle_exit_state.velocity;

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_exit_state, end_state, 4, 1.1 * circle_velocity, max_thrust,
          2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addForwardHeading(&exit_trajectory);

  quadrotor_common::Trajectory breaking_trajectory;
  breaking_trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  maneuver_list_.push_back(enter_trajectory_1);
  maneuver_list_.push_back(enter_trajectory_2);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(exit_trajectory);

  if (break_at_end) {
    // append breaking trajectory at end
    quadrotor_common::TrajectoryPoint end_state_hover;
    end_state_hover.position =
        (end_state.position + end_state.velocity);
    end_state_hover.velocity = Eigen::Vector3d::Zero();
    breaking_trajectory =
        trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
            end_state, end_state_hover, 4, 1.1 * circle_velocity, 15.0,
            max_roll_pitch_rate, exec_loop_rate);
    trajectory_generation_helper::heading::addForwardHeading(
        &breaking_trajectory);
    maneuver_list_.push_back(breaking_trajectory);
  }

  return !(enter_trajectory_1.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           enter_trajectory_2.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           breaking_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendLoops(
    const int n_loops, const double& circle_velocity, const double& radius,
    const Eigen::Vector3d& circle_center_offset,
    const Eigen::Vector3d& circle_center_offset_end, const bool break_at_end,
    const double& traj_sampling_freq) {
  printf("appending loop\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  printf(
      "Enter state of loop maneuver: Pos: %.2f, %.2f, %.2f | Vel: %.2f, %.2f, "
      "%.2f\n",
      init_state.position.x(), init_state.position.y(), init_state.position.z(),
      init_state.velocity.x(), init_state.velocity.y(),
      init_state.velocity.z());

  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = 0.0;

  const double figure_z_rotation_angle = 0.0;
  //  const double figure_z_rotation_angle = 0.785398163;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      init_state.position + q_W_P.inverse() * circle_center_offset;

  printf("circle center: %.2f, %.2f, %.2f\n", circle_center.x(),
         circle_center.y(), circle_center.z());

  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity, 2.0) / radius;
  const double max_roll_pitch_rate = 3.0;

  // Compute Circle trajectory
  printf("compute circle trajectory\n");

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, radius, circle_velocity,
          M_PI / 2.0, -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();

  // Start position relative to circle center
  quadrotor_common::TrajectoryPoint start_state;
  start_state = init_state;

  // enter trajectory
  printf("compute enter trajectory\n");
  //  printf("Maximum speed: %.3f, current speed: %.3f\n", 1.1*circle_velocity,
  //  start_state.velocity.norm());
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4,
          1.1 * std::max(start_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &enter_trajectory);

  // End position RELATIVE to circle center
  printf("compute exit trajectory\n");

  const Eigen::Vector3d end_pos_P =
      circle_center_offset_end;  // Eigen::Vector3d(circle_center_offset.x(),
  // scircle_center_offset.y(),-circle_center_offset.z()); // nice breaking
  // forward flip
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = q_W_P * end_pos_P + circle_center;
  end_state.velocity = q_W_P * Eigen::Vector3d(circle_velocity, 0.0, 0.0);

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_enter_state, end_state, 4, 1.1 * circle_velocity, max_thrust,
          2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &exit_trajectory);

  quadrotor_common::Trajectory breaking_trajectory;
  breaking_trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(exit_trajectory);

  if (break_at_end) {
    // append breaking trajectory at end
    quadrotor_common::TrajectoryPoint end_state_hover;
    end_state_hover.position =
        (end_state.position + Eigen::Vector3d(2.0, 0.0, 0.0));
    end_state_hover.velocity = Eigen::Vector3d::Zero();
    breaking_trajectory =
        trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
            end_state, end_state_hover, 4, 1.1 * circle_velocity, 15.0,
            max_roll_pitch_rate, exec_loop_rate);
    trajectory_generation_helper::heading::addConstantHeading(
        0.0, &breaking_trajectory);
    maneuver_list_.push_back(breaking_trajectory);
  }

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           breaking_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendRandomStraight(const double& velocity,
                                             const double& traj_sampling_freq) {
  printf("appending random straight\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = 0.0;
  const double figure_z_rotation_angle = 0.0;

  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // end state relative to start position
  double random_angle = 2 * M_PI * ((double)rand() / (RAND_MAX));
  double random_radius = 0.5 + 3.5 * ((double)rand() / (RAND_MAX));
  double random_z = -0.5 + ((double)rand() / (RAND_MAX));
  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(random_angle, Eigen::Vector3d::UnitZ()));

  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = init_state.position +
                       q_W_P * Eigen::Vector3d(random_radius, 0.0, random_z);

  const double max_thrust = 20.0;
  const double max_roll_pitch_rate = 3.0;

  quadrotor_common::Trajectory trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          init_state, end_state, 4,
          1.1 * std::max(init_state.velocity.norm(), velocity), max_thrust,
          max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &trajectory);

  maneuver_list_.push_back(trajectory);

  return !(trajectory.trajectory_type ==
           quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendStraight(const Eigen::Vector3d& end_position,
                                       const Eigen::Vector3d& end_velocity,
                                       const double& end_yaw,
                                       const double& max_velocity,
                                       const double& traj_sampling_freq) {
  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  const double exec_loop_rate = traj_sampling_freq;

  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = end_position;
  end_state.velocity = end_velocity;

  const double max_thrust = 20.0;
  const double max_roll_pitch_rate = 3.0;

  quadrotor_common::Trajectory trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          init_state, end_state, 4,
          1.1 * std::max(init_state.velocity.norm(), max_velocity), max_thrust,
          max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(0.0, &trajectory);

  maneuver_list_.push_back(trajectory);

  return !(trajectory.trajectory_type ==
           quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

void AcrobaticSequence::manualAddTrajectory(
    const quadrotor_common::Trajectory trajectory) {
  maneuver_list_.push_back(trajectory);
}

bool AcrobaticSequence::appendCorkScrew(
    const int n_loops, const double& circle_velocity, const double& radius,
    const Eigen::Vector3d& circle_center_offset,
    const Eigen::Vector3d& circle_center_offset_end, const bool break_at_end,
    const double& traj_sampling_freq) {
  printf("appending cork screw\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  printf(
      "Enter state of cork screw maneuver: Pos: %.2f, %.2f, %.2f | Vel: %.2f, "
      "%.2f, %.2f\n",
      init_state.position.x(), init_state.position.y(), init_state.position.z(),
      init_state.velocity.x(), init_state.velocity.y(),
      init_state.velocity.z());

  const double exec_loop_rate = traj_sampling_freq;
  const double circle_radius = radius;
  double corkscrew_velocity = 0.5;
  const double max_thrust =
      9.81 + 1.5 * pow(circle_velocity, 2.0) / circle_radius;
  const double max_roll_pitch_rate = 3.0;
  const double figure_z_rotation_angle = M_PI / 2.0;

  quadrotor_common::TrajectoryPoint circle_enter_state;
  circle_enter_state.position = init_state.position + circle_center_offset;
  circle_enter_state.velocity =
      Eigen::Vector3d(corkscrew_velocity, circle_velocity, 0.0);

  // compute enter trajectory
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          init_state, circle_enter_state, 4,
          1.5 * std::max(init_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(0.0,
                                                            &enter_trajectory);
  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      circle_enter_state.position + Eigen::Vector3d(0.0, 0.0, circle_radius);
  printf("circle center: %.2f, %.2f, %.2f\n", circle_center.x(),
         circle_center.y(), circle_center.z());

  // Compute Circle trajectory
  printf("compute cork screw trajectory\n");
  double rotate_loop = M_PI / 2.0;
  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::
          computeVerticalCircleTrajectoryCorkScrew(
              circle_center, rotate_loop, circle_radius, circle_velocity,
              M_PI / 2.0, -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI,
              corkscrew_velocity, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(0,
                                                            &circle_trajectory);

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));

  // End position RELATIVE to circle center
  printf("compute exit trajectory\n");

  //  const Eigen::Vector3d end_pos_P = Eigen::Vector3d(4.0, 0.0, 0.5); // nice
  //  breaking forward flip
  quadrotor_common::TrajectoryPoint end_state;
  //  end_state.position = q_W_P * end_pos_P + circle_center;
  //  end_state.velocity = q_W_P * Eigen::Vector3d(circle_velocity, 0.0, 0.0);
  //  end_state.position = init_state.position + Eigen::Vector3d(10.0, 0.0,
  //  0.0);
  end_state.position =
      circle_trajectory.points.back().position + circle_center_offset_end;
  end_state.velocity = Eigen::Vector3d(circle_velocity, 0.0, 0.0);

  quadrotor_common::TrajectoryPoint circle_exit_state =
      circle_trajectory.points.back();

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_exit_state, end_state, 4, 1.5 * circle_velocity, max_thrust,
          2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(0.0,
                                                            &exit_trajectory);

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(exit_trajectory);

  quadrotor_common::Trajectory breaking_trajectory;
  breaking_trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;
  if (break_at_end) {
    // append breaking trajectory at end
    quadrotor_common::TrajectoryPoint end_state_hover;
    end_state_hover.position =
        (end_state.position + Eigen::Vector3d(2.0, 0.0, 0.0));
    end_state_hover.velocity = Eigen::Vector3d::Zero();
    breaking_trajectory =
        trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
            end_state, end_state_hover, 4, 1.1 * circle_velocity, 15.0,
            max_roll_pitch_rate, exec_loop_rate);
    trajectory_generation_helper::heading::addConstantHeading(
        0.0, &breaking_trajectory);

    maneuver_list_.push_back(breaking_trajectory);
  }

  // Debug output
  std::cout << static_cast<int>(enter_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(circle_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(exit_trajectory.trajectory_type) << std::endl;

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           breaking_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendSplitS(const double& circle_velocity,
                                     const double& traj_sampling_freq) {
  printf("appending Split-S\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();
  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = 0.0;

  const double circle_radius = 0.75;
  const double figure_z_rotation_angle = 0.0;
  const double max_thrust = 30.0;
  const double max_roll_pitch_rate = 30.0;
  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // enter trajectory
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = init_state.position;
  start_state.velocity = init_state.velocity;
  quadrotor_common::TrajectoryPoint circle_enter_state;
  circle_enter_state.position =
      init_state.position + Eigen::Vector3d(4.0, 0.0, 0.0);
  circle_enter_state.velocity = Eigen::Vector3d(circle_velocity, 0.0, 0.0);
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4,
          1.5 * std::max(start_state.velocity.norm(), circle_velocity),
          0.5 * max_thrust, 0.5 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(desired_heading,
                                                            &enter_trajectory);

  // half-circle trajectory
  double start_angle = M_PI / 2.0;
  double end_angle = -M_PI / 2.0;
  const Eigen::Vector3d circle_center =
      circle_enter_state.position + Eigen::Vector3d(0.0, 0.0, circle_radius);

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, circle_radius,
          circle_velocity, start_angle, end_angle, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &circle_trajectory);

  // spin trajectory
  quadrotor_common::TrajectoryPoint circle_exit_state =
      circle_trajectory.points.back();
  double spin_time = 0.3;
  double init_accel_time = 0.0;
  double init_lin_acc = 20.0;
  double spin_angle = M_PI;
  double coast_acc = 2.5;  // thrust applied during the spin maneuver
  double final_hover_time = 0.0;
  quadrotor_common::Trajectory spin_trajectory =
      trajectory_generation_helper::flips::computeFlipTrajectory(
          circle_exit_state, init_accel_time, init_lin_acc, spin_angle,
          spin_time, coast_acc, final_hover_time, exec_loop_rate);

  printf("spin exit orientation: %.2f, %.2f, %.2f, %.2f | heading: %.2f \n",
         spin_trajectory.points.back().orientation.w(),
         spin_trajectory.points.back().orientation.x(),
         spin_trajectory.points.back().orientation.y(),
         spin_trajectory.points.back().orientation.z(),
         spin_trajectory.points.back().heading);

  // exit trajectory
  quadrotor_common::TrajectoryPoint spin_exit_state;
  spin_exit_state.position = spin_trajectory.points.back().position;
  spin_exit_state.velocity = spin_trajectory.points.back().velocity;
  spin_exit_state.orientation = spin_trajectory.points.back().orientation;

  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = init_state.position + Eigen::Vector3d(-4.0, 0.0, 0.0);
  end_state.orientation = quadrotor_common::eulerAnglesZYXToQuaternion(
      Eigen::Vector3d(0.0, 0.0, 0.0));
  end_state.heading = M_PI;

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          spin_exit_state, end_state, 4, spin_exit_state.velocity.norm(),
          max_thrust, max_roll_pitch_rate, exec_loop_rate);
  // after trajectory computation, the trajectory points do not contain any
  // orientation information yet! the functionality add heading will also fill
  // this information...
  trajectory_generation_helper::heading::addConstantHeading(M_PI,
                                                            &exit_trajectory);

  // Debug output
  std::cout << static_cast<int>(enter_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(circle_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(spin_trajectory.trajectory_type) << std::endl;
  std::cout << static_cast<int>(exit_trajectory.trajectory_type) << std::endl;

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(spin_trajectory);
  maneuver_list_.push_back(exit_trajectory);

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           spin_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendHover(const double& hover_time,
                                    const double& traj_sampling_freq) {
  printf("appending hover\n");

  // get start state
  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = 0.0;
  const double circle_velocity = 3.5;
  const double figure_z_rotation_angle = 0.0;
  const double max_thrust = 20.0;
  const double max_roll_pitch_rate = 3.0;

  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();
  double sampling_time = 1.0 / exec_loop_rate;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // compute braking trajectory
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = init_state.position + init_state.velocity * 1.0;

  quadrotor_common::Trajectory braking_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          init_state, end_state, 4, 1.1 * circle_velocity, max_thrust,
          max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &braking_trajectory);

  quadrotor_common::Trajectory hover_trajectory_end;
  hover_trajectory_end.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;
  for (double t = 0.0; t < hover_time; t += sampling_time) {
    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(t);
    point.position = end_state.position;
    point.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
    hover_trajectory_end.points.push_back(point);
  }
  maneuver_list_.push_back(braking_trajectory);
  maneuver_list_.push_back(hover_trajectory_end);
}

bool AcrobaticSequence::appendCrazyLoop(
    const int n_loops, const double& circle_velocity, const double& radius,
    const double& revolutions_enter, const double& revolutions_orbit,
    const double& revolutions_connect, const double& revolutions_loop,
    const double& revolutions_exit, const Eigen::Vector3d& circle_center_offset,
    const Eigen::Vector3d& circle_center_offset_end,
    const double& traj_sampling_freq) {
  printf("appending crazy loop\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  // hardcoded trajectory parameters
  const double exec_loop_rate = traj_sampling_freq;
  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity, 2.0) / radius;
  const double max_roll_pitch_rate = 3.0;
  const double figure_z_rotation_angle = 0.0;
  double current_heading = 0.0;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));

  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      init_state.position + q_W_P.inverse() * circle_center_offset;

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          circle_center, radius, circle_velocity, M_PI / 2.0,
          -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      revolutions_enter, revolutions_enter + revolutions_orbit,
      &circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();

  // Start position relative to circle center
  quadrotor_common::TrajectoryPoint start_state;
  start_state = init_state;

  // enter trajectory
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4,
          1.1 * std::max(start_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      0.0, revolutions_enter, &enter_trajectory);

  quadrotor_common::Trajectory loop_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, radius, circle_velocity,
          M_PI / 2.0, -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      revolutions_enter + revolutions_orbit + revolutions_connect,
      revolutions_enter + revolutions_orbit + revolutions_connect +
          revolutions_loop,
      &loop_trajectory);

  quadrotor_common::TrajectoryPoint loop_enter_state =
      loop_trajectory.points.front();

  quadrotor_common::Trajectory connect_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_enter_state, loop_enter_state, 4, 1.1 * circle_velocity,
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      revolutions_enter + revolutions_orbit,
      revolutions_enter + revolutions_orbit + revolutions_connect,
      &connect_trajectory);

  const Eigen::Vector3d end_pos_P = circle_center_offset_end;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = q_W_P * end_pos_P + circle_center;
  end_state.velocity = q_W_P * Eigen::Vector3d(circle_velocity, 0.0, 0.0);

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          loop_enter_state, end_state, 4, 1.1 * circle_velocity, max_thrust,
          2.0 * max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      revolutions_enter + revolutions_orbit + revolutions_connect +
          revolutions_loop,
      revolutions_enter + revolutions_orbit + revolutions_connect +
          revolutions_loop + revolutions_exit,
      &exit_trajectory);

  // append breaking trajectory at end
  quadrotor_common::TrajectoryPoint end_state_hover;
  end_state_hover.position =
      q_W_P * (end_pos_P + Eigen::Vector3d(2.0, 0.0, 0.0)) + circle_center;
  end_state_hover.velocity = Eigen::Vector3d::Zero();
  quadrotor_common::Trajectory breaking_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          end_state, end_state_hover, 4, 1.1 * circle_velocity, 15.0,
          max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      revolutions_enter + revolutions_orbit + revolutions_connect +
          revolutions_loop + revolutions_exit,
      &breaking_trajectory);

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(connect_trajectory);
  maneuver_list_.push_back(loop_trajectory);
  maneuver_list_.push_back(exit_trajectory);
  maneuver_list_.push_back(breaking_trajectory);

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           connect_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           loop_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           breaking_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

bool AcrobaticSequence::appendMattyLoop(
    const int n_loops, const double& circle_velocity, const double& radius,
    const Eigen::Vector3d& circle_center_offset,
    const Eigen::Vector3d& circle_center_offset_end,
    const double& traj_sampling_freq) {
  printf("appending matty loop\n");

  // get start state
  quadrotor_common::TrajectoryPoint init_state =
      maneuver_list_.back().points.back();

  printf(
      "Enter state of loop maneuver: Pos: %.2f, %.2f, %.2f | Vel: %.2f, %.2f, "
      "%.2f\n",
      init_state.position.x(), init_state.position.y(), init_state.position.z(),
      init_state.velocity.x(), init_state.velocity.y(),
      init_state.velocity.z());

  const double exec_loop_rate = traj_sampling_freq;
  const double desired_heading = M_PI;

  const double figure_z_rotation_angle = 0.0;
  //  const double figure_z_rotation_angle = 0.785398163;

  const Eigen::Quaterniond q_W_P = Eigen::Quaterniond(
      Eigen::AngleAxisd(figure_z_rotation_angle, Eigen::Vector3d::UnitZ()));
  double desired_heading_loop = quadrotor_common::wrapMinusPiToPi(
      desired_heading + figure_z_rotation_angle);

  // cirlce center RELATIVE to start position
  const Eigen::Vector3d circle_center =
      init_state.position + q_W_P.inverse() * circle_center_offset;

  printf("circle center: %.2f, %.2f, %.2f\n", circle_center.x(),
         circle_center.y(), circle_center.z());

  const double max_thrust = 9.81 + 1.5 * pow(circle_velocity, 2.0) / radius;
  const double max_roll_pitch_rate = 3.0;

  // Compute Circle trajectory
  printf("compute circle trajectory\n");

  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          circle_center, figure_z_rotation_angle, radius, circle_velocity,
          M_PI / 2.0, -(3.0 / 2.0 + 2 * (n_loops - 1)) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      desired_heading_loop, &circle_trajectory);

  quadrotor_common::TrajectoryPoint circle_enter_state =
      circle_trajectory.points.front();

  // Start position relative to circle center
  quadrotor_common::TrajectoryPoint start_state;
  start_state = init_state;

  // enter trajectory
  printf("compute enter trajectory\n");
  //  printf("Maximum speed: %.3f, current speed: %.3f\n", 1.1*circle_velocity,
  //  start_state.velocity.norm());
  quadrotor_common::Trajectory enter_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, circle_enter_state, 4,
          1.1 * std::max(start_state.velocity.norm(), circle_velocity),
          max_thrust, 2.0 * max_roll_pitch_rate, exec_loop_rate);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      0.0, M_PI, &enter_trajectory);
  //  trajectory_generation_helper::heading::addConstantHeading(desired_heading_loop,
  //                                                            &enter_trajectory);

  // End position RELATIVE to circle center
  printf("compute exit trajectory\n");

  const Eigen::Vector3d end_pos_P =
      circle_center_offset_end;  // Eigen::Vector3d(circle_center_offset.x(),
  // scircle_center_offset.y(),-circle_center_offset.z()); // nice breaking
  // forward flip
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = q_W_P * end_pos_P + circle_center;
  end_state.velocity = q_W_P * Eigen::Vector3d(circle_velocity, 0.0, 0.0);

  quadrotor_common::Trajectory exit_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          circle_enter_state, end_state, 4, 1.1 * circle_velocity, max_thrust,
          2.0 * max_roll_pitch_rate, exec_loop_rate);
  //  trajectory_generation_helper::heading::addConstantHeading(desired_heading_loop,
  //                                                            &exit_trajectory);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      M_PI, 2 * M_PI, &exit_trajectory);

  // append breaking trajectory at end
  quadrotor_common::TrajectoryPoint end_state_hover;
  end_state_hover.position =
      (end_state.position + Eigen::Vector3d(2.0, 0.0, 0.0));
  end_state_hover.velocity = Eigen::Vector3d::Zero();
  quadrotor_common::Trajectory breaking_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          end_state, end_state_hover, 4, 1.1 * circle_velocity, 15.0,
          max_roll_pitch_rate, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(
      0.0, &breaking_trajectory);

  maneuver_list_.push_back(enter_trajectory);
  maneuver_list_.push_back(circle_trajectory);
  maneuver_list_.push_back(exit_trajectory);
  maneuver_list_.push_back(breaking_trajectory);

  return !(enter_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           circle_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           exit_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED ||
           breaking_trajectory.trajectory_type ==
               quadrotor_common::Trajectory::TrajectoryType::UNDEFINED);
}

std::list<quadrotor_common::Trajectory> AcrobaticSequence::getManeuverList() {
  return maneuver_list_;
}

}  // namespace fpv_aggressive_trajectories
