//
// Created by elia on 27.09.19.
//

#pragma once

#include <quadrotor_common/trajectory.h>
#include <list>

namespace fpv_aggressive_trajectories {
class AcrobaticSequence {
 public:
  explicit AcrobaticSequence(
      const quadrotor_common::TrajectoryPoint& start_state);

  virtual ~AcrobaticSequence();

  bool appendLoops(const int n_loops, const double& circle_velocity,
                   const double& radius,
                   const Eigen::Vector3d& circle_center_offset,
                   const Eigen::Vector3d& circle_center_offset_end,
                   const bool break_at_end, const double& traj_sampling_freq);

  bool appendRandomStraight(const double& velocity,
                            const double& traj_sampling_freq);

  bool appendStraight(const Eigen::Vector3d& end_position,
                      const double& end_yaw, const double& max_velocity,
                      const double& traj_sampling_freq);

  bool appendCorkScrew(const int n_loops, const double& circle_velocity,
                       const double& radius,
                       const Eigen::Vector3d& circle_center_offset,
                       const Eigen::Vector3d& circle_center_offset_end,
                       const bool break_at_end,
                       const double& traj_sampling_freq);

  bool appendSplitS(const double& circle_velocity,
                    const double& traj_sampling_freq);

  bool appendLoopli(const int n_loops, const double& circle_velocity,
                    const double& radius, const double& traj_sampling_freq);

  bool appendHover(const double& hover_time, const double& traj_sampling_freq);

  bool appendCrazyLoop(const int n_loops, const double& circle_velocity,
                       const double& radius, const double& revolutions_enter,
                       const double& revolutions_orbit,
                       const double& revolutions_connect,
                       const double& revolutions_loop,
                       const double& revolutions_exit,
                       const Eigen::Vector3d& circle_center_offset,
                       const Eigen::Vector3d& circle_center_offset_end,
                       const double& traj_sampling_freq);

  bool appendMattyLoop(const int n_loops, const double& circle_velocity,
                       const double& radius,
                       const Eigen::Vector3d& circle_center_offset,
                       const Eigen::Vector3d& circle_center_offset_end,
                       const double& traj_sampling_freq);

  std::list<quadrotor_common::Trajectory> getManeuverList();

 private:
  void computeLimits();

  std::list<quadrotor_common::Trajectory> maneuver_list_;
};
}  // namespace fpv_aggressive_trajectories
