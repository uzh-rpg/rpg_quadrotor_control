#pragma once

#include <quadrotor_common/trajectory.h>
#include <list>

namespace fpv_aggressive_trajectories {
class AcrobaticSequence {
 public:
  explicit AcrobaticSequence(
      const quadrotor_common::TrajectoryPoint& start_state);

  virtual ~AcrobaticSequence();

  bool appendStraight(const Eigen::Vector3d& end_position,
                      const Eigen::Vector3d& end_velocity,
                      const double& end_yaw, const double& max_velocity,
                      const double& traj_sampling_freq,
                      const bool& minimum_snap = true);

  std::list<quadrotor_common::Trajectory> getManeuverList();

 private:
  void computeLimits();

  std::list<quadrotor_common::Trajectory> maneuver_list_;
};
}  // namespace fpv_aggressive_trajectories
