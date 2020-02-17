#pragma once

#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include "position_controller/position_controller_params.h"

namespace position_controller {

class PositionController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PositionController();
  ~PositionController();

  quadrotor_common::ControlCommand off();
  quadrotor_common::ControlCommand run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::Trajectory& reference_trajectory,
      const PositionControllerParams& config);

 private:
  quadrotor_common::ControlCommand computeNominalReferenceInputs(
      const quadrotor_common::TrajectoryPoint& reference_state,
      const Eigen::Quaterniond& attitude_estimate) const;

  void computeAeroCompensatedReferenceInputs(
      const quadrotor_common::TrajectoryPoint& reference_state,
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const PositionControllerParams& config,
      quadrotor_common::ControlCommand* reference_inputs,
      Eigen::Vector3d* drag_accelerations) const;

  Eigen::Vector3d computePIDErrorAcc(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::TrajectoryPoint& reference_state,
      const PositionControllerParams& config) const;

  double computeDesiredCollectiveMassNormalizedThrust(
      const Eigen::Quaterniond& attitude_estimate,
      const Eigen::Vector3d& desired_acc,
      const PositionControllerParams& config) const;

  Eigen::Quaterniond computeDesiredAttitude(
      const Eigen::Vector3d& desired_acceleration, const double reference_yaw,
      const Eigen::Quaterniond& attitude_estimate) const;
  Eigen::Vector3d computeRobustBodyXAxis(
      const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
      const Eigen::Vector3d& y_C,
      const Eigen::Quaterniond& attitude_estimate) const;

  Eigen::Vector3d computeFeedBackControlBodyrates(
      const Eigen::Quaterniond& desired_attitude,
      const Eigen::Quaterniond& attitude_estimate,
      const PositionControllerParams& config) const;

  bool almostZero(const double value) const;
  bool almostZeroThrust(const double thrust_value) const;

  // Constants
  static constexpr double kMinNormalizedCollectiveThrust_ = 1.0;
  static constexpr double kAlmostZeroValueThreshold_ = 0.001;
  static constexpr double kAlmostZeroThrustThreshold_ = 0.01;

  const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
};

}  // namespace position_controller
