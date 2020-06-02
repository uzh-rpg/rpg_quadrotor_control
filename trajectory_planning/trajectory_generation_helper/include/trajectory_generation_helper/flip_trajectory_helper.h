#pragma once

#include <quadrotor_common/trajectory.h>
#include <Eigen/Dense>

namespace trajectory_generation_helper {

namespace flips {

quadrotor_common::Trajectory computeFlipTrajectory(
    const quadrotor_common::TrajectoryPoint start_point,
    const double init_accel_time, const double init_lin_acc,
    const double rotation_angle, const double flip_time, const double coast_acc,
    const double hover_time, const double sampling_frequency);

quadrotor_common::Trajectory computeCoastFlipTrajectory(
    const quadrotor_common::TrajectoryPoint start_point,
    const double rotation_angle, const double flip_time, const double coast_acc,
    const double sampling_frequency);

}  // namespace flips

}  // namespace trajectory_generation_helper
