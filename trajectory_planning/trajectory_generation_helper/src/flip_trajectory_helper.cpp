#include "trajectory_generation_helper/flip_trajectory_helper.h"

#include <quadrotor_common/math_common.h>
#include <quadrotor_common/trajectory_point.h>
#include <fstream>

namespace trajectory_generation_helper {

namespace flips {

/// \brief generates a trajectory that flips the quadrotor around its own axis
/// \param start_point initial state of the maneuver
/// \param init_accel_time accelerate the quadrotor before starting the flip
/// maneuver \param init_lin_acc desired acceleration before flip maneuver
/// \param rotation_angle around x-axis of platform
/// \param flip_time requested time to perform the flip
/// \param coast_acc mass normalized thrust to be applied during the flip
/// \param hover_time optional hover time before and after the maneuver
/// \param sampling_frequency
/// \return
quadrotor_common::Trajectory computeFlipTrajectory(
    const quadrotor_common::TrajectoryPoint start_point,
    const double init_accel_time, const double init_lin_acc,
    const double rotation_angle, const double flip_time, const double coast_acc,
    const double hover_time, const double sampling_frequency) {
  quadrotor_common::Trajectory trajectory;
  trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  Eigen::Vector3d curr_acceleration = start_point.acceleration;
  Eigen::Vector3d curr_velocity = start_point.velocity;
  Eigen::Vector3d curr_position = start_point.position;
  Eigen::Vector3d curr_bodyrates = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Quaterniond q_orientation = start_point.orientation;
  Eigen::Vector3d euler_angles_start =
      quadrotor_common::quaternionToEulerAnglesZYX(start_point.orientation);
  double dt = 1.0 / sampling_frequency;

  /**************************************
   * accelerate upwards before flipping *
   **************************************/
  double accel_time = init_accel_time;
  double mass_normalized_thrust = init_lin_acc;
  double curr_spin_angle = 0.0;

  for (double dt_flip = 0.0; dt_flip < accel_time; dt_flip += dt) {
    // constant orientation
    Eigen::Vector3d euler_angles = euler_angles_start;
    // integrate velocity & position
    curr_acceleration =
        Eigen::Vector3d(0.0, 0.0, mass_normalized_thrust - 9.81);
    curr_velocity = curr_velocity + curr_acceleration * dt;
    curr_position = curr_position + curr_velocity * dt;

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(dt_flip);
    point.position = curr_position;
    point.velocity = curr_velocity;
    point.acceleration = curr_acceleration;
    point.jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    point.orientation =
        quadrotor_common::eulerAnglesZYXToQuaternion(euler_angles);
    point.bodyrates = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    trajectory.points.push_back(point);
  }

  /**************************************
   * compute angular acceleration needed to perform the desired spin angle in
   the desired time we split the maneuver in two parts, angular acceleration
   phase and a symmetric deceleration phase use constant acceleration model ->
   rot_angle/2 = 1/2 a * (t/2)^2, where 'a' is the angular acceleration and 't'
   is the desired total spin time
   **************************************/
  double angular_acceleration = rotation_angle / std::pow(flip_time / 2.0, 2.0);
  double curr_angular_acceleration = angular_acceleration;
  Eigen::Vector3d euler_angles;
  Eigen::Vector3d orientation_incr;
  Eigen::Quaterniond q_incr;
  bool inverted_angular_acceleration = false;
  for (double dt_flip = 0.0; dt_flip < flip_time; dt_flip += dt) {
    if (dt_flip > flip_time / 2.0 && inverted_angular_acceleration == false) {
      curr_angular_acceleration *= -1.0;
      inverted_angular_acceleration = true;
    }
    curr_bodyrates += Eigen::Vector3d(curr_angular_acceleration * dt, 0.0, 0.0);
    orientation_incr = Eigen::Vector3d(curr_bodyrates.x() * dt, 0.0, 0.0);
    q_incr = quadrotor_common::eulerAnglesZYXToQuaternion(orientation_incr);

    q_orientation *= q_incr;

    mass_normalized_thrust = coast_acc;
    curr_acceleration =
        q_orientation * Eigen::Vector3d(0.0, 0.0, mass_normalized_thrust) -
        Eigen::Vector3d(0.0, 0.0, 9.81);
    curr_velocity = curr_velocity + curr_acceleration * dt;
    curr_position = curr_position + curr_velocity * dt;

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(accel_time + dt_flip);
    point.position = curr_position;
    point.velocity = curr_velocity;
    point.acceleration = curr_acceleration;

    point.orientation = q_orientation;
    point.bodyrates = Eigen::Vector3d(angular_acceleration * dt_flip, 0.0, 0.0);
    point.angular_acceleration =
        Eigen::Vector3d(angular_acceleration, 0.0, 0.0);
    point.angular_jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    Eigen::Vector3d thrust =
        curr_acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    double inv_thrust = 1.0 / thrust.norm();
    point.jerk =
        (point.orientation.inverse() *
         (inv_thrust * Eigen::Matrix<double, 3, 3>::Identity() -
          thrust * thrust.transpose() / (inv_thrust * inv_thrust * inv_thrust)))
            .inverse() *
        Eigen::Vector3d(0.0, -point.bodyrates.x(), 0.0);
    point.snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    trajectory.points.push_back(point);
  }

  /********************************************
   * some hovering at the end of the maneuver *
   ********************************************/
  for (double dt_hover = 0.0; dt_hover < hover_time; dt_hover += dt) {
    // constant orientation
    Eigen::Vector3d euler_angles = Eigen::Vector3d(0.0, 0.0, 0.0);
    // integrate velocity & position
    curr_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    curr_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(dt_hover + accel_time + flip_time);
    point.position = curr_position;
    point.velocity = curr_velocity;
    point.acceleration = curr_acceleration;
    point.jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    point.orientation =
        quadrotor_common::eulerAnglesZYXToQuaternion(euler_angles);
    point.bodyrates = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    trajectory.points.push_back(point);
  }

  return trajectory;
}

quadrotor_common::Trajectory computeCoastFlipTrajectory(
    const quadrotor_common::TrajectoryPoint start_point,
    const double rotation_angle, const double maneuver_time,
    const double coast_acc, const double sampling_frequency) {
  quadrotor_common::Trajectory trajectory;
  trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  Eigen::Vector3d curr_acceleration = start_point.acceleration;
  Eigen::Vector3d curr_velocity = start_point.velocity;
  Eigen::Vector3d curr_position = start_point.position;
  Eigen::Vector3d curr_bodyrates =
      Eigen::Vector3d(0.0, 0.0, 0.0);  // start_point.bodyrates; //
  Eigen::Quaterniond q_orientation = start_point.orientation;
  Eigen::Vector3d euler_angles_start =
      quadrotor_common::quaternionToEulerAnglesZYX(start_point.orientation);
  double dt = 1.0 / sampling_frequency;

  /**************************************
   * rotate quad around body-y axis by desired rotation angle
   * compute angular acceleration needed to perform the desired spin angle in
   the desired time we split the maneuver in two parts, angular acceleration
   phase and a symmetric deceleration phase use constant acceleration model ->
   rot_angle/2 = 1/2 a * (t/2)^2, where 'a' is the angular acceleration and 't'
   is the desired total spin time
   **************************************/
  double angular_acceleration =
      rotation_angle / std::pow(maneuver_time / 2.0, 2.0);
  double curr_angular_acceleration = angular_acceleration;
  Eigen::Vector3d euler_angles;
  Eigen::Vector3d orientation_incr;
  Eigen::Quaterniond q_incr;
  double coast_acc1 = coast_acc;
  bool inverted_angular_acceleration = false;
  for (double dt_flip = 0.0; dt_flip < maneuver_time; dt_flip += dt) {
    if (dt_flip > maneuver_time / 2.0 &&
        inverted_angular_acceleration == false) {
      // after first half of maneuver, invert angular acceleration to break
      // rotation
      curr_angular_acceleration *= -1.0;
      inverted_angular_acceleration = true;
    }
    printf("angular acceleration: %.2f \n", curr_angular_acceleration);
    printf("body rates: %.2f, %.2f, %.2f \n", curr_bodyrates.x(),
           curr_bodyrates.y(), curr_bodyrates.z());
    curr_bodyrates += curr_angular_acceleration * dt * Eigen::Vector3d::UnitY();
    orientation_incr = curr_bodyrates.y() * dt * Eigen::Vector3d::UnitY();
    q_incr = quadrotor_common::eulerAnglesZYXToQuaternion(orientation_incr);
    q_orientation *= q_incr;

    //    mass_normalized_thrust = coast_acc;
    curr_acceleration = q_orientation * Eigen::Vector3d(0.0, 0.0, coast_acc1) -
                        Eigen::Vector3d(0.0, 0.0, 9.81);
    curr_velocity = curr_velocity + curr_acceleration * dt;
    curr_position = curr_position + curr_velocity * dt;

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(dt_flip);
    point.position = curr_position;
    point.velocity = curr_velocity;
    point.acceleration = curr_acceleration;

    point.orientation = q_orientation;
    point.bodyrates = angular_acceleration * dt_flip * Eigen::Vector3d::UnitY();
    point.angular_acceleration =
        angular_acceleration * Eigen::Vector3d::UnitY();
    point.angular_jerk = Eigen::Vector3d(0.0, 0.0, 0.0);
    point.angular_snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    Eigen::Vector3d thrust =
        curr_acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    double inv_thrust = 1.0 / thrust.norm();
    point.jerk =
        (point.orientation.inverse() *
         (inv_thrust * Eigen::Matrix<double, 3, 3>::Identity() -
          thrust * thrust.transpose() / (inv_thrust * inv_thrust * inv_thrust)))
            .inverse() *
        Eigen::Vector3d(0.0, -point.bodyrates.x(), 0.0);
    point.snap = Eigen::Vector3d(0.0, 0.0, 0.0);

    trajectory.points.push_back(point);
  }

  return trajectory;
}

}  // namespace flips

}  // namespace trajectory_generation_helper
