#include "rpg_common/test_main.h"

#include "trajectory_generation_helper/circle_trajectory_helper.h"
#include "trajectory_generation_helper/heading_trajectory_helper.h"

#include <cmath>

using Quaternion = Eigen::Quaterniond;
using Vector = Eigen::Vector3d;
using Scalar = double;

Quaternion quat_mult(const Quaternion& q, const Quaternion& p) {
  Scalar w, x, y, z;
  w = q.w() * p.w() - q.x() * p.x() - q.y() * p.y() - q.z() * p.z();
  x = q.x() * p.w() + q.w() * p.x() + q.y() * p.z() - q.z() * p.y();
  y = q.y() * p.w() + q.w() * p.y() + q.z() * p.x() - q.x() * p.z();
  z = q.z() * p.w() + q.w() * p.z() + q.x() * p.y() - q.y() * p.x();
  Quaternion result(w, x, y, z);

  return result;
}

bool checkVector3Error(const Vector& derivative,
                       const Vector& integration_start,
                       const Vector& integration_end,
                       const Scalar& integration_time,
                       const std::string& derivative_name,
                       const std::string& integral_name) {
  Vector numeric_derivative =
      (integration_end - integration_start) / integration_time;
  Vector abs_error = derivative - numeric_derivative;
  Vector rel_error = abs_error / derivative.norm();

  if (rel_error.norm() > 0.05 && abs_error.norm() > 0.2) {
    std::printf("Vector<3> error\n");
    std::printf("==========================\n");
    std::printf("Integration time: %.4f\n", integration_time);
    std::printf("%s absolute error:  %.4f\n", derivative_name.c_str(),
                abs_error.norm());
    std::printf("%s relative error:  %.4f\n", derivative_name.c_str(),
                rel_error.norm());
    std::printf("%s start: %.4f, %.4f, %.4f\n", integral_name.c_str(),
                integration_start.x(), integration_start.y(),
                integration_start.z());
    std::printf("%s end: %.4f, %.4f, %.4f\n", integral_name.c_str(),
                integration_end.x(), integration_end.y(), integration_end.z());
    std::printf("%s analytical: %.4f, %.4f, %.4f\n", derivative_name.c_str(),
                derivative.x(), derivative.y(), derivative.z());
    std::printf("%s numeric: %.4f, %.4f, %.4f\n", derivative_name.c_str(),
                numeric_derivative.x(), numeric_derivative.y(),
                numeric_derivative.z());
    std::printf("==========================\n");
    return false;
  }
  return true;
}

bool checkBodyrateError(const Vector& bodyrates, const Quaternion& att_start,
                        const Quaternion& att_end,
                        const Scalar& integration_time,
                        const std::string& bodyrates_name,
                        const std::string& orientation_name) {
  Quaternion numeric_bodyrates_q = att_start.inverse() * att_end;
  Vector numeric_bodyrates =
      2.0 / integration_time *
      Vector(numeric_bodyrates_q.x(), numeric_bodyrates_q.y(),
             numeric_bodyrates_q.z());
  // compare with expected bodyrates
  Vector abs_error = bodyrates - numeric_bodyrates;

  // time derivative of the quaternion
  Eigen::Vector4d quat_der_vec =
      (att_end.coeffs() - att_start.coeffs()) / integration_time;
  Quaternion quat_der(quat_der_vec[3], quat_der_vec[0], quat_der_vec[1],
                      quat_der_vec[2]);

  std::cout << "quat_der: " << quat_der.coeffs().transpose()
            << " | norm: " << quat_der.coeffs().norm() << std::endl;

  Eigen::Vector4d numeric_bodyrates_2 =
      2.0 * (att_end.inverse() * quat_der).coeffs();
  std::cout << "numeric_bodyrates_2: " << numeric_bodyrates_2.transpose()
            << std::endl;
  // let's go completely freestyle: would the integrated bodyrates result in the
  // expected end attitude?
  Quaternion q_incr =
      Quaternion(Eigen::AngleAxisd(numeric_bodyrates.norm() * integration_time,
                                   numeric_bodyrates.normalized()));

  Quaternion q_incr_analytic = Quaternion(Eigen::AngleAxisd(
      bodyrates.norm() * integration_time, bodyrates.normalized()));

  Quaternion q_end_numeric = att_start * q_incr;
  Quaternion q_end_analytic = att_start * q_incr_analytic;
  std::cout << "att_end_gt: " << att_end.coeffs().transpose() << std::endl;
  std::cout << "att_end_nu: " << q_end_numeric.coeffs().transpose()
            << std::endl;
  std::cout << "att_end_an: " << q_end_analytic.coeffs().transpose()
            << std::endl;

  Vector rel_error = abs_error / bodyrates.norm();
  if (rel_error.norm() > 0.1 && abs_error.norm() > 0.01) {
    std::printf("Bodyrate error\n");
    std::printf("==========================\n");
    std::printf("Integration time: %.4f\n", integration_time);
    std::printf("%s absolute error:  %.4f\n", bodyrates_name.c_str(),
                abs_error.norm());
    std::printf("%s relative error:  %.4f\n", bodyrates_name.c_str(),
                rel_error.norm());
    std::printf("%s start: %.4f, %.4f, %.4f, %.4f\n", orientation_name.c_str(),
                att_start.w(), att_start.x(), att_start.y(), att_start.z());
    std::printf("%s end: %.4f, %.4f, %.4f, %.4f\n", orientation_name.c_str(),
                att_end.w(), att_end.x(), att_end.y(), att_end.z());
    std::printf("%s analytical: %.4f, %.4f, %.4f\n", bodyrates_name.c_str(),
                bodyrates.x(), bodyrates.y(), bodyrates.z());
    std::printf("%s numeric: %.4f, %.4f, %.4f\n", bodyrates_name.c_str(),
                numeric_bodyrates.x(), numeric_bodyrates.y(),
                numeric_bodyrates.z());
    // print body-x-axis
    Vector body_x_world = att_start * Vector::UnitX();
    std::printf("body_x_world: %.4f, %.4f, %.4f\n", body_x_world.x(),
                body_x_world.y(), body_x_world.z());

    std::printf("==========================\n");
    return false;
  }
  return true;
}

bool checkTrajectoryLimits(
    const std::vector<quadrotor_common::TrajectoryPoint>& setpoints) {
  Vector GVEC(0.0, 0.0, -9.81);
  Scalar max_thrust = 0.0;
  Scalar max_acceleration = 0.0;
  Scalar max_velocity = 0.0;
  Scalar max_bodyrate_x = 0.0;
  Scalar max_bodyrate_y = 0.0;
  Scalar max_bodyrate_z = 0.0;
  Scalar duration = 0.0;
  int counter = 0;
  int non_smooth_points = 0;
  int traj_spec_counter = 0;
  int traj_spec_non_smooth_points = 0;

  Scalar prev_time_from_start = setpoints.front().time_from_start.toSec();

  Vector prev_position = setpoints.front().position;
  Vector prev_velocity = setpoints.front().velocity;
  Vector prev_acceleration = setpoints.front().acceleration;
  Quaternion prev_orientation = setpoints.front().orientation;
  Vector prev_bodyrates = setpoints.front().bodyrates;

  int points_in_trajectory = setpoints.size();
  for (auto setpoint : setpoints) {
    // check trajectory limits
    const Scalar thrust = (setpoint.acceleration + GVEC).norm();
    max_thrust = std::max(max_thrust, thrust);
    max_acceleration = std::max(max_acceleration, setpoint.acceleration.norm());
    max_velocity = std::max(max_velocity, setpoint.velocity.norm());
    max_bodyrate_x = std::max(max_bodyrate_x, setpoint.bodyrates.x());
    max_bodyrate_y = std::max(max_bodyrate_y, setpoint.bodyrates.y());
    max_bodyrate_z = std::max(max_bodyrate_z, setpoint.bodyrates.z());

    // check trajectory integrity
    if (counter > 0) {
      Scalar integration_time =
          setpoint.time_from_start.toSec() - prev_time_from_start;
      duration += integration_time;
      if (!checkVector3Error(setpoint.velocity, prev_position,
                             setpoint.position, integration_time, "Velocity",
                             "Position") ||
          !checkVector3Error(setpoint.acceleration, prev_velocity,
                             setpoint.velocity, integration_time,
                             "Acceleration", "Velocity") ||
          !checkBodyrateError(setpoint.bodyrates, prev_orientation,
                              setpoint.orientation, integration_time,
                              "Bodyrates", "Orientation")) {
        std::printf("Found discontinuity at point [%d / %d].\n",
                    traj_spec_counter, points_in_trajectory);
        Vector prev_thrust = prev_acceleration - GVEC;
        Vector curr_thrust = setpoint.acceleration - GVEC;
        Vector crossprod = prev_thrust.cross(curr_thrust).normalized();
        Vector crossprod_body = setpoint.orientation.inverse() * crossprod;
        std::printf("Prev thrust: %.2f, %.2f, %.2f\n", prev_thrust.x(),
                    prev_thrust.y(), prev_thrust.z());
        std::printf("Curr thrust: %.2f, %.2f, %.2f\n", curr_thrust.x(),
                    curr_thrust.y(), curr_thrust.z());
        std::printf("crossprod: %.2f, %.2f, %.2f\n", crossprod.x(),
                    crossprod.y(), crossprod.z());
        std::printf("crossprod_body: %.2f, %.2f, %.2f\n", crossprod_body.x(),
                    crossprod_body.y(), crossprod_body.z());

        traj_spec_non_smooth_points += 1;
      }
    }

    prev_time_from_start = setpoint.time_from_start.toSec();

    prev_position = setpoint.position;
    prev_velocity = setpoint.velocity;
    prev_acceleration = setpoint.acceleration;
    prev_orientation = setpoint.orientation;
    prev_bodyrates = setpoint.bodyrates;
    counter += 1;
    traj_spec_counter += 1;
  }
  std::printf("Found [%d] non-smooth points in trajectory segment.\n",
              traj_spec_non_smooth_points);
  non_smooth_points += traj_spec_non_smooth_points;
  std::printf("Checked trajectory limits:\n");
  std::printf("%d / %d points have non-smooth derivatives.\n",
              non_smooth_points, counter);
  std::printf("Max nominal thrust: %.2f\n", max_thrust);
  std::printf("Max nominal acceleration: %.2f\n", max_acceleration);
  std::printf("Max nominal velocity: %.2f\n", max_velocity);
  std::printf("Max nominal bodyrates: %.2f, %.2f, %.2f\n", max_bodyrate_x,
              max_bodyrate_y, max_bodyrate_z);
  std::printf("Maneuver duration: %.2f\n", duration);

  return (traj_spec_non_smooth_points == 0);
}

TEST(MathChecks, NaN) {
  EXPECT_TRUE(std::isfinite(0.0));
  EXPECT_FALSE(std::isfinite(NAN));
  EXPECT_FALSE(std::isfinite(INFINITY));
}

TEST(HeadingCheck, YawRate) {
  ros::Time::init();
  // Compute Circle trajectory
  printf("compute circle trajectory\n");
  int n_loops = 1;
  double exec_loop_rate = 500.0;
  double circle_velocity = 5.0;
  double radius = 2.0;
  Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
  quadrotor_common::Trajectory circle_trajectory =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          circle_center, radius, circle_velocity, M_PI_2,
          -(0.5 + 2 * n_loops) * M_PI, exec_loop_rate);
  trajectory_generation_helper::heading::addConstantHeading(0.0,
                                                            &circle_trajectory);

  std::vector<quadrotor_common::TrajectoryPoint> sampled_trajectory;
  for (auto point : circle_trajectory.points) {
    sampled_trajectory.push_back(point);
    //    std::printf("bodyrates: %.2f, %.2f, %.2f\n", point.bodyrates.x(),
    //                point.bodyrates.y(), point.bodyrates.z());
  }
  checkTrajectoryLimits(sampled_trajectory);
}

RPG_COMMON_TEST