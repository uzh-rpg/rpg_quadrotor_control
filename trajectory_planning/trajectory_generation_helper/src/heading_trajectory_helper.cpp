#include "trajectory_generation_helper/heading_trajectory_helper.h"

#include <quadrotor_common/math_common.h>

namespace trajectory_generation_helper {

namespace heading {

void addConstantHeading(const double heading,
                        quadrotor_common::Trajectory* trajectory) {
  auto iterator(trajectory->points.begin());
  auto iterator_prev(trajectory->points.begin());
  iterator_prev = std::prev(iterator_prev);
  auto iterator_next(trajectory->points.begin());
  iterator_next = std::next(iterator_next);
  auto last_element = trajectory->points.end();
  last_element = std::prev(last_element);
  double time_step;

  for (int i = 0; i < trajectory->points.size(); i++) {
    // do orientation first, since bodyrate conversion will depend on it
    Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
    Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(
        I_eZ_I, iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81));

//    double adapted_heading = 0.0;
//    double angle_error = 0.0;
//    int iter_counter = 0;
//    Eigen::Vector3d x_body_world;
//    do {
//      //      std::printf("iter_counter: %d\n", iter_counter);
//      Eigen::Quaternion<double> q_heading_adapt =
//          Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
//              adapted_heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
//      Eigen::Quaterniond q_combined = quatDes * q_heading_adapt;
//      // TODO: set yaw such that the projection of the body-x axis on the
//      // world xy plane aligns with the world x-axis
//      // 1. compute angle between the two axes
//      x_body_world = q_combined * Eigen::Vector3d::UnitX();
//      x_body_world[2] = 0.0;  // project on xy-plane
//      x_body_world.normalize();
//      angle_error = std::acos(x_body_world.dot(Eigen::Vector3d::UnitX()));
//
//      //      std::printf("angle_error: %.5f\n", angle_error);
//      adapted_heading += 0.001;
//      iter_counter++;
//    } while (angle_error > 0.01);
//    std::printf(
//        "body heading of %.2f resulted in %.3f angle error (found solution in "
//        "%d steps)\n",
//        adapted_heading, angle_error, iter_counter);
//    std::printf("body_x_world: %.4f, %.4f, %.4f\n", x_body_world.x(),
//                x_body_world.y(), x_body_world.z());
    // set full orientation and heading to zero
    Eigen::Quaternion<double> q_heading =
        Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
            heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
//    Eigen::Quaternion<double> q_heading =
//        Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
//            adapted_heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
    Eigen::Quaternion<double> q_orientation = quatDes * q_heading;
    iterator->orientation = q_orientation;
    iterator->heading = 0.0;  // heading is now absorbed in orientation
    iterator->heading_rate = 0.0;
    iterator->heading_acceleration = 0.0;

    Eigen::Vector3d thrust_1;
    Eigen::Vector3d thrust_2;
    // catch case of first and last element
    if (i == 0) {
      thrust_1 = iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      time_step =
          (iterator_next->time_from_start - iterator->time_from_start).toSec();
      thrust_2 = iterator_next->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    } else if (i < trajectory->points.size() - 1) {
      thrust_1 = iterator_prev->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      time_step =
          (iterator_next->time_from_start - iterator_prev->time_from_start)
              .toSec();
      thrust_2 = iterator_next->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    } else {
      // at the last point, we extrapolate the acceleration
      thrust_1 = iterator_prev->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      thrust_2 = iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81) +
                 time_step / 2.0 * iterator->jerk;
    }

    thrust_1.normalize();
    thrust_2.normalize();

    Eigen::Vector3d crossProd =
        thrust_1.cross(thrust_2);  // direction of omega, in inertial axes
    Eigen::Vector3d angular_rates_wf = Eigen::Vector3d(0, 0, 0);
    if (crossProd.norm() > 0.0) {
      angular_rates_wf = std::acos(thrust_1.dot(thrust_2)) / time_step *
                         crossProd / crossProd.norm();
    }
    // rotate bodyrates to bodyframe
    iterator->bodyrates = q_orientation.inverse() * angular_rates_wf;

    iterator_prev++;
    iterator++;
    iterator_next++;
  }
}

void addForwardHeading(quadrotor_common::Trajectory* trajectory) {
  auto iterator(trajectory->points.begin());
  auto iterator_prev(trajectory->points.begin());
  iterator_prev = std::prev(iterator_prev);
  auto iterator_next(trajectory->points.begin());
  iterator_next = std::next(iterator_next);
  auto last_element = trajectory->points.end();
  last_element = std::prev(last_element);
  double time_step;

  for (int i = 0; i < trajectory->points.size(); i++) {
    // do orientation first, since bodyrate conversion will depend on it
    Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
    Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(
        I_eZ_I, iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81));

    double heading = std::atan2(iterator->velocity.y(), iterator->velocity.x());
    // set full orientation and heading to zero
    Eigen::Quaternion<double> q_heading = Eigen::Quaternion<double>(
        Eigen::AngleAxis<double>(heading, Eigen::Vector3d::UnitZ()));
    Eigen::Quaternion<double> q_orientation = quatDes * q_heading;
    iterator->orientation = q_orientation;
    iterator->heading = 0.0;  // heading is now absorbed in orientation
    iterator->heading_rate = 0.0;
    iterator->heading_acceleration = 0.0;

    Eigen::Vector3d thrust_1;
    Eigen::Vector3d thrust_2;
    // catch case of first and last element
    if (i == 0) {
      thrust_1 = iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      time_step =
          (iterator_next->time_from_start - iterator->time_from_start).toSec();
      thrust_2 = iterator_next->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    } else if (i < trajectory->points.size() - 1) {
      thrust_1 = iterator_prev->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      time_step =
          (iterator_next->time_from_start - iterator_prev->time_from_start)
              .toSec();
      thrust_2 = iterator_next->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    } else {
      // at the last point, we extrapolate the acceleration
      thrust_1 = iterator_prev->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
      thrust_2 = iterator->acceleration + Eigen::Vector3d(0.0, 0.0, 9.81) +
                 time_step / 2.0 * iterator->jerk;
    }

    thrust_1.normalize();
    thrust_2.normalize();

    Eigen::Vector3d crossProd =
        thrust_1.cross(thrust_2);  // direction of omega, in inertial axes
    Eigen::Vector3d angular_rates_wf = Eigen::Vector3d(0, 0, 0);
    if (crossProd.norm() > 0.0) {
      angular_rates_wf = std::acos(thrust_1.dot(thrust_2)) / time_step *
                         crossProd / crossProd.norm();
    }
    // rotate bodyrates to bodyframe
    iterator->bodyrates = q_orientation.inverse() * angular_rates_wf;

    iterator_prev++;
    iterator++;
    iterator_next++;
  }
}

void addConstantHeadingRate(const double initial_heading,
                            const double final_heading,
                            quadrotor_common::Trajectory* trajectory) {
  if (trajectory->points.size() < 2) {
    return;
  }
  const double delta_angle =
      final_heading -
      initial_heading;  // quadrotor_common::wrapAngleDifference(initial_heading,
                        // final_heading);
  const double trajectory_duration =
      (trajectory->points.back().time_from_start -
       trajectory->points.front().time_from_start)
          .toSec();
  const double heading_rate = delta_angle / trajectory_duration;
  const double delta_heading = delta_angle / trajectory->points.size();

  double heading = initial_heading;
  std::list<quadrotor_common::TrajectoryPoint>::iterator it;
  for (auto& point : trajectory->points) {
    // do orientation first, since bodyrate conversion will depend on it
    Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
    Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(
        I_eZ_I, point.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81));

    // set full orientation and heading to zero
    Eigen::Quaternion<double> q_heading =
        Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
            heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
    Eigen::Quaternion<double> q_orientation = quatDes * q_heading;
    point.orientation = q_orientation;
    point.heading = 0.0;  // heading is now absorbed in orientation
    point.heading_rate = 0.0;
    point.heading_acceleration = 0.0;

    heading += delta_heading;

    // since we know the full orientation at this point, we can compute the
    // feedforward bodyrates
    double time_step = 0.02;
    Eigen::Vector3d acc1 = point.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
    Eigen::Vector3d acc2 =
        point.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81) +
        time_step *
            point.jerk;  // should be acceleration at next trajectory point

    acc1.normalize();
    acc2.normalize();

    Eigen::Vector3d crossProd =
        acc1.cross(acc2);  // direction of omega, in inertial axes
    Eigen::Vector3d bodyrates_wf = Eigen::Vector3d(0, 0, 0);
    if (crossProd.norm() > 0.0) {
      bodyrates_wf =
          std::acos(acc1.dot(acc2)) / time_step * crossProd / crossProd.norm();
    }
    // rotate angular rates to bodyframe
    point.bodyrates = q_orientation.inverse() * bodyrates_wf;
  }
}

}  // namespace heading

}  // namespace trajectory_generation_helper
