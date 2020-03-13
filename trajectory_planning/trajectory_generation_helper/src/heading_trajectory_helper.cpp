#include "trajectory_generation_helper/heading_trajectory_helper.h"

#include <quadrotor_common/math_common.h>

namespace trajectory_generation_helper {

namespace heading {

void addConstantHeading(const double heading,
                        quadrotor_common::Trajectory* trajectory) {
  for (auto point : trajectory->points) {
    point.heading = heading;
    point.heading_rate = 0.0;
    point.heading_acceleration = 0.0;

    double time_step = 0.02;
    Eigen::Vector3d acc1 = point.acceleration;
    Eigen::Vector3d acc2 =
        point.acceleration +
        time_step *
            point.jerk;  // should be acceleration at next trajectory point

    acc1.normalize();
    acc2.normalize();

    Eigen::Vector3d crossProd =
        acc1.cross(acc2);  // direction of omega, in inertial axes
    Eigen::Vector3d bodyrates_wf = Eigen::Vector3d(0, 0, 0);
    if (crossProd.norm()) {
      bodyrates_wf = acos(acc1.dot(acc2)) / time_step) * crossProd.normalize();
    }
    // rotate bodyrates to bodyframe this is only valid if heading is zero!!!
    point.bodyrates = bodyrates_wf * point.orientation;
  }
}

void addConstantHeadingRate(const double initial_heading,
                            const double final_heading,
                            quadrotor_common::Trajectory* trajectory) {
  if (trajectory->points.size() < 2) {
    return;
  }
  const double delta_angle =
      quadrotor_common::wrapAngleDifference(initial_heading, final_heading);
  const double trajectory_duration =
      (trajectory->points.back().time_from_start -
       trajectory->points.front().time_from_start)
          .toSec();

  const double heading_rate = delta_angle / trajectory_duration;

  std::list<quadrotor_common::TrajectoryPoint>::iterator it;
  for (it = trajectory->points.begin(); it != trajectory->points.end(); it++) {
    const double duration_ratio =
        (it->time_from_start - trajectory->points.front().time_from_start)
            .toSec() /
        trajectory_duration;
    it->heading = initial_heading + duration_ratio * delta_angle;
    it->heading_rate = heading_rate;
    it->heading_acceleration = 0.0;
  }
}

}  // namespace heading

}  // namespace trajectory_generation_helper
