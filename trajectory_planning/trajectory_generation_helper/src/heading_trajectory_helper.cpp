#include "trajectory_generation_helper/heading_trajectory_helper.h"

#include <quadrotor_common/math_common.h>

namespace trajectory_generation_helper {

namespace heading {

void addConstantHeading(const double heading,
                        quadrotor_common::Trajectory* trajectory) {
  std::list<quadrotor_common::TrajectoryPoint>::iterator it;
  for (it = trajectory->points.begin(); it != trajectory->points.end(); it++) {
    it->heading = heading;
    it->heading_rate = 0.0;
    it->heading_acceleration = 0.0;
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
