#include "polynomial_trajectories/polynomial_trajectory.h"

namespace polynomial_trajectories {

PolynomialTrajectory::PolynomialTrajectory()
    : trajectory_type(TrajectoryType::UNDEFINED),
      coeff(),
      T(ros::Duration(0.0)),
      start_state(),
      end_state(),
      number_of_segments(0),
      segment_times(),
      optimization_cost(0.0) {}

PolynomialTrajectory::~PolynomialTrajectory() {}

}  // namespace polynomial_trajectories
