#include "rpg_quadrotor_integration_test/rpg_quadrotor_integration_test.h"

#include <gtest/gtest.h>
#include <vector>

#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

namespace rpg_quadrotor_integration_test {

QuadrotorIntegrationTest::QuadrotorIntegrationTest()
    : executing_trajectory_(false),
      sum_position_error_squared_(0.0),
      max_position_error_(0.0),
      sum_thrust_direction_error_squared_(0.0),
      max_thrust_direction_error_(0.0) {
  ros::NodeHandle nh;

  arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);

  measure_tracking_timer_ =
      nh_.createTimer(ros::Duration(1.0 / kExecLoopRate_),
                      &QuadrotorIntegrationTest::measureTracking, this);
}

QuadrotorIntegrationTest::~QuadrotorIntegrationTest() {}

void QuadrotorIntegrationTest::measureTracking(const ros::TimerEvent& time) {
  if (executing_trajectory_) {
    // Position error
    const double position_error =
        autopilot_helper_.getCurrentPositionError().norm();
    sum_position_error_squared_ += pow(position_error, 2.0);
    if (position_error > max_position_error_) {
      max_position_error_ = position_error;
    }

    // Thrust direction error
    const Eigen::Vector3d ref_thrust_direction =
        autopilot_helper_.getCurrentReferenceOrientation() *
        Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d thrust_direction =
        autopilot_helper_.getCurrentOrientationEstimate() *
        Eigen::Vector3d::UnitZ();

    const double thrust_direction_error =
        acos(ref_thrust_direction.dot(thrust_direction));
    sum_thrust_direction_error_squared_ += pow(thrust_direction_error, 2.0);
    if (thrust_direction_error > max_thrust_direction_error_) {
      max_thrust_direction_error_ = thrust_direction_error;
    }
  }
}

void QuadrotorIntegrationTest::run() {
  ros::Rate command_rate(kExecLoopRate_);

  // Make sure everything is up and running
  // Wait for Autopilot feedback with assert
  ASSERT_TRUE(autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_))
      << "Did not receive autopilot feedback within 10 seconds.";

  ros::Duration(3.0).sleep();

  // Arm bridge
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);

  ///////////////
  // Check off command
  ///////////////

  // Takeoff
  autopilot_helper_.sendStart();

  // Wait for autopilot to go to start
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::START, 0.5, kExecLoopRate_))
      << "Autopilot did not switch to start after sending start command.";

  // Abort start and send off
  autopilot_helper_.sendOff();

  // Wait for autopilot to go to off
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::OFF, 0.1, kExecLoopRate_))
      << "Autopilot could not be forced to off during take off.";

  ///////////////
  // Check take off
  ///////////////

  // Takeoff for real
  autopilot_helper_.sendStart();

  // Wait for autopilot to go to hover
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_))
      << "Autopilot did not switch to hover after take off.";

  executing_trajectory_ = true;  // Start measuring errors

  ///////////////
  // Check velocity control
  ///////////////

  // Send velocity commands
  const Eigen::Vector3d vel_cmd = Eigen::Vector3d(1.0, -0.8, 0.3);
  const double heading_rate_cmd = -1.0;

  ros::Time start_sending_vel_cmds = ros::Time::now();
  while (ros::ok()) {
    autopilot_helper_.sendVelocityCommand(vel_cmd, heading_rate_cmd);
    if ((ros::Time::now() - start_sending_vel_cmds) > ros::Duration(2.0)) {
      EXPECT_TRUE((autopilot_helper_.getCurrentAutopilotState() ==
                   autopilot::States::VELOCITY_CONTROL))
          << "Autopilot did not switch to velocity control correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check go to pose
  ///////////////

  // Send pose command
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0.0, 0.0, 1.0);
  const double heading_cmd = 0.0;

  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);

  // Wait for autopilot to go to got to pose state
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_))
      << "Autopilot did not switch to trajectory control because of go to pose "
         "action correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  // Check if we are at the requested pose
  EXPECT_TRUE(
      (autopilot_helper_.getCurrentReferenceState().position - position_cmd)
          .norm() < 0.01)
      << "Go to pose action did not end up at the right position.";

  EXPECT_TRUE(autopilot_helper_.getCurrentReferenceHeading() - heading_cmd <
              0.01)
      << "Go to pose action did not end up at the right heading.";

  ///////////////
  // Check sending reference states
  ///////////////

  // Generate trajectory, sample it and send it as reference states
  const double max_vel = 2.0;
  const double max_thrust = 15.0;
  const double max_roll_pitch_rate = 0.5;

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = position_cmd;
  start_state.heading = 0.0;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = Eigen::Vector3d(1.5, 1.7, 1.2);
  end_state.heading = M_PI;

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, end_state, 5, max_vel, max_thrust, max_roll_pitch_rate,
          kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      start_state.heading, end_state.heading, &manual_traj);

  bool autopilot_was_in_reference_control_mode = false;
  while (ros::ok() && !manual_traj.points.empty()) {
    autopilot_helper_.sendReferenceState(manual_traj.points.front());
    manual_traj.points.pop_front();
    ros::spinOnce();
    if (!autopilot_was_in_reference_control_mode &&
        autopilot_helper_.getCurrentAutopilotState() ==
            autopilot::States::REFERENCE_CONTROL) {
      autopilot_was_in_reference_control_mode = true;
    }
    command_rate.sleep();
  }

  EXPECT_TRUE(autopilot_was_in_reference_control_mode)
      << "Autopilot did not switch to reference control correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 2.0, kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check trajectory control
  ///////////////

  // Generate trajectories and send them as complete trajectories
  // One polynomial to enter a ring and a ring to check execution of
  // consecutive trajectories

  // Ring trajectory with enter segment
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(-0.5, 0.0, 1.5));
  way_points.push_back(Eigen::Vector3d(1.5, -1.5, 0.6));
  way_points.push_back(Eigen::Vector3d(3.5, 0.0, 2.0));
  way_points.push_back(Eigen::Vector3d(1.5, 2.0, 0.6));

  Eigen::VectorXd initial_ring_segment_times =
      Eigen::VectorXd::Ones(int(way_points.size()));
  polynomial_trajectories::PolynomialTrajectorySettings
      ring_trajectory_settings;
  ring_trajectory_settings.continuity_order = 4;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
  ring_trajectory_settings.minimization_weights = minimization_weights;
  ring_trajectory_settings.polynomial_order = 11;
  ring_trajectory_settings.way_points = way_points;

  quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
      polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
          initial_ring_segment_times, ring_trajectory_settings, max_vel,
          max_thrust, max_roll_pitch_rate, kExecLoopRate_);

  polynomial_trajectories::PolynomialTrajectorySettings
      enter_trajectory_settings = ring_trajectory_settings;
  enter_trajectory_settings.way_points.clear();

  quadrotor_common::Trajectory enter_traj =
      trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
          Eigen::VectorXd::Ones(1), end_state, ring_traj.points.front(),
          enter_trajectory_settings, 1.03 * max_vel, 1.03 * max_thrust,
          max_roll_pitch_rate, kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      end_state.heading, 0.0, &enter_traj);
  trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                &ring_traj);

  autopilot_helper_.sendTrajectory(enter_traj);
  autopilot_helper_.sendTrajectory(ring_traj);

  // Check if autopilot goes to trajectory control state
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check enforcing hover
  ///////////////

  // Before trajectory finishes force autopilot to hover
  while (autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration() >
         ros::Duration(1.5)) {
    ros::spinOnce();
    command_rate.sleep();
  }
  executing_trajectory_ = false;  // Stop measuring errors

  autopilot_helper_.sendForceHover();

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 1.0, kExecLoopRate_))
      << "Autopilot did not switch to hover after being forced to hover.";

  ///////////////
  // Check landing
  ///////////////

  // Land
  autopilot_helper_.sendLand();

  // Wait for autopilot to go to land
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::LAND, 1.0, kExecLoopRate_))
      << "Autopilot did not switch to land after sending land command within "
         "timeout.";

  // Wait for autopilot to go to off
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::OFF, 10.0, kExecLoopRate_))
      << "Autopilot did not switch to off after landing within timeout.";

  ///////////////
  // Check sending control commands
  ///////////////

  ros::Duration(0.2).sleep();

  // Send control command to spin motors
  quadrotor_common::ControlCommand control_command;
  control_command.armed = true;
  control_command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  control_command.collective_thrust = 3.0;

  ros::Time start_sending_cont_cmds = ros::Time::now();
  while (ros::ok()) {
    autopilot_helper_.sendControlCommandInput(control_command);
    if ((ros::Time::now() - start_sending_cont_cmds) > ros::Duration(0.5)) {
      EXPECT_TRUE((autopilot_helper_.getCurrentAutopilotState() ==
                   autopilot::States::COMMAND_FEEDTHROUGH))
          << "Autopilot did not switch to command feedthrough correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }

  autopilot_helper_.sendOff();

  // Check tracking performance
  EXPECT_LT(max_position_error_, 0.15)
      << "Max position error (||est - ref||) from autopilot too large";
  EXPECT_LT(sum_position_error_squared_, 2.0)
      << "Sum of position errors (||est - ref||^2) squared over all received "
         "feedback messages from the autopilot too large";
  EXPECT_LT(max_thrust_direction_error_, 0.25)
      << "Max thrust direction error (acos(des.dot(est))) from autopilot "
         "too large";
  EXPECT_LT(sum_thrust_direction_error_squared_, 15.0)
      << "Sum of thrust direction (acos(des.dot(est))^2) squared over all "
         "received feedback messages from the autopilot too large";
}

TEST(QuadrotorIntegrationTest, AutopilotFunctionality) {
  QuadrotorIntegrationTest rpg_quadrotor_integration_test;
  rpg_quadrotor_integration_test.run();
}

}  // namespace rpg_quadrotor_integration_test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rpg_quadrotor_integration_test");

  return RUN_ALL_TESTS();
}
