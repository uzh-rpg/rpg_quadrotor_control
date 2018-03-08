#include "rpg_quadrotor_integration_test/rpg_quadrotor_integration_test.h"

#include <gtest/gtest.h>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

namespace rpg_quadrotor_integration_test
{

QuadrotorIntegrationTest::QuadrotorIntegrationTest() :
    autopilot_feedback_received_(false), executing_trajectory_(false),
    sum_position_error_squared_(0.0), max_position_error_(0.0),
    sum_thrust_direction_error_squared_(0.0), max_thrust_direction_error_(0.0)
{
  ros::NodeHandle nh;

  arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);
  start_pub_ = nh.advertise<std_msgs::Empty>("autopilot/start", 1);
  land_pub_ = nh.advertise<std_msgs::Empty>("autopilot/land", 1);
  off_pub_ = nh.advertise<std_msgs::Empty>("autopilot/off", 1);

  pose_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      "autopilot/pose_command", 1);
  velocity_command_pub_ = nh.advertise<geometry_msgs::TwistStamped>(
      "autopilot/velocity_command", 1);
  reference_state_pub_ = nh.advertise<quadrotor_msgs::TrajectoryPoint>(
      "autopilot/reference_state", 1);
  trajectory_pub_ = nh.advertise<quadrotor_msgs::Trajectory>(
      "autopilot/trajectory", 1);
  control_command_pub_ = nh.advertise<quadrotor_msgs::ControlCommand>(
      "autopilot/control_command_input", 1);
  force_hover_pub_ = nh.advertise<std_msgs::Empty>("autopilot/force_hover", 1);

  autopilot_feedback_sub_ = nh.subscribe(
      "autopilot/feedback", 1,
      &QuadrotorIntegrationTest::autopilotFeedbackCallback, this);
}

QuadrotorIntegrationTest::~QuadrotorIntegrationTest()
{
}

void QuadrotorIntegrationTest::autopilotFeedbackCallback(
    const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg)
{
  autopilot_feedback_ = *msg;

  if (!autopilot_feedback_received_)
  {
    autopilot_feedback_received_ = true;
  }

  if (executing_trajectory_)
  {
    // Position error
    const double position_error = (quadrotor_common::geometryToEigen(
        msg->reference_state.pose.position)
        - quadrotor_common::geometryToEigen(
            msg->state_estimate.pose.pose.position)).norm();
    sum_position_error_squared_ += pow(position_error, 2.0);
    if (position_error > max_position_error_)
    {
      max_position_error_ = position_error;
    }

    // Thrust direction error
    const Eigen::Vector3d des_thrust_direction =
        quadrotor_common::geometryToEigen(msg->reference_state.pose.orientation)
            * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d thrust_direction = quadrotor_common::geometryToEigen(
        msg->state_estimate.pose.pose.orientation) * Eigen::Vector3d::UnitZ();

    const double thrust_direction_error = acos(
        des_thrust_direction.dot(thrust_direction));
    sum_thrust_direction_error_squared_ += pow(thrust_direction_error, 2.0);
    if (thrust_direction_error > max_thrust_direction_error_)
    {
      max_thrust_direction_error_ = thrust_direction_error;
    }
  }
}

void QuadrotorIntegrationTest::run()
{
  ros::Rate command_rate(kExecLoopRate_);

  // Make sure everything is up and running
  // Wait for Autopilot feedback with assert
  ros::Time start_time = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_time) < ros::Duration(10.0))
  {
    ros::spinOnce();
    if (autopilot_feedback_received_)
    {
      break;
    }
    command_rate.sleep();
  }
  ASSERT_TRUE(autopilot_feedback_received_)<< "Did not receive autopilot feedback within 10 seconds.";

  ros::Duration(3.0).sleep();

  // Arm bridge
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);

  ///////////////
  // Check off command
  ///////////////

  // Takeoff
  start_pub_.publish(std_msgs::Empty());

  // Wait for autopilot to go to start
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::START, 0.5))
      << "Autopilot did not switch to start after sending start command.";

  // Abort start and send off
  off_pub_.publish(std_msgs::Empty());

  // Wait for autopilot to go to off
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::OFF, 0.1))
      << "Autopilot could not be forced to off during take off.";

  ///////////////
  // Check take off
  ///////////////

  // Takeoff for real
  start_pub_.publish(std_msgs::Empty());

  // Wait for autopilot to go to hover
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, 10.0))
      << "Autopilot did not switch to hover after take off.";

  executing_trajectory_ = true; // Start measuring errors

  ///////////////
  // Check velocity control
  ///////////////

  // Send velocity commands
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.twist.linear.x = 1.0;
  vel_cmd.twist.linear.y = -0.8;
  vel_cmd.twist.linear.z = 0.3;
  vel_cmd.twist.angular.z = -1.0;

  ros::Time start_sending_vel_cmds = ros::Time::now();
  while (ros::ok())
  {
    velocity_command_pub_.publish(vel_cmd);
    if ((ros::Time::now() - start_sending_vel_cmds) > ros::Duration(2.0))
    {
      EXPECT_TRUE(
          (autopilot_feedback_.autopilot_state
              == autopilot_feedback_.VELOCITY_CONTROL))
          << "Autopilot did not switch to velocity control correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, 10.0))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check go to pose
  ///////////////

  // Send pose command
  geometry_msgs::PoseStamped pose_cmd;
  pose_cmd.pose.position.x = 0.0;
  pose_cmd.pose.position.y = 0.0;
  pose_cmd.pose.position.z = 1.0;
  pose_cmd.pose.orientation.w = 1.0;

  pose_command_pub_.publish(pose_cmd);

  // Wait for autopilot to go to got to pose state
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::TRAJECTORY_CONTROL, 2.0))
      << "Autopilot did not switch to trajectory control because of go to pose action correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, 10.0))
      << "Autopilot did not switch back to hover correctly.";

  // Check if we are at the requested pose
  EXPECT_TRUE(
      (quadrotor_common::geometryToEigen(autopilot_feedback_.reference_state.pose.position) -
          quadrotor_common::geometryToEigen(pose_cmd.pose.position)).norm() < 0.01)
      << "Go to pose action did not end up at the right position.";

  EXPECT_TRUE(quadrotor_common::geometryToEigen(
          autopilot_feedback_.reference_state.pose.orientation).angularDistance(
          quadrotor_common::geometryToEigen(pose_cmd.pose.orientation)) < 0.01)
      << "Go to pose action did not end up at the right orientation.";

  ///////////////
  // Check sending reference states
  ///////////////

  // Generate trajectory, sample it and send it as reference states
  const double max_vel = 2.0;
  const double max_thrust = 15.0;
  const double max_roll_pitch_rate = 0.5;

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = quadrotor_common::geometryToEigen(
      pose_cmd.pose.position);
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
  while (ros::ok() && !manual_traj.points.empty())
  {
    reference_state_pub_.publish(manual_traj.points.front().toRosMessage());
    manual_traj.points.pop_front();
    ros::spinOnce();
    if (!autopilot_was_in_reference_control_mode
        && autopilot_feedback_.autopilot_state
            == quadrotor_msgs::AutopilotFeedback::REFERENCE_CONTROL)
    {
      autopilot_was_in_reference_control_mode = true;
    }
    command_rate.sleep();
  }

  EXPECT_TRUE(
      autopilot_was_in_reference_control_mode)
      << "Autopilot did not switch to reference control correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, 2.0))
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

  Eigen::VectorXd initial_ring_segment_times = Eigen::VectorXd::Ones(
      int(way_points.size()));
  polynomial_trajectories::PloynomialTrajectorySettings ring_trajectory_settings;
  ring_trajectory_settings.continuity_order = 4;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
  ring_trajectory_settings.minimization_weights = minimization_weights;
  ring_trajectory_settings.polynomial_order = 11;
  ring_trajectory_settings.way_points = way_points;

  quadrotor_common::Trajectory ring_traj =
      trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
          initial_ring_segment_times, ring_trajectory_settings, max_vel,
          max_thrust, max_roll_pitch_rate, kExecLoopRate_);

  polynomial_trajectories::PloynomialTrajectorySettings enter_trajectory_settings =
      ring_trajectory_settings;
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

  trajectory_pub_.publish(enter_traj.toRosMessage());
  trajectory_pub_.publish(ring_traj.toRosMessage());

  // Check if autopilot goes to trajectory control state
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::TRAJECTORY_CONTROL, 2.0))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check enforcing hover
  ///////////////

  // Before trajectory finishes force autopilot to hover
  while (autopilot_feedback_.trajectory_execution_left_duration
      > ros::Duration(1.5))
  {
    ros::spinOnce();
    command_rate.sleep();
  }
  executing_trajectory_ = false; // Stop measuring errors

  force_hover_pub_.publish(std_msgs::Empty());

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::HOVER, 1.0))
      << "Autopilot did not switch to hover after being forced to hover.";

  ///////////////
  // Check landing
  ///////////////

  // Land
  land_pub_.publish(std_msgs::Empty());

  // Wait for autopilot to go to land
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::LAND, 1.0))
      << "Autopilot did not switch to land after sending land command within "
          "timeout.";

  // Wait for autopilot to go to off
  EXPECT_TRUE(
      waitForAutopilotState(quadrotor_msgs::AutopilotFeedback::OFF, 10.0))
      << "Autopilot did not switch to off after landing within timeout.";

  ///////////////
  // Check sending control commands
  ///////////////

  ros::Duration(0.2).sleep();

  // Send control command to spin motors
  quadrotor_msgs::ControlCommand control_command;
  control_command.armed = true;
  control_command.control_mode = control_command.BODY_RATES;
  control_command.collective_thrust = 3.0;

  ros::Time start_sending_cont_cmds = ros::Time::now();
  while (ros::ok())
  {
    control_command_pub_.publish(control_command);
    if ((ros::Time::now() - start_sending_cont_cmds) > ros::Duration(0.5))
    {
      EXPECT_TRUE(
          (autopilot_feedback_.autopilot_state
              == autopilot_feedback_.COMMAND_FEEDTHROUGH))
          << "Autopilot did not switch to command feedthrough correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }

  off_pub_.publish(std_msgs::Empty());

  // Check tracking performance
  EXPECT_LT(sum_position_error_squared_, 1.0)
      << "Sum of position errors (||des - est||^2) squared over all received "
          "feedback messages from the flight controller too large";
  EXPECT_LT(max_position_error_, 0.15)
      << "Max position error (||des - est||) from flight controller too large";
  EXPECT_LT(sum_thrust_direction_error_squared_, 10.0)
      << "Sum of thrust direction (acos(des.dot(est))^2) squared over all "
          "received feedback messages from the flight controller too large";
  EXPECT_LT(max_thrust_direction_error_, 0.3)
      << "Max thrust direction error (acos(des.dot(est))) from flight "
          "controller too large";

}

bool QuadrotorIntegrationTest::waitForAutopilotState(const uint state,
                                                     const double timeout)
{
  ros::Time start_wait = ros::Time::now();
  ros::Rate loop_rate(kExecLoopRate_);
  while (ros::ok() && (ros::Time::now() - start_wait) <= ros::Duration(timeout))
  {
    ros::spinOnce();
    if (autopilot_feedback_.autopilot_state == state)
    {
      return true;
    }
    loop_rate.sleep();
  }

  return false;
}

TEST(QuadrotorIntegrationTest, AutopilotFunctionality)
{
  QuadrotorIntegrationTest rpg_quadrotor_integration_test;
  rpg_quadrotor_integration_test.run();
}

} // namespace rpg_quadrotor_integration_test

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rpg_quadrotor_integration_test");

  return RUN_ALL_TESTS();
}
