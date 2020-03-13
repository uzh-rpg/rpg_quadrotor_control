#pragma once

#include <list>

#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>

namespace trajectory_visualizer {

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer();
  virtual ~TrajectoryVisualizer();

 private:
  ros::NodeHandle nh_;

  ros::Publisher marker_pub_gt_;
  ros::Publisher marker_pub_ref_;
  ros::Publisher marker_pub_se_;
  ros::Publisher odometry_ref_pub_;
  ros::Publisher bodyrates_pub_;
  ros::Subscriber autopilot_feedback_sub_;
  ros::Subscriber odometry_gt_sub_;

  bool loadParameters();
  void autopilotFeedbackCallback(
      const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg);
  void odometryGTCallback(const nav_msgs::OdometryConstPtr& msg);

  std::list<quadrotor_msgs::AutopilotFeedback> feedback_queue_;
  std::list<nav_msgs::Odometry> odometry_queue_;

  // Parameters
  double n_points_to_visualize_;
};

}  // namespace trajectory_visualizer
