#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>
#include <thread>

#include "quad_msgs/ControlCommand.h"
#include "quad_msgs/OnboardStatus.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_msgs/QuadGpioPwmCtrl.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"

#include "quad_common/math_common.h"
#include "quad_common/geometry_eigen_conversions.h"

#include "ctrl_bridge/serial.hpp"

namespace ctrl_bridge {

class CtrlBridge
{
public:
  CtrlBridge(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  CtrlBridge() : CtrlBridge(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~CtrlBridge();

private:
  bool loadParams();
  void publishData(const uint64_t time, const msgs::upstream_payload_t& data);
  void setControlActiveCallback(const std_msgs::BoolConstPtr &msg);
  void controlCommandCallback(const quad_msgs::ControlCommand& msg);
  void stateEstimateCallback(const quad_msgs::QuadStateEstimate& msg);
  void offCallback(const std_msgs::Empty&);

  void receive();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Publishers
  ros::Publisher imu_pub_;
  ros::Publisher ctrl_attitude_pub_;
  ros::Publisher ctrl_status_pub_;

  // Subscribers
  ros::Subscriber control_command_sub_;
  ros::Subscriber set_control_active_sub_;
  ros::Subscriber state_estimate_sub_;
  ros::Subscriber off_sub_;

  ros::Time start_time_;
  SerialPort serial_port_;
  std::string port_name_;
  int baud_rate_;
  bool control_active_;

  double mass_;
  Eigen::Quaterniond q_IB_{1, 0, 0, 0};

  std::thread receive_thread_;
  bool shutdown_{false};
};

} // namespace ctrl_bridge
