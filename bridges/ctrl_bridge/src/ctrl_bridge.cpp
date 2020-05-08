#include <ctrl_bridge/ctrl_bridge.hpp>

#include <Eigen/Dense>
#include <limits>
#include "ctrl_bridge/msg_encoding.hpp"
#include "ctrl_bridge/msgs_defs.hpp"
#include "quad_common/parameter_helper.h"

namespace ctrl_bridge {


CtrlBridge::CtrlBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), control_active_(false)
{
  // Fetch parameters
  ROS_INFO("[%s] Initializing...!", pnh_.getNamespace().c_str());
  if (!loadParams())
  {
    ROS_ERROR("[%s] Could not load parameters.",
      pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  start_time_ = ros::Time::now();

  // Publishers
  ROS_INFO("[%s] Announcing topics!", pnh_.getNamespace().c_str());
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  ctrl_attitude_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(
    "onboard_attitude_est", 1);
  ctrl_status_pub_ = nh_.advertise<quad_msgs::OnboardStatus>(
    "onboard_status", 1);

  // Subscribers
  ROS_INFO("[%s] Subscribing to topics!", pnh_.getNamespace().c_str());
  set_control_active_sub_ = nh_.subscribe(
    "control_active", 1,
    &CtrlBridge::setControlActiveCallback, this);
  off_sub_ = nh_.subscribe("copilot/off", 1, &CtrlBridge::offCallback, this);
  control_command_sub_ = nh_.subscribe(
    "control_command", 1, &CtrlBridge::controlCommandCallback, this);
  state_estimate_sub_ = nh_.subscribe("state_estimate", 1,
    &CtrlBridge::stateEstimateCallback, this);

  // Open serial port
  ROS_INFO("[%s] Creating Serial object!", pnh_.getNamespace().c_str());
  if (!serial_port_.start(port_name_, baud_rate_))
  {
    ROS_ERROR("[%s] Could not start serial port!", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("[%s] Starting Ctrl Bridge!", pnh_.getNamespace().c_str());
  receive_thread_ = std::thread(&CtrlBridge::receive, this);
  ROS_INFO("[%s] Started Ctrl Bridge!", pnh_.getNamespace().c_str());

  quad_msgs::ControlCommand init_msg;
  init_msg.control_mode = init_msg.NONE;
  init_msg.off = true;
  for(size_t i = 0; i<50; ++i)
  {
    init_msg.header.stamp = ros::Time::now();
    controlCommandCallback(init_msg);
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

}

CtrlBridge::~CtrlBridge()
{
  shutdown_ = true;
  if(serial_port_.isOpen())
    serial_port_.stop();
}

void CtrlBridge::receive()
{
  // double t_last = 0.0;
  while(!shutdown_)
  {
    char buffer[SerialPort::BUFFERSIZE];
    const size_t bytes = serial_port_.receive(buffer);
    if(!bytes) continue;

    msgs::serial_container_t container;
    
    if(SerialContainerDecode((uint8_t*)buffer, bytes, &container))
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Decoding error!",
        pnh_.getNamespace().c_str());
        continue;
    }

    for(size_t i = 0; i < container.num_messages; ++i)
    {
      if(container.messages[i].mode != msgs::CTRLMODE::FEEDBACK)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] Received unexpected message!",
          pnh_.getNamespace().c_str());
        continue;
      }

      msgs::upstream_payload_t data =
        *((msgs::upstream_payload_t*) container.messages[i].payload);
      uint64_t time_ns = container.messages[i].time;
      publishData(time_ns, data);
      ros::spinOnce();
    }
  }
}


void CtrlBridge::publishData(
  const uint64_t time_ns,
  const msgs::upstream_payload_t& data)
{
  if(data.control_mode >= msgs::CTRLMODE::ARM && !control_active_)
    ROS_INFO("[%s] Control active", pnh_.getNamespace().c_str());

  control_active_ = data.control_mode >= msgs::CTRLMODE::ARM &&
    data.control_mode <= msgs::CTRLMODE::EMERGENCY;

  sensor_msgs::Imu imu_msg;

  static ros::Time last = ros::Time(0, 0);

  const ros::Time time_ros = start_time_ +  
    ros::Duration(time_ns / 1000000000u, time_ns % 1000000000u);

  if(last.nsec >= 0 && last.sec >= 0 && (time_ros - last).toSec()>0.1)
    ROS_WARN("To old message %1.3fms", (time_ros - last).toSec()*1000);

  if(time_ros < last) return;
  last = time_ros;

  imu_msg.header.stamp = time_ros;
  imu_msg.angular_velocity.x = data.angular_vel_x;
  imu_msg.angular_velocity.y = data.angular_vel_y;
  imu_msg.angular_velocity.z = data.angular_vel_z;
  imu_msg.linear_acceleration.x = data.linear_accel_x;
  imu_msg.linear_acceleration.y = data.linear_accel_y;
  imu_msg.linear_acceleration.z = data.linear_accel_z;
  imu_pub_.publish(imu_msg);

  quad_msgs::OnboardStatus ctrl_status_msg;
  ctrl_status_msg.header.stamp = time_ros;
  switch(data.control_mode)
  {
    case msgs::CTRLMODE::BODY_RATE:
      ctrl_status_msg.control_mode = quad_msgs::OnboardStatus::RATE_MODE;
      break;
    case msgs::CTRLMODE::ATTITUDE:
      ctrl_status_msg.control_mode = quad_msgs::OnboardStatus::ATTITUDE_MODE;
      break;
    default:
      ctrl_status_msg.control_mode = 0u;
  }
  ctrl_status_msg.battery_voltage = data.battery_voltage;
  ctrl_status_pub_.publish(ctrl_status_msg);

  geometry_msgs::QuaternionStamped ctrl_attitude_msg;
  const Eigen::Quaterniond q =
    Eigen::Quaterniond(
      data.quaternion_w,
      data.quaternion_x,
      data.quaternion_y,
      data.quaternion_z).normalized();
  ctrl_attitude_msg.header.stamp = time_ros;
  ctrl_attitude_msg.quaternion.w = q.w();
  ctrl_attitude_msg.quaternion.x = q.x();
  ctrl_attitude_msg.quaternion.y = q.y();
  ctrl_attitude_msg.quaternion.z = q.z();
  ctrl_attitude_pub_.publish(ctrl_attitude_msg);
}

// Subscriber Callbacks
void CtrlBridge::stateEstimateCallback(const quad_msgs::QuadStateEstimate& msg)
{
  const Eigen::Quaterniond q(
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);

  if(q.norm() > 0.0)
    q_IB_ = q.normalized();
}

void CtrlBridge::offCallback(const std_msgs::Empty&)
{
  ROS_INFO("[%s] OFF Command", pnh_.getNamespace().c_str());
  const ros::Duration ros_time_since_start = ros::Time::now()  - start_time_;
  const uint64_t time_ns =
    ros_time_since_start.sec * 1e9 + ros_time_since_start.nsec;

  msgs::serial_container_t container;
  container.num_messages = 1u;

  msgs::serial_message_t& off_msg =  container.messages[0];
  off_msg.time = time_ns;
  off_msg.mode = msgs::CTRLMODE::OFF;

  char buffer[msgs::SERIAL_CONTAINER_MAX_LENGTH];
  uint8_t length = msgs::SERIAL_CONTAINER_MAX_LENGTH;

  if(SerialContainerEncode(&container, (uint8_t*)&buffer[1], &length))
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] Encoding error!",
      pnh_.getNamespace().c_str());
    return;
  }
  buffer[0] = msgs::SERIAL_CONTAINER_DELIMITER; length++;
  /* append at least 1 delimiter and pad to a multiple of 8 bytes in length */
  do
  {
    buffer[length++] = msgs::SERIAL_CONTAINER_DELIMITER;
  } while (length % 16 != 0);

  const size_t sent_bytes = serial_port_.send(buffer, length);
  if(sent_bytes != length)
    ROS_WARN("[%s] Error sending to serial port!",pnh_.getNamespace().c_str());
}

void CtrlBridge::controlCommandCallback(const quad_msgs::ControlCommand& msg)
{
  const ros::Duration ros_time_since_start = ros::Time::now() - start_time_;
  const uint64_t time_ns =
    ros_time_since_start.sec * 1e9 + ros_time_since_start.nsec;
  
  msgs::serial_container_t container;
  container.num_messages = 1u;

  msgs::serial_message_t& ctrl_msg =  container.messages[0];
  ctrl_msg.time = time_ns;
  
  static const double angle_scale =
    std::numeric_limits<int16_t>::max() / msgs::ANGLE_RANGE;
  static const double rate_scale =
    std::numeric_limits<int16_t>::max() / msgs::ANGULAR_VELOCITY_RANGE;
  static const double force_scale =
    std::numeric_limits<int16_t>::max() / msgs::FORCE_RANGE;

  if(!control_active_ && !(msg.control_mode == msg.NONE && msg.off))
  {
    // ROS_WARN_THROTTLE(1, "[%s] Control not activated!",
    //   pnh_.getNamespace().c_str());
    ctrl_msg.mode = msgs::CTRLMODE::INVALID;
    ROS_INFO("[%s] Sending invalid", pnh_.getNamespace().c_str());
    // return;
  }
  else if(msg.control_mode == msg.NONE && msg.off)
  {
    ctrl_msg.mode = msgs::CTRLMODE::OFF;
    ROS_INFO("[%s] Sending off", pnh_.getNamespace().c_str());
  }
  else if(msg.off)
  {
    ctrl_msg.mode = msgs::CTRLMODE::INVALID;
    ROS_INFO("[%s] Sending invalid", pnh_.getNamespace().c_str());
    // return;
  }
  else if(msg.control_mode == msg.ANGLE)
  {
    ctrl_msg.mode = msgs::CTRLMODE::ATTITUDE;
    msgs::attitude_payload_t attitude_payload;
    const Eigen::Quaterniond q_ID = Eigen::Quaterniond(
      msg.orientation.w,
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z).normalized();
    const Eigen::Quaterniond q_IBz =
      Eigen::Quaterniond(q_IB_.w(), 0, 0, q_IB_.z()).normalized();
    const Eigen::Quaterniond q_BzD = q_IBz.conjugate() * q_ID;
    const Eigen::Vector3d dir =
      q_BzD.toRotationMatrix() * Eigen::Vector3d::UnitZ();
    attitude_payload.attitude[0] = (int16_t)(dir.x() * angle_scale);
    attitude_payload.attitude[1] = (int16_t)(dir.y() * angle_scale);
    attitude_payload.attitude[2] = (int16_t)(dir.z() * angle_scale);
    attitude_payload.yaw_body_rate = (int16_t)(msg.bodyrates.z * rate_scale);
    attitude_payload.thrust = (int16_t)(msg.thrust * force_scale)*mass_;
    memcpy(
      ctrl_msg.payload,
      &attitude_payload,
      getPayloadLength(msgs::CTRLMODE::ATTITUDE));
    ROS_INFO("[%s] Sending attitude", pnh_.getNamespace().c_str());
    ROS_INFO("Direction: [%1.3f, %1.3f, %1.3f]", dir.x(), dir.y(), dir.z());
  }
  else if(msg.control_mode == msg.ANGLERATE)
  {
    ctrl_msg.mode = msgs::CTRLMODE::BODY_RATE;
    msgs::body_rate_payload_t body_rate_payload;
    body_rate_payload.body_rate[0] = (int16_t)(msg.bodyrates.x * rate_scale);
    body_rate_payload.body_rate[1] = (int16_t)(msg.bodyrates.y * rate_scale);
    body_rate_payload.body_rate[2] = (int16_t)(msg.bodyrates.z * rate_scale);
    body_rate_payload.thrust = (int16_t)(msg.thrust * force_scale)*mass_;
    memcpy(
      ctrl_msg.payload,
      &body_rate_payload,
      getPayloadLength(msgs::CTRLMODE::BODY_RATE));
    ROS_INFO("[%s] Sending bodyrate", pnh_.getNamespace().c_str());
  }
  else if(msg.control_mode == msg.ROTTHROTTLE)
  {
    ctrl_msg.mode = msgs::CTRLMODE::ROTOR_THROTTLE;
    msgs::rotor_throttle_payload_t rotor_throttle_payload;
    rotor_throttle_payload.throttle[0] = (int16_t)(msg.mot_throttle[0]);
    rotor_throttle_payload.throttle[1] = (int16_t)(msg.mot_throttle[1]);
    rotor_throttle_payload.throttle[2] = (int16_t)(msg.mot_throttle[2]);
    rotor_throttle_payload.throttle[3] = (int16_t)(msg.mot_throttle[3]);
    memcpy(
      ctrl_msg.payload,
      &rotor_throttle_payload,
      getPayloadLength(msgs::CTRLMODE::ROTOR_THROTTLE));
    ROS_INFO("[%s] Sending rotor throttles", pnh_.getNamespace().c_str());
  }
  else
  {
    ctrl_msg.mode = msgs::CTRLMODE::OFF;
    ROS_WARN("[%s] Unsupported control mode!",
      pnh_.getNamespace().c_str());
  }


  char buffer[msgs::SERIAL_CONTAINER_MAX_LENGTH];
  uint8_t length = msgs::SERIAL_CONTAINER_MAX_LENGTH;

  if(SerialContainerEncode(&container, (uint8_t*)&buffer[1], &length))
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] Encoding error!",
      pnh_.getNamespace().c_str());
    return;
  }
  buffer[0] = msgs::SERIAL_CONTAINER_DELIMITER; length++;
  /* append at least 1 delimiter and pad to a multiple of 8 bytes in length */
  do
  {
    buffer[length++] = msgs::SERIAL_CONTAINER_DELIMITER;
  } while (length % 16 != 0);

  const size_t sent_bytes = serial_port_.send(buffer, length);
  if(sent_bytes != length)
    ROS_WARN("[%s] Error sending to serial port!",pnh_.getNamespace().c_str());
}

void CtrlBridge::setControlActiveCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == false)
  {
    control_active_ = false;
    ROS_INFO("[%s] Control inactive", pnh_.getNamespace().c_str());
  }
  else
  {
    ROS_INFO("[%s] Activate Control", pnh_.getNamespace().c_str());
    const ros::Duration ros_time_since_start = ros::Time::now()  - start_time_;
    const uint64_t time_ns =
      ros_time_since_start.sec * 1e9 + ros_time_since_start.nsec;
    
    msgs::serial_container_t container;
    container.num_messages = 1u;

    msgs::serial_message_t& arm_msg =  container.messages[0];
    arm_msg.time = time_ns;
    arm_msg.mode = msgs::CTRLMODE::ARM;

    char buffer[msgs::SERIAL_CONTAINER_MAX_LENGTH];
    uint8_t length = msgs::SERIAL_CONTAINER_MAX_LENGTH;

    if(SerialContainerEncode(&container, (uint8_t*)&buffer[1], &length))
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Encoding error!",
        pnh_.getNamespace().c_str());
      return;
    }
    buffer[0] = msgs::SERIAL_CONTAINER_DELIMITER; length++;
    /* append at least 1 delimiter and pad to a multiple of 8 bytes in length */
    do
    {
      buffer[length++] = msgs::SERIAL_CONTAINER_DELIMITER;
    } while (length % 16 != 0);

    const size_t sent_bytes = serial_port_.send(buffer, length);
    if(sent_bytes != length)
      ROS_WARN("[%s] Error sending to serial port!",pnh_.getNamespace().c_str());
  }
}

bool CtrlBridge::loadParams()
{
  bool check = true;
  check &= pnh_.getParam("mass", mass_);
  check &= pnh_.getParam("port_name", port_name_);
  check &= pnh_.getParam("baud_rate", baud_rate_);
  return check;
}

} // namespace ctrl_bridge

