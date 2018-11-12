/*
 * parameter_handler.cpp
 *
 *  Created on: May 2, 2014
 *      Author: ffontana
 */

#include "laird_bridge/parameter_handler.h"
#include "laird_bridge/laird_bridge.h"

namespace laird_bridge
{

ParameterHandler::ParameterHandler(LairdBridge * laird_bridge, int basecomputer_sys_id, int basecomputer_comp_id,
                                   int px4_sys_id, int px4_comp_id) :
    nh_(), requested_parameter_received_(false), requested_parameter_name_()
{
  laird_bridge_ = laird_bridge;
  px4_comp_id_ = px4_comp_id;
  px4_sys_id_ = px4_sys_id;
  basecomputer_sys_id_ = basecomputer_sys_id;
  basecomputer_comp_id_ = basecomputer_comp_id;

  node_name_ = ros::this_node::getName();

  // this node handle uses a seperate callback queue to not block the other messages while we read parameters.
  nh_.setCallbackQueue(&service_callback_queue_);
  spin_thread_ = boost::thread(&ParameterHandler::spinThread, this);

  write_onboard_parameter_server_ = nh_.advertiseService("write_onboard_parameter",
                                                         &ParameterHandler::writeOnboardParameterCallback, this);
  read_onboard_parameter_server_ = nh_.advertiseService("read_onboard_parameter",
                                                        &ParameterHandler::readOnboardParameterCallback, this);

  load_onboard_parameters_from_flash_server_ = nh_.advertiseService(
      "load_onboard_parameters_from_flash", &ParameterHandler::loadOnboardParametersFromFlashCallback, this);
  save_onboard_parameters_to_flash_server_ = nh_.advertiseService(
      "save_onboard_parameters_to_flash", &ParameterHandler::saveOnboardParametersToFlashCallback, this);
}

ParameterHandler::~ParameterHandler()
{
}

void ParameterHandler::spinThread()
{
  while (ros::ok())
  {
    service_callback_queue_.callAvailable(ros::WallDuration(0.1));
  }
}

void ParameterHandler::handleOnboardParameterMsg(const mavlink_message_t& mavlink_msg)
{
  quad_msgs::OnboardParameter received_parameter;

  char param_id[17];
  mavlink_msg_param_value_get_param_id(&mavlink_msg, param_id);
  param_id[16] = '\0';
  received_parameter.param_id = param_id;
  received_parameter.param_value = mavlink_msg_param_value_get_param_value(&mavlink_msg);
  received_parameter.param_type = mavlink_msg_param_value_get_param_type(&mavlink_msg);
  received_parameter.param_count = mavlink_msg_param_value_get_param_count(&mavlink_msg);
  received_parameter.param_index = mavlink_msg_param_value_get_param_index(&mavlink_msg);

  // check if it is the parameter we requested
  if (received_parameter.param_id == requested_parameter_name_)
  {
    boost::mutex::scoped_lock lock(received_parameter_mutex_);
    requested_parameter_received_ = true;
    received_parameter_ = received_parameter;
  }
}

bool ParameterHandler::writeOnboardParameterCallback(quad_srvs::OnboardParameterRequest & request,
                                                     quad_srvs::OnboardParameterResponse & response)
{
  // create the mavlink message to write a parameter
  mavlink_message_t mavlink_msg;
  mavlink_msg_param_set_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, px4_sys_id_, px4_comp_id_,
                             request.parameter.param_id.c_str(), request.parameter.param_value,
                             request.parameter.param_type);

  // sometimes the parameter write request gets lost, therefore we retry a couple of times
  for (int i = 0; i < PARAMETER_READ_WRITE_MAX_NR_RETRIES; i++)
  {
    // reset flags
    requested_parameter_received_ = false;
    requested_parameter_name_ = request.parameter.param_id;

    // send the write request
    laird_bridge_->sendMavlinkMessage(mavlink_msg);

    // wait for the parameter answer, if we timeout we start again
    if (!requestedParameterReceived())
    {
      continue;
    }

    boost::mutex::scoped_lock lock(received_parameter_mutex_);

    // double check if we received the right parameter?
    if (requested_parameter_name_ == received_parameter_.param_id)
    {
      // was the parameter updated?
      if (areEqual(request.parameter.param_value, received_parameter_.param_value))
      {
//        ROS_INFO("[%s] parameter %s was updated on the px4", node_name_.c_str(), request.parameter.param_id.c_str() );
        response.parameter = received_parameter_;
        return true;
      }
    }
  } // end for
  ROS_INFO("[%s] parameter %s was not updated on the px4", node_name_.c_str(), request.parameter.param_id.c_str());
  return false;
}

bool ParameterHandler::readOnboardParameterCallback(quad_srvs::OnboardParameterRequest & request,
                                                    quad_srvs::OnboardParameterResponse & response)
{
  // create the mavlink message to read a parameter
  int param_index = -1; // ignore the parameter index, use the name instead
  mavlink_message_t mavlink_msg;
  mavlink_msg_param_request_read_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, px4_sys_id_,
                                      px4_comp_id_, request.parameter.param_id.c_str(), param_index);

  // sometimes the parameter read request gets lost, therefore we retry a couple of times
  for (int i = 0; i < PARAMETER_READ_WRITE_MAX_NR_RETRIES; i++)
  {
    // reset flags
    requested_parameter_received_ = false;
    requested_parameter_name_ = request.parameter.param_id;

    // send the write request
    laird_bridge_->sendMavlinkMessage(mavlink_msg);

    // wait for the parameter answer, if we timeout we start again
    if (!requestedParameterReceived())
    {
      continue;
    }

    boost::mutex::scoped_lock lock(received_parameter_mutex_);

    // double check if we received the right parameter?
    if (requested_parameter_name_ == received_parameter_.param_id)
    {
      response.parameter = received_parameter_;
      return true;
    }
  } // end for
  ROS_INFO("[%s] parameter %s could not be read", node_name_.c_str(), request.parameter.param_id.c_str());
  return false;
}

bool ParameterHandler::loadOnboardParametersFromFlashCallback(std_srvs::EmptyRequest & request,
                                                              std_srvs::EmptyResponse & response)
{
  mavlink_message_t mavlink_msg;
  // param1 == 0 -> loading onboard parameters
  // param1 == 1 -> saving onboard parameters
  // param7 == 0 -> using eeprom
  // param7 == 1 -> using sd card
  mavlink_msg_command_long_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, px4_sys_id_, px4_comp_id_,
                                MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 0, 0, 0, 0, 0, 1);
  laird_bridge_->sendMavlinkMessage(mavlink_msg);
  return true;
}

bool ParameterHandler::saveOnboardParametersToFlashCallback(std_srvs::EmptyRequest & request,
                                                            std_srvs::EmptyResponse & response)
{
  mavlink_message_t mavlink_msg;
  // param1 == 0 -> loading onboard parameters
  // param1 == 1 -> saving onboard parameters
  // param7 == 0 -> using eeprom
  // param7 == 1 -> using sd card
  mavlink_msg_command_long_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, px4_sys_id_, px4_comp_id_,
                                MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 1);
  laird_bridge_->sendMavlinkMessage(mavlink_msg);
  return true;
}

bool ParameterHandler::areEqual(const double & a, const double & b)
{
  if (fabs(a - b) <= 0.0001)
    return true;
  return false;
}

bool ParameterHandler::requestedParameterReceived()
{
  ros::Time parameter_request_sent_stamp(ros::Time::now());
  // loop until the px4 replies or we timeout
  while (ros::ok() && !requested_parameter_received_
      && (ros::Time::now() - parameter_request_sent_stamp).toSec() < PARAMETER_READ_WRITE_TIMEOUT)
  {
    ros::Duration(0.001).sleep(); // check with 1kHz if we received something
  }

  if (requested_parameter_received_)
    return true;
  ROS_INFO("[%s] Timeout on waiting for parameter %s", node_name_.c_str(), requested_parameter_name_.c_str());
  return false;
}

//void ParameterHandler::getAllOnboardParametersCallback(const std_msgs::Empty::ConstPtr& msg)
//{
//  mavlink_message_t mavlink_msg;
//  mavlink_msg_param_request_list_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, px4_sys_id_, px4_comp_id_);
//  laird_bridge_->sendMavlinkMessage( mavlink_msg );
//}

} /* namespace laird_bridge */
