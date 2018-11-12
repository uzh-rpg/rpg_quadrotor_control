/*
 * parameter_handler.h
 *
 *  Created on: May 2, 2014
 *      Author: ffontana
 */

#pragma once

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "std_srvs/Empty.h"
#include "quad_srvs/OnboardParameter.h"
#include "quad_msgs/OnboardParameter.h"

#include "mavlink/v1.0/rpg/mavlink.h"

namespace pixhawk_bridge
{

class PixHawkBridge;

const double PARAMETER_READ_WRITE_TIMEOUT = 0.5;
const int PARAMETER_READ_WRITE_MAX_NR_RETRIES = 3;

class ParameterHandler
{
public:
  ParameterHandler() {};
  ParameterHandler(PixHawkBridge * pixhawk_bridge, int basecomputer_sys_id, int basecomputer_comp_id, int px4_sys_id,
                   int px4_comp_id);

  virtual ~ParameterHandler();

  void spinThread();
  void handleOnboardParameterMsg(const mavlink_message_t& mavlink_msg);

  bool writeOnboardParameterCallback(quad_srvs::OnboardParameterRequest & request,
                                     quad_srvs::OnboardParameterResponse & response);

  bool readOnboardParameterCallback(quad_srvs::OnboardParameterRequest & request,
                                    quad_srvs::OnboardParameterResponse & response);

  bool loadOnboardParametersFromFlashCallback(std_srvs::EmptyRequest & request, std_srvs::EmptyResponse & response);

  bool saveOnboardParametersToFlashCallback(std_srvs::EmptyRequest & request, std_srvs::EmptyResponse & response);

  boost::thread spin_thread_;

  ros::NodeHandle nh_;
  std::string node_name_;
  PixHawkBridge * pixhawk_bridge_;
  ros::CallbackQueue service_callback_queue_;

  ros::ServiceServer write_onboard_parameter_server_;
  ros::ServiceServer read_onboard_parameter_server_;

  ros::ServiceServer load_onboard_parameters_from_flash_server_;
  ros::ServiceServer save_onboard_parameters_to_flash_server_;

  bool requested_parameter_received_;
  std::string requested_parameter_name_;

  boost::mutex received_parameter_mutex_;
  quad_msgs::OnboardParameter received_parameter_;

  int basecomputer_sys_id_;
  int basecomputer_comp_id_;
  int px4_sys_id_;
  int px4_comp_id_;

private:
  bool areEqual(const double & a, const double & b);
  bool requestedParameterReceived();
};

} /* namespace pixhawk_bridge */
