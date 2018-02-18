/*
 * copilot_helper.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: Alessandro
 */

#include <limits>  // std::numeric_limits<double>

#include "copilot/copilot_helper.h"
#include "copilot/states.h"
#include "quad_msgs/ControllerFeedback.h"

namespace copilot
{
CopilotHelper::CopilotHelper()
  : controller_common::FlightControllerBaseHelper("copilot/feedback")
{
  feedthrough_pub_ = nh_.advertise<std_msgs::Bool>("copilot/feedthrough", 1);
}

CopilotHelper::~CopilotHelper()
{
}

bool CopilotHelper::enableFeedthrough(const double loop_rate_hz,
                                      const bool copilot_must_hover) const
{
  ros::spinOnce();  // update copilot_feedback_msg_ / received_copilot_feedback_

  // Check if copilot's feedback is being received
  if (!received_controller_feedback_)
  {
    ROS_ERROR("[%s] Can't enable feedthrough: Copilot not sending feedback",
              ros::this_node::getName().c_str());
    return false;
  }

  // Check age of last received copilot feedback message
  const double kMaxFeedbackAge = 2.0;
  if (feedbackMessageAge() > kMaxFeedbackAge)
  {
    ROS_ERROR("[%s] Will not enable feedthrough: Last received copilot "
              "feedback message is too long ago",
              ros::this_node::getName().c_str());
    return false;
  }

  // Now feedthrough can be enabled
  ROS_INFO("[%s] Activating Feedthrough", ros::this_node::getName().c_str());
  std_msgs::Bool feedthrough_msg;
  feedthrough_msg.data = true;
  const size_t kMaxSwitchingAttempts = 10;
  for (size_t attempt = 1; attempt <= kMaxSwitchingAttempts; attempt++)
  {
    feedthrough_pub_.publish(feedthrough_msg);

    // Wait for Copilot to switch to feedthrough
    ROS_INFO("[%s] Waiting for Copilot to switch to feedthrough...",
                  ros::this_node::getName().c_str());

    if (waitForSpecificControllerState(copilot::states::FEEDTHROUGH,
                                       kTimeoutS / kMaxSwitchingAttempts,
                                       loop_rate_hz))
    {
      // Switching was succesful, stop trying
      break;
    }

    if (attempt == kMaxSwitchingAttempts)
    {
      ROS_ERROR("[%s] Timeout reached waiting for Copilot to switch to "
                "feedthrough",
                ros::this_node::getName().c_str());
      return false;
    }
  }

  // No errors encountered, successfully switched to feedthrough
  ROS_INFO("[%s] Feedthrough active", ros::this_node::getName().c_str());
  return true;
}

bool CopilotHelper::disableFeedthrough(const double loop_rate_hz) const
{
  ROS_INFO("[%s] Deactivating Feedthrough", ros::this_node::getName().c_str());

  std_msgs::Bool feedthrough_msg;
  feedthrough_msg.data = false;
  feedthrough_pub_.publish(feedthrough_msg);

  // Wait for copilot to return to hover mode
  ROS_INFO("[%s] Waiting for Copilot to switch to Hover...",
                ros::this_node::getName().c_str());
  if (!waitForSpecificControllerState(copilot::states::HOVER, kTimeoutS,
                                      loop_rate_hz))
  {
    ROS_ERROR("[%s] Timeout reached waiting for Copilot to switch to Hover",
              ros::this_node::getName().c_str());
    return false;
  }

  return true;
}

// Publish a new desired state to the flight controller
void CopilotHelper::publishDesiredState(
    const quad_common::QuadDesiredState& state) const
{
  // Don't update the desired state
  ROS_WARN("[%s] I will not publish a desired state to the copilot. Send it to "
           "the FlightController instead! The copilot is here to recover in "
           "case something goes wrong.",
           ros::this_node::getName().c_str());
}

}  // namespace copilot
