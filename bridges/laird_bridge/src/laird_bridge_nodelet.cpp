#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include <memory>
#include <laird_bridge/laird_bridge.h>

namespace laird_bridge
{

class LairdBridgeNodelet : public nodelet::Nodelet
{
public:
  LairdBridgeNodelet() = default;
  virtual ~LairdBridgeNodelet() {}
private:
  virtual void onInit();
  std::unique_ptr<LairdBridge> laird_bridge_;
};

void LairdBridgeNodelet::onInit()
{
  printf("init\n");

  ros::NodeHandle nh_private(getPrivateNodeHandle());
  ros::NodeHandle nh(getNodeHandle());
  laird_bridge_.reset(new laird_bridge::LairdBridge(nh, nh_private));
}

}

PLUGINLIB_EXPORT_CLASS(laird_bridge::LairdBridgeNodelet, nodelet::Nodelet)

