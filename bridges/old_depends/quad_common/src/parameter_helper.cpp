#include "quad_common/parameter_helper.h"

namespace quad_common
{

bool getParamVector3d(const std::string& name, Eigen::Vector3d& vector, const ros::NodeHandle & pnh)
{
  if (getParam(name + "/x", vector.x(), pnh) && getParam(name + "/y", vector.y()) && getParam(name + "/z", vector.z()))
    return true;
  return false;
}

bool getParamQuaterniond(const std::string& name, Eigen::Quaterniond& q, const ros::NodeHandle & pnh)
{
  if (getParam(name + "/x", q.x()) && getParam(name + "/y", q.y()) && getParam(name + "/z", q.z())
      && getParam(name + "/w", q.w()))
    return true;
  return false;
}

} // quad_common
