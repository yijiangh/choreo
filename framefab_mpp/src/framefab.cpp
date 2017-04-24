#include "../include/framefab.h"

namespace framefab
{

FrameFab::FrameFab(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  ROS_INFO("FrameFab node started.");

  // readParameters
  readParameters();

}

FrameFab::~FrameFab()
{
  node_handle_.shutdown();
}

bool FrameFab::readParameters()
{
  //FrameFab Parameters
  node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));

  return true;
}
}
