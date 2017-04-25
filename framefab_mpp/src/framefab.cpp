#include <framefab.h>

namespace framefab
{

FrameFab::FrameFab(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  ROS_INFO("FrameFab node started.");

  readParameters();

}

FrameFab::~FrameFab()
{

}

bool FrameFab::readParameters()
{

  return true;
}

void FrameFab::debug()
{
  ROS_INFO("FrameFab: ROS debug function called.");

}

}