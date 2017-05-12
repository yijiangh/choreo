#include <framefab_planner.h>

namespace framefab
{

FrameFabPlanner::FrameFabPlanner(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  ROS_INFO("FrameFabPlanner node started.");

  readParameters();

}

FrameFabPlanner::~FrameFabPlanner()
{

}

bool FrameFabPlanner::readParameters()
{

  return true;
}

void FrameFabPlanner::debug()
{
  ROS_INFO("FrameFabPlanner: ROS debug function called.");

}

}// namespace frammefab