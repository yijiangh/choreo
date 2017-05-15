/*
 * framefab_node.cpp
 * 
 * Created on:  April 7, 2017
 * Author:      Yijiang Huang, Thomas Cook
 * Institute:   MIT, Digital Structure Group, Building Tech
*/

// boost smart_ptr
#include <boost/shared_ptr.hpp>

// ros
#include <ros/ros.h>

// framefab_mpp
#include <framefab_planner.h>

#include <framefab_msgs/AdvanceRobot.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "framefab_mpp_node");
  ros::NodeHandle node_handle("framefab_mpp_node");

  ros::CallbackQueue rviz_ui_queue;

  // init framefab_planner
  framefab::FrameFabPlanner framefab_planner(node_handle);

  ros::ServiceServer ss = node_handle.advertiseService("advance_robot",
                                                       &framefab::FrameFabPlanner::setRobotHomePose, &framefab_planner);
  // spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // This sleep is ONLY to allow Rviz to come up
  // sleep(5);

  ros::spin();

  return 0;
}
