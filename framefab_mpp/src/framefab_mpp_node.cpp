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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "framefab_node");
  ros::NodeHandle node_handle("framefab_node");

  ros::CallbackQueue rviz_ui_queue;

  // init framefab_planner
  framefab::FrameFabPlannerPtr ptr_framefab_planner = boost::make_shared<framefab::FrameFabPlanner>(node_handle);

  // spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(10.0);

  ptr_framefab_planner->setRobotHomePose();

  ros::spin();

  return 0;
}
