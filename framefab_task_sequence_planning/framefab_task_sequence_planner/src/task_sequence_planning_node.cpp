//
// Created by yijiangh on 12/21/17.
//

#include <ros/ros.h>

// framefab dependencies
#include "framefab_task_sequence_planner/FiberPrintPARM.h"
#include "framefab_task_sequence_planner/utils/WireFrame.h"
#include "framefab_task_sequence_planner/FiberPrintPlugIn.h"

// srv
#include <framefab_msgs/TaskSequencePlanning.h>

// Globals
const static std::string DEFAULT_TASK_SEQUENCE_PLANNING_SERVICE = "task_sequence_planning";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_task_sequence_planning");
  ros::NodeHandle nh, pnh("~");

  std::string world_frame, hotend_group, hotend_tcp, robot_model_plugin;
  pnh.param<std::string>("/world_frame", world_frame, "/world_frame");
  pnh.param<std::string>("hotend_group", hotend_group, "manipulator_tcp");
  pnh.param<std::string>("hotend_tcp", hotend_tcp, "tcp_frame");
  pnh.param<std::string>("robot_model_plugin", robot_model_plugin, "");

  // IK Plugin parameter must be specified
  if (robot_model_plugin.empty())
  {
    ROS_ERROR_STREAM("MUST SPECIFY PARAMETER 'robot_model_plugin' for framefab_process_planning node");
    return -1;
  }

  FiberPrintPlugIn fiber_print_plugin(world_frame, hotend_group, hotend_tcp, robot_model_plugin);

  ros::ServiceServer task_sequence_planning_server =
      nh.advertiseService(DEFAULT_TASK_SEQUENCE_PLANNING_SERVICE,
                          &FiberPrintPlugIn::handleTaskSequencePlanning, &fiber_print_plugin);

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[tsp] sequence task planning server online");
  ros::spin();

  return 0;
}