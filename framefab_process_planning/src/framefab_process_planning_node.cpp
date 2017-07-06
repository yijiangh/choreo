//
// Created by yijiangh on 6/18/17.
//

#include <ros/ros.h>
// Process Services
#include <framefab_process_planning/framefab_process_planning.h>

// Globals
const static std::string DEFAULT_PRINT_PLANNING_SERVICE = "framefab_process_planning";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_process_planning");

  // Load local parameters
  ros::NodeHandle nh, pnh("~");
  std::string world_frame, hotend_group, hotend_tcp, robot_model_plugin;
  pnh.param<std::string>("world_frame", world_frame, "world_frame");
  pnh.param<std::string>("hotend_group", hotend_group, "manipulator_tcp");
  pnh.param<std::string>("hotend_tcp", hotend_tcp, "tcp_frame");
  pnh.param<std::string>("robot_model_plugin", robot_model_plugin, "");

  // IK Plugin parameter must be specified
  if (robot_model_plugin.empty())
  {
    ROS_ERROR_STREAM("MUST SPECIFY PARAMETER 'robot_model_plugin' for framefab_process_planning node");
    return -1;
  }

  using framefab_process_planning::ProcessPlanningManager;

  // Creates a planning manager that will create the appropriate planning classes and perform
  // all required initialization. It exposes member functions to handle each kind of processing
  // event.
  ProcessPlanningManager manager(world_frame, hotend_group, hotend_tcp, robot_model_plugin);
  // Plumb in the appropriate ros services
  ros::ServiceServer print_server = nh.advertiseService(
      DEFAULT_PRINT_PLANNING_SERVICE, &ProcessPlanningManager::handlePrintPlanning, &manager);

  // Serve and wait for shutdown
  ROS_INFO_STREAM("framefab Process Planning Server Online");
  ros::spin();

  return 0;
}
