//
// Created by yijiangh on 6/18/17.
//

#include <ros/ros.h>
// Process Services
#include <framefab_process_planning/framefab_process_planning.h>

// Globals
const static std::string DEFAULT_PRINT_PLANNING_SERVICE = "process_planning";
const static std::string MOVE_TO_TARGET_POSE_SERVICE = "move_to_target_pose";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_process_planning");

  // Load local parameters
  ros::NodeHandle nh, pnh("~");
  std::string world_frame, hotend_group, hotend_tcp, hotend_base, robot_model_plugin;
  pnh.param<std::string>("world_frame", world_frame, "");
  pnh.param<std::string>("hotend_group", hotend_group, ""); // planning group name
  pnh.param<std::string>("hotend_tcp", hotend_tcp, "");     // tool frame name
  pnh.param<std::string>("hotend_base", hotend_base, "");   // work object/reference frame name
  pnh.param<std::string>("robot_model_plugin", robot_model_plugin, "");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

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
  ProcessPlanningManager manager(world_frame, hotend_group, hotend_tcp, world_frame, robot_model_plugin);

  // Plumb in the appropriate ros services
  ros::ServiceServer print_server = nh.advertiseService(
      DEFAULT_PRINT_PLANNING_SERVICE, &ProcessPlanningManager::handlePrintPlanning, &manager);

   ros::ServiceServer move_to_target_pose_server = nh.advertiseService(
       MOVE_TO_TARGET_POSE_SERVICE, &ProcessPlanningManager::handleMoveToTargetPosePlanAndExecution, &manager);

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[PP] process planning server online");
  ros::spin();

  return 0;
}
