//
// Created by yijiangh on 6/18/17.
//

// Process Services
#include <choreo_process_planning/choreo_process_planning.h>

// srv
#include <choreo_msgs/ProcessPlanning.h>

#include <ros/ros.h>

// Globals
const static std::string PROCESS_PLANNING_SERVICE = "process_planning";
const static std::string MOVE_TO_TARGET_POSE_SERVICE = "move_to_target_pose";

namespace
{
}// anon util namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "choreo_process_planning");

  // TODO: these parmaters' names are misleading, should be more app-neutral
  // Load local parameters
  ros::NodeHandle nh, pnh("~");
  std::string world_frame, hotend_group, hotend_tcp, hotend_base, robot_model_plugin;
  pnh.param<std::string>("world_frame", world_frame, "");
  pnh.param<std::string>("hotend_group", hotend_group, ""); // planning group name
  pnh.param<std::string>("hotend_tcp", hotend_tcp, "");     // tool frame name
  pnh.param<std::string>("hotend_base", hotend_base, "");   // work object/reference frame name
  pnh.param<std::string>("robot_model_plugin", robot_model_plugin, "");

  // IK Plugin parameter must be specified
  if (robot_model_plugin.empty())
  {
    ROS_ERROR_STREAM("[process planning] MUST SPECIFY PARAMETER 'robot_model_plugin' for choreo_process_planning node");
    return -1;
  }

  using choreo_process_planning::ProcessPlanningManager;

  // Creates a planning manager that will create the appropriate planning classes and perform
  // all required initialization. It exposes member functions to handle each kind of processing
  // event.
  ProcessPlanningManager manager(world_frame, hotend_group, hotend_tcp, world_frame, robot_model_plugin);

  // Plumb in the appropriate ros services
  ros::ServiceServer pp_server = nh.advertiseService(
      PROCESS_PLANNING_SERVICE, &ProcessPlanningManager::handleProcessPlanning, &manager);

   ros::ServiceServer move_to_target_pose_server = nh.advertiseService(
       MOVE_TO_TARGET_POSE_SERVICE, &ProcessPlanningManager::handleMoveToTargetPosePlanAndExecution, &manager);

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[PP] process planning server online");
  ros::spin();

  return 0;
}