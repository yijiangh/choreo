#ifndef CHOREO_PROCESS_PLANNING_H
#define CHOREO_PROCESS_PLANNING_H

// service
#include <choreo_msgs/ProcessPlanning.h>
#include <choreo_msgs/MoveToTargetPose.h>

// msg
#include <choreo_msgs/ElementCandidatePoses.h>
#include <moveit_msgs/CollisionObject.h>

// robot model
#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

/*
 * This class wraps Descartes planning methods and provides functionality for configuration
 * and for planning for choreo support and spatial element printing paths.
 *
 * The general plannning approach is:
 *
 */
namespace choreo_process_planning
{

class ProcessPlanningManager
{
 public:
  ProcessPlanningManager(const std::string& world_frame,
                         const std::string& hotend_group, const std::string& hotend_tcp, const std::string& hotend_base,
                         const std::string& robot_model_plugin);

  // srv serving function for general process planning
  // deliver request to specific planning server according to input assembly type
  bool handleProcessPlanning(choreo_msgs::ProcessPlanning::Request& req,
                           choreo_msgs::ProcessPlanning::Response& res);

  // srv serving function for spatial extrusion
  bool handlePrintPlanning(choreo_msgs::ProcessPlanning::Request& req,
                           choreo_msgs::ProcessPlanning::Response& res);

  // srv serving function for picknplace
  bool handlePickNPlacePlanning(
      choreo_msgs::ProcessPlanning::Request& req,
      choreo_msgs::ProcessPlanning::Response& res);

  // srv serving function for single-query move pose to pose
  bool handleMoveToTargetPosePlanAndExecution(
      choreo_msgs::MoveToTargetPose::Request& req,
      choreo_msgs::MoveToTargetPose::Response& res);

 private:
  // TODO: rename this! should be sth like descartes_model_
  // descartes robot model
  descartes_core::RobotModelPtr hotend_model_;

  // moveit robot model
  moveit::core::RobotModelConstPtr moveit_model_;

  pluginlib::ClassLoader<descartes_core::RobotModel> plugin_loader_; // kept around so code doesn't get unloaded

  std::string hotend_group_name_;
  std::string world_frame_;

  // planning scene service client
  ros::NodeHandle nh_;
  ros::ServiceClient planning_scene_diff_client_;
};
}

#endif