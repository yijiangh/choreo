#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <ros/ros.h>

// service
#include <framefab_msgs/ElementNumberRequest.h>
#include <framefab_msgs/VisualizeSelectedPath.h>

// msgs
#include <framefab_msgs/FrameFabParameters.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>
#include <framefab_msgs/RobotInputParameters.h>
#include <framefab_msgs/OutputPathInputParameters.h>
#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/UnitProcessPlan.h>

// actions
#include <framefab_msgs/PathPlanningAction.h>
#include <framefab_msgs/ProcessPlanningAction.h>
#include <framefab_msgs/ProcessExecutionAction.h>
#include <framefab_msgs/SimulateMotionPlanAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// core service instances
#include <framefab_core/visual_tools/framefab_visual_tool.h>
#include "framefab_core/trajectory_library.h"

/**
 * Associates a name with a joint trajectory
 */
struct ProcessPlanResult
{
  std::vector<framefab_msgs::UnitProcessPlan> plans;
};

class FrameFabCoreService
{
 public:
  FrameFabCoreService();

  bool init();
  void run();

 private:
  bool load_model_input_parameters(const std::string& filename);
  void save_model_input_parameters(const std::string& filename);
  bool load_path_input_parameters(const std::string& filename);
  void save_path_input_parameters(const std::string& filename);
  bool load_robot_input_parameters(const std::string& filename);
  void save_robot_input_parameters(const std::string& filename);
  bool load_output_path_input_parameters(const std::string& filename);
  void save_output_path_input_parameters(const std::string& filename);

  // Service callbacks
  bool framefab_parameters_server_callback(framefab_msgs::FrameFabParameters::Request& req,
                                           framefab_msgs::FrameFabParameters::Response& res);

  bool element_number_sequest_server_callback(framefab_msgs::ElementNumberRequest::Request& req,
                                              framefab_msgs::ElementNumberRequest::Response& res);

  bool visualize_selected_path_server_callback(framefab_msgs::VisualizeSelectedPath::Request& req,
                                               framefab_msgs::VisualizeSelectedPath::Response& res);

  // Action callbacks
  void pathPlanningActionCallback(const framefab_msgs::PathPlanningGoalConstPtr &goal);
  void processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal);
  void simulateMotionPlansActionCallback(const framefab_msgs::SimulateMotionPlanGoalConstPtr& goal_in);

  // Process Planning - these process planning related
  // methods are defined in src/framefab_core_service_process_planning.cpp
  bool generateMotionLibrary(
      const int selected_path_index,
      framefab_core_service::TrajectoryLibrary& traj_lib);

  ProcessPlanResult generateProcessPlan(const int index);

  // immediate plan & execution
  bool moveToTargetJointPose(std::vector<double> joint_pose);

 private:
  // Services offered by this class
  ros::ServiceServer framefab_parameters_server_;
  ros::ServiceServer element_number_sequest_server_;
  ros::ServiceServer visualize_selected_path_server_;

  // Services subscribed to by this class
  ros::ServiceClient path_post_processing_client_;
  ros::ServiceClient process_planning_client_;
  ros::ServiceClient move_to_pose_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<framefab_msgs::PathPlanningAction> path_planning_server_;
  framefab_msgs::PathPlanningFeedback path_planning_feedback_;
  framefab_msgs::PathPlanningResult path_planning_result_;

  actionlib::SimpleActionServer<framefab_msgs::ProcessPlanningAction> process_planning_server_;
  framefab_msgs::ProcessPlanningFeedback process_planning_feedback_;
  framefab_msgs::ProcessPlanningResult process_planning_result_;

  actionlib::SimpleActionServer<framefab_msgs::SimulateMotionPlanAction> simulate_motion_plan_server_;
  framefab_msgs::SimulateMotionPlanFeedback simulate_motion_plan_feedback_;
  framefab_msgs::SimulateMotionPlanResult simulate_motion_plan_result_;

  // Actions subscribed to by this class
  actionlib::SimpleActionClient<framefab_msgs::ProcessExecutionAction> framefab_exe_client_;

  // Current state publishers

  // Core Service Instances
  framefab_visual_tools::FrameFabVisualTool visual_tool_;

  // working environment collision objects
  std::vector<moveit_msgs::CollisionObject> env_objs_;
  // path results
  std::vector<framefab_msgs::ElementCandidatePoses> process_paths_;

  // Trajectory library
  int selected_path_id_;
  framefab_core_service::TrajectoryLibrary trajectory_library_;

  // Parameters
  framefab_msgs::ModelInputParameters 	model_input_params_;
  framefab_msgs::PathInputParameters 	path_input_params_;
  framefab_msgs::RobotInputParameters   robot_input_params_;
  framefab_msgs::OutputPathInputParameters 	output_path_input_params_;

  framefab_msgs::ModelInputParameters 	default_model_input_params_;
  framefab_msgs::PathInputParameters 	default_path_input_params_;
  framefab_msgs::RobotInputParameters   default_robot_input_params_;
  framefab_msgs::OutputPathInputParameters 	default_output_path_input_params_;

  // Parameter loading and saving
  bool save_data_;
  std::string save_location_;
  std::string param_cache_prefix_;
};

#endif
