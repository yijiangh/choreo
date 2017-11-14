#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <ros/ros.h>

// served service
#include <framefab_msgs/ElementNumberRequest.h>
#include <framefab_msgs/VisualizeSelectedPath.h>
#include <framefab_msgs/GetAvailableProcessPlans.h>
#include <framefab_msgs/OutputProcessPlans.h>
#include <framefab_msgs/QueryComputationRecord.h>

// msgs
#include <framefab_msgs/FrameFabParameters.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>
#include <framefab_msgs/RobotInputParameters.h>
#include <framefab_msgs/OutputSaveDirInputParameters.h>
#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/UnitProcessPlan.h>

// actions
#include <framefab_msgs/TaskSequenceProcessingAction.h>
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
  bool loadModelInputParameters(const std::string &filename);
  void saveModelInputParameters(const std::string &filename);
  bool loadTaskSequenceInputParameters(const std::string& filename);
  void saveTaskSequenceInputParameters(const std::string& filename);
  bool loadRobotInputParameters(const std::string& filename);
  void saveRobotInputParameters(const std::string& filename);
  bool loadOutputSaveDirInputParameters(const std::string& filename);
  void saveOutputSaveDirInputParameters(const std::string& filename);
  int  checkSavedLadderGraphSize(const std::string& filename);

  // Service callbacks
  bool framefabParametersServerCallback(framefab_msgs::FrameFabParameters::Request& req,
                                        framefab_msgs::FrameFabParameters::Response& res);

  bool elementNumberRequestServerCallback(framefab_msgs::ElementNumberRequest::Request& req,
                                          framefab_msgs::ElementNumberRequest::Response& res);

  bool visualizeSelectedPathServerCallback(framefab_msgs::VisualizeSelectedPath::Request& req,
                                           framefab_msgs::VisualizeSelectedPath::Response& res);

  bool getAvailableProcessPlansCallback(framefab_msgs::GetAvailableProcessPlans::Request& req,
                                        framefab_msgs::GetAvailableProcessPlans::Response& res);

  bool outputProcessPlansCallback(framefab_msgs::OutputProcessPlans::Request& req,
                                  framefab_msgs::OutputProcessPlans::Response& res);

  bool queryComputationResultCallback(framefab_msgs::QueryComputationRecord::Request& req,
                                      framefab_msgs::QueryComputationRecord::Response& res);

  // Action callbacks
  void taskSequenceProcessingActionCallback(const framefab_msgs::TaskSequenceProcessingGoalConstPtr &goal);
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

  void adjustSimSpeed(double sim_speed);

 private:
  // Services offered by this class
  ros::ServiceServer framefab_parameters_server_;
  ros::ServiceServer element_number_sequest_server_;
  ros::ServiceServer visualize_selected_path_server_;
  ros::ServiceServer get_available_process_plans_server_;
  ros::ServiceServer output_process_plans_server_;
  ros::ServiceServer query_computation_result_server_;

  // Services subscribed to by this class
  ros::ServiceClient task_sequence_processing_srv_client_;
  ros::ServiceClient process_planning_client_;
  ros::ServiceClient move_to_pose_client_;
  ros::ServiceClient output_processing_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<framefab_msgs::TaskSequenceProcessingAction> task_sequence_processing_server_;
  framefab_msgs::TaskSequenceProcessingFeedback task_sequence_processing_feedback_;
  framefab_msgs::TaskSequenceProcessingResult task_sequence_processing_result_;

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
  // formulated task sequence results
  std::vector<framefab_msgs::ElementCandidatePoses> task_sequence_;

  // Trajectory library
  int selected_task_id_;
  bool use_saved_graph_;
  framefab_core_service::TrajectoryLibrary trajectory_library_;

  // Parameters
  framefab_msgs::ModelInputParameters 	model_input_params_;
  framefab_msgs::TaskSequenceInputParameters 	task_sequence_input_params_;
  framefab_msgs::RobotInputParameters   robot_input_params_;
  framefab_msgs::OutputSaveDirInputParameters 	output_save_dir_input_params_;

  framefab_msgs::ModelInputParameters 	default_model_input_params_;
  framefab_msgs::TaskSequenceInputParameters 	default_task_sequence_input_params_;
  framefab_msgs::RobotInputParameters   default_robot_input_params_;
  framefab_msgs::OutputSaveDirInputParameters 	default_output_save_dir_input_params_;

  // Parameter loading and saving
  bool save_data_;
  std::string save_location_;
  std::string param_cache_prefix_;
};

#endif
