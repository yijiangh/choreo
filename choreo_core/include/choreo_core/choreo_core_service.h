#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <ros/ros.h>

// served service
#include <choreo_msgs/ElementNumberRequest.h>
#include <choreo_msgs/VisualizeSelectedPath.h>
#include <choreo_msgs/GetAvailableProcessPlans.h>
#include <choreo_msgs/OutputProcessPlans.h>
#include <choreo_msgs/QueryComputationRecord.h>

// msgs
#include <choreo_msgs/ChoreoParameters.h>
#include <choreo_msgs/ModelInputParameters.h>
#include <choreo_msgs/TaskSequenceInputParameters.h>
#include <choreo_msgs/RobotInputParameters.h>
#include <choreo_msgs/OutputSaveDirInputParameters.h>
#include <choreo_msgs/ElementCandidatePoses.h>
#include <choreo_msgs/UnitProcessPlan.h>

// actions
#include <choreo_msgs/TaskSequenceProcessingAction.h>
#include <choreo_msgs/TaskSequencePlanningAction.h>
#include <choreo_msgs/ProcessPlanningAction.h>
#include <choreo_msgs/ProcessExecutionAction.h>
#include <choreo_msgs/SimulateMotionPlanAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// helper functions
#include "choreo_core/trajectory_library.h"

// visualizer
#include <choreo_visual_tools/choreo_visual_tools.h>

// TODO: we should avoid using this kind of "alias". It decrease code's readability.
struct ProcessPlanResult
{
  std::vector<choreo_msgs::UnitProcessPlan> plans;
};

// this package is the central monitoring node that connects
// Choreo's specialized packages together. It receives requests from UI
// package and dispatch / receive computation request to responsible packages.

class ChoreoCoreService
{
 public:
  ChoreoCoreService();

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
  bool choreoParametersServerCallback(choreo_msgs::ChoreoParameters::Request& req,
                                        choreo_msgs::ChoreoParameters::Response& res);

  // reponding to <path selection> (until which index we compute)
  // and <plan selection> element number query in UI
  bool elementNumberRequestServerCallback(choreo_msgs::ElementNumberRequest::Request& req,
                                          choreo_msgs::ElementNumberRequest::Response& res);

  // visualize selected assembly sequence and grasp (w or w/o end effector)
  bool visualizeSelectedPathServerCallback(choreo_msgs::VisualizeSelectedPath::Request& req,
                                           choreo_msgs::VisualizeSelectedPath::Response& res);

  bool getAvailableProcessPlansCallback(choreo_msgs::GetAvailableProcessPlans::Request& req,
                                        choreo_msgs::GetAvailableProcessPlans::Response& res);

  bool outputProcessPlansCallback(choreo_msgs::OutputProcessPlans::Request& req,
                                  choreo_msgs::OutputProcessPlans::Response& res);

  bool queryComputationResultCallback(choreo_msgs::QueryComputationRecord::Request& req,
                                      choreo_msgs::QueryComputationRecord::Response& res);

  // Action callbacks
  void taskSequenceProcessingActionCallback(const choreo_msgs::TaskSequenceProcessingGoalConstPtr &goal);
  void taskSequencePlanningActionCallback(const choreo_msgs::TaskSequencePlanningGoalConstPtr &goal);
  void processPlanningActionCallback(const choreo_msgs::ProcessPlanningGoalConstPtr &goal);
  void simulateMotionPlansActionCallback(const choreo_msgs::SimulateMotionPlanGoalConstPtr& goal_in);

  // Process Planning - these process planning related
  // methods are defined in src/choreo_core_service_process_planning.cpp
  bool generateMotionLibrary(
      const int selected_path_index,
      choreo_core_service::TrajectoryLibrary& traj_lib);

  // call task sequence planner to request collision objects
  // and then call process planner node to trigger process planning
  // TODO:
  ProcessPlanResult generateProcessPlan(const int& index);

  // immediate plan & execution for resetting the robot
  bool moveToTargetJointPose(std::vector<double> joint_pose);

  // TODO: complete this
  void adjustSimSpeed(double sim_speed);

  bool generatePicknPlaceMotionLibrary();

 private:
  // Services offered by this class
  ros::ServiceServer choreo_parameters_server_;
  ros::ServiceServer element_number_sequest_server_;
  ros::ServiceServer visualize_selected_path_server_;
  ros::ServiceServer get_available_process_plans_server_;
  ros::ServiceServer output_process_plans_server_;
  ros::ServiceServer query_computation_result_server_;

  // Services subscribed to by this class
  ros::ServiceClient task_sequence_processing_srv_client_;
  ros::ServiceClient task_sequence_planning_srv_client_;
  ros::ServiceClient process_planning_client_;
  ros::ServiceClient move_to_pose_client_;
  ros::ServiceClient output_processing_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<choreo_msgs::TaskSequenceProcessingAction> task_sequence_processing_server_;
  choreo_msgs::TaskSequenceProcessingFeedback task_sequence_processing_feedback_;
  choreo_msgs::TaskSequenceProcessingResult task_sequence_processing_result_;

  actionlib::SimpleActionServer<choreo_msgs::TaskSequencePlanningAction> task_sequence_planning_server_;
  choreo_msgs::TaskSequencePlanningFeedback task_sequence_planning_feedback_;
  choreo_msgs::TaskSequencePlanningResult task_sequence_planning_result_;

  actionlib::SimpleActionServer<choreo_msgs::ProcessPlanningAction> process_planning_server_;
  choreo_msgs::ProcessPlanningFeedback process_planning_feedback_;
  choreo_msgs::ProcessPlanningResult process_planning_result_;

  actionlib::SimpleActionServer<choreo_msgs::SimulateMotionPlanAction> simulate_motion_plan_server_;









  choreo_msgs::SimulateMotionPlanFeedback simulate_motion_plan_feedback_;
  choreo_msgs::SimulateMotionPlanResult simulate_motion_plan_result_;

  // Actions subscribed to by this class
  actionlib::SimpleActionClient<choreo_msgs::ProcessExecutionAction> choreo_exe_client_;

  // Visualizer for imported geometry, assembly sequence, and grasps
  choreo_visual_tools::ChoreoVisualTools visual_tools_;

  // TODO: should remove this
  // working environment collision objects
  std::vector<moveit_msgs::CollisionObject> env_objs_;

  // formulated task sequence results, parsed from task sequence processor
  std::vector<choreo_msgs::ElementCandidatePoses> task_sequence_;

  // TODO: parsed assembly seqence
  choreo_msgs::AssemblySequencePickNPlace as_pnp_;

  // Trajectory library
  int selected_task_id_;
  bool use_saved_graph_;
  choreo_core_service::TrajectoryLibrary trajectory_library_;

  // Parameters
  choreo_msgs::ModelInputParameters model_input_params_;
  choreo_msgs::TaskSequenceInputParameters task_sequence_input_params_;
  choreo_msgs::RobotInputParameters robot_input_params_;
  choreo_msgs::OutputSaveDirInputParameters 	output_save_dir_input_params_;

  choreo_msgs::ModelInputParameters default_model_input_params_;
  choreo_msgs::TaskSequenceInputParameters default_task_sequence_input_params_;
  choreo_msgs::RobotInputParameters default_robot_input_params_;
  choreo_msgs::OutputSaveDirInputParameters default_output_save_dir_input_params_;

  // Parameter loading and saving
  bool save_data_;
  std::string save_location_;
  std::string param_cache_prefix_;
  std::string world_frame_;

  std::string assembly_type_;
};

#endif