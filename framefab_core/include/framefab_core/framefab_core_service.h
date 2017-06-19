/*
  Copyright May 7, 2014 Southwest Research Institute
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <framefab_msgs/ProcessPlan.h>

#include <framefab_msgs/BlendProcessPlanning.h>
#include <framefab_msgs/PathPlanning.h>
#include <framefab_msgs/PathPlanningParameters.h>

#include <framefab_msgs/ProcessExecutionAction.h>
#include <framefab_msgs/ProcessPlanningAction.h>
#include <framefab_msgs/SelectMotionPlanAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <framefab_process_path_generation/VisualizeBlendingPlan.h>
#include <framefab_process_path_generation/utils.h>
#include <framefab_process_path_generation/polygon_utils.h>

#include <services/trajectory_library.h>
/*#include <coordination/data_coordinator.h>*/

//  marker namespaces
/*const static std::string BOUNDARY_NAMESPACE = "process_boundary";*/
/*const static std::string PATH_NAMESPACE = "process_path";*/
/*const static std::string TOOL_NAMESPACE = "process_tool";*/
namespace framfab_core
{
struct ProcessPathDetails
{
  std::vector <std::vector<geometry_msgs::PoseArray>> blend_poses_;
  std::vector <geometry_msgs::PoseArray> edge_poses_;
  std::vector <std::vector<geometry_msgs::PoseArray>> scan_poses_;
};

/**
 * Associates a name with a visual msgs marker which contains a pose and sequence of points defining
 * a path
 */
struct ProcessPathResult
{
  typedef std::pair <std::string, std::vector<geometry_msgs::PoseArray>> value_type;
  std::vector <value_type> paths;
};

/**
 * Associates a name with a joint trajectory
 */
struct ProcessPlanResult
{
  typedef std::pair <std::string, framefab_msgs::ProcessPlan> value_type;
  std::vector <value_type> plans;
};

class FrameFabCoreService
{
 public:
  FrameFabCoreService();

  bool init();
  void run();

 private:
  bool load_blend_parameters(const std::string &filename);
  void save_blend_parameters(const std::string &filename);
  bool load_path_planning_parameters(const std::string &filename);
  void save_path_planning_parameters(const std::string &filename);

  /**
   * The following path generation and planning methods are defined in
   * src/blending_service_path_generation.cpp
   */
  bool generate_process_plan(framefab_process_path_generation::VisualizeBlendingPlan &process_plan);

  // Service callbacks, these components drive this class by signalling events
  // from the user
  bool surface_detection_server_callback(framefab_msgs::SurfaceDetection::Request &req,
                                         framefab_msgs::SurfaceDetection::Response &res);

  bool select_surface_server_callback(framefab_msgs::SelectSurface::Request &req,
                                      framefab_msgs::SelectSurface::Response &res);

  void processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal);

  void selectMotionPlansActionCallback(const framefab_msgs::SelectMotionPlanGoalConstPtr &goal_in);

  bool
  surface_blend_parameters_server_callback(framefab_msgs::FrameFabCoreParameters::Request &req,
                                           framefab_msgs::FrameFabCoreParameters::Response &res);

  // Reads from the surface selection server and generates blend/scan paths for each
  framefab_surface_detection::TrajectoryLibrary
  generateMotionLibrary(const framefab_msgs::PathPlanningParameters &params);

  bool generateProcessPath(const int &id, ProcessPathResult &result);

  bool generateProcessPath(const int &id,
                           const std::string &name,
                           const pcl::PolygonMesh &mesh,
                           const framefab_surface_detection::detection::CloudRGB::Ptr,
                           ProcessPathResult &result);

  bool generateBlendPath(const framefab_msgs::PathPlanningParameters &params,
                         const pcl::PolygonMesh &mesh,
                         std::vector <geometry_msgs::PoseArray> &result);

  bool generateEdgePath(framefab_surface_detection::detection::CloudRGB::Ptr surface,
                        std::vector <geometry_msgs::PoseArray> &result);

  ProcessPlanResult generateProcessPlan(const std::string &name,
                                        const std::vector <geometry_msgs::PoseArray> &path,
                                        const framefab_msgs::BlendingPlanParameters &params,
                                        const framefab_msgs::ScanPlanParameters &scan_params);

  void visualizePaths();

  void visualizePathPoses();

  void visualizePathStrips();

  std::string getBlendToolPlanningPluginName() const;

  std::string getScanToolPlanningPluginName() const;

  // Services offered by this class
  ros::ServiceServer surface_detect_server_;
  ros::ServiceServer select_surface_server_;
  ros::ServiceServer surf_blend_parameters_server_;

  ros::ServiceServer get_motion_plans_server_;
  ros::ServiceServer load_save_motion_plan_server_;
  ros::ServiceServer rename_suface_server_;

  // Services subscribed to by this class
  ros::ServiceClient process_path_client_;
  ros::ServiceClient trajectory_planner_client_;

  ros::ServiceClient blend_planning_client_;
  ros::ServiceClient keyence_planning_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer <framefab_msgs::ProcessPlanningAction> process_planning_server_;
  actionlib::SimpleActionServer <framefab_msgs::SelectMotionPlanAction> select_motion_plan_server_;
  framefab_msgs::ProcessPlanningFeedback process_planning_feedback_;
  framefab_msgs::ProcessPlanningResult process_planning_result_;

  // Actions subscribed to by this class
  actionlib::SimpleActionClient <framefab_msgs::ProcessExecutionAction> blend_exe_client_;

  // Current state publishers
  ros::Publisher selected_surf_changed_pub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher tool_path_markers_pub_;
  ros::Publisher blend_visualization_pub_;
  ros::Publisher edge_visualization_pub_;
  ros::Publisher scan_visualization_pub_;

  // Timers
  bool stop_tool_animation_;

//  // marker server instance
//  framefab_surface_detection::interactive::InteractiveSurfaceServer surface_server_;
  // data coordinator
//  framefab_surface_detection::data::DataCoordinator data_coordinator_;

  // parameters
  framefab_msgs::BlendingPlanParameters default_blending_plan_params_;
  framefab_msgs::ScanPlanParameters default_scan_params_;
  framefab_msgs::PathPlanningParameters default_path_planning_params_;
  framefab_msgs::BlendingPlanParameters blending_plan_params_;
  framefab_msgs::PathPlanningParameters path_planning_params_;

  // results
  ProcessPathDetails process_path_results_;
  std::vector <ProcessPlanResult::value_type> process_poses_;
  std::vector <std::vector<ros::Duration>>    duration_results_;
  // returned by visualize plan service, needed by trajectory planner

  // parameters
  bool save_data_;
  std::string save_location_;

  // msgs

  int marker_counter_;

  // Parameter loading and saving
  std::string param_cache_prefix_;
};
} // namespace

#endif
