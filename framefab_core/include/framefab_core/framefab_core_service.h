#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

// service
#include <framefab_msgs/PathPlanning.h>

// msgs
#include <framefab_msgs/ModelInputParameters.h>

// actions
#include <framefab_msgs/ProcessExecutionAction.h>
#include <framefab_msgs/ProcessPlanningAction.h>
#include <framefab_msgs/SimulateMotionPlanAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

namespace framfab_core
{
struct ProcessPathDetails
{
  std::vector <std::vector<geometry_msgs::PoseArray>> blend_poses_;
  std::vector <geometry_msgs::PoseArray>              edge_poses_;
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

  void processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal);

  void simulateMotionPlansActionCallback(const framefab_msgs::SimulateMotionPlanGoalConstPtr &goal_in);

  bool
  framefab_parameters_server_callback(framefab_msgs::FrameFabCoreParameters::Request &req,
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
  ros::ServiceServer framefab_parameters_server_;

  // Services subscribed to by this class
  ros::ServiceClient process_path_client_;
  ros::ServiceClient trajectory_planner_client_;

  ros::ServiceClient blend_planning_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer <framefab_msgs::ProcessPlanningAction>    process_planning_server_;
  actionlib::SimpleActionServer <framefab_msgs::SimulateMotionPlanAction> simulate_motion_plan_server_;
  framefab_msgs::ProcessPlanningFeedback  process_planning_feedback_;
  framefab_msgs::ProcessPlanningResult    process_planning_result_;

  // Actions subscribed to by this class
  actionlib::SimpleActionClient <framefab_msgs::ProcessExecutionAction> process_exe_client_;

  // Current state publishers
  ros::Publisher selected_surf_changed_pub_;
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
  framefab_msgs::ModelInputParameters       default_model_input_plan_params_;
  framefab_msgs::ProcessPlanningParameters  default_process_planning_params_;

  framefab_msgs::ModelInputParameters       model_input_plan_params_;
  framefab_msgs::ProcessPlanningParameters  process_planning_params_;

  // results
  ProcessPathDetails process_path_results_;
  std::vector <ProcessPlanResult::value_type> process_poses_;
  std::vector <std::vector<ros::Duration>>    duration_results_;
  // returned by visualize plan service, needed by trajectory planner

  // parameters
  bool save_data_;
  std::string save_location_;

  int marker_counter_;

  // Parameter loading and saving
  std::string param_cache_prefix_;
};
} // namespace

#endif
