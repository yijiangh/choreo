#include <framefab_core/framefab_core_service.h>

// srv
#include <framefab_msgs/TrajectoryExecution.h>

// Process Planning
#include <framefab_msgs/BlendProcessPlanning.h>
#include <framefab_msgs/KeyenceProcessPlanning.h>
#include <framefab_msgs/PathPlanning.h>

#include <framefab_param_helpers/framefab_param_helpers.h>

// topics and services
const static std::string SAVE_DATA_BOOL_PARAM = "save_data";
const static std::string SAVE_LOCATION_PARAM = "save_location";
const static std::string TRAJECTORY_PLANNING_SERVICE = "trajectory_planner";
const static std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";
const static std::string PROCESS_PATH_SERVICE = "process_path";
const static std::string PATH_GENERATION_SERVICE = "process_path_generator";
const static std::string VISUALIZE_BLENDING_PATH_SERVICE = "visualize_path_generator";

const static std::string PROCESS_EXECUTION_SERVICE = "process_execution";
const static std::string PROCESS_PLANNING_SERVICE = "process_planning";

const static std::string TOOL_PATH_PREVIEW_TOPIC = "tool_path_preview";
const static std::string EDGE_VISUALIZATION_TOPIC = "edge_visualization";
const static std::string BLEND_VISUALIZATION_TOPIC = "blend_visualization";
const static std::string SCAN_VISUALIZATION_TOPIC = "scan_visualization";
const static std::string ROBOT_SCAN_PATH_PREVIEW_TOPIC = "robot_scan_path_preview";

const static std::string EDGE_IDENTIFIER = "_edge_";

//  tool visual properties
//const static float TOOL_DIA = .050;
//const static float TOOL_THK = .005;
//const static float TOOL_SHAFT_DIA = .006;
//const static float TOOL_SHAFT_LEN = .045;
//const static std::string TOOL_FRAME_ID = "process_tool";

// Default filepaths and namespaces for caching stored parameters
const static std::string MODEL_INPUT_PARAMS_FILE = "framefab_model_input_parameters.msg";
//const static std::string PATH_PLANNING_PARAMS_FILE = "framefab_path_planning_parameters.msg";
const static std::string PROCESS_PLANNING_PARAMS_FILE = "framefab_process_planning_parameters.msg";

// action server name
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "process_execution_as";
const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME = "process_planning_as";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";
const static int PROCESS_EXE_BUFFER = 5;  // Additional time [s] buffer between when blending should end and timeout

FrameFabCoreService::FrameFabCoreService() : save_data_(false),
  process_exe_client_(PROCESS_EXE_ACTION_SERVER_NAME, true),
  process_planning_server_(nh_, PROCESS_PLANNING_ACTION_SERVER_NAME,
                           boost::bind(&FrameFabCoreService::processPlanningActionCallback, this, _1), false),
  simulate_motion_plan_server_(nh_, SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME,
                             boost::bind(&FrameFabCoreService::simulateMotionPlansActionCallback, this, _1), false)
{}

bool FrameFabCoreService::init()
{
  ros::NodeHandle ph("~");

  // loading parameters
  ph.getParam(SAVE_DATA_BOOL_PARAM, save_data_);
  ph.getParam(SAVE_LOCATION_PARAM, save_location_);

  // Load the 'prefix' that will be combined with parameters msg base names to save to disk
  ph.param<std::string>("param_cache_prefix", param_cache_prefix_, "");

  if (!this->load_process_planning_parameters(param_cache_prefix_ + PROCESS_PLANNING_PARAMS_FILE))
    ROS_WARN("Unable to load framefab process parameters.");

  if (!this->load_model_input_parameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load framefab process parameters.");

  //TODO: make sure action (sequence) planning component is loaded

  // save default parameters
  default_model_input_plan_params_ = model_input_plan_params_;

//  if (surface_server_.init() && data_coordinator_.init())
//  {
//    // adding callbacks
//    ROS_INFO_STREAM("Surface detection service initialization succeeded");
//  }
//  else
//  {
//    ROS_ERROR_STREAM("Surface detection service had an initialization error");
//  }

  // start server
//  interactive::InteractiveSurfaceServer::SelectionCallback f =
//      boost::bind(&FrameFabCoreService::publish_selected_surfaces_changed, this);
//  surface_server_.add_selection_callback(f);

  // service clients
//  process_path_client_ = nh_.serviceClient<framefab_msgs::PathPlanning>(PATH_GENERATION_SERVICE);

  // Process Execution Parameters
  blend_planning_client_ = nh_.serviceClient<framefab_msgs::BlendProcessPlanning>(PROCESS_PLANNING_SERVICE);

  surf_blend_parameters_server_ =
      nh_.advertiseService(FRAMEFAB_PARAMETERS_SERVICE,
                          &FrameFabCoreService::framefab_parameters_server_callback, this);

  get_motion_plans_server_ = nh_.advertiseService(
      SIMULATE_MOTION_PLANS_SERVICE, &FrameFabCoreService::simulateMotionPlansCallback, this);

  // publishers
  tool_path_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(TOOL_PATH_PREVIEW_TOPIC, 1, true);
  blend_visualization_pub_ = nh_.advertise<geometry_msgs::PoseArray>(BLEND_VISUALIZATION_TOPIC, 1, true);
  edge_visualization_pub_ = nh_.advertise<geometry_msgs::PoseArray>(EDGE_VISUALIZATION_TOPIC, 1, true);
  scan_visualization_pub_ = nh_.advertise<geometry_msgs::PoseArray>(SCAN_VISUALIZATION_TOPIC, 1, true);

  // action servers
  process_planning_server_.start();
  simulate_motion_plan_server_.start();

  return true;
}

void FrameFabCoreService::run()
{
  surface_server_.run();

  ros::Duration loop_duration(1.0f);
  while (ros::ok())
  {
    loop_duration.sleep();
  }
}

// Blending Parameters
bool FrameFabCoreService::load_model_input_parameters(const std::string& filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if (framefab_param_helpers::fromFile(filename, model_input_plan_params_))
  {
    return true;
  }
  // otherwise default to the parameter server
  ros::NodeHandle nh("~/model_input");
  return loadParam(nh, "ref_pt_x", blending_plan_params_.ref_pt_x) &&
         loadParam(nh, "ref_pt_y", blending_plan_params_.ref_pt_x) &&
         loadParam(nh, "ref_pt_z", blending_plan_params_.ref_pt_x) &&
         loadParam(nh, "unit_type", blending_plan_params_.unit_type) &&
         loadParam(nh, "file_name", blending_plan_params_.file_name);
}


void FrameFabCoreService::save_model_input_parameters(const std::string& filename)
{
  if (!framefab_param_helpers::toFile(filename, model_input_plan_params_))
  {
    ROS_WARN_STREAM("Unable to save model input parameters to: " << filename);
  }
}


bool FrameFabCoreService::load_process_planning_parameters(const std::string & filename)
{
  if(framefab_param_helpers::fromFile(filename, path_planning_params_))
  {
    return true;
  }
  return false;
}


void FrameFabCoreService::save_process_planning_parameters(const std::string & filename)
{
  if(!framefab_param_helpers::toFile(filename, path_planning_params_))
  {
    ROS_WARN_STREAM("Unable to save path-planning parameters to: " << filename);
  }
}

void FrameFabCoreService::processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal_in)
{
  switch (goal_in->action)
  {
    case framefab_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW:
    {
      process_planning_feedback_.last_completed = "Recieved request to generate motion plan";
      process_planning_server_.publishFeedback(process_planning_feedback_);
      process_planning_feedback_.last_completed = "Finished planning. Visualizing...";
      process_planning_server_.publishFeedback(process_planning_feedback_);
      visualizePaths();
      process_planning_result_.succeeded = true;
      process_planning_server_.setSucceeded(process_planning_result_);
      break;
    }
    case framefab_msgs::ProcessPlanningGoal::PREVIEW_TOOL_PATH:
    {
      process_planning_feedback_.last_completed = "Recieved request to preview tool path";
      process_planning_server_.publishFeedback(process_planning_feedback_);
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Unknown action code '" << goal_in->action << "' request");
      break;
    }
  }

  process_planning_result_.succeeded = false;
}

bool FrameFabCoreService::framefab_parameters_server_callback(
    framefab_msgs::FrameFabCoreParameters::Request& req,
    framefab_msgs::FrameFabCoreParameters::Response& res)
{
  switch (req.action)
  {
  case framefab_msgs::FrameFabCoreParameters::Request::GET_CURRENT_PARAMETERS:

    res.model_input_params  = model_input_params_;
    res.process_plan_params = process_plan_params_;
    break;

  case framefab_msgs::FrameFabCoreParameters::Request::GET_DEFAULT_PARAMETERS:

    res.model_input_params  = default_model_input_params_;
    res.process_plan_params = default_process_plan_params_;
    break;

  // Update the current parameters in this service
  case framefab_msgs::FrameFabCoreParameters::Request::SET_PARAMETERS:
  case framefab_msgs::FrameFabCoreParameters::Request::SAVE_PARAMETERS:
    model_input_params_       = req.model_input_params_;
    process_planning_params_  = req.process_plan_params;

    if (req.action == framefab_msgs::FrameFabCoreParameters::Request::SAVE_PARAMETERS)
    {
      this->save_model_input_parameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE);
      this->save_process_planning_parameters(param_cache_prefix_ + PROCESS_PLANNING_PARAMS_FILE);
    }
    break;
  }

  return true;
}

void FrameFabCoreService::simulateMotionPlansActionCallback(const framefab_msgs::SimulateMotionPlanGoalConstPtr& goal_in)
{
  framefab_msgs::SimulateMotionPlanResult res;

//  // If plan does not exist, abort and return
//  if (trajectory_library_.get().find(goal_in->name) == trajectory_library_.get().end())
//  {
//    ROS_WARN_STREAM("Motion plan " << goal_in->name << " does not exist. Cannot execute.");
//    res.code = framefab_msgs::SimulateMotionPlanResponse::NO_SUCH_NAME;
//    simulate_motion_plan_server_.setAborted(res);
//    return;
//  }

  bool is_blend = trajectory_library_.get()[goal_in->name].type == framefab_msgs::ProcessPlan::BLEND_TYPE;

  // Send command to execution server
  framefab_msgs::ProcessExecutionActionGoal goal;
  goal.goal.trajectory_approach = trajectory_library_.get()[goal_in->name].trajectory_approach;
  goal.goal.trajectory_depart = trajectory_library_.get()[goal_in->name].trajectory_depart;
  goal.goal.trajectory_process = trajectory_library_.get()[goal_in->name].trajectory_process;
  goal.goal.wait_for_execution = goal_in->wait_for_execution;
  goal.goal.simulate = goal_in->simulate;

  actionlib::SimpleActionClient<framefab_msgs::ProcessExecutionAction> *exe_client = process_exe_client_;
  exe_client->sendGoal(goal.goal);

  ros::Duration process_time(goal.goal.trajectory_depart.points.back().time_from_start);
  ros::Duration buffer_time(PROCESS_EXE_BUFFER);
  if(exe_client->waitForResult(process_time + buffer_time))
  {
    res.code = framefab_msgs::SimulateMotionPlanResult::SUCCESS;
    simulate_motion_plan_server_.setSucceeded(res);
  }
  else
  {
    res.code=framefab_msgs::simulateMotionPlanResult::TIMEOUT;
    simulate_motion_plan_server_.setAborted(res);
  }
}

void FrameFabCoreService::visualizePaths()
{
  visualizePathPoses();

  visualizePathStrips();
}

void FrameFabCoreservice::visualizepathposes()
{
  // publish poses
  geometry_msgs::posearray blend_poses, edge_poses, scan_poses;
  blend_poses.header.frame_id = edge_poses.header.frame_id = scan_poses.header.frame_id = "world_frame";
  blend_poses.header.stamp = edge_poses.header.stamp = scan_poses.header.stamp = ros::time::now();

  for (const auto& path : process_path_results_.blend_poses_)
  {
    for (const auto& pose_array : path)
    {
      blend_poses.poses.insert(blend_poses.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
    }
  }

  for(const auto& pose_array : process_path_results_.edge_poses_)
    edge_poses.poses.insert(edge_poses.poses.end(), pose_array.poses.begin(), pose_array.poses.end());

  for (const auto& path : process_path_results_.scan_poses_)
  {
    for(const auto& pose_array : path)
    {
      scan_poses.poses.insert(scan_poses.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
    }
  }

  blend_visualization_pub_.publish(blend_poses);
  edge_visualization_pub_.publish(edge_poses);
  scan_visualization_pub_.publish(scan_poses);
}

static visualization_msgs::Marker makeLineStripMarker(const std::string& ns, const int id, const std_msgs::ColorRGBA& color,
                                                      const geometry_msgs::PoseArray& segment)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_frame";
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.ns = ns;
  marker.pose.orientation.w = 1;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.004;
  marker.color = color;
  marker.lifetime = ros::Duration(0.0);

  for (const auto& pose : segment.poses)
    marker.points.push_back(pose.position);

  return marker;
}

void FrameFabCoreService::visualizePathStrips()
{
  visualization_msgs::MarkerArray path_visualization;

  // Visualize Blending Paths
  std_msgs::ColorRGBA blend_color;
  blend_color.r = 1.0;
  blend_color.b = 0.0;
  blend_color.g = 0.0;
  blend_color.a = 0.8;

  const std::string blend_ns = "blend_paths";

  int blend_path_id = 0;

  for (const auto& path : process_path_results_.blend_poses_) // for a given surface
  {
    for (const auto& segment : path) // for a given path segment on the surface
    {
      path_visualization.markers.push_back(makeLineStripMarker(blend_ns, blend_path_id++, blend_color, segment));
    }
  }

  // Visualize scan paths
  std_msgs::ColorRGBA scan_color;
  scan_color.r = 1.0;
  scan_color.b = 0.0;
  scan_color.g = 1.0;
  scan_color.a = 0.8;

  const std::string scan_ns = "scan_paths";

  int scan_path_id = 0;

  for (const auto& path : process_path_results_.scan_poses_) // for a given surface
  {
    for (const auto& segment : path) // for a given path segment on the surface
    {
      path_visualization.markers.push_back(makeLineStripMarker(scan_ns, scan_path_id++, scan_color, segment));
    }
  }

  tool_path_markers_pub_.publish(path_visualization);
}

std::string FrameFabCoreService::getBlendToolPlanningPluginName() const
{
  ros::NodeHandle pnh ("~");
  std::string name;
  if (!pnh.getParam(BLEND_TOOL_PLUGIN_PARAM, name))
  {
    ROS_WARN("Unable to load blend tool planning plugin from ros param '%s'",
             BLEND_TOOL_PLUGIN_PARAM.c_str());
  }

  return name;
}

std::string FrameFabCoreService::getScanToolPlanningPluginName() const
{
  ros::NodeHandle pnh ("~");
  std::string name;
  if (!pnh.getParam(SCAN_TOOL_PLUGIN_PARAM, name))
  {
    ROS_WARN("Unable to load scan tool planning plugin from ros param '%s'",
             SCAN_TOOL_PLUGIN_PARAM.c_str());
  }

  return name;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_core_service");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  FrameFabCoreService core_srv;

  if (core_srv.init())
  {
    ROS_INFO("FrameFab Core Service successfully initialized and now running");
    core_srv.run();
  }

  ros::waitForShutdown();
}
