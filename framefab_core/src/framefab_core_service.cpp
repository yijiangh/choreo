#include <framefab_core/framefab_core_service.h>

#include <framefab_param_helpers/framefab_param_helpers.h>

//subscribed services
#include <framefab_msgs/PathPostProcessing.h>
#include <framefab_msgs/ProcessPlanning.h>

// For visualizing in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// topics and services
const static std::string SAVE_DATA_BOOL_PARAM = "save_data";
const static std::string SAVE_LOCATION_PARAM = "save_location";

// provided services
const static std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";
const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH_SERVICE= "visualize_select_path";

// subscribed services
const static std::string PATH_POST_PROCESSING_SERVICE = "path_post_processing";
const static std::string PROCESS_PLANNING_SERVICE = "process_planning";

// Default filepaths and namespaces for caching stored parameters
const static std::string MODEL_INPUT_PARAMS_FILE = "model_input_parameters.msg";
const static std::string PATH_INPUT_PARAMS_FILE = "path_input_parameters.msg";

// Visualization Maker topics
const static std::string PATH_VISUAL_TOPIC = "path_visualization";

// action server name - note: must be same to client's name
const static std::string FRAMEFAB_EXE_ACTION_SERVER_NAME = "framefab_process_execution_as";
const static std::string PATH_PLANNING_ACTION_SERVER_NAME = "path_planning_action";
const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME = "process_planning_action";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

const static int PROCESS_EXE_BUFFER = 5;  // Additional time [s] buffer between when blending should end and timeout

FrameFabCoreService::FrameFabCoreService()
    : save_data_(false),
      framefab_exe_client_(FRAMEFAB_EXE_ACTION_SERVER_NAME, true),
      path_planning_server_(nh_, PATH_PLANNING_ACTION_SERVER_NAME,
                            boost::bind(&FrameFabCoreService::pathPlanningActionCallback, this, _1), false),
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

  if (!this->load_model_input_parameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load model input parameters.");

  if (!this->load_path_input_parameters(param_cache_prefix_ + PATH_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load path input parameters.");

  // load plugins (if-need-be)

  // TODO: save default parameters

  // service servers
  framefab_parameters_server_ =
      nh_.advertiseService(FRAMEFAB_PARAMETERS_SERVICE,
                           &FrameFabCoreService::framefab_parameters_server_callback, this);

  element_number_sequest_server_ =
      nh_.advertiseService(ELEMENT_NUMBER_REQUEST_SERVICE,
                           &FrameFabCoreService::element_number_sequest_server_callback, this);

  visualize_selected_path_server_ =
      nh_.advertiseService(VISUALIZE_SELECTED_PATH_SERVICE,
                           &FrameFabCoreService::visualize_selected_path_server_callback, this);

  // start local instances
  visual_tool_.init("world_frame", PATH_VISUAL_TOPIC);

  // start server

  // service clients
  path_post_processing_client_ = nh_.serviceClient<framefab_msgs::PathPostProcessing>(PATH_POST_PROCESSING_SERVICE);
  process_planning_client_ = nh_.serviceClient<framefab_msgs::ProcessPlanning>(PROCESS_PLANNING_SERVICE);

  // publishers

  // action servers
  path_planning_server_.start();
  process_planning_server_.start();
  simulate_motion_plan_server_.start();

  return true;
}

void FrameFabCoreService::run()
{
  ros::Duration loop_duration(1.0f);
  while (ros::ok())
  {
    loop_duration.sleep();
  }
}

bool FrameFabCoreService::load_model_input_parameters(const std::string & filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if(framefab_param_helpers::fromFile(filename, model_input_params_))
  {
    return true;
  }

  // otherwise default to the parameter server
  ros::NodeHandle nh("~/model_input");
  return loadParam(nh, "ref_pt_x", model_input_params_.ref_pt_x) &&
      loadParam(nh, "ref_pt_y", model_input_params_.ref_pt_y) &&
      loadParam(nh, "ref_pt_z", model_input_params_.ref_pt_z) &&
      loadParam(nh, "unit_type", model_input_params_.unit_type) &&
      loadParam(nh, "element_diameter", model_input_params_.element_diameter) &&
      loadParam(nh, "shrink_length", model_input_params_.shrink_length);
}


void FrameFabCoreService::save_model_input_parameters(const std::string& filename)
{
  if (!framefab_param_helpers::toFile(filename, model_input_params_))
  {
    ROS_WARN_STREAM("Unable to save model input parameters to: " << filename);
  }
}

bool FrameFabCoreService::load_path_input_parameters(const std::string & filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if(framefab_param_helpers::fromFile(filename, path_input_params_))
  {
    return true;
  }

  // otherwise default to the parameter server
  ros::NodeHandle nh("~/path_input_params");
  return loadParam(nh, "file_path", path_input_params_.file_path);
}

void FrameFabCoreService::save_path_input_parameters(const std::string & filename)
{
  if(!framefab_param_helpers::toFile(filename, path_input_params_))
  {
    ROS_WARN_STREAM("Unable to save path input parameters to: " << filename);
  }
}

bool FrameFabCoreService::framefab_parameters_server_callback(
    framefab_msgs::FrameFabParameters::Request& req,
    framefab_msgs::FrameFabParameters::Response& res)
{
  switch (req.action)
  {
    case framefab_msgs::FrameFabParameters::Request::GET_CURRENT_PARAMETERS:
      res.model_params = model_input_params_;
      res.path_params  = path_input_params_;
      break;

    case framefab_msgs::FrameFabParameters::Request::GET_DEFAULT_PARAMETERS:
      res.model_params = default_model_input_params_;
      res.path_params  = default_path_input_params_;
      break;

      // Update the current parameters in this service
    case framefab_msgs::FrameFabParameters::Request::SET_PARAMETERS:
    case framefab_msgs::FrameFabParameters::Request::SAVE_PARAMETERS:
      model_input_params_ = req.model_params;
      path_input_params_ = req.path_params;

      if (req.action == framefab_msgs::FrameFabParameters::Request::SAVE_PARAMETERS)
      {
        this->save_model_input_parameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE);
        this->save_path_input_parameters(param_cache_prefix_ + PATH_INPUT_PARAMS_FILE);
      }
      break;
  }

  res.succeeded = true;
  return true;
}

bool FrameFabCoreService::element_number_sequest_server_callback(
    framefab_msgs::ElementNumberRequest::Request& req,
    framefab_msgs::ElementNumberRequest::Response& res)
{
  res.element_number = visual_tool_.getPathArraySize();
}

bool FrameFabCoreService::visualize_selected_path_server_callback(
    framefab_msgs::VisualizeSelectedPath::Request& req,
    framefab_msgs::VisualizeSelectedPath::Response& res)
{
  if(req.index != -1)
  {
    visual_tool_.visualizePath(req.index);
    visual_tool_.visualizeFeasibleOrientations(req.index, true);
    res.succeeded = true;
  }
  else
  {
    visual_tool_.cleanUpAllPaths();
    res.succeeded = true;
  }
}

void FrameFabCoreService::pathPlanningActionCallback(const framefab_msgs::PathPlanningGoalConstPtr &goal_in)
{
  switch (goal_in->action)
  {
    case framefab_msgs::PathPlanningGoal::FIND_AND_PROCESS:
    {
      path_planning_feedback_.last_completed = "Recieved request to post process plan\n";
      path_planning_server_.publishFeedback(path_planning_feedback_);

      // call path_post_processing srv
      framefab_msgs::PathPostProcessing srv;
      srv.request.action = srv.request.PROCESS_PATH_AND_MARKER;
      srv.request.model_params = model_input_params_;
      srv.request.path_params = path_input_params_;

      if(!path_post_processing_client_.call(srv))
      {
        ROS_WARN_STREAM("Unable to call path post processing service");
        path_planning_feedback_.last_completed = "Failed to call Path Post Processing Service!\n";
        path_planning_server_.publishFeedback(path_planning_feedback_);
        path_planning_result_.succeeded = false;
        path_planning_server_.setAborted(path_planning_result_);
      }
      else
      {
        // take srv output, save them
        path_planning_feedback_.last_completed = "Finished path post processing. Visualizing...\n";
        path_planning_server_.publishFeedback(path_planning_feedback_);

        // import data into visual_tools
        visual_tool_.setProcessPath(srv.response.process);
        visual_tool_.visualizeAllPaths();

        // import data into process_planning_visualizer
        process_paths_ = srv.response.process;

        path_planning_result_.succeeded = true;
        path_planning_server_.setSucceeded(path_planning_result_);
      }
      break;
    }
    default:
    {
      // NOT SUPPORTING OTHER ACTION GOAL NOW
      ROS_ERROR_STREAM("Unknown action code '" << goal_in->action << "' request");
      break;
    }
  }
  path_planning_result_.succeeded = false;
}

void FrameFabCoreService::processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal_in)
{
  switch (goal_in->action)
  {
    case framefab_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW:
    {
      process_planning_feedback_.last_completed = "Recieved request to generate motion plan\n";
      process_planning_server_.publishFeedback(process_planning_feedback_);

      visual_tool_.cleanUpAllPaths();
      visual_tool_.visualizePath(goal_in->index);
      visual_tool_.visualizeFeasibleOrientations(goal_in->index, false);

      // TODO: make a trajectory library and ui for user to choose
      bool success = generateMotionLibrary(goal_in->index, trajectory_library_);

      if(success)
      {
        process_planning_feedback_.last_completed = "Finished planning. Visualizing...\n";
        process_planning_server_.publishFeedback(process_planning_feedback_);
//      visualizePaths();
        process_planning_result_.succeeded = true;
        process_planning_server_.setSucceeded(process_planning_result_);

        return;
      }
      else
      {
        process_planning_feedback_.last_completed = "Process Planning action failed.\n";
        process_planning_server_.publishFeedback(process_planning_feedback_);
//      visualizePaths();
        process_planning_result_.succeeded = false;
        process_planning_server_.setAborted(process_planning_result_);
      }
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Unknown action code '" << goal_in->action << "' request");
      break;
    }
  }
}

void FrameFabCoreService::simulateMotionPlansActionCallback(const framefab_msgs::SimulateMotionPlanGoalConstPtr& goal_in)
{
  framefab_msgs::SimulateMotionPlanResult res;

  std::string lib_sort_id = std::to_string(goal_in->index);

  // If plan does not exist, abort and return
  if (trajectory_library_.get().find(lib_sort_id) == trajectory_library_.get().end())
  {
    ROS_WARN_STREAM("Motion plan #" << lib_sort_id << " does not exist. Cannot execute.");
    res.code = framefab_msgs::SimulateMotionPlanResult::NO_SUCH_NAME;
    simulate_motion_plan_server_.setAborted(res);
    return;
  }

  // Send command to execution server
  framefab_msgs::ProcessExecutionActionGoal goal;
  goal.goal.trajectory_connection = trajectory_library_.get()[lib_sort_id].trajectory_connection;
  goal.goal.trajectory_approach = trajectory_library_.get()[lib_sort_id].trajectory_approach;
  goal.goal.trajectory_process = trajectory_library_.get()[lib_sort_id].trajectory_process;
  goal.goal.trajectory_depart = trajectory_library_.get()[lib_sort_id].trajectory_depart;
  goal.goal.wait_for_execution = goal_in->wait_for_execution;
  goal.goal.simulate = goal_in->simulate;

  actionlib::SimpleActionClient<framefab_msgs::ProcessExecutionAction> *exe_client = &framefab_exe_client_;
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
    res.code = framefab_msgs::SimulateMotionPlanResult::TIMEOUT;
    simulate_motion_plan_server_.setAborted(res);
  }
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
