#include <framefab_core/framefab_core_service.h>

#include <framefab_param_helpers/framefab_param_helpers.h>

//services
#include <framefab_msgs/PathPostProcessing.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// Process Planning

// topics and services
const static std::string SAVE_DATA_BOOL_PARAM = "save_data";
const static std::string SAVE_LOCATION_PARAM = "save_location";

// provided services
const static std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";
const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH_SERVICE= "visualize_select_path";

// subscribed services
const static std::string PATH_POST_PROCESSING_SERVICE = "path_post_processing";

// Default filepaths and namespaces for caching stored parameters
const static std::string MODEL_INPUT_PARAMS_FILE = "model_input_parameters.msg";
const static std::string PATH_INPUT_PARAMS_FILE = "path_input_parameters.msg";

// Visualization Maker topics
const static std::string PATH_VISUAL_TOPIC = "path_visualization";

// action server name - note: must be same to client's name
const static std::string PATH_PLANNING_ACTION_SERVER_NAME = "path_planning_action";

FrameFabCoreService::FrameFabCoreService()
    : save_data_(false),
      path_planning_server_(nh_, PATH_PLANNING_ACTION_SERVER_NAME,
                            boost::bind(&FrameFabCoreService::pathPlanningActionCallback, this, _1), false)
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
  visual_tool_.init("arm_base_link", PATH_VISUAL_TOPIC);

  // start server

  // service clients
  path_post_processing_client_ = nh_.serviceClient<framefab_msgs::PathPostProcessing>(PATH_POST_PROCESSING_SERVICE);

  // publishers

  // action servers
  path_planning_server_.start();

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
//  ros::NodeHandle nh("~/path_input_params");
//  return loadParam(nh, "file_path", path_input_params_.file_path);
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

        visual_tool_.setProcessPath(srv.response.process);
        visual_tool_.visualizeAllPaths();

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
