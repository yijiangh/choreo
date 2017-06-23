#include <framefab_core/framefab_core_service.h>

#include <framefab_param_helpers/framefab_param_helpers.h>

// Process Planning

// topics and services
const static std::string SAVE_DATA_BOOL_PARAM = "save_data";
const static std::string SAVE_LOCATION_PARAM = "save_location";
const static std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";

// Default filepaths and namespaces for caching stored parameters
const static std::string MODEL_INPUT_PARAMS_FILE = "model_input_parameters.msg";
const static std::string PATH_INPUT_PARAMS_FILE = "path_input_parameters.msg";

// action server name

FrameFabCoreService::FrameFabCoreService()
    : save_data_(false)
{}

bool FrameFabCoreService::init()
{
  ros::NodeHandle ph("~");

  // loading parameters
  ph.getParam(SAVE_DATA_BOOL_PARAM, save_data_);
  ph.getParam(SAVE_LOCATION_PARAM, save_location_);

  // Load the 'prefix' that will be combined with parameters msg base names to save to disk
  ph.param<std::string>("param_cache_prefix", param_cache_prefix_, "");

  // service servers
  framefab_parameters_server_ =
      nh_.advertiseService(FRAMEFAB_PARAMETERS_SERVICE,
                           &FrameFabCoreService::framefab_parameters_server_callback, this);

  // publishers

  // action servers

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
  if(framefab_param_helpers::fromFile(filename, model_input_params_))
  {
    return true;
  }
  return false;
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
  if(framefab_param_helpers::fromFile(filename, path_input_params_))
  {
    return true;
  }
  return false;
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

  return true;
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
