#include <framefab_core/framefab_core_service.h>

#include <framefab_param_helpers/framefab_param_helpers.h>

//subscribed services
#include <framefab_msgs/TaskSequenceProcessing.h>
#include <framefab_msgs/ProcessPlanning.h>
#include <framefab_msgs/MoveToTargetPose.h>
#include <framefab_msgs/OutputProcessing.h>

// For visualizing in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// msgs
#include <descartes_msgs/LadderGraphList.h>

// topics and services
const static std::string SAVE_DATA_BOOL_PARAM = "save_data";
const static std::string SAVE_LOCATION_PARAM = "save_location";

// provided services
const static std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";
const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH_SERVICE= "visualize_select_path";
const static std::string GET_AVAILABLE_PROCESS_PLANS_SERVICE= "get_available_process_plans";
const static std::string OUTPUT_PROCESS_PLANS_SERVICE= "output_process_plans";
const static std::string QUERY_COMPUTATION_RESULT="query_computation_result";

// subscribed services
const static std::string TASK_SEQUENCE_PROCESSING_SERVICE = "task_sequence_processing";
const static std::string PROCESS_PLANNING_SERVICE = "process_planning";
const static std::string MOVE_TO_TARGET_POSE_SERVICE = "move_to_target_pose";
const static std::string OUTPUT_PROCESSING_SERVICE = "output_processing";

// Default filepaths and namespaces for caching stored parameters
const static std::string MODEL_INPUT_PARAMS_FILE = "model_input_parameters.msg";
const static std::string TASK_SEQUENCE_INPUT_PARAMS_FILE = "task_sequence_input_parameters.msg";
const static std::string ROBOT_INPUT_PARAMS_FILE = "robot_input_parameters.msg";
const static std::string OUTPUT_SAVE_DIR_INPUT_PARAMS_FILE = "output_save_dir_input_parameters.msg";

// Visualization Maker topics
const static std::string PATH_VISUAL_TOPIC = "path_visualization";

// subscribed action server name - note: must be same to client's name
const static std::string FRAMEFAB_EXE_ACTION_SERVER_NAME =              "framefab_execution_as";
const static std::string TASK_SEQUENCE_PROCESSING_ACTION_SERVER_NAME =  "task_sequence_processing_action";
const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME =          "process_planning_action";

// serving action
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

const static int PROCESS_EXE_BUFFER = 5;  // Additional time [s] buffer between when blending should end and timeout

FrameFabCoreService::FrameFabCoreService()
    : save_data_(false),
      selected_task_id_(0),
      framefab_exe_client_(FRAMEFAB_EXE_ACTION_SERVER_NAME, true),
      task_sequence_processing_server_(nh_, TASK_SEQUENCE_PROCESSING_ACTION_SERVER_NAME,
                            boost::bind(&FrameFabCoreService::taskSequenceProcessingActionCallback, this, _1), false),
      process_planning_server_(nh_, PROCESS_PLANNING_ACTION_SERVER_NAME,
                               boost::bind(&FrameFabCoreService::processPlanningActionCallback, this, _1), false),
      simulate_motion_plan_server_(nh_, SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME,
                                   boost::bind(&FrameFabCoreService::simulateMotionPlansActionCallback, this, _1), false)
{}

bool FrameFabCoreService::init()
{
  ros::NodeHandle ph("~");

  // loading parameters
  // TODO: save location is default to $HOME/.ros/ now,
  // should be oriented to some customized path

//  ph.getParam(SAVE_DATA_BOOL_PARAM, save_data_);
//  ph.getParam(SAVE_LOCATION_PARAM, save_location_);

  // Load the 'prefix' that will be combined with parameters msg base names to save to disk
  ph.param<std::string>("param_cache_prefix", param_cache_prefix_, "");

  if (!this->loadModelInputParameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load model input parameters.");

  if (!this->loadTaskSequenceInputParameters(param_cache_prefix_ + TASK_SEQUENCE_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load path input parameters.");

  if (!this->loadRobotInputParameters(param_cache_prefix_ + ROBOT_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load robot input parameters.");

  if (!this->loadOutputSaveDirInputParameters(param_cache_prefix_ + OUTPUT_SAVE_DIR_INPUT_PARAMS_FILE))
    ROS_WARN("Unable to load output path input parameters.");

  // load plugins (if-need-be)

  // TODO: save default parameters

  // service servers
  framefab_parameters_server_ =
      nh_.advertiseService(FRAMEFAB_PARAMETERS_SERVICE,
                           &FrameFabCoreService::framefabParametersServerCallback, this);

  element_number_sequest_server_ =
      nh_.advertiseService(ELEMENT_NUMBER_REQUEST_SERVICE,
                           &FrameFabCoreService::elementNumberRequestServerCallback, this);

  visualize_selected_path_server_ =
      nh_.advertiseService(VISUALIZE_SELECTED_PATH_SERVICE,
                           &FrameFabCoreService::visualizeSelectedPathServerCallback, this);

  get_available_process_plans_server_ = nh_.advertiseService(
      GET_AVAILABLE_PROCESS_PLANS_SERVICE, &FrameFabCoreService::getAvailableProcessPlansCallback, this);

  output_process_plans_server_ = nh_.advertiseService(
      OUTPUT_PROCESS_PLANS_SERVICE, &FrameFabCoreService::outputProcessPlansCallback, this);

  query_computation_result_server_ = nh_.advertiseService(
      QUERY_COMPUTATION_RESULT, &FrameFabCoreService::queryComputationResultCallback, this);

  // start local instances
  visual_tool_.init("world_frame", PATH_VISUAL_TOPIC);

  // start server

  // service clients
  task_sequence_processing_srv_client_ = nh_.serviceClient<framefab_msgs::TaskSequenceProcessing>(TASK_SEQUENCE_PROCESSING_SERVICE);
  process_planning_client_ = nh_.serviceClient<framefab_msgs::ProcessPlanning>(PROCESS_PLANNING_SERVICE);
  move_to_pose_client_  = nh_.serviceClient<framefab_msgs::MoveToTargetPose>(MOVE_TO_TARGET_POSE_SERVICE);
  output_processing_client_  = nh_.serviceClient<framefab_msgs::OutputProcessing>(OUTPUT_PROCESSING_SERVICE);

  // publishers

  // action servers
  task_sequence_processing_server_.start();
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

bool FrameFabCoreService::loadModelInputParameters(const std::string & filename)
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


void FrameFabCoreService::saveModelInputParameters(const std::string& filename)
{
  if (!framefab_param_helpers::toFile(filename, model_input_params_))
  {
    ROS_WARN_STREAM("Unable to save model input parameters to: " << filename);
  }
}

bool FrameFabCoreService::loadTaskSequenceInputParameters(const std::string & filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if(framefab_param_helpers::fromFile(filename, task_sequence_input_params_))
  {
    return true;
  }

  // otherwise default to the parameter server
  ros::NodeHandle nh("~/task_sequence_input");
  return loadParam(nh, "file_path", task_sequence_input_params_.file_path);
}

void FrameFabCoreService::saveTaskSequenceInputParameters(const std::string & filename)
{
  if(!framefab_param_helpers::toFile(filename, task_sequence_input_params_))
  {
    ROS_WARN_STREAM("Unable to save path input parameters to: " << filename);
  }
}

bool FrameFabCoreService::loadRobotInputParameters(const std::string & filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if(framefab_param_helpers::fromFile(filename, robot_input_params_))
  {
    return true;
  }

  // otherwise default to the parameter server
  ros::NodeHandle nh("~/robot_input");
  return loadParam(nh, "init_pose", robot_input_params_.init_pose);
}

void FrameFabCoreService::saveRobotInputParameters(const std::string& filename)
{
  if (!framefab_param_helpers::toFile(filename, robot_input_params_))
  {
    ROS_WARN_STREAM("Unable to save robot input parameters to: " << filename);
  }
}

bool FrameFabCoreService::loadOutputSaveDirInputParameters(const std::string & filename)
{
  using framefab_param_helpers::loadParam;
  using framefab_param_helpers::loadBoolParam;

  if(framefab_param_helpers::fromFile(filename, output_save_dir_input_params_))
  {
    return true;
  }

  // otherwise default to the parameter server
  ros::NodeHandle nh("~/output_save_dir_input");
  return loadParam(nh, "file_path", output_save_dir_input_params_.file_path);
}

void FrameFabCoreService::saveOutputSaveDirInputParameters(const std::string & filename)
{
  if(!framefab_param_helpers::toFile(filename, output_save_dir_input_params_))
  {
    ROS_WARN_STREAM("Unable to save output path input parameters to: " << filename);
  }
}

int FrameFabCoreService::checkSavedLadderGraphSize(const std::string& filename)
{
  descartes_msgs::LadderGraphList graph_list_msg;

  if(framefab_param_helpers::fromFile(filename, graph_list_msg))
  {
    ROS_INFO_STREAM("[Core] saved ladder graph record found!");
    return graph_list_msg.graph_list.size();
  }
  else
  {
    ROS_WARN_STREAM("[Core] NO saved ladder graph record found.");
    return 0;
  }
}

bool FrameFabCoreService::framefabParametersServerCallback(
    framefab_msgs::FrameFabParameters::Request& req,
    framefab_msgs::FrameFabParameters::Response& res)
{
  switch (req.action)
  {
    case framefab_msgs::FrameFabParameters::Request::GET_CURRENT_PARAMETERS:
      res.model_params = model_input_params_;
      res.task_sequence_params  = task_sequence_input_params_;
      res.robot_params = robot_input_params_;
      res.output_save_dir_params = output_save_dir_input_params_;
      break;

    case framefab_msgs::FrameFabParameters::Request::GET_DEFAULT_PARAMETERS:
      res.model_params = default_model_input_params_;
      res.task_sequence_params  = default_task_sequence_input_params_;
      res.robot_params = default_robot_input_params_;
      res.output_save_dir_params = default_output_save_dir_input_params_;
      break;

      // Update the current parameters in this service
    case framefab_msgs::FrameFabParameters::Request::SET_PARAMETERS:
    case framefab_msgs::FrameFabParameters::Request::SAVE_PARAMETERS:
      model_input_params_ = req.model_params;
      task_sequence_input_params_ = req.task_sequence_params;
      robot_input_params_ = req.robot_params;
      output_save_dir_input_params_ = req.output_save_dir_params;

      if (req.action == framefab_msgs::FrameFabParameters::Request::SAVE_PARAMETERS)
      {
        this->saveModelInputParameters(param_cache_prefix_ + MODEL_INPUT_PARAMS_FILE);
        this->saveTaskSequenceInputParameters(param_cache_prefix_ + TASK_SEQUENCE_INPUT_PARAMS_FILE);
        this->saveRobotInputParameters(param_cache_prefix_ + ROBOT_INPUT_PARAMS_FILE);
        this->saveOutputSaveDirInputParameters(param_cache_prefix_ + OUTPUT_SAVE_DIR_INPUT_PARAMS_FILE);
      }
      break;
  }

  res.succeeded = true;
  return true;
}

bool FrameFabCoreService::elementNumberRequestServerCallback(
    framefab_msgs::ElementNumberRequest::Request& req,
    framefab_msgs::ElementNumberRequest::Response& res)
{
  switch (req.action)
  {
    case framefab_msgs::ElementNumberRequest::Request::REQUEST_ELEMENT_NUMBER:
    {
      res.element_number = visual_tool_.getPathArraySize();
      break;
    }
    case framefab_msgs::ElementNumberRequest::Request::REQUEST_SELECTED_TASK_NUMBER:
    {
      res.element_number = selected_task_id_;
      break;
    }
    default:
    {
      res.element_number = 0;
      ROS_ERROR_STREAM("Unknown parameter loading request in selection widget");
      break;
    }
  }
}

bool FrameFabCoreService::visualizeSelectedPathServerCallback(
    framefab_msgs::VisualizeSelectedPath::Request& req,
    framefab_msgs::VisualizeSelectedPath::Response& res)
{
  if(req.index != -1)
  {
    visual_tool_.visualizePathUntil(req.index);
    visual_tool_.visualizeFeasibleOrientations(req.index, true);
    res.succeeded = true;
  }
  else
  {
    visual_tool_.cleanUpAllPaths();
    res.succeeded = true;
  }
}

bool FrameFabCoreService::getAvailableProcessPlansCallback(
    framefab_msgs::GetAvailableProcessPlans::Request&,
    framefab_msgs::GetAvailableProcessPlans::Response& res)
{
  typedef framefab_core_service::TrajectoryLibrary::TrajectoryMap::const_iterator MapIter;
  for (MapIter it = trajectory_library_.get().begin(); it != trajectory_library_.get().end(); ++it)
  {
    res.names.push_back(it->first);
  }
  return true;
}

bool FrameFabCoreService::outputProcessPlansCallback(
    framefab_msgs::OutputProcessPlans::Request& req,
    framefab_msgs::OutputProcessPlans::Response& res)
{
  // call output_processing srv
  framefab_msgs::OutputProcessing srv;

  // fill in file_path
  srv.request.file_path = output_save_dir_input_params_.file_path;

  // fill in plans
  for(auto id : req.names)
  {
    if (trajectory_library_.get().find(id) == trajectory_library_.get().end())
    {
      ROS_ERROR_STREAM("Plan #" << id << " does not exist. Cannot output.");
      return false;
    }
    else
    {
      srv.request.plans.push_back(trajectory_library_.get()[id]);
    }
  }

  if(!output_processing_client_.call(srv))
  {
    ROS_WARN_STREAM("[framefab_core_service] Unable to call output processing service");
    return false;
  }
  else
  {
    return true;
  }
}

bool FrameFabCoreService::queryComputationResultCallback(
    framefab_msgs::QueryComputationRecord::Request &req,
    framefab_msgs::QueryComputationRecord::Response &res)
{
  const int found_saved_graphs_size = checkSavedLadderGraphSize(req.file_name);

  ROS_INFO_STREAM("[CORE] saved graph query - file name: " << req.file_name);

  res.record_found = bool(found_saved_graphs_size);
  res.found_record_size = found_saved_graphs_size;

  return true;
}

void FrameFabCoreService::taskSequenceProcessingActionCallback(const framefab_msgs::TaskSequenceProcessingGoalConstPtr &goal_in)
{
  switch (goal_in->action)
  {
    case framefab_msgs::TaskSequenceProcessingGoal::FIND_AND_PROCESS:
    {
      task_sequence_processing_feedback_.last_completed = "[Core] Recieved request to process task sequence plan\n";
      task_sequence_processing_server_.publishFeedback(task_sequence_processing_feedback_);

      // call task_sequence_processing srv
      framefab_msgs::TaskSequenceProcessing srv;
      srv.request.action = srv.request.PROCESS_TASK_AND_MARKER;
      srv.request.model_params = model_input_params_;
      srv.request.task_sequence_params = task_sequence_input_params_;

      if(!task_sequence_processing_srv_client_.call(srv))
      {
        ROS_WARN_STREAM("[Core] Unable to call task sequence processing service");
        task_sequence_processing_feedback_.last_completed = "[Core] Failed to call Task Sequence Processing Service!\n";
        task_sequence_processing_server_.publishFeedback(task_sequence_processing_feedback_);
        task_sequence_processing_result_.succeeded = false;
        task_sequence_processing_server_.setAborted(task_sequence_processing_result_);
      }
      else
      {
        // take srv output, save them
        task_sequence_processing_feedback_.last_completed = "Finished task sequence processing. Visualizing...\n";
        task_sequence_processing_server_.publishFeedback(task_sequence_processing_feedback_);

        // import data into visual_tools
        visual_tool_.setProcessPath(srv.response.process);
        visual_tool_.visualizeAllPaths();

        // import data into process_planning_visualizer
        task_sequence_ = srv.response.process;
        env_objs_ = srv.response.env_collision_objs;

        task_sequence_processing_result_.succeeded = true;
        task_sequence_processing_server_.setSucceeded(task_sequence_processing_result_);
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
  task_sequence_processing_result_.succeeded = false;
}

void FrameFabCoreService::processPlanningActionCallback(const framefab_msgs::ProcessPlanningGoalConstPtr &goal_in)
{
  switch (goal_in->action)
  {
    case framefab_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW:
    {
      process_planning_feedback_.last_completed = "Recieved request to generate motion plan\n";
      process_planning_server_.publishFeedback(process_planning_feedback_);

      selected_task_id_ = goal_in->index;
      use_saved_graph_ = goal_in->use_saved_graph;

      // reset Robot's pose to init pose
      if(!moveToTargetJointPose(robot_input_params_.init_pose))
      {
        process_planning_feedback_.last_completed = "Reset to init robot's pose planning & execution failed\n";
        process_planning_server_.publishFeedback(process_planning_feedback_);
        process_planning_result_.succeeded = false;
        process_planning_server_.setAborted(process_planning_result_);
        return;
      }

      visual_tool_.cleanUpAllPaths();
      visual_tool_.visualizePathUntil(goal_in->index);
      visual_tool_.visualizeFeasibleOrientations(goal_in->index, false);

      bool success = generateMotionLibrary(goal_in->index, trajectory_library_);

      if(success)
      {
        process_planning_feedback_.last_completed = "Finished planning. Visualizing...\n";
        process_planning_server_.publishFeedback(process_planning_feedback_);
        process_planning_result_.succeeded = true;
        process_planning_server_.setSucceeded(process_planning_result_);

        return;
      }
      else
      {
        process_planning_feedback_.last_completed = "Process Planning action failed.\n";
        process_planning_server_.publishFeedback(process_planning_feedback_);
        process_planning_result_.succeeded = false;
        process_planning_server_.setAborted(process_planning_result_);
      }
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("[Core] Unknown action code '" << goal_in->action << "' request");
      break;
    }
  }
}

//namespace{
//void appendTrajectoryHeaders(const trajectory_msgs::JointTrajectory &orig_traj,
//                             trajectory_msgs::JointTrajectory &traj,
//                             const double sim_time_scale)
//{
//  traj.joint_names = orig_traj.joint_names;
//  traj.header.frame_id = orig_traj.header.frame_id;
//  traj.header.stamp = orig_traj.header.stamp + orig_traj.points.back().time_from_start;
//
//  // set time_from_start relative to first point
//  ros::Duration base_time = traj.points[0].time_from_start;
//
//  for (int i = 0; i < traj.points.size(); i++)
//  {
//    traj.points[i].time_from_start -= base_time;
//
//    //sim speed tuning
//    traj.points[i].time_from_start *= sim_time_scale;
//  }
//}
//}
//
//void FrameFabCoreService::adjustSimSpeed(double sim_speed)
//{
//  // global time_stamp rescale
//  trajectory_msgs::JointTrajectory last_filled_jts = trajectory_library_.get().begin()->second.sub_process_array[0].joint_array;
//
//  // shift first jts array
//  for (int i = 0; i < last_filled_jts.points().size(); i++)
//  {
//    last_filled_jts.points[i].time_from_start *= sim_speed;
//  }
//
//  // inline function for append trajectory headers (adjust time frame)
//  void adjustTrajectoryHeaders = [sim_speed](trajectory_msgs::JointTrajectory& last_filled_jts, framefab_msgs::SubProcess& sp)
//  {
//    appendTrajectoryHeaders(last_filled_jts, sp.joint_array, sim_speed);
//    last_filled_jts = sp.joint_array;
//  };
//
//  for(auto it = trajectory_library_.get().begin(); it != trajectory_library_.get().end(); ++it)
//  {
//    for (size_t j = 0; j < it->second.sub_process_array.size(); j++)
//    {
//      adjustTrajectoryHeaders(last_filled_jts, it->second.sub_process_array[j]);
//    }
//  }
//}

void FrameFabCoreService::simulateMotionPlansActionCallback(const framefab_msgs::SimulateMotionPlanGoalConstPtr& goal_in)
{
  std::string lib_sort_id = std::to_string(goal_in->index);

  // If plan does not exist, abort and return
  if (trajectory_library_.get().find(lib_sort_id) == trajectory_library_.get().end())
  {
    ROS_WARN_STREAM("[Core] Motion plan #" << lib_sort_id << " does not exist. Cannot execute.");
    simulate_motion_plan_result_.code = framefab_msgs::SimulateMotionPlanResult::NO_SUCH_NAME;
    simulate_motion_plan_server_.setAborted(simulate_motion_plan_result_);
    return;
  }
  else
  {
    ROS_INFO_STREAM("[Core] Motion plan #" << lib_sort_id << " found");
  }

  if(0 == goal_in->index)
  {
    // reset Robot's pose to init pose
    if (!moveToTargetJointPose(robot_input_params_.init_pose))
    {
      ROS_ERROR("[Core] Reset to init robot's pose planning & execution failed");
      simulate_motion_plan_result_.code = framefab_msgs::SimulateMotionPlanResult::RESET_POSE_FAIL;
      simulate_motion_plan_server_.setAborted(simulate_motion_plan_result_);
    }
  }

  // Send command to execution server
  framefab_msgs::ProcessExecutionGoal goal;
  ros::Duration process_time(0);

  for(auto sp : trajectory_library_.get()[lib_sort_id].sub_process_array)
  {
    goal.joint_traj_array.push_back(sp.joint_array);
    process_time += sp.joint_array.points.back().time_from_start;
  }

  goal.wait_for_execution = goal_in->wait_for_execution;
  goal.simulate = goal_in->simulate;

  actionlib::SimpleActionClient<framefab_msgs::ProcessExecutionAction> *exe_client = &framefab_exe_client_;
  exe_client->sendGoal(goal);

//  ros::Duration process_time(goal.joint_traj_array.back().points.back().time_from_start);
  ros::Duration buffer_time(PROCESS_EXE_BUFFER);

  visual_tool_.visualizePathUntil(goal_in->index);

  ROS_INFO_STREAM("[Core] Simulation time: " << process_time);

  if(exe_client->waitForResult(process_time + buffer_time))
  {
    simulate_motion_plan_result_.code = framefab_msgs::SimulateMotionPlanResult::SUCCESS;
    simulate_motion_plan_server_.setSucceeded(simulate_motion_plan_result_);
  }
  else
  {
    simulate_motion_plan_result_.code = framefab_msgs::SimulateMotionPlanResult::TIMEOUT;
    simulate_motion_plan_server_.setAborted(simulate_motion_plan_result_);
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
    ROS_INFO("[Core] framefab core service online.");
    core_srv.run();
  }

  ros::waitForShutdown();
}
