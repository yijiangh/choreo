#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <ros/ros.h>

// service

// msgs
#include <framefab_msgs/FrameFabParameters.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>

// actions
#include <framefab_msgs/PathPlanningAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

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

  // Service callbacks, these components drive this class by signalling events
  // from the user
  bool framefab_parameters_server_callback(framefab_msgs::FrameFabParameters::Request& req,
                                           framefab_msgs::FrameFabParameters::Response& res);

  void pathPlanningActionCallback(const framefab_msgs::PathPlanningGoalConstPtr &goal);

 private:
  // Services offered by this class
  ros::ServiceServer framefab_parameters_server_;

  // Services subscribed to by this class
  ros::ServiceClient path_post_processing_client_;

  // Actions offered by this class
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<framefab_msgs::PathPlanningAction> path_planning_server_;
  framefab_msgs::PathPlanningFeedback path_planning_feedback_;
  framefab_msgs::PathPlanningResult path_planning_result_;

  // Actions subscribed to by this class

  // Current state publishers

  // Parameters
  framefab_msgs::ModelInputParameters 	model_input_params_;
  framefab_msgs::PathInputParameters 	path_input_params_;

  framefab_msgs::ModelInputParameters 	default_model_input_params_;
  framefab_msgs::PathInputParameters 	default_path_input_params_;

  // Parameter loading and saving
  bool save_data_;
  std::string save_location_;
  std::string param_cache_prefix_;
};

#endif
