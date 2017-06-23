#ifndef FRAMEFAB_CORE_SERVICE_H
#define FRAMEFAB_CORE_SERVICE_H

#include <ros/ros.h>

// service
#include <framefab_msgs/PathPlanning.h>

// msgs
#include <framefab_msgs/FrameFabParameters.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>

// actions

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

  bool
  framefab_parameters_server_callback(framefab_msgs::FrameFabParameters::Request& req,
                                      framefab_msgs::FrameFabParameters::Response& res);

  // Services offered by this class
  ros::ServiceServer framefab_parameters_server_;

  // Services subscribed to by this class

  // Actions offered by this class
  ros::NodeHandle nh_;

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
