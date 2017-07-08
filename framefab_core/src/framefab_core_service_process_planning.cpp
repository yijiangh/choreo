//
// Created by yijiangh on 7/5/17.
//

#include "framefab_core/framefab_core_service.h"

// services
#include <framefab_msgs/ProcessPlanning.h>

bool FrameFabCoreService::generateMotionLibrary(
    const int selected_path_index, framefab_core_service::TrajectoryLibrary& traj_lib)
{
  framefab_core_service::TrajectoryLibrary lib;

  ros::NodeHandle nh;

  // call process_processing srv
  framefab_msgs::ProcessPlanning srv;
//  srv.request.params = process_planning_params_;
  srv.request.index = selected_path_index;
  srv.request.process_path = process_paths_;

  if(!process_planning_client_.call(srv))
  {
    ROS_WARN_STREAM("Unable to call process processing service");
    process_planning_feedback_.last_completed = "Failed to call Process Planning Service!\n";
    process_planning_server_.publishFeedback(process_planning_feedback_);
    return false;
  }

  trajectory_library_ = traj_lib;
  return true;
}