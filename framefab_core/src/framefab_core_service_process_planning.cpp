//
// Created by yijiangh on 7/5/17.
//
#include <string>

#include "framefab_core/framefab_core_service.h"

// services
#include <framefab_msgs/ProcessPlanning.h>
#include <framefab_msgs/MoveToTargetPose.h>

bool FrameFabCoreService::generateMotionLibrary(
    const int selected_path_index, framefab_core_service::TrajectoryLibrary& traj_lib)
{
  ProcessPlanResult plan = generateProcessPlan(selected_path_index);

  traj_lib.get().clear();
  for (std::size_t k = 0; k < plan.plans.size(); ++k)
  {
    traj_lib.get()[std::to_string(k)] = plan.plans[k];
  }

  return true;
}

ProcessPlanResult FrameFabCoreService::generateProcessPlan(const int selected_path_index)
{
  ProcessPlanResult result;

  bool success = false;
  std::vector<framefab_msgs::UnitProcessPlan> process_plan;

  // get task sequence file name as ladder graph file name
  std::string saved_graph_file_name = task_sequence_input_params_.file_path;
  std::replace(saved_graph_file_name.begin(), saved_graph_file_name.end(), '/', '_');

  // call process_processing srv
  framefab_msgs::ProcessPlanning srv;
//  srv.request.params = process_planning_params_;
  srv.request.index = selected_path_index;
  srv.request.task_sequence = task_sequence_;
  srv.request.env_collision_objs = env_objs_;
  srv.request.use_saved_graph = use_saved_graph_;
  srv.request.file_name = saved_graph_file_name;

  success = process_planning_client_.call(srv);
  process_plan = srv.response.plan;

  if (success)
  {
    for (auto v : process_plan)
    {
      result.plans.push_back(v);
    }
  }
  else
  {
    ROS_ERROR_STREAM("[Core] Failed to plan for path #" << selected_path_index << ", planning failed.");
  }

  return result;
}

bool FrameFabCoreService::moveToTargetJointPose(std::vector<double> joint_pose)
{
  framefab_msgs::MoveToTargetPose srv;

  srv.request.type = srv.request.JOINT_POSE;
  srv.request.pose = joint_pose;

  return move_to_pose_client_.call(srv);
}
