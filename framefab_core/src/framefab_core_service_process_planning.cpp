//
// Created by yijiangh on 7/5/17.
//
#include <string>

#include "framefab_core/framefab_core_service.h"

// services
#include <framefab_msgs/ProcessPlanning.h>

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

  // call process_processing srv
  framefab_msgs::ProcessPlanning srv;
//  srv.request.params = process_planning_params_;
  srv.request.index = selected_path_index;
  srv.request.process_path = process_paths_;

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
    ROS_ERROR_STREAM("Failed to plan for path #" << selected_path_index << ", unable to connect to process planning node.");
  }

  return result;
}