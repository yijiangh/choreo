//
// Created by yijiangh on 7/5/17.
//

#include "framefab_core/framefab_core_service.h"

framefab_core_service::TrajectoryLibrary FrameFabCoreService::generateMotionLibrary(
    const int selected_path_index)
{
  framefab_core_service::TrajectoryLibrary lib;

  ros::NodeHandle nh;

//  // Generate trajectory plans from motion plan
//  {
//    for (std::size_t j = 0; j < paths.paths.size(); ++j)
//    {
//      ProcessPlanResult plan = generateProcessPlan(paths.paths[j].first, paths.paths[j].second, blend_params,
//                                                   scan_params);
//
//      for (std::size_t k = 0; k < plan.plans.size(); ++k)
//        lib.get()[plan.plans[k].first] = plan.plans[k].second;
//    }
//  }

  return lib;
}