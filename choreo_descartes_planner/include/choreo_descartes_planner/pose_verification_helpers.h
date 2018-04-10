//
// Created by yijiangh on 4/9/18.
//

#ifndef FRAMEFAB_MPP_POSE_VERIFICATION_HELPERS_H
#define FRAMEFAB_MPP_POSE_VERIFICATION_HELPERS_H

#include "pose_sampling_helpers.h"
#include "choreo_descartes_planner/capsulated_ladder_tree.h"

namespace descartes_planner
{

// if pose is feasible, return true and start and end joint solution
// otherwise return empty joint solution
bool checkFeasibility(
    descartes_core::RobotModel& model,
    const std::vector<Eigen::Affine3d>& poses, descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert);

bool domainDiscreteEnumerationCheck(
    descartes_core::RobotModel& model,
    descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert);

}

#endif //FRAMEFAB_MPP_POSE_VERIFICATION_HELPERS_H
