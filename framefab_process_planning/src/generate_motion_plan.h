//
// Created by yijiangh on 7/8/17.
//

#ifndef FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
#define FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H

#include <descartes_core/robot_model.h>
#include <descartes_core/trajectory_pt.h>
#include <framefab_msgs/UnitProcessPlan.h>

#include "common_utils.h"

namespace framefab_process_planning
{

bool generateMotionPlan(const descartes_core::RobotModelPtr model,
                        const std::vector<DescartesUnitProcess>& trajs,
                        const std::vector<moveit_msgs::CollisionObject>& collision_objs,
                        moveit::core::RobotModelConstPtr moveit_model,
                        const std::string& move_group_name,
                        const std::vector<double>& start_state,
                        std::vector<framefab_msgs::UnitProcessPlan>& plan);

}

#endif //FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
