//
// Created by yijiangh on 7/8/17.
//

#ifndef FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
#define FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H

#include <descartes_core/robot_model.h>
#include <descartes_core/trajectory_pt.h>
#include <descartes_planner/graph_builder.h>

#include <framefab_msgs/UnitProcessPlan.h>
#include <framefab_msgs/ElementCandidatePoses.h>
#include <moveit_msgs/CollisionObject.h>

#include "common_utils.h"

namespace framefab_process_planning
{

bool generateMotionPlan(descartes_core::RobotModelPtr model,
                        std::vector<descartes_planner::ConstrainedSegment>& segs,
                        const std::vector<framefab_msgs::ElementCandidatePoses>& task_seq,
                        const std::vector<std::vector<moveit_msgs::CollisionObject>>& collision_obj_lists,
                        const bool use_saved_graph,
                        const std::string& saved_graph_file_name,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient& planning_scene_diff_client,
                        const std::string& hotend_group_name,
                        const std::vector<double>& start_state,
                        std::vector<framefab_msgs::UnitProcessPlan>& plan);

}

#endif //FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
