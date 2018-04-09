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
#include <framefab_msgs/WireFrameCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "common_utils.h"

namespace framefab_process_planning
{
// TODO: should replace with a more general one, quickfix is an overload
void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                             const std::vector <framefab_msgs::WireFrameCollisionObject> &wf_collision_objs,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked_approach,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked_depart,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_full);

void CLTRRTforProcessROSTraj(descartes_core::RobotModelPtr model,
                             std::vector <descartes_planner::ConstrainedSegment> &segs,
                             const double clt_rrt_unit_process_timeout,
                             const double clt_rrt_timeout,
                             const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_approach,
                             const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_depart,
                             const std::vector <framefab_msgs::ElementCandidatePoses> &task_seq,
                             std::vector <framefab_msgs::UnitProcessPlan> &plans,
                             const std::string &saved_graph_file_name,
                             bool use_saved_graph);

void retractionPlanning(descartes_core::RobotModelPtr model,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_approach,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_depart,
                        const std::vector <descartes_planner::ConstrainedSegment> &segs,
                        std::vector <framefab_msgs::UnitProcessPlan> &plans);

void transitionPlanning(std::vector <framefab_msgs::UnitProcessPlan> &plans,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient &planning_scene_diff_client,
                        const std::string &move_group_name,
                        const std::vector<double> &start_state,
                        std::vector <planning_scene::PlanningScenePtr> &planning_scenes);

void adjustTrajectoryTiming(std::vector <framefab_msgs::UnitProcessPlan> &plans,
                            const std::vector <std::string> &joint_names,
                            const std::string world_frame);

void appendTCPPoseToPlans(const descartes_core::RobotModelPtr model,
                          const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked,
                          const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_full,
                          std::vector <framefab_msgs::UnitProcessPlan> &plans);

bool generateMotionPlan(descartes_core::RobotModelPtr model,
                        std::vector<descartes_planner::ConstrainedSegment>& segs,
                        const std::vector<framefab_msgs::ElementCandidatePoses>& task_seq,
                        const std::vector<framefab_msgs::WireFrameCollisionObject>& collision_obj_lists,
                        const std::string world_frame,
                        const bool use_saved_graph,
                        const std::string& saved_graph_file_name,
                        const double clt_rrt_unit_process_timeout,
                        const double clt_rrt_timeout,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient& planning_scene_diff_client,
                        const std::string& hotend_group_name,
                        const std::vector<double>& start_state,
                        std::vector<framefab_msgs::UnitProcessPlan>& plan);

}

#endif //FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
