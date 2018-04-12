//
// Created by yijiangh on 7/8/17.
//

#ifndef FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H
#define FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H

#include <descartes_core/robot_model.h>
#include <choreo_descartes_planner/constrained_segment.h>

#include <framefab_msgs/UnitProcessPlan.h>
#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/WireFrameCollisionObject.h>
#include <framefab_msgs/AssemblySequencePickNPlace.h>
#include <moveit_msgs/CollisionObject.h>

namespace framefab_process_planning
{
// retraction planning for spatial extrusion
// not used in picknplace
void retractionPlanning(descartes_core::RobotModelPtr model,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_approach,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_depart,
                        const std::vector <descartes_planner::ConstrainedSegment> &segs,
                        std::vector <framefab_msgs::UnitProcessPlan> &plans);

// TODO: need overload for picknplace (as the task sequence alternating pattern is different)
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

// overload for spatial extrusion planning request
bool generateMotionPlan(const std::string world_frame,
                        const bool use_saved_graph,
                        const std::string &saved_graph_file_name,
                        const double clt_rrt_unit_process_timeout,
                        const double clt_rrt_timeout,
                        const std::string &move_group_name,
                        const std::vector<double> &start_state,
                        const std::vector<framefab_msgs::ElementCandidatePoses> &task_seq,
                        const std::vector<framefab_msgs::WireFrameCollisionObject> &wf_collision_objs,
                        std::vector<descartes_planner::ConstrainedSegment> &segs,
                        descartes_core::RobotModelPtr model,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient &planning_scene_diff_client,
                        std::vector <framefab_msgs::UnitProcessPlan> &plans);

// TODO: add ConstrainedSegmentPickNPlace here
// overload for picknplace planning request
bool generateMotionPlan(const std::string world_frame,
                        const bool use_saved_graph,
                        const std::string &saved_graph_file_name,
                        const double clt_rrt_unit_process_timeout,
                        const double clt_rrt_timeout,
                        const double& linear_vel,
                        const double& linear_disc,
                        const std::string &move_group_name,
                        const std::vector<double> &start_state,
                        const framefab_msgs::AssemblySequencePickNPlace& as_pnp,
                        descartes_core::RobotModelPtr model,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient &planning_scene_diff_client,
                        std::vector <framefab_msgs::UnitProcessPlan> &plans);

}

#endif //FRAMEFAB_PROCESS_PLANNING_GENERATE_MOTION_PLAN_H