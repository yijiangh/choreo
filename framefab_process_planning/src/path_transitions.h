//
// Created by yijiangh on 7/5/17.
//

#ifndef FRAMEFAB_PROCESS_PLANNING_PATH_TRANSITIONS_H
#define FRAMEFAB_PROCESS_PLANNING_PATH_TRANSITIONS_H

#include "common_utils.h"

// msgs
#include <framefab_msgs/AssemblySequencePickNPlace.h>
#include <framefab_msgs/ElementCandidatePoses.h>
#include <eigen_conversions/eigen_msg.h>

#include <choreo_descartes_planner/constrained_segment.h>

#include <descartes_core/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>

namespace framefab_process_planning
{


struct ConstrainedSegParameters
{
  double linear_vel;
  double linear_disc;

  // used only in spatial extrusion
  double angular_disc;
  double retract_dist;
};

// TODO: overload for picknplace
std::vector<descartes_planner::ConstrainedSegmentPickNPlace> toDescartesConstrainedPath(
    const framefab_msgs::AssemblySequencePickNPlace& task_sequence,
    const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scene_pick,
    const double& linear_vel, const double& linear_disc);

// TODO: should update this to a more universal one
// this version is dedicated to spatial extrusion
std::vector<descartes_planner::ConstrainedSegment> toDescartesConstrainedPath(
    const std::vector<framefab_msgs::ElementCandidatePoses>& task_sequence,
    const ConstrainedSegParameters& transition_params);

std::vector<descartes_planner::ConstrainedSegment> toDescartesConstrainedPath(
    const std::vector<framefab_msgs::ElementCandidatePoses>& task_sequence,
    const double& linear_vel, const double& linear_disc,
    const double& angular_disc, const double& retract_dist);

bool retractPath(const std::vector<double>& start_joint, double retract_dist, double TCP_speed,
                 const std::vector<Eigen::Matrix3d>& eef_directions,
                 descartes_core::RobotModelPtr& descartes_model,
                 std::vector<std::vector<double>>& retract_jt_traj);
}

#endif //FRAMEFAB_PROCESS_PLANNING_PATH_TRANSITIONS_H