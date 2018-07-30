//
// Created by yijiangh on 7/5/17.
//

#include <choreo_process_planning/choreo_process_planning.h>
#include <ros/console.h>

// msg
#include <choreo_msgs/ElementCandidatePoses.h>

// service
#include <choreo_msgs/ProcessPlanning.h>

// descartes
#include "path_transitions.h"
#include "common_utils.h"
#include "generate_motion_plan.h"

namespace choreo_process_planning
{
// Planning Constants
const double PRINT_ANGLE_DISCRETIZATION =
    M_PI / 12.0; // The discretization of the tool's pose about the z axis

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

bool ProcessPlanningManager::handlePrintPlanning(choreo_msgs::ProcessPlanning::Request &req,
                                                 choreo_msgs::ProcessPlanning::Response &res)
{
  // Enable Collision Checks
  hotend_model_->setCheckCollisions(true);

  if (req.task_sequence.empty())
  {
    ROS_WARN("[Process Planning] Planning request contained no process path. Nothing to be done.");
    return true;
  }

  const static double LINEAR_VEL = 0.1; // (m/s)
  const static double RETRACT_DISTANCE = 0.010; // meters

  // 0.005
  const static double LINEAR_DISCRETIZATION = 0.05; // meters

  // the distance between angular steps about z for each orientationcreateCollisionObject
  const static double ANGULAR_DISCRETIZATION = PRINT_ANGLE_DISCRETIZATION; // radians

  ConstrainedSegParameters constrained_seg_params;
  constrained_seg_params.linear_vel = LINEAR_VEL;
  constrained_seg_params.linear_disc = LINEAR_DISCRETIZATION;
  constrained_seg_params.angular_disc = ANGULAR_DISCRETIZATION;
  constrained_seg_params.retract_dist = RETRACT_DISTANCE;

  //  Eigen::Affine3d start_home_pose;
  //  hotend_model_->getFK(current_joints, start_home_pose);
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  // construct segs for descartes & copy chosen task sequence
  const std::vector<choreo_msgs::ElementCandidatePoses>
      chosen_task_seq(req.task_sequence.begin(), req.task_sequence.begin() + req.index + 1);

  std::vector<descartes_planner::ConstrainedSegment> constrained_segs =
      toDescartesConstrainedPath(chosen_task_seq, constrained_seg_params);

  // clear existing objs from previous planning
  clearAllCollisionObjects(planning_scene_diff_client_);

  // add fixed extra collision objects in the work environment, e.g. heating bed (adjustable)
  for(const auto& obj : req.env_collision_objs)
  {
    addCollisionObject(planning_scene_diff_client_, obj);
  }

  if(generateMotionPlan(
      world_frame_,
      req.use_saved_graph, req.file_name, req.clt_rrt_unit_process_timeout, req.clt_rrt_timeout,
      hotend_group_name_,
      current_joints,
      chosen_task_seq,
      req.wf_collision_objs,
      constrained_segs,
      hotend_model_,
      moveit_model_,
      planning_scene_diff_client_,
      res.plan))
  {
    return true;
  }
  else
  {
    return false;
  }
}

}// end namespace
