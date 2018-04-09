//
// Created by yijiangh on 4/3/18.
//

#include <framefab_process_planning/framefab_process_planning.h>
#include <ros/console.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

// msg
#include <trajectory_msgs/JointTrajectory.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/Grasp.h>

#include "path_transitions.h"
#include "common_utils.h"

// for immediate execution
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace {

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

} // end anon util namespace

namespace framefab_process_planning
{

bool ProcessPlanningManager::handlePickNPlacePlanning(
    framefab_msgs::ProcessPlanning::Request& req,
    framefab_msgs::ProcessPlanning::Response& res)
{
//  // Enable Collision Checks
//  hotend_model_->setCheckCollisions(true);
//
//  if (req.task_sequence.empty())
//  {
//    ROS_WARN("[Process Planning] Planning request contained no process path. Nothing to be done.");
//    return true;
//  }
//
//  const static double LINEAR_VEL = 0.1; // (m/s)
//  const static double RETRACT_DISTANCE = 0.010; // meters
//
//  const static double LINEAR_DISCRETIZATION = 0.005; // meters
//  // the distance between angular steps about z for each orientationcreateCollisionObject
//  const static double ANGULAR_DISCRETIZATION = PRINT_ANGLE_DISCRETIZATION; // radians
//
//  ConstrainedSegParameters constrained_seg_params;
//  constrained_seg_params.linear_vel = LINEAR_VEL;
//  constrained_seg_params.linear_disc = LINEAR_DISCRETIZATION;
//  constrained_seg_params.angular_disc = ANGULAR_DISCRETIZATION;
//  constrained_seg_params.retract_dist = RETRACT_DISTANCE;
//
//  //  Eigen::Affine3d start_home_pose;
//  //  hotend_model_->getFK(current_joints, start_home_pose);
//  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);
//
//  // construct segs for descartes & copy chosen task sequence
//  std::vector<descartes_planner::ConstrainedSegment> constrained_segs =
//      toDescartesConstrainedPath(req.task_sequence, req.index, constrained_seg_params);
//  const std::vector<framefab_msgs::ElementCandidatePoses>
//      chosen_task_seq(req.task_sequence.begin(), req.task_sequence.begin() + constrained_segs.size());
//
//  // clear existing objs from previous planning
//  clearAllCollisionObjects(planning_scene_diff_client_);
//
//  // add fixed extra collision objects in the work environment, e.g. heating bed (adjustable)
//  for(const auto& obj : req.env_collision_objs)
//  {
//    addCollisionObject(planning_scene_diff_client_, obj);
//  }
//
//  if(generateMotionPlan(hotend_model_, constrained_segs, chosen_task_seq, req.wf_collision_objs, world_frame_,
//                        req.use_saved_graph, req.file_name,
//                        req.clt_rrt_unit_process_timeout, req.clt_rrt_timeout,
//                        moveit_model_, planning_scene_diff_client_,
//                        hotend_group_name_, current_joints, res.plan))
//  {
//    return true;
//  }
//  else
//  {
//    return false;
//  }

}

}// end ff_process_planning ns