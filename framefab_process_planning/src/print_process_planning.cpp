//
// Created by yijiangh on 7/5/17.
//

#include <framefab_process_planning/framefab_process_planning.h>
#include <ros/console.h>

// msg
#include <framefab_msgs/ElementCandidatePoses.h>

// service
#include <framefab_msgs/ProcessPlanning.h>

// descartes
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "path_transitions.h"
#include "common_utils.h"
#include "generate_motion_plan.h"

namespace framefab_process_planning
{
// Planning Constants
const double PRINT_ANGLE_DISCRETIZATION =
    M_PI / 12.0; // The discretization of the tool's pose about the z axis

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

const double SEG_LINEAR_DISC = 0.02; // approx linear discretization (m)
const double SEG_LINEAR_VEL = 0.01; // approximate linear velocity (m/s)
const double SEG_Z_AXIS_DISC = 0.1; // angle discretization about z (radians)

bool ProcessPlanningManager::handlePrintPlanning(framefab_msgs::ProcessPlanning::Request &req,
                                                 framefab_msgs::ProcessPlanning::Response &res)
{
  // selected path index
  int index = req.index;

  // Enable Collision Checks
  hotend_model_->setCheckCollisions(true);

  std::vector<framefab_msgs::ElementCandidatePoses> process_path = req.process_path;
  std::vector<moveit_msgs::CollisionObject> env_objs = req.env_collision_objs;

  if (process_path.empty())
  {
    ROS_WARN("Planning request contained no process path. Nothing to be done.");
    return true;
  }

  const static double LINEAR_VEL = 0.01; // (m/s)
  const static double LINEAR_DISCRETIZATION = 0.01; // meters
  // the distance between angular steps about z for each orientation
  const static double ANGULAR_DISCRETIZATION = 0.1; // radians
  const static double RETRACT_DISTANCE = 0.005; // meters

  ConstrainedSegParameters constrained_seg_params;
  constrained_seg_params.linear_vel = LINEAR_VEL;
  constrained_seg_params.linear_disc = LINEAR_DISCRETIZATION;
  constrained_seg_params.angular_disc = ANGULAR_DISCRETIZATION;
  constrained_seg_params.retract_dist = RETRACT_DISTANCE;

  Eigen::Affine3d start_home_pose;
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);
//  hotend_model_->getFK(current_joints, start_home_pose);

  std::vector<descartes_planner::ConstrainedSegment> constrained_segs =
      toDescartesConstrainedPath(req.process_path, index, 0.01, constrained_seg_params);

  // extract collision objs from process_path
  std::vector<moveit_msgs::CollisionObject> collision_objs;
  for (auto v : process_path)
  {
    collision_objs.push_back(v.collision_cylinder);
  }

  // add working env collision objs (table etc.)
  for (auto obj : env_objs)
  {
    addCollisionObject(planning_scene_diff_client_, obj);
    ROS_INFO_STREAM("collision object added: " << obj.id);
  }

  if(generateMotionPlan(hotend_model_, constrained_segs, collision_objs, moveit_model_, planning_scene_diff_client_,
                        hotend_group_name_, current_joints, res.plan))
  {
    return true;
  }
  else
  {
    return false;
  }
}

}// end namespace