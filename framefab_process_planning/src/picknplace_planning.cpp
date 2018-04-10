//
// Created by yijiangh on 4/3/18.
//

#include <framefab_process_planning/framefab_process_planning.h>

#include "generate_motion_plan.h"
#include "path_transitions.h"
#include "common_utils.h"

#include <framefab_msgs/SequencedElement.h>

#include <ros/console.h>

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
  // Enable Collision Checks
  hotend_model_->setCheckCollisions(true);

  assert(req.as_pnp.sequenced_elements.size() > 0);

  const static double LINEAR_VEL = 0.1; // (m/s)
  const static double LINEAR_DISCRETIZATION = 0.005; // meters
//  // the distance between angular steps about z for each orientationcreateCollisionObject
//  const static double ANGULAR_DISCRETIZATION = PRINT_ANGLE_DISCRETIZATION; // radians
//
//  ConstrainedSegParameters constrained_seg_params;
//  constrained_seg_params.linear_vel = LINEAR_VEL;
//  constrained_seg_params.linear_disc = LINEAR_DISCRETIZATION;
//  constrained_seg_params.angular_disc = ANGULAR_DISCRETIZATION;
//  constrained_seg_params.retract_dist = RETRACT_DISTANCE;

  // TODO: assuming current robot pose is the home pose, this should be read from ros parameter
  //  Eigen::Affine3d start_home_pose;
  //  hotend_model_->getFK(current_joints, start_home_pose);
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);
//
//  // construct segs for descartes & copy chosen task sequence
//  std::vector<descartes_planner::ConstrainedSegment> constrained_segs =
//      toDescartesConstrainedPath(req.task_sequence, req.index, constrained_seg_params);

  // copy & crop up to the required index
  assert(req.index > 0 && req.index < req.as_pnp.sequenced_elements.size());
  framefab_msgs::AssemblySequencePickNPlace as_pnp = req.as_pnp;
  as_pnp.sequenced_elements = std::vector<framefab_msgs::SequencedElement>(
      as_pnp.sequenced_elements.begin(), as_pnp.sequenced_elements.begin() + req.index);

  // TODO: this shouldn't remove collision objs in xacro?
  // clear existing objs from previous planning
  clearAllCollisionObjects(planning_scene_diff_client_);

  // TODO: these "still" collision obj construction should be moved in the constructPlanningScene function
//  // add fixed extra collision objects in the work environment, e.g. heating bed (adjustable)
//  for(const auto& obj : req.env_collision_objs)
//  {
//    addCollisionObject(planning_scene_diff_client_, obj);
//  }

  // TODO: remember to crop the assembly seq according to the requested id
  if(generateMotionPlan(world_frame_,
                        req.use_saved_graph,
                        req.file_name,
                        req.clt_rrt_unit_process_timeout,
                        req.clt_rrt_timeout,
                        hotend_group_name_,
                        current_joints,
                        as_pnp,
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

}// end ff_process_planning ns