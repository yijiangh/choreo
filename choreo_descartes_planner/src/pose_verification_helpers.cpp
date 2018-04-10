//
// Created by yijiangh on 4/9/18.
//

#include "pose_verification_helpers.h"

namespace descartes_planner
{

bool checkFeasibility(
    descartes_core::RobotModel& model,
    const std::vector<Eigen::Affine3d>& poses, descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert)
{
  // sanity check
  assert(poses.size() == cap_rung.path_pts_.size());

  std::vector<double> st_jt;
  std::vector<double> end_jt;

  std::vector<std::vector<double>> joint_poses;

  // check ik feasibility for each of the path points
  for(size_t c_id = 0; c_id < poses.size(); c_id++)
  {
    joint_poses.clear();

    if(c_id < poses.size() - 1)
    {
      model.setPlanningScene(cap_rung.planning_scene_);
    }
    else
    {
      // TODO: this is temporal workaround
      // the last pose, prevent eef collide with element being printed
      model.setPlanningScene(
          cap_rung.planning_scene_completed_ ? cap_rung.planning_scene_completed_ : cap_rung.planning_scene_);
    }

    model.getAllIK(poses[c_id], joint_poses);

    if(joint_poses.empty())
    {
      // current capsule is invalid if there exists one path point without feasible kinematics solution.
      return false;
    }
    else
    {
      // only store kinematics family for only start and end path point
      if(0 == c_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          st_jt.insert(st_jt.end(), sol.cbegin(), sol.cend());
        }
        cap_vert.start_joint_data_ = st_jt;
      }
      if(poses.size()-1 == c_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          end_jt.insert(end_jt.end(), sol.cbegin(), sol.cend());
        }
        cap_vert.end_joint_data_ = end_jt;
      }
    }
  }// end loop over poses

  // all poses are valid (have feasible ik solutions)!
  return true;
}

bool domainDiscreteEnumerationCheck(
    descartes_core::RobotModel& model,
    descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert)
{
  // direct enumeration of domain
  const std::vector<Eigen::Vector3d> pts = cap_rung.path_pts_;
  std::vector<Eigen::Affine3d> poses;
  poses.reserve(cap_rung.path_pts_.size());

  const auto n_angle_disc = std::lround( 2 * M_PI / cap_rung.z_axis_disc_);
  const auto angle_step = 2 * M_PI / n_angle_disc;

  for (const auto& orientation : cap_rung.orientations_)
  {
    for (long i = 0; i < n_angle_disc; ++i)
    {
      const auto angle = angle_step * i;
      poses.clear();

      for(const auto& pt : pts)
      {
        poses.push_back(makePose(pt, orientation, angle));
      }

      if(checkFeasibility(model, poses, cap_rung, cap_vert))
      {
        cap_vert.orientation_ = orientation;
        cap_vert.z_axis_angle_ = angle;
        return true;
      }
    }
  }

  return false;
}

}