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

  // first check conflict indices
  std::vector<size_t> full_ids(poses.size());
  std::iota(full_ids.begin(), full_ids.end(), 0); // fill 0 ~ n-1
  std::set<size_t> full_ids_set(full_ids.begin(), full_ids.end());

  std::set<size_t> last_check_ids;
  std::set_difference(full_ids_set.begin(), full_ids_set.end(),
                      cap_rung.conflict_ids_.begin(), cap_rung.conflict_ids_.end(),
                      std::inserter(last_check_ids, last_check_ids.end()));

  std::vector<std::vector<double>> joint_poses;
  for(size_t c_id = 0; c_id < poses.size(); c_id++)
  {
    joint_poses.clear();

    if(c_id < poses.size() - 1)
    {
      model.setPlanningScene(cap_rung.planning_scene_);
    }
    else
    {
      // the last pose, prevent eef collide with element being printed
      model.setPlanningScene(cap_rung.planning_scene_completed_);
    }

    model.getAllIK(poses[c_id], joint_poses);

    if(joint_poses.empty())
    {
      return false;
    }
    else
    {
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