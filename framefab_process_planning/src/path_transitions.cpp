//
// Created by yijiangh on 7/7/17.
//
#include "path_transitions.h"

#include <algorithm>

#include <Eigen/Geometry>

// msgs
#include <geometry_msgs/Vector3.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

// for extra descartes graph building feature
#include <descartes_planner/graph_builder.h>

namespace // anon namespace to hide utility functions
{
  // convert orientations representing by Eigen3d::vector into Eigen::Matrix3d
  std::vector<Eigen::Matrix3d> convertOrientationVector(
      const std::vector<geometry_msgs::Vector3>& orients_msg)
  {
    std::vector<Eigen::Matrix3d> m_orients;

    for(auto v : orients_msg)
    {
      Eigen::Vector3d eigen_vec;
      tf::vectorMsgToEigen(v, eigen_vec);
      eigen_vec.normalize();

      const Eigen::Vector3d& x_vec = eigen_vec.cross(Eigen::Vector3d::UnitZ());

      Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
      if(0 != x_vec.norm())
      {
        double rot_angle = acos(eigen_vec.dot(Eigen::Vector3d::UnitZ()));
        m = m * Eigen::AngleAxisd(rot_angle, Eigen::Vector3d::UnitZ());
        m_orients.push_back(m);
      }

      m_orients.push_back(m);
    }

    return m_orients;
  }
} // end utility function ns

using DescartesConstrainedPathConversionFunc =
boost::function<descartes_planner::ConstrainedSegment (const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                       const std::vector<Eigen::Vector3d> &)>;

std::vector<descartes_planner::ConstrainedSegment>
framefab_process_planning::toDescartesConstrainedPath(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                                                      const int index,
                                                      const double process_speed, const TransitionParameters& transition_params)
{
  using ConstrainedSegment = descartes_planner::ConstrainedSegment;

  int selected_path_num = index + 1;
  std::vector<ConstrainedSegment> segs(selected_path_num);

  // Inline function for adding a sequence of motions
  auto add_segment = [process_speed, transition_params]
      (ConstrainedSegment& seg, const framefab_msgs::ElementCandidatePoses path_pose)
  {
      tf::pointMsgToEigen(path_pose.start_pt, seg.start);
      tf::pointMsgToEigen(path_pose.end_pt, seg.end);
  };

//  for (std::size_t i = 0; i < selected_path_num; ++i)
//  {
//    add_segment(trajs[i].connect_path, process_path_poses[i].connect, false);
//
//    add_segment(trajs[i].approach_path, process_path_poses[i].approach, false);
//
//    add_segment(trajs[i].print_path, process_path_poses[i].print, false);
//
//    add_segment(trajs[i].depart_path, process_path_poses[i].depart, false);
//
//  } // end segments

  return segs;
}