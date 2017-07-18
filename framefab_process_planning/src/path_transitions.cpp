//
// Created by yijiangh on 7/7/17.
//
#include "path_transitions.h"

#include <algorithm>

#include <Eigen/Geometry>

// rviz visual debug
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

// for extra descartes graph building feature
#include <descartes_planner/graph_builder.h>

using DescartesConstrainedPathConversionFunc =
boost::function<descartes_planner::ConstrainedSegment (const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                       const std::vector<Eigen::Vector3d> &)>;

std::vector<descartes_planner::ConstrainedSegment>
framefab_process_planning::toDescartesConstrainedPath(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                                                      const int index,
                                                      const double process_speed, const TransitionParameters& transition_params,
                                                      DescartesConstrainedPathConversionFunc conversion_fn)
{
  using ConstrainedSegment = descartes_planner::ConstrainedSegment;

  int selected_path_num = index + 1;
  std::vector<ConstrainedSegment> segs(selected_path_num);

  // Inline function for adding a sequence of motions
  auto add_segment = [process_speed, conversion_fn, transition_params]
      (ConstrainedSegment& seg, const framefab_msgs::ElementCandidatePoses)
  {
    // Create Descartes trajectory for the segment path
//    for (std::size_t j = 0; j < poses.size(); ++j)
    {


//      double dt = 0;
//      traj.push_back(conversion_fn(this_pose, dt));
    }
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