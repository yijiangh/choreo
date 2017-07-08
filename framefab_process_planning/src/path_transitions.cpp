//
// Created by yijiangh on 7/7/17.
//
#include "path_transitions.h"

#include <algorithm>

#include <Eigen/Geometry>

// rviz visual debug
#include <rviz_visual_tools/rviz_visual_tools.h>

struct Vector3dAvrSort
{
  double key_;
  Eigen::Vector3d v_;

  Vector3dAvrSort(const Eigen::Vector3d& v) : key_(-1), v_(v) {}

  void setKey(Eigen::Vector3d avr_e) { key_ = (v_ - avr_e).norm(); }
  bool operator < (const Vector3dAvrSort& v) const
  {
    return (key_ < v.key_);
  }
};

void framefab_process_planning::generatePrintPoses(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                          std::vector<ProcessPathPose>& process_path_poses)
{
  process_path_poses.resize(process_path.size());
  for(int i = 0; i < process_path.size(); i++)
  {
    // fill in element info
    Eigen::Affine3d start_pose; // = Eigen::Affine3d::Identity();
    Eigen::Affine3d end_pose; // = Eigen::Affine3d::Identity();

    Eigen::Vector3d start_v;
    Eigen::Vector3d end_v;

    tf::pointMsgToEigen(process_path[i].start_pt, start_v);
    tf::pointMsgToEigen(process_path[i].end_pt, end_v);

    // compute average feasible orientation
    Eigen::Vector3d avr_vec = Eigen::Vector3d(0, 0, 0);
    int m = process_path[i].feasible_orients.size();
    std::vector <Vector3dAvrSort> sort_vlist;

    for (int j = 0; j < m; j++)
    {
      Eigen::Vector3d e;
      tf::vectorMsgToEigen(process_path[i].feasible_orients[j], e);
      avr_vec = avr_vec + e;
      sort_vlist.push_back(Vector3dAvrSort(e));
    }
    avr_vec.normalize();

    // construct the sort list
    for (Vector3dAvrSort v : sort_vlist)
    {
      v.setKey(avr_vec);
    }
    std::sort(sort_vlist.begin(), sort_vlist.end());

    Eigen::Vector3d z_axis = sort_vlist.begin()->v_;

    // generate start & end pose using generated vector as axis
    Eigen::Vector3d global_z_axis(0.0, 0.0, 1.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(global_z_axis, z_axis);

    start_pose = (Eigen::Translation3d(start_v) * quat);
    end_pose = (Eigen::Translation3d(end_v) * quat);

    ProcessPathPose unit_process;
    unit_process.print.push_back(start_pose);
    unit_process.print.push_back(end_pose);

    process_path_poses[i] = unit_process;
  }
}

using DescartesConversionFunc =
boost::function<descartes_core::TrajectoryPtPtr (const Eigen::Affine3d &, const double)>;

std::vector<framefab_process_planning::DescartesTraj>
framefab_process_planning::toDescartesTraj(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                                        const double process_speed, const TransitionParameters& transition_params,
                                        DescartesConversionFunc conversion_fn)
{
  // visual debug
  rviz_visual_tools::RvizVisualToolsPtr visual_tool;
  visual_tool.reset(
      new rviz_visual_tools::RvizVisualTools("world_frame", "pose_visualization"));
  double visual_axis_length = 0.01;
  double visual_axis_diameter = 0.001;

  // sort for every ElementCandidatePoses's feasible orientation
  // just for experiment, take the one closest to average orientation
  // convert all process path into pose_array (st_pt, end_pt, take the selected vector as z axis)
  std::vector<ProcessPathPose> process_path_poses;
  process_path_poses.resize(process_path.size());

  // generate pose for print start & end
  generatePrintPoses(process_path, process_path_poses);

  for(auto v : process_path_poses)
  {
    visual_tool->publishAxis(v.print[0], visual_axis_length, visual_axis_diameter, "pose_axis");
    visual_tool->publishAxis(v.print[1], visual_axis_length, visual_axis_diameter, "pose_axis");
  }
  visual_tool->trigger();

  // add retraction point
//  generateTransitions(process_path_poses, transition_params);

  std::vector<DescartesTraj> trajs;
//  Eigen::Affine3d last_pose = createNominalTransform(segments.front().poses.front());

  // Convert pose arrays to Eigen types
//  auto eigen_segments = toEigenArrays(segments);

  // Inline function for adding a sequence of motions
//  auto add_segment = [&traj, &last_pose, process_speed, conversion_fn, transition_params]
//      (const EigenSTL::vector_Affine3d& poses, bool free_last)
//  {
//    // Create Descartes trajectory for the segment path
//    for (std::size_t j = 0; j < poses.size(); ++j)
//    {
//      Eigen::Affine3d this_pose = createNominalTransform(poses[j], transition_params.z_adjust);
//      // O(1) jerky - may need to revisit this time parameterization later. This at least allows
//      // Descartes to perform some optimizations in its graph serach.
//      double dt = (this_pose.translation() - last_pose.translation()).norm() / process_speed;
//
//      if (dt < 1e-4)
//      {
//        continue;
//      }
//
//      if (j == poses.size() - 1 && free_last)
//      {
//        dt = 0.0;
//      }
//      traj.push_back( conversion_fn(this_pose, dt) );
//      last_pose = this_pose;
//    }
//  };
//
//  for (std::size_t i = 0; i < segments.size(); ++i)
//  {
//    add_segment(transitions[i].approach, false);
//
//    add_segment(eigen_segments[i], false);
//
//    add_segment(transitions[i].depart, false);
//
//    if (i != segments.size() - 1)
//    {
//      // To keep the robot at a safe height while we have no model of the parts we're working on, this code enforces a
//      // linear travel between poses. The call to closestRotationalPose allows the linear interpolation to happen to the
//      // pose that is 180 degrees off (about Z) from the nominal one. The discretization in Descartes takes care of the rest.
//      auto connection = interpolateCartesian(transitions[i].depart.back(),
//                                             closestRotationalPose(transitions[i].depart.back(), transitions[i+1].approach.front()),
//                                             transition_params.linear_disc, transition_params.angular_disc);
//      add_segment(connection, false);
//    }
//  } // end segments

  return trajs;
}