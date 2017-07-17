//
// Created by yijiangh on 7/7/17.
//
#include "path_transitions.h"

#include <algorithm>

#include <Eigen/Geometry>

// rviz visual debug
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

// full extra descartes graph building feature
#include <descartes_planner/graph_builders.h>

static double quaternionDistance(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
  return std::acos(2.0 * std::pow(a.dot(b), 2.0) - 1.0);
}

static Eigen::Affine3d closestRotationalPose(const Eigen::Affine3d& start, const Eigen::Affine3d& nominal_stop)
{
  Eigen::Affine3d alt_candidate = nominal_stop * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1));

  const auto distance1 = quaternionDistance(Eigen::Quaterniond(start.rotation()),
                                            Eigen::Quaterniond(nominal_stop.rotation()));
  const auto distance2 = quaternionDistance(Eigen::Quaterniond(start.rotation()),
                                            Eigen::Quaterniond(alt_candidate.rotation()));
  if (distance1 < distance2)
  {
    return nominal_stop;
  }
  else
  {
    return alt_candidate;
  }
}

static EigenSTL::vector_Affine3d interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                                                      const double ds, const double dt)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse()*stop; //This the stop pose represented in the start pose coordinate system
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Calculate number of steps
  unsigned steps_translation = static_cast<unsigned>(delta_translation.norm() / ds) + 1;
  unsigned steps_rotation = static_cast<unsigned>(delta_rotation.angle() / dt) + 1;
  unsigned steps = std::max(steps_translation, steps_rotation);

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q (start.rotation());
  Eigen::Quaterniond stop_q (stop.rotation());
  double slerp_ratio = 1.0 / steps;

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Affine3d pose;
  result.reserve(steps+1);
  for (unsigned i = 0; i <= steps; ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

static EigenSTL::vector_Affine3d retractPath(const Eigen::Affine3d& pose, double retract_dist,
                                             const double linear_disc, const double angular_disc)
{
  Eigen::Affine3d retract_pose = Eigen::Translation3d(0, 0, retract_dist) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  retract_pose = pose * retract_pose;

  auto segment_retract = interpolateCartesian(pose, retract_pose, linear_disc, angular_disc);

  EigenSTL::vector_Affine3d result;
  result.insert(result.end(), segment_retract.begin(), segment_retract.end());

  return result;
}

void framefab_process_planning::generateTransitions(std::vector<ProcessPathPose>& process_path_poses,
                         const TransitionParameters& params)
{
  for (std::size_t i = 0; i < process_path_poses.size(); i++)
  {
    Eigen::Affine3d start_pose = process_path_poses[i].print[0];
    Eigen::Affine3d end_pose = process_path_poses[i].print[1];

    // generate intermediate waypoints
    auto approach = retractPath(start_pose, params.retract_dist,
                                               params.linear_disc, params.angular_disc);

    auto depart = retractPath(end_pose, params.retract_dist,
                                               params.linear_disc, params.angular_disc);
    std::reverse(approach.begin(), approach.end());

    process_path_poses[i].approach = approach;
    process_path_poses[i].depart = depart;
  }
}

void framefab_process_planning::generateConnectPoses(std::vector<ProcessPathPose>& process_path_poses,
                          const Eigen::Affine3d& start_pose,
                          const TransitionParameters& params)
{
  Eigen::Affine3d last_pose = createNominalTransform(start_pose);

  for (std::size_t i = 0; i < process_path_poses.size(); i++)
  {
    auto connection = interpolateCartesian(last_pose,
                                           closestRotationalPose(last_pose,
                                                                 process_path_poses[i].approach.front()),
                                           params.linear_disc, params.angular_disc);

    process_path_poses[i].connect = connection;
    last_pose = process_path_poses[i].depart.back();
  }
}

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

std::vector<framefab_process_planning::DescartesUnitProcess>
framefab_process_planning::toDescartesTraj(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                                           const int index, const Eigen::Affine3d& start_pose,
                                           const double process_speed, const TransitionParameters& transition_params,
                                           DescartesConversionFunc conversion_fn)
{
  // sort for every ElementCandidatePoses's feasible orientation
  // just for experiment, take the one closest to average orientation
  // convert all process path into pose_array (st_pt, end_pt, take the selected vector as z axis)
  std::vector<ProcessPathPose> process_path_poses;

  // generate pose for print start & end
  generatePrintPoses(process_path, process_path_poses);

  // add retract pose
  generateTransitions(process_path_poses, transition_params);

  // add connect poses
  generateConnectPoses(process_path_poses, start_pose, transition_params);

  int selected_path_num = index + 1;
  std::vector<DescartesUnitProcess> trajs(selected_path_num);

  // Inline function for adding a sequence of motions
  auto add_segment = [process_speed, conversion_fn, transition_params]
      (DescartesTraj& traj, const EigenSTL::vector_Affine3d& poses, bool free_last)
  {
    // Create Descartes trajectory for the segment path
    for (std::size_t j = 0; j < poses.size(); ++j)
    {
      Eigen::Affine3d this_pose = createNominalTransform(poses[j]);

      double dt = 0;
      traj.push_back(conversion_fn(this_pose, dt));
    }
  };

  std::vector<EigenSTL::vector_Affine3d> result(selected_path_num);

  for (std::size_t i = 0; i < selected_path_num; ++i)
  {
    add_segment(trajs[i].connect_path, process_path_poses[i].connect, false);

    add_segment(trajs[i].approach_path, process_path_poses[i].approach, false);

    add_segment(trajs[i].print_path, process_path_poses[i].print, false);

    add_segment(trajs[i].depart_path, process_path_poses[i].depart, false);

//    result[i].insert(result[i].end(), process_path_poses[i].connect.begin(), process_path_poses[i].connect.end());
//    result[i].insert(result[i].end(), process_path_poses[i].approach.begin(), process_path_poses[i].approach.end());
//    result[i].insert(result[i].end(), process_path_poses[i].print.begin(), process_path_poses[i].print.end());
//    result[i].insert(result[i].end(), process_path_poses[i].depart.begin(), process_path_poses[i].depart.end());
  } // end segments

  // visual debug
//  rviz_visual_tools::RvizVisualToolsPtr visual_tool;
//  visual_tool.reset(
//      new rviz_visual_tools::RvizVisualTools("world_frame", "pose_visualization"));
//  visual_tool->deleteAllMarkers();
//
//  double visual_axis_length = 0.01;
//  double visual_axis_diameter = 0.0005;

//  int display_id = (1 >= selected_path_num) ? 0 : selected_path_num - 2;
//
//  for(auto v : result[display_id])
//  {
//    visual_tool->publishAxis(v, visual_axis_length, visual_axis_diameter, "pose_axis");
//    visual_tool->trigger();
//  }

//  for(auto v : result.back())
//  {
//    visual_tool->publishAxis(v, visual_axis_length, visual_axis_diameter, "pose_axis");
//    visual_tool->trigger();
//  }

  return trajs;
}