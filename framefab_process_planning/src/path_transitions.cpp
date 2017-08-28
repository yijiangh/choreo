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
  void convertOrientationVector(
      const std::vector<geometry_msgs::Vector3>& orients_msg,
      descartes_planner::ConstrainedSegment::OrientationVector& m_orients)
  {
    m_orients.clear();

    for(auto v : orients_msg)
    {
      Eigen::Vector3d eigen_vec;
      tf::vectorMsgToEigen(v, eigen_vec);
      eigen_vec *= -1.0;
      eigen_vec.normalize();
      ROS_INFO_STREAM("Candiate:" << eigen_vec.transpose());

      // JM
      Eigen::Vector3d candidate_dir = Eigen::Vector3d::UnitX();
      if ( std::abs(eigen_vec.dot(Eigen::Vector3d::UnitX())) > 0.8 )
      {
        candidate_dir = Eigen::Vector3d::UnitY();
      }

      Eigen::Vector3d y_vec = eigen_vec.cross(candidate_dir).normalized();

      Eigen::Vector3d x_vec = y_vec.cross(eigen_vec).normalized();




      // JM

//      const Eigen::Vector3d& x_vec = eigen_vec.cross(Eigen::Vector3d::UnitZ());

      Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
      m.col(0) = x_vec;
      m.col(1) = y_vec;
      m.col(2) = eigen_vec;
//      if(0 != x_vec.norm())
//      {
//        double rot_angle = acos(eigen_vec.dot(Eigen::Vector3d::UnitZ()));
//        m = m * Eigen::AngleAxisd(rot_angle, Eigen::Vector3d::UnitZ());
//      }

//      m = m * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
      ROS_INFO_STREAM("Output\n" << m << "\n");
      m_orients.push_back(m);
    }
  }
} // end utility function ns

using DescartesConstrainedPathConversionFunc =
boost::function<descartes_planner::ConstrainedSegment (const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                       const std::vector<Eigen::Vector3d> &)>;

std::vector<descartes_planner::ConstrainedSegment>
framefab_process_planning::toDescartesConstrainedPath(const std::vector<framefab_msgs::ElementCandidatePoses>& process_path,
                                                      const int index,
                                                      const double process_speed, const ConstrainedSegParameters& seg_params)
{
  using ConstrainedSegment = descartes_planner::ConstrainedSegment;

  int selected_path_num = index + 1;
  std::vector<ConstrainedSegment> segs(selected_path_num);

  // Inline function for adding a sequence of motions
  auto add_segment = [process_speed, seg_params]
      (ConstrainedSegment& seg, const framefab_msgs::ElementCandidatePoses path_pose)
  {
    tf::pointMsgToEigen(path_pose.start_pt, seg.start);
    tf::pointMsgToEigen(path_pose.end_pt, seg.end);
    convertOrientationVector(path_pose.feasible_orients, seg.orientations);

    seg.linear_vel = seg_params.linear_vel;
    seg.linear_disc = seg_params.linear_disc;
    seg.z_axis_disc = seg_params.angular_disc;
    seg.retract_dist = seg_params.retract_dist;
  };

  for (std::size_t i = 0; i < selected_path_num; ++i)
  {
    add_segment(segs[i], process_path[i]);
  } // end segments

  return segs;
}