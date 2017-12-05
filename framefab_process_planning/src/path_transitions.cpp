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

#include "trajectory_utils.h"

const static double JTS_DISC_DELTA = 0.05; // radians

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
      // eigen_vec = local z axis
      Eigen::Vector3d eigen_vec;
      tf::vectorMsgToEigen(v, eigen_vec);
      eigen_vec *= -1.0;
      eigen_vec.normalize();

      // construct local x axis & y axis
      Eigen::Vector3d candidate_dir = Eigen::Vector3d::UnitX();
      if ( std::abs(eigen_vec.dot(Eigen::Vector3d::UnitX())) > 0.8 )
      {
        // if z axis = UnitX,
        candidate_dir = Eigen::Vector3d::UnitY();
      }

      Eigen::Vector3d y_vec = eigen_vec.cross(candidate_dir).normalized();

      Eigen::Vector3d x_vec = y_vec.cross(eigen_vec).normalized();

      // JM
      Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
      m.col(0) = x_vec;
      m.col(1) = y_vec;
      m.col(2) = eigen_vec;

      m_orients.push_back(m);
    }
  }
} // end utility function ns

using DescartesConstrainedPathConversionFunc =
boost::function<descartes_planner::ConstrainedSegment (const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                       const std::vector<Eigen::Vector3d> &)>;

std::vector<descartes_planner::ConstrainedSegment>
framefab_process_planning::toDescartesConstrainedPath(const std::vector<framefab_msgs::ElementCandidatePoses>& task_sequence,
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
    add_segment(segs[i], task_sequence[i]);
  } // end segments

  return segs;
}

bool framefab_process_planning::retractPath(
    const std::vector<double>& start_joint, double retract_dist, double TCP_speed,
    descartes_core::RobotModelPtr& descartes_model,
    std::vector<std::vector<double>>& retract_jt_traj)
{
  const int dof = descartes_model->getDOF();

  // solve FK to retrieve start eef plane
  Eigen::Affine3d start_pose;
  descartes_model->getFK(start_joint, start_pose);

  // transit the start eef plane along the direction of the local z axis
  Eigen::Affine3d retract_pose = start_pose * Eigen::Translation3d(retract_dist * start_pose.linear().col(2));

  // debug visualization
//  auto rvz = rviz_visual_tools::RvizVisualTools("world_frame", "pose_v");
//  rvz.publishXArrow(retract_pose, rviz_visual_tools::RED, rviz_visual_tools::XXXXSMALL, 0.1);
//  rvz.publishYArrow(retract_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXXSMALL, 0.1);
//  rvz.publishZArrow(retract_pose, rviz_visual_tools::BLUE, rviz_visual_tools::XXXXSMALL, 0.1);
//  rvz.trigger();

  // solve IK for retract plane, and use a two_rung ladder graph to
  // pick the one with the minimal joint distance to start joint config
  std::vector<std::vector<double>> retract_jt_vec;
  descartes_model->getAllIK(retract_pose, retract_jt_vec);

  if(retract_jt_vec.size() == 0)
  {
    ROS_WARN_STREAM("[RetractPath] failed to directly generate feasible IK for retracted pose!");
    return false;
  }
  else
  {
    // sample around the eef orientation
  }

  const int end_size = retract_jt_vec.size() / dof;

  double delta = 10e4;
  int min_id = 0;
  int cnt = 0;

  for(auto& retract_jt : retract_jt_vec)
  {
    double temp_delta = 0;
    // calculate joint dist
    for(size_t i = 0; i < dof; i++)
    {
      temp_delta += abs(retract_jt[i] - start_joint[i]);
    }

    // update min delta value and id
    if(temp_delta < delta)
    {
      delta = temp_delta;
      min_id = cnt;
    }

    cnt++;
  }

  // interpolate start pose and target pose
  retract_jt_traj = interpolateJoint(start_joint, retract_jt_vec[min_id], JTS_DISC_DELTA);

  // check the validity of the pose, if not, send out warning and return nothing
  for(auto& jt : retract_jt_traj)
  {
    if(!descartes_model->isValid(jt))
    {
      ROS_ERROR("[retract planning] interpolated joint pose is not valid!");
      return false;
    }
  }

  return true;
}