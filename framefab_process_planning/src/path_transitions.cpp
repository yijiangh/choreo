//
// Created by yijiangh on 7/7/17.
//
#include "path_transitions.h"
#include "trajectory_utils.h"

#include <choreo_descartes_planner/pose_sampling_helpers.h>

// for extra descartes graph building feature
#include <descartes_planner/ladder_graph.h>
#include <choreo_descartes_planner/choreo_ladder_graph_builder.h>

// declare for descartes edge policy
namespace descartes_planner
{
typedef boost::function<double(const double*, const double*)> CostFunction;
}

#include <descartes_planner/planning_graph_edge_policy.h>
#include <descartes_planner/ladder_graph_dag_search.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Vector3.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

const static double JTS_DISC_DELTA = 0.01; // radians
const static double RETRACT_OFFSET_SAMPLE_TIMEOUT = 5.0;
const static double MIN_RETRACTION_DIST = 0.003; // m

namespace // anon namespace to hide utility functions
{
// TODO: should be moved to geometry_conversion_helpers
// convert orientations representing by Eigen3d::vector into Eigen::Matrix3d
void convertOrientationVector(
    const std::vector<geometry_msgs::Vector3>& orients_msg,
    std::vector<Eigen::Matrix3d>& m_orients)
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
    if (std::abs(eigen_vec.dot(Eigen::Vector3d::UnitX())) > 0.8)
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

namespace framefab_process_planning
{
std::vector<descartes_planner::ConstrainedSegmentPickNPlace> toDescartesConstrainedPath(
    const framefab_msgs::AssemblySequencePickNPlace& as_pnp,
    const std::vector<planning_scene::PlanningScenePtr>& planning_scene_pick,
    const std::vector<planning_scene::PlanningScenePtr>& planning_scene_place,
    const double& linear_vel, const double& linear_disc)
{
  using ConstrainedSegmentPickNPlace = descartes_planner::ConstrainedSegmentPickNPlace;

  assert(linear_disc > 0 && linear_vel > 0);
  assert(as_pnp.sequenced_elements.size() > 0);
  assert(as_pnp.sequenced_elements.size() == planning_scene_pick.size());
  assert(as_pnp.sequenced_elements.size() == planning_scene_place.size());

  std::vector<ConstrainedSegmentPickNPlace> segs(as_pnp.sequenced_elements.size());

  for(int i=0;i <as_pnp.sequenced_elements.size(); i++)
  {
    const auto& se = as_pnp.sequenced_elements[i];

    assert(se.grasps.size() > 0);
    segs[i].path_pts.resize(4);

    segs[i].path_pts[0].resize(2);
    tf::pointMsgToEigen(se.grasps[0].pick_grasp_approach_pose.position, segs[i].path_pts[0][0]);
    tf::pointMsgToEigen(se.grasps[0].pick_grasp_pose.position, segs[i].path_pts[0][1]);

    segs[i].path_pts[1].resize(2);
    tf::pointMsgToEigen(se.grasps[0].pick_grasp_pose.position, segs[i].path_pts[1][0]);
    tf::pointMsgToEigen(se.grasps[0].pick_grasp_retreat_pose.position, segs[i].path_pts[1][1]);

    segs[i].path_pts[2].resize(2);
    tf::pointMsgToEigen(se.grasps[0].place_grasp_approach_pose.position, segs[i].path_pts[2][0]);
    tf::pointMsgToEigen(se.grasps[0].place_grasp_pose.position, segs[i].path_pts[2][1]);

    segs[i].path_pts[3].resize(2);
    tf::pointMsgToEigen(se.grasps[0].place_grasp_pose.position, segs[i].path_pts[3][0]);
    tf::pointMsgToEigen(se.grasps[0].place_grasp_retreat_pose.position, segs[i].path_pts[3][1]);

    // six path points in total
    segs[i].orientations.resize(4);

    for(int j=0; j < se.grasps.size(); j++)
    {
      const auto& g = se.grasps[j];
      Eigen::Quaterniond q;

      // convert geometry_msgs::Quaterion back to eigen::Quaterion
      // then back to matrix3d
      tf::quaternionMsgToEigen(g.pick_grasp_approach_pose.orientation, q);
      segs[i].orientations[0].push_back(q.toRotationMatrix());

      tf::quaternionMsgToEigen(g.pick_grasp_pose.orientation, q);
      segs[i].orientations[1].push_back(q.toRotationMatrix());

      tf::quaternionMsgToEigen(g.place_grasp_approach_pose.orientation, q);
      segs[i].orientations[2].push_back(q.toRotationMatrix());

      tf::quaternionMsgToEigen(g.place_grasp_pose.orientation, q);
      segs[i].orientations[3].push_back(q.toRotationMatrix());
    }

    segs[i].planning_scenes.push_back(planning_scene_pick[i]);
    segs[i].planning_scenes.push_back(planning_scene_pick[i]);
    segs[i].planning_scenes.push_back(planning_scene_place[i]);
    segs[i].planning_scenes.push_back(planning_scene_place[i]);

    segs[i].linear_vel = linear_vel;
    segs[i].linear_disc = linear_disc;
  }

  return segs;
}

std::vector <descartes_planner::ConstrainedSegment>
toDescartesConstrainedPath(const std::vector <framefab_msgs::ElementCandidatePoses> &task_sequence,
                           const ConstrainedSegParameters &seg_params)
{
  using ConstrainedSegment = descartes_planner::ConstrainedSegment;

  assert(task_sequence.size() > 0);
  std::vector<ConstrainedSegment> segs(task_sequence.size());

  assert(seg_params.linear_disc > 0 && seg_params.linear_vel > 0);
  assert(seg_params.angular_disc > 0 && seg_params.retract_dist > 0);

  // Inline function for adding a sequence of motions
  auto add_segment = [seg_params]
      (ConstrainedSegment &seg, const framefab_msgs::ElementCandidatePoses path_pose)
  {
    tf::pointMsgToEigen(path_pose.start_pt, seg.start);
    tf::pointMsgToEigen(path_pose.end_pt, seg.end);
    convertOrientationVector(path_pose.feasible_orients, seg.orientations);

    seg.linear_vel = seg_params.linear_vel;
    seg.linear_disc = seg_params.linear_disc;

    // TODO: spatial extrusion specific
    switch (path_pose.type)
    {
      case framefab_msgs::ElementCandidatePoses::SUPPORT:
      {
        seg.process_type = ConstrainedSegment::PROCESS_TYPE::SUPPORT;
        break;
      }
      case framefab_msgs::ElementCandidatePoses::CREATE:
      {
        seg.process_type = ConstrainedSegment::PROCESS_TYPE::CREATE;
        break;
      }
      case framefab_msgs::ElementCandidatePoses::CONNECT:
      {
        seg.process_type = ConstrainedSegment::PROCESS_TYPE::CONNECT;
        break;
      }
    }

    seg.z_axis_disc = seg_params.angular_disc;
    seg.retract_dist = seg_params.retract_dist;
  };

  for (std::size_t i = 0; i < task_sequence.size(); ++i)
  {
    add_segment(segs[i], task_sequence[i]);
  } // end segments

  return segs;
}

std::vector <descartes_planner::ConstrainedSegment> toDescartesConstrainedPath(
    const std::vector<framefab_msgs::ElementCandidatePoses> &task_sequence,
    const double &linear_vel, const double &linear_disc, const double &angular_disc, const double &retract_dist)
{
  ConstrainedSegParameters sp;
  sp.linear_vel = linear_vel;
  sp.linear_disc = linear_disc;
  sp.angular_disc = angular_disc;
  sp.retract_dist = retract_dist;

  return toDescartesConstrainedPath(task_sequence, sp);
}

bool retractPath(
    const std::vector<double> &start_joint, double retract_dist, double TCP_speed,
    const std::vector <Eigen::Matrix3d> &eef_directions,
    descartes_core::RobotModelPtr &model,
    std::vector <std::vector<double>> &retract_jt_traj)
{
  if (retract_dist < MIN_RETRACTION_DIST)
  {
    ROS_ERROR_STREAM("[process planning] recursive retraction sampling fails, retraction dist "
                         << retract_dist << "/" << MIN_RETRACTION_DIST);
    return false;
  }

  const int dof = model->getDOF();

  // solve FK to retrieve start eef plane
  Eigen::Affine3d start_pose;
  if (!model->getFK(start_joint, start_pose))
  {
    return false;
  }

  const auto retract_sample_start = ros::Time::now();

  std::vector <Eigen::Affine3d> retract_eef_poses;
  descartes_planner::LadderGraph graph(model->getDOF());

  const descartes_core::TimingConstraint timing(retract_dist / TCP_speed);

  // sample feasible directions for offset movement vector
  bool first_try = true;
  bool exist_pose_not_feasible = true;

  while ((ros::Time::now() - retract_sample_start).toSec() < RETRACT_OFFSET_SAMPLE_TIMEOUT)
  {
    // generate index sample
    retract_eef_poses.clear();
    graph.clear();

    Eigen::Matrix3d offset_vec;
    if (first_try)
    {
      // transit the start eef plane along the direction of the local z axis
      offset_vec = start_pose.matrix().block<3, 3>(0, 0);
    }
    else
    {
      int sample_id = descartes_planner::randomSampleInt(0, eef_directions.size() - 1);
      offset_vec = eef_directions[sample_id];
    }

    Eigen::Affine3d retract_pose =
        start_pose * Eigen::Translation3d(retract_dist * offset_vec.col(2));

    auto st_pt = start_pose.matrix().col(3).head<3>();
    auto end_pt = retract_pose.matrix().col(3).head<3>();
    double ds = (st_pt - end_pt).norm() / 2;

    std::vector <Eigen::Vector3d> path_pts = descartes_planner::discretizePositions(st_pt, end_pt, ds);
    graph.resize(path_pts.size());

    for (auto &pt : path_pts)
    {
      Eigen::Affine3d pose = start_pose;
      pose.matrix().col(3).head<3>() = pt;

      retract_eef_poses.push_back(pose);
    }

    exist_pose_not_feasible = false;

    for (int i = 0; i < retract_eef_poses.size(); i++)
    {
      std::vector <std::vector<double>> retract_jt_vec;
      retract_jt_vec.clear();

      if (0 == i)
      {
        retract_jt_vec.push_back(start_joint);
      }
      else
      {
        if (!model->getAllIK(retract_eef_poses[i], retract_jt_vec))
        {
          exist_pose_not_feasible = true;
          break;
        }
      }

      graph.assignRung(i, descartes_core::TrajectoryID::make_nil(), timing, retract_jt_vec);
    }

    if (!exist_pose_not_feasible)
    {
      if (!first_try)
      {
        ROS_INFO_STREAM("[retraction planning] sampled retraction pose used.");
      }

      break;
    }

    first_try = false;
  }

  for (std::size_t i = 0; i < graph.size(); i++)
  {
    if (0 == graph.rungSize(i) || exist_pose_not_feasible)
    {
      ROS_WARN_STREAM("[process planning] retraction sampling fails, recursively decrease retract dist to "
                          << retract_dist * 0.8);
      return retractPath(start_joint, retract_dist * 0.8, TCP_speed, eef_directions, model, retract_jt_traj);
    }
  }

  // build edges
  for (std::size_t i = 0; i < graph.size() - 1; ++i)
  {
    const auto start_idx = i;
    const auto end_idx = i + 1;
    const auto &joints1 = graph.getRung(start_idx).data;
    const auto &joints2 = graph.getRung(end_idx).data;
    const auto &tm = graph.getRung(end_idx).timing;

    const auto start_size = joints1.size() / dof;
    const auto end_size = joints2.size() / dof;

    descartes_planner::DefaultEdgesWithTime builder(start_size, end_size, dof, tm.upper,
                                                    model->getJointVelocityLimits());
    for (size_t k = 0; k < start_size; k++) // from rung
    {
      const auto start_index = k * dof;

      for (size_t j = 0; j < end_size; j++) // to rung
      {
        const auto end_index = j * dof;

        builder.consider(&joints1[start_index], &joints2[end_index], j);
      }
      builder.next(k);
    }

    std::vector <descartes_planner::LadderGraph::EdgeList> edges = builder.result();

    graph.assignEdges(i, std::move(edges));
  }

  descartes_planner::DAGSearch search(graph);
  double cost = search.run();
  auto path_idxs = search.shortestPath();

  retract_jt_traj.clear();
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto *data = graph.vertex(j, idx);
    retract_jt_traj.push_back(std::vector<double>(data, data + dof));
  }

  return true;
}

}