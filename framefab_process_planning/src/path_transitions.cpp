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
#include <descartes_planner/ladder_graph.h>
#include <descartes_planner/graph_builder.h>
#include "descartes_planner/planning_graph_edge_policy.h"
#include "descartes_planner/ladder_graph_dag_search.h"

#include "trajectory_utils.h"

const static double JTS_DISC_DELTA = 0.01; // radians
const static double RETRACT_OFFSET_SAMPLE_TIMEOUT = 5.0;

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

static int randomSampleInt(int lower, int upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_int_distribution<int> int_distr(lower, upper);
    return int_distr(gen);
  }
  else
  {
    return lower;
  }
}

std::vector<Eigen::Vector3d> discretizePositions(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds)
{
  auto dist = (stop - start).norm();

  size_t n_intermediate_points = 0;
  if (dist > ds)
  {
    n_intermediate_points = static_cast<size_t>(std::lround(dist / ds));
  }

  const auto total_points = 2 + n_intermediate_points;

  std::vector<Eigen::Vector3d> result;
  result.reserve(total_points);

  for (std::size_t i = 0; i < total_points; ++i)
  {
    const double r = i / static_cast<double>(total_points - 1);
    Eigen::Vector3d point = start + (stop - start) * r;
    result.push_back(point);
  }
  return result;
}

} // end utility function ns

using DescartesConstrainedPathConversionFunc =
boost::function<descartes_planner::ConstrainedSegment (const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                       const std::vector<Eigen::Vector3d> &)>;

std::vector<descartes_planner::ConstrainedSegment>
framefab_process_planning::toDescartesConstrainedPath(const std::vector<framefab_msgs::ElementCandidatePoses>& task_sequence,
                                                      const int index, const ConstrainedSegParameters& seg_params)
{
  using ConstrainedSegment = descartes_planner::ConstrainedSegment;

  // index is under 0-index convention in C++, thus + 1 for full size
  int selected_path_num = index + 1;
  std::vector<ConstrainedSegment> segs(selected_path_num);

  // Inline function for adding a sequence of motions
  auto add_segment = [seg_params]
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
    const std::vector<Eigen::Matrix3d>& eef_directions,
    descartes_core::RobotModelPtr& model,
    std::vector<std::vector<double>>& retract_jt_traj)
{
  const int dof = model->getDOF();

  // solve FK to retrieve start eef plane
  Eigen::Affine3d start_pose;
  model->getFK(start_joint, start_pose);

  const auto retract_sample_start = ros::Time::now();

  // TODO: better strategy to generate retract pose?
  // transit the start eef plane along the direction of the local z axis
  std::vector<Eigen::Affine3d> retract_eef_poses;
  descartes_planner::LadderGraph graph(model.getDOF());

  // sample feasible directions for offset movement vector
  while((ros::Time::now() - retract_sample_start).toSec() < RETRACT_OFFSET_SAMPLE_TIMEOUT)
  {
    // generate index sample
    retract_eef_poses.clear();
    graph.clear();

    int sample_id = randomSampleInt(0, eef_directions.size()-1);
    Eigen::Matrix3d offset_vec = eef_directions[sample_id];

    Eigen::Affine3d retract_pose =
        start_pose * Eigen::Translation3d(retract_dist * offset_vec.col(2));

    auto st_pt = start_pose.matrix().col(3).head<3>();
    auto end_pt = retract_pose.matrix().col(3).head<3>();
    double ds = (st_pt - end_pt).norm() / 5;

    std::vector<Eigen::Vector3d> path_pts = discretizePositions(st_pt, end_pt, ds);
    const descartes_core::TimingConstraint timing(retract_dist / TCP_speed);

    for(auto& pt : path_pts)
    {
      Eigen::Affine3d pose = start_pose;
      pose.matrix().col(3).head<3>() = pt;

      retract_eef_poses.push_back(pose);
    }

    bool exist_pose_not_feasible = false;

    for(int i=0; i < retract_eef_poses.size(); i++)
    {
      std::vector <std::vector<double>> retract_jt_vec;
      model->getAllIK(retract_eef_poses[i], retract_jt_vec);

      if(0 == retract_jt_vec.size())
      {
        exist_pose_not_feasible = true;
        break;
      }
      else
      {
        graph.assignRung(i, descartes_core::TrajectoryID::make_nil(), timing, retract_jt_vec);
      }
    }

    if(!exist_pose_not_feasible && graph.size() == retract_eef_poses.size())
    {
      break;
    }
  }

  if(graph.size() != retract_eef_poses.size())
  {
    ROS_WARN_STREAM("[process planning] retraction sampling failed to find feasible retraction pose.");
    return false;
  }

  // build edges
  for (std::size_t i = 0; i < graph.size() - 1; ++i)
  {
    const auto start_idx = i;
    const auto end_idx = i + 1;
    const auto& joints1 = graph.getRung(start_idx).data;
    const auto& joints2 = graph.getRung(end_idx).data;
    const auto& tm = graph.getRung(end_idx).timing;
    const auto dof = model.getDOF();

    const auto start_size = joints1.size() / dof;
    const auto end_size = joints2.size() / dof;

    descartes_planner::DefaultEdgesWithTime builder (start_size, end_size, dof, tm.upper,
                                                     model.getJointVelocityLimits());
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

    std::vector<descartes_planner::LadderGraph::EdgeList> edges = builder.result();

    graph.assignEdges(i, std::move(edges));
  }

  descartes_planner::DAGSearch search(unified_graph);
  double cost = search.run();
  auto path_idxs = search.shortestPath();

  retract_jt_traj.clear();
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto* data = graph.vertex(j, idx);
    retract_jt_traj.push_back(std::vector<double>(data, data + 6));
  }

  return true;
}