#include <choreo_descartes_planner/choreo_ladder_graph_builder.h>

#include "../include/choreo_descartes_planner/pose_sampling_helpers.h"
#include "../include/choreo_descartes_planner/pose_verification_helpers.h"

// declare for descartes edge policy
namespace descartes_planner
{
typedef boost::function<double(const double*, const double*)> CostFunction;
}

#include "descartes_planner/planning_graph_edge_policy.h"

namespace // anon namespace to hide utility functions
{

void concatenate(descartes_planner::LadderGraph& dest, const descartes_planner::LadderGraph& src)
{
  assert(dest.size() > 0);
  assert(src.size() > 0);
  assert(dest.size() == src.size()); // same number of rungs
  // TODO
  // Combines the joints and edges from one graph, src, into another, dest
  // The joints copy straight over, but the index of the points needs to be transformed
  for (std::size_t i = 0; i < src.size(); ++i)
  {
    // Copy the joints
    auto& dest_joints = dest.getRung(i).data;
    const auto& src_joints = src.getRung(i).data;
    dest_joints.insert(dest_joints.end(), src_joints.begin(), src_joints.end());

    if (i != src.size() - 1)
    {
      // Copy the edges and transform them
      const auto next_rung_size = dest.rungSize(i + 1);
      auto& dest_edges = dest.getEdges(i);
      const auto& src_edges = src.getEdges(i);
      for (const auto& edge : src_edges)
      {
        auto edge_copy = edge;
        for (auto& e : edge_copy)
          e.idx += next_rung_size;
        dest_edges.push_back(edge_copy);
      }
    }
  }
}

} // end anon utility function ns

namespace descartes_planner
{

LadderGraph generateLadderGraphFromPoses(const descartes_core::RobotModel &model,
                                         const std::vector <Eigen::Affine3d> &ps,
                                         const double dt)
{
  descartes_planner::LadderGraph graph(model.getDOF());
  graph.resize(ps.size());

  const descartes_core::TimingConstraint timing(dt);

  // Solve IK for each point
  for (std::size_t i = 0; i < ps.size(); ++i)
  {
    std::vector <std::vector<double>> joint_poses;
    model.getAllIK(ps[i], joint_poses);

    graph.assignRung(i, descartes_core::TrajectoryID::make_nil(), timing, joint_poses);
  }

  bool has_edges_t = true;

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  // for the last rung, there's no out edge
  for (std::size_t i = 0; i < graph.size() - 1; ++i)
  {
    const auto start_idx = i;
    const auto end_idx = i + 1;
    const auto &joints1 = graph.getRung(start_idx).data;
    const auto &joints2 = graph.getRung(end_idx).data;
    const auto &tm = graph.getRung(end_idx).timing;
    const auto dof = model.getDOF();

    const auto start_size = joints1.size() / dof;
    const auto end_size = joints2.size() / dof;

    descartes_planner::DefaultEdgesWithTime builder(start_size, end_size, dof, tm.upper,
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

    std::vector <descartes_planner::LadderGraph::EdgeList> edges = builder.result();
    if (!builder.hasEdges())
    {
//      ROS_WARN("No edges");
    }

    has_edges_t = has_edges_t && builder.hasEdges();
    graph.assignEdges(i, std::move(edges));
  } // end edge loop

  return graph;
}

LadderGraph sampleSingleConfig(const descartes_core::RobotModel &model,
                               const std::vector <Eigen::Vector3d> &origins,
                               const double dt,
                               const Eigen::Matrix3d &orientation,
                               const double z_axis_angle)
{
  std::vector <Eigen::Affine3d> poses;

  for (const auto &o : origins)
  {
    poses.push_back(makePose(o, orientation, z_axis_angle));
  }

  return generateLadderGraphFromPoses(model, poses, dt);
}

LadderGraph sampleSingleConfig(const descartes_core::RobotModel &model,
                               const std::vector <Eigen::Vector3d> &origins,
                               const Eigen::Matrix3d &orientation,
                               const double dt)
{
  std::vector <Eigen::Affine3d> poses;

  for (const auto &o : origins)
  {
    poses.push_back(makePose(o, orientation, z_axis_angle));
  }

  return generateLadderGraphFromPoses(model, poses);
}

LadderGraph sampleConstrainedPaths(const descartes_core::RobotModel &model,
                                   ConstrainedSegment &segment)
{
  // Determine the linear points
  // TODO: should have a lower bound for discretization
  // if sampled point num < threshold, discretize using threshold, otherwise use input linear_disc
  auto points = discretizePositions(segment.start, segment.end, segment.linear_disc);

  // Compute the number of angle steps
  static const auto min_angle = -M_PI;
  static const auto max_angle = M_PI;
  const auto n_angle_disc = std::lround((max_angle - min_angle) / segment.z_axis_disc);
  const auto angle_step = (max_angle - min_angle) / n_angle_disc;

  // Compute the expected time step for each linear point
  double traverse_length = (segment.end - segment.start).norm();
  const auto dt = traverse_length / segment.linear_vel;

  LadderGraph graph(model.getDOF());

  // there will be a ladder rung for each point that we must solve
  graph.resize(points.size());

  // We will build up our graph one configuration at a time: a configuration is a single orientation and z angle disc
  for (const auto &orientation : segment.orientations)
  {
    std::vector <Eigen::Vector3d> process_pts = points;

    for (long i = 0; i < n_angle_disc; ++i)
    {
      const auto angle = angle_step * i;

      LadderGraph single_config_graph = sampleSingleConfig(model, process_pts, dt, orientation, angle);
      concatenate(graph, single_config_graph);
    }
  }

  // TODO: should invoke fine resolution if any rung has no vertex

  return graph;
}

// TODO:
void appendInTime(LadderGraph &current, const LadderGraph &next)
{
  const auto ref_size = current.size();
  const auto new_total_size = ref_size + next.size();

  // So step 1 is to literally add the two sets of joints and edges together to make
  // one longer graph
  current.resize(new_total_size);
  for (std::size_t i = 0; i < next.size(); ++i)
  {
    current.getRung(ref_size + i) = next.getRung(i);
  }

  // The second problem is that we now need to 'connect' the two graphs.
  if (ref_size > 0 && next.size() > 0)
  {
    const auto dof = current.dof();
    auto &a_rung = current.getRung(ref_size - 1);
    auto &b_rung = current.getRung(ref_size);

    const auto n_start = a_rung.data.size() / dof;
    const auto n_end = b_rung.data.size() / dof;

    descartes_planner::DefaultEdgesWithoutTime builder(n_start, n_end, dof);

    for (size_t k = 0; k < n_start; k++) // from rung
    {
      const auto start_index = k * dof;

      for (size_t j = 0; j < n_end; j++) // to rung
      {
        const auto end_index = j * dof;

        builder.consider(&a_rung.data[start_index], &b_rung.data[end_index], j);
      }
      builder.next(k);
    }

    std::vector <descartes_planner::LadderGraph::EdgeList> edges = builder.result();

    if (!builder.hasEdges())
    {
      ROS_ERROR("[Descartes graph builder] No edges!!!");
    }

    current.assignEdges(ref_size - 1, std::move(edges));
  }
}
}
