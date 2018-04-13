//
// Created by yijiangh on 11/28/17.
//

#ifndef CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
#define CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H

#include "constrained_segment.h"
#include "capsulated_ladder_tree.h"

#include <moveit/planning_scene/planning_scene.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

namespace descartes_planner
{
// RRT* on capsulated ladder tree
class CapsulatedLadderTreeRRTstar
{
 public:
  // for spatial extrusion
  CapsulatedLadderTreeRRTstar(
      const std::vector<ConstrainedSegment>& segs,
      const std::vector<planning_scene::PlanningScenePtr>& planning_scenes,
      const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_completed
      = std::vector<planning_scene::PlanningScenePtr>());

  // for picknplace
  CapsulatedLadderTreeRRTstar(
      const std::vector<ConstrainedSegmentPickNPlace>& segs);

  ~CapsulatedLadderTreeRRTstar();

  std::vector<std::vector<int>> getGraphPartitionIds() const
  {
    assert(cap_rungs_.size() > 0);
    std::vector<std::vector<int>> p_ids;

    for(const auto& c : cap_rungs_)
    {
      p_ids.push_back(c.sub_segment_ids_);
    }

    return p_ids;
  }

  // TODO: should pass generate sample, feasibility checking function in
  // use RRT* on a ladder tree to get optimal capsulated solution
  // return the cost of the solution, if no sol found, return numerical max
  double solve(descartes_core::RobotModel& model, double clt_rrt_unit_process_timeout, double clt_rrt_timeout);

  // TODO: temporal function for solve PnP, this should be merged to a more universal solve function
  double solvePickNPlace(descartes_core::RobotModel& model, double clt_rrt_unit_process_timeout, double clt_rrt_timeout);

  // construct ladder graph for each capsule and apply DAG search to get full trajectory solution
  void extractSolution(descartes_core::RobotModel& model,
                       std::vector<descartes_core::TrajectoryPtPtr>& sol,
                       std::vector<descartes_planner::LadderGraph>& graphs,
                       std::vector<int>& graph_indices,
                       const bool use_saved_graph);

 private:
  std::vector<CapRung> cap_rungs_;

  // TODO
  // template:
  // std::vector<Eigen::Affine3d> generateSample(const descartes_planner::CapRung& cap_rung,
  // descartes_planner::CapVert& cap_vert)
//  descartes_planner::SegmentSamplingFunction custom_sampling_fn_;

  // TODO
  // template:

};
}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
