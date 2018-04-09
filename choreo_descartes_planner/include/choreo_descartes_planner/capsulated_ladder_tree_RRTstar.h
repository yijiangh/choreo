//
// Created by yijiangh on 11/28/17.
//

#ifndef CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
#define CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H

#include <moveit/planning_scene/planning_scene.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

// for ConstrainedSegment
#include "choreo_ladder_graph_builder.h"
#include "capsulated_ladder_tree.h"

namespace descartes_planner
{
// RRT* on capsulated ladder tree
class CapsulatedLadderTreeRRTstar
{
 public:
  explicit CapsulatedLadderTreeRRTstar(
      const std::vector<ConstrainedSegment>& segs,
      const std::vector<planning_scene::PlanningScenePtr>& planning_scenes,
      const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_completed);

  ~CapsulatedLadderTreeRRTstar();

  // use RRT* on a ladder tree to get optimal capsulated solution
  // return the cost of the solution, if no sol found, return numerical max
  double solve(descartes_core::RobotModel& model, double clt_rrt_unit_process_timeout, double clt_rrt_timeout);

  // construct ladder graph for each capsule and apply DAG search to get full trajectory solution
  void extractSolution(descartes_core::RobotModel& model,
                       std::vector<descartes_core::TrajectoryPtPtr>& sol,
                       std::vector<descartes_planner::LadderGraph>& graphs,
                       std::vector<int>& graph_indices,
                       const bool use_saved_graph);

 private:
  std::vector<CapRung> cap_rungs_;
};
}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
