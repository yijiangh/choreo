//
// Created by yijiangh on 4/10/18.
//

//// declare for choreo descartes planners' sampling and feasibility checking function
//namespace descartes_planner
//{
//
////  generateSample(const descartes_planner::,
//// descartes_planner::)
//
//typedef boost::function<std::vector<Eigen::Affine3d>(const CapRung& cap_rung, const CapVert& cap_vert)> SegmentSamplingFunction;
//
//}

#include "semi_constrained_cartesian_planning.h"
#include "common_utils.h"
#include"path_transitions.h"

// cap ladder tree RRTstar
#include <choreo_descartes_planner/choreo_ladder_graph_builder.h>
#include <choreo_descartes_planner/capsulated_ladder_tree_RRTstar.h>

#include <descartes_planner/ladder_graph_dag_search_lazy_collision.h>
#include <descartes_planner/ladder_graph_dag_search.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_core/trajectory_pt.h>

#include <moveit/planning_scene/planning_scene.h>

// for serializing ladder graph
#include <descartes_parser/descartes_parser.h>

// serialization
#include <framefab_param_helpers/framefab_param_helpers.h>

#include <descartes_msgs/LadderGraphList.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace {

bool saveLadderGraph(const std::string &filename, const descartes_msgs::LadderGraphList &graph_list_msg)
{
  if (!framefab_param_helpers::toFile(filename, graph_list_msg))
  {
    ROS_WARN_STREAM("Unable to save ladder graph list to: " << filename);
  }
}

} // anon util namespace

namespace framefab_process_planning
{

// TODO: overhead for spatial extrusion
void CLTRRTforProcessROSTraj(descartes_core::RobotModelPtr model,
                             std::vector <descartes_planner::ConstrainedSegment> &segs,
                             const double clt_rrt_unit_process_timeout,
                             const double clt_rrt_timeout,
                             const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_approach,
                             const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_depart,
                             const std::vector <framefab_msgs::ElementCandidatePoses> &task_seq,
                             std::vector <framefab_msgs::UnitProcessPlan> &plans,
                             const std::string &saved_graph_file_name,
                             bool use_saved_graph)
{
  // sanity check
  assert(segs.size() == task_seq.size());

  const auto clt_start = ros::Time::now();

  framefab_process_planning::DescartesTraj sol;

  std::vector <descartes_planner::LadderGraph> graphs;
  std::vector<int> graph_indices;
  descartes_msgs::LadderGraphList graph_list_msg;
  double clt_cost = 0;

  if (use_saved_graph)
  {
    if (framefab_param_helpers::fromFile(saved_graph_file_name, graph_list_msg))
    {
      ROS_INFO_STREAM("[CLR RRT*] use saved ladder graph.");

      //parse saved ladder graph
      graphs = descartes_parser::convertToLadderGraphList(graph_list_msg);
    }
    else
    {
      ROS_WARN_STREAM("[CLT RRT*] read saved ladder graph list fails. Reconstruct graph.");
      // reading msg fails, reconstruct ladder graph
      use_saved_graph = false;
    }
  }

  descartes_planner::CapsulatedLadderTreeRRTstar CLT_RRT(segs, planning_scenes_approach, planning_scenes_depart);

  if (!use_saved_graph || segs.size() > graphs.size())
  {
    // reconstruct and search, output sol, graphs, graph_indices
    clt_cost = CLT_RRT.solve(*model, clt_rrt_unit_process_timeout, clt_rrt_timeout);
    CLT_RRT.extractSolution(*model,
                            sol,
                            graphs,
                            graph_indices,
                            use_saved_graph);

    graph_list_msg = descartes_parser::convertToLadderGraphListMsg(graphs);
    saveLadderGraph(saved_graph_file_name, graph_list_msg);
  }
  else
  {
    // default start from begin
    std::vector <descartes_planner::LadderGraph> chosen_graphs(graphs.begin(), graphs.begin() + segs.size());
    CLT_RRT.extractSolution(*model, sol,
                            chosen_graphs,
                            graph_indices, use_saved_graph);
  }

  const auto clt_end = ros::Time::now();
  ROS_INFO_STREAM("[CLT RRT*] CLT RRT* Search took " << (clt_end - clt_start).toSec()
                                                     << " seconds");

  trajectory_msgs::JointTrajectory ros_traj = framefab_process_planning::toROSTrajectory(sol, *model);

  auto it = ros_traj.points.begin();
  for (size_t i = 0; i < task_seq.size(); i++)
  {
    framefab_msgs::SubProcess sub_process;

    sub_process.unit_process_id = i;
    sub_process.process_type = framefab_msgs::SubProcess::PROCESS;
    sub_process.main_data_type = framefab_msgs::SubProcess::CART;

    switch (task_seq[i].type)
    {
      case framefab_msgs::ElementCandidatePoses::SUPPORT:
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::SUPPORT;
        break;
      }
      case framefab_msgs::ElementCandidatePoses::CREATE:
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::CREATE;
        break;
      }
      case framefab_msgs::ElementCandidatePoses::CONNECT:
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::CONNECT;
        break;
      }
      default:
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::NONE;
        ROS_WARN_STREAM("[Process Planning] printing process #" << i << " have no element process type!");
      }
    }

    sub_process.joint_array.points = std::vector<trajectory_msgs::JointTrajectoryPoint>(it, it + graph_indices[i]);

    plans[i].sub_process_array.push_back(sub_process);

    it = it + graph_indices[i];
  }
}

// TODO: overload for picknplace
void CLTRRTforProcessROSTraj(descartes_core::RobotModelPtr model,
                             const framefab_msgs::AssemblySequencePickNPlace& as_pnp,
                             const double clt_rrt_unit_process_timeout,
                             const double clt_rrt_timeout,
                             const double& linear_vel,
                             const double& linear_disc,
                             const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_subprocess,
                             std::vector <framefab_msgs::UnitProcessPlan>& plans,
                             const std::string &saved_graph_file_name,
                             bool use_saved_graph)
{
  // sanity check, segs' size is used as req.index
  const auto clt_start = ros::Time::now();

  framefab_process_planning::DescartesTraj sol;

  std::vector <descartes_planner::LadderGraph> graphs;
  std::vector<std::vector<int>> graph_indices;
  std::vector<int> flat_graph_ids; // un-partitioned graph indices

  descartes_msgs::LadderGraphList graph_list_msg;
  double clt_cost = 0;

  if (use_saved_graph)
  {
    if(framefab_param_helpers::fromFile(saved_graph_file_name, graph_list_msg))
    {
      ROS_INFO_STREAM("[CLR RRT*] use saved ladder graph.");

      //parse saved ladder graph & partition indices
      graphs = descartes_parser::convertToLadderGraphList(graph_list_msg, graph_indices);
    }
    else
    {
      ROS_WARN_STREAM("[CLT RRT*] read saved ladder graph list fails. Reconstruct graph.");
      // reading msg fails, reconstruct ladder graph
      use_saved_graph = false;
    }
  }

  // construct segs for descartes & copy chosen task sequence
  const std::vector<descartes_planner::ConstrainedSegmentPickNPlace> segs =
      toDescartesConstrainedPath(as_pnp, planning_scenes_subprocess, linear_vel, linear_disc);

  ROS_INFO_STREAM("[CLT RRT*] Constrained segment constructed.");

  // partition indices will be init inside CLT_RRT
  descartes_planner::CapsulatedLadderTreeRRTstar CLT_RRT(segs);

  // if request size bigger than the saved one, recompute
  if (!use_saved_graph || segs.size() > graphs.size())
  {
    // TODO: do we need customized extractSolution function?
    // reconstruct and search, output sol, graphs, graph_indices
    clt_cost = CLT_RRT.solvePickNPlace(*model, clt_rrt_unit_process_timeout, clt_rrt_timeout);
    CLT_RRT.extractSolution(*model,
                            sol,
                            graphs,
                            flat_graph_ids,
                            use_saved_graph);

    graph_indices = CLT_RRT.getGraphPartitionIds();

    assert(graph_indices.size() == graphs.size());
    graph_list_msg = descartes_parser::convertToLadderGraphListMsg(graphs, graph_indices);
    saveLadderGraph(saved_graph_file_name, graph_list_msg);
  }
  else
  {
    // TODO: do we need customized extractSolution function?
    // default start from begin
    std::vector <descartes_planner::LadderGraph> chosen_graphs(graphs.begin(), graphs.begin() + segs.size());
    CLT_RRT.extractSolution(*model,
                            sol,
                            chosen_graphs,
                            flat_graph_ids,
                            use_saved_graph);

    graph_indices = CLT_RRT.getGraphPartitionIds();

    // sanity check
    assert(graph_indices.size() == flat_graph_ids.size());
    for(int k=0; k<flat_graph_ids.size(); k++)
    {
      int sum = 0;
      for(auto& chuck_num : graph_indices[k])
      {
        sum += chuck_num;
      }
      assert(sum == flat_graph_ids[k]);
    }
  }

  const auto clt_end = ros::Time::now();
  ROS_INFO_STREAM("[CLT RRT*] CLT RRT* Search took " << (clt_end - clt_start).toSec()
                                                     << " seconds");

  trajectory_msgs::JointTrajectory ros_traj = framefab_process_planning::toROSTrajectory(sol, *model);

  auto it = ros_traj.points.begin();
  for (size_t i = 0; i < segs.size(); i++)
  {
    framefab_msgs::SubProcess sub_process;

    sub_process.unit_process_id = i;
    sub_process.process_type = framefab_msgs::SubProcess::RETRACTION;
    sub_process.main_data_type = framefab_msgs::SubProcess::CART;

    // must contain two index partition: pick approach, pick depart, place approach, place depart
    assert(graph_indices[i].size() == 4);

    for(int k = 0; k < 4; k++)
    {
      if(k % 2 == 0)
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::APPROACH;
      }
      else
      {
        sub_process.element_process_type = framefab_msgs::SubProcess::DEPART;
      }

      if(k <= 1)
      {
        sub_process.comment = "pick";
      }
      else
      {
        sub_process.comment = "place";
      }

      sub_process.joint_array.points = std::vector<trajectory_msgs::JointTrajectoryPoint>(it, it + graph_indices[i][k]);
      plans[i].sub_process_array.push_back(sub_process);

      it = it + graph_indices[i][k];
    }
  }
}

}