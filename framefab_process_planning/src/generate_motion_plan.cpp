//
// Created by yijiangh on 7/8/17.
//
#include <ros/console.h>

#include <Eigen/Geometry>

#include "generate_motion_plan.h"
#include "trajectory_utils.h"
#include "path_transitions.h"
#include "common_utils.h"
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

#include <descartes_planner/ladder_graph_dag_search_lazy_collision.h>
#include <descartes_planner/ladder_graph_dag_search.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/graph_builder.h>
#include <moveit/planning_scene/planning_scene.h>

// cap ladder tree RRTstar
#include <descartes_planner/capsulated_ladder_tree_RRTstar.h>

// pose conversion
#include <eigen_conversions/eigen_msg.h>

// for serializing ladder graph
#include <descartes_parser/descartes_parser.h>

// for immediate execution
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// msg
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <framefab_msgs/SubProcess.h>
#include <framefab_msgs/ElementCandidatePoses.h>

// srv
#include <moveit_msgs/ApplyPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

// serialization
#include <framefab_param_helpers/framefab_param_helpers.h>

#define SANITY_CHECK(cond) do { if ( !(cond) ) { throw std::runtime_error(#cond); } } while (false);

namespace{ //util function namespace

void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                             const std::vector<moveit_msgs::CollisionObject>& collision_objs,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes)
{
  const auto build_scenes_start = ros::Time::now();

  planning_scenes.clear();
  planning_scenes.reserve(collision_objs.size());

  planning_scene::PlanningScenePtr root_scene(new planning_scene::PlanningScene(moveit_model));
  root_scene->getCurrentStateNonConst().setToDefaultValues();
  root_scene->getCurrentStateNonConst().update();
  planning_scenes.push_back(root_scene);

  for (std::size_t i = 0; i < collision_objs.size() - 1; ++i) // We use all but the last collision object
  {
    auto last_scene = planning_scenes.back();
    auto child = last_scene->diff();

    if (!child->processCollisionObjectMsg(collision_objs[i]))
    {
      ROS_WARN("[Process Planning] Failed to process collision object");
    }
    planning_scenes.push_back(child);
  }

  const auto build_scenes_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] constructing planning scenes took " << (build_scenes_end-build_scenes_start).toSec()
                                                                          << " seconds.");
}

void constructLadderGraphSegments(descartes_core::RobotModelPtr model,
                                  moveit::core::RobotModelConstPtr moveit_model,
                                  std::vector<descartes_planner::ConstrainedSegment>& segs,
                                  const std::vector<planning_scene::PlanningScenePtr>& planning_scenes,
                                  const std::string& move_group_name,
                                  const std::vector<std::string> &joint_names,
                                  std::vector<descartes_planner::LadderGraph>& saved_graphs,
                                  std::vector<descartes_planner::LadderGraph>& graphs,
                                  std::vector<std::size_t>& graph_indices)
{
  auto graph_build_start = ros::Time::now();

  graphs.clear();
  graphs.reserve(segs.size());

  bool use_ladder_graph = bool(saved_graphs.size());

  for (std::size_t i = 0; i < segs.size(); ++i)
  {
    model->setPlanningScene(planning_scenes[i]);

    if (use_ladder_graph && i < saved_graphs.size())
    {
      ROS_INFO_STREAM("[Process Planning] process #" << i << ", orient size" << segs[i].orientations.size()
                                                     << " (saved graph used)");
      graphs.push_back(saved_graphs[i]);
    }
    else
    {
      descartes_planner::ConstrainedSegment &seg = segs[i];
      ROS_INFO_STREAM("[Process Planning] process #" << i << ", orient size: " << seg.orientations.size());
      graphs.push_back(descartes_planner::sampleConstrainedPaths(*model, segs[i]));
    }
    graph_indices.push_back(graphs.back().size());
  }

  auto graph_build_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Ladder Graph building took: "
                      << (graph_build_end - graph_build_start).toSec() << " seconds");
}

void appendLadderGraphSegments(std::vector<descartes_planner::LadderGraph>& graphs,
                                descartes_planner::LadderGraph& final_graph)
{
  const auto append_start = ros::Time::now();

  int append_id = 0;
  for (const auto& graph : graphs)
  {
    ROS_INFO_STREAM("[Process Planning] appending graph #" << append_id);
    descartes_planner::appendInTime(final_graph, graph);
    append_id++;
  }

  const auto append_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Graph appending took: " << (append_end - append_start).toSec() << " seconds");
}

void searchLadderGraphforProcessROSTraj(descartes_core::RobotModelPtr model,
                                        const descartes_planner::LadderGraph& final_graph,
                                        const std::vector<std::size_t>& graph_indices,
                                        const std::vector<int> seg_type_tags,
                                        std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  const auto search_start = ros::Time::now();
  descartes_planner::DAGSearch search(final_graph);
  double cost = search.run();

  const auto search_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] DAG Search took " << (search_end-search_start).toSec()
                                                        << " seconds and produced a result with dist = " << cost);

  // Harvest shortest path in graph to retract trajectory
  auto path_idxs = search.shortestPath();
  ROS_INFO_STREAM("[Process Planning] DAG shortest path completed.");

  framefab_process_planning::DescartesTraj sol;
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto* data = final_graph.vertex(j, idx);
    const auto& tm = final_graph.getRung(j).timing;
    auto pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(
        std::vector<double>(data, data + 6), tm));
    sol.push_back(pt);
  }

  ROS_INFO_STREAM("[Process Planning] descartes traj sol packed.");

  trajectory_msgs::JointTrajectory ros_traj = framefab_process_planning::toROSTrajectory(sol, *model);

  auto it = ros_traj.points.begin();
  for(size_t i = 0; i < seg_type_tags.size(); i++)
  {
    framefab_msgs::SubProcess sub_process;

    sub_process.unit_process_id = i;
    sub_process.process_type = framefab_msgs::SubProcess::PROCESS;
    sub_process.main_data_type = framefab_msgs::SubProcess::CART;

    switch(seg_type_tags[i])
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

    sub_process.joint_array.points =  std::vector<trajectory_msgs::JointTrajectoryPoint>(it, it + graph_indices[i]);

    plans[i].sub_process_array.push_back(sub_process);

    it = it + graph_indices[i];
  }
}

void retractionPlanning(std::vector<framefab_msgs::UnitProcessPlan>& plans,
                        descartes_core::RobotModelPtr model,
                        std::vector<planning_scene::PlanningScenePtr>& planning_scenes,
                        const double& retract_dist,
                        const double& retract_TCP_speed)
{
  if (plans.size() == 0)
  {
    ROS_ERROR("[retraction Planning] plans size = 0!");
    assert(false);
  }

  const auto ret_planning_start = ros::Time::now();

  for(size_t i = 0; i < plans.size(); i++)
  {
    ROS_INFO_STREAM("[retraction Planning] process #" << i);

    const std::vector<double> start_process_joint = plans[i].sub_process_array.back().joint_array.points.front().positions;
    const std::vector<double> end_process_joint = plans[i].sub_process_array.back().joint_array.points.back().positions;

    if(0 != i)
    {
      const auto last_process_end_joint = plans[i-1].sub_process_array.back().joint_array.points.back().positions;

      // TODO: user option if start process pt = end process pt, skip retraction planning
      if(last_process_end_joint == start_process_joint)
      {
        // skip retraction planning
        ROS_INFO_STREAM("[retraction Planning] process #" << i << "retraction planning skipped.");
        continue;
      }
    }

    model->setPlanningScene(planning_scenes[i]);

    std::vector<std::vector<double>> approach_retract_traj;
    if(!framefab_process_planning::retractPath(start_process_joint, retract_dist, retract_TCP_speed,
                                               model, approach_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible approach retract motion!");
    }
    std::reverse(approach_retract_traj.begin(), approach_retract_traj.end());

    std::vector<std::vector<double>> depart_retract_traj;
    if(!framefab_process_planning::retractPath(end_process_joint, retract_dist, retract_TCP_speed,
                                               model, depart_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible depart retract motion!");
    }

    trajectory_msgs::JointTrajectory approach_ros_traj =
        framefab_process_planning::toROSTrajectory(approach_retract_traj, *model);
    trajectory_msgs::JointTrajectory depart_ros_traj =
        framefab_process_planning::toROSTrajectory(depart_retract_traj, *model);

    framefab_msgs::SubProcess sub_process_approach;

    sub_process_approach.process_type = framefab_msgs::SubProcess::RETRACTION;
    sub_process_approach.main_data_type = framefab_msgs::SubProcess::CART;
    sub_process_approach.element_process_type = framefab_msgs::SubProcess::APPROACH;
    sub_process_approach.joint_array = approach_ros_traj;

    framefab_msgs::SubProcess sub_process_depart;

    sub_process_depart.process_type = framefab_msgs::SubProcess::RETRACTION;
    sub_process_depart.main_data_type = framefab_msgs::SubProcess::CART;
    sub_process_depart.element_process_type = framefab_msgs::SubProcess::DEPART;
    sub_process_depart.joint_array = depart_ros_traj;

    // retract_approach - process - retract depart
    plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process_approach);
    plans[i].sub_process_array.insert(plans[i].sub_process_array.end(), sub_process_depart);
  }

  const auto ret_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[retraction planning] Retraction Planning took " << (ret_planning_end-ret_planning_start).toSec()
                                                                    << " seconds.");
}

void transitionPlanning(std::vector<framefab_msgs::UnitProcessPlan>& plans,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient& planning_scene_diff_client,
                        const std::string& move_group_name,
                        const std::vector<double>& start_state,
                        std::vector<planning_scene::PlanningScenePtr>& planning_scenes)
{
  if(plans.size() == 0)
  {
    ROS_ERROR("[transionPlanning] plans size = 0!");
    assert(false);
  }

  const auto tr_planning_start = ros::Time::now();

  std::vector<double> last_joint_pose = start_state;
  std::vector<double> current_first_joint_pose;

  for(size_t i = 0; i < plans.size(); i++)
  {
    ROS_INFO_STREAM("[Transition Planning] process #" << i);

    if(0 != i)
    {
      last_joint_pose = plans[i-1].sub_process_array.back().joint_array.points.back().positions;
    }

    current_first_joint_pose = plans[i].sub_process_array.front().joint_array.points.front().positions;

    if(last_joint_pose == current_first_joint_pose)
    {
      // skip transition planning
      continue;
    }

    // update the planning scene
    if(!planning_scene_diff_client.waitForExistence())
    {
      ROS_ERROR_STREAM("[Tr Planning] cannot connect with planning scene diff server...");
    }

    moveit_msgs::ApplyPlanningScene srv;
    planning_scenes[i]->getPlanningSceneMsg(srv.request.scene);
    if(!planning_scene_diff_client.call(srv))
    {
      ROS_ERROR_STREAM("[Tr Planning] Failed to publish planning scene diff srv!");
    }

    trajectory_msgs::JointTrajectory ros_trans_traj =
        framefab_process_planning::getMoveitTransitionPlan(move_group_name,
                                                           last_joint_pose,
                                                           current_first_joint_pose,
                                                           start_state,
                                                           moveit_model);

    framefab_msgs::SubProcess sub_process;

    sub_process.process_type = framefab_msgs::SubProcess::TRANSITION;
    sub_process.main_data_type = framefab_msgs::SubProcess::JOINT;
    sub_process.joint_array = ros_trans_traj;

    plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process);
  }

  const auto tr_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Transition Planning took " << (tr_planning_end-tr_planning_start).toSec()
                                                                 << " seconds.");
}

void adjustTrajectoryTiming(std::vector<framefab_msgs::UnitProcessPlan>& plans,
                            const std::vector<std::string> &joint_names)
{
  if (plans.size() == 0)
  {
    ROS_ERROR("[ProcessPlanning : adjustTrajectoryTiming] plans size = 0!");
    assert(false);
  }

  if (0 == plans[0].sub_process_array.size())
  {
    ROS_ERROR("[ProcessPlanning : adjustTrajectoryTiming] plans[0] doesn't have sub process!");
    assert(false);
  }

  framefab_process_planning::fillTrajectoryHeaders(joint_names, plans[0].sub_process_array[0].joint_array);
  auto last_filled_jts = plans[0].sub_process_array[0].joint_array;

  // inline function for append trajectory headers (adjust time frame)
  auto adjustTrajectoryHeaders = [](trajectory_msgs::JointTrajectory& last_filled_jts, framefab_msgs::SubProcess& sp, double sim_speed)
  {
    framefab_process_planning::appendTrajectoryHeaders(last_filled_jts, sp.joint_array, sim_speed);
    last_filled_jts = sp.joint_array;
  };

  for(size_t i = 0; i < plans.size(); i++)
  {
    for (size_t j = 0; j < plans[i].sub_process_array.size(); j++)
    {
      plans[i].sub_process_array[j].unit_process_id = i;
      plans[i].sub_process_array[j].sub_process_id = j;

      double sim_speed = 1.0;
      if(2 != plans[i].sub_process_array[j].process_type)
      {
        sim_speed = 6.0;
      }

      adjustTrajectoryHeaders(last_filled_jts, plans[i].sub_process_array[j], sim_speed);
    }
  }
}

bool saveLadderGraph(const std::string& filename, const descartes_msgs::LadderGraphList& graph_list_msg)
{
  if (!framefab_param_helpers::toFile(filename, graph_list_msg))
  {
    ROS_WARN_STREAM("Unable to save ladder graph list to: " << filename);
    return false;
  }
  return true;
}

void appendTCPPosetoPlans(const descartes_core::RobotModelPtr model,
                          const std::vector<planning_scene::PlanningScenePtr>& planning_scenes,
                          std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  int process_id_count = 0;
  for(auto& unit_plan : plans)
  {
    model->setPlanningScene(planning_scenes[process_id_count]);

    for(auto& sub_process : unit_plan.sub_process_array)
    {
      for(const auto& jt_pt : sub_process.joint_array.points)
      {
        Eigen::Affine3d TCP_pose = Eigen::Affine3d::Identity();
        geometry_msgs::Pose geo_pose_msg;

        // convert it to TCP pose
        if(!model->getFK(jt_pt.positions, TCP_pose))
        {
          ROS_ERROR_STREAM("FK solution failed at unit process #" << sub_process.unit_process_id
                                                                  << ", subprocess #" << sub_process.sub_process_id
                                                                  << ", process type: " << sub_process.process_type
                                                                  << " (0: process, 1: near_model, 2: transition)");
        }

        // affine3d to geometry_msg/pose
        tf::poseEigenToMsg(TCP_pose, geo_pose_msg);

        sub_process.TCP_pose_array.push_back(geo_pose_msg);
      }
    }
    process_id_count++;
  }
}

} // end util namespace

bool framefab_process_planning::generateMotionPlan(
    descartes_core::RobotModelPtr model,
    std::vector<descartes_planner::ConstrainedSegment>& segs,
    const std::vector<moveit_msgs::CollisionObject>& collision_objs,
    const std::vector<int>& seg_type_tags,
    const bool use_saved_graph,
    const std::string& saved_graph_file_name,
    moveit::core::RobotModelConstPtr moveit_model,
    ros::ServiceClient& planning_scene_diff_client,
    const std::string& move_group_name,
    const std::vector<double>& start_state,
    std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  // Step 0: Sanity checks
  if (segs.size() == 0 || collision_objs.size() != segs.size() || seg_type_tags.size() != segs.size())
  {
    ROS_ERROR_STREAM("[Process Planning] input descartes Constrained Segment size" << segs.size()
                                                                                   << ", collision objs size: " << collision_objs.size()
                                                                                   << ", seg type size" << seg_type_tags.size());
    return false;
  }

  plans.resize(segs.size());
  const std::vector<std::string> &joint_names =
      moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

  // Step 1: Let's create all of our planning scenes to collision check against
  std::vector<planning_scene::PlanningScenePtr> planning_scenes;
  constructPlanningScenes(moveit_model, collision_objs, planning_scenes);

//  // temp test CLT RRT*
//  descartes_planner::CapsulatedLadderTreeRRTstar CLT_RRT(segs, planning_scenes);
//  double clt_cost = 0;
//
//  clt_cost = CLT_RRT.solve(*model);
//
//  ROS_WARN_STREAM("CLT RRT* return cost: " << clt_cost);

  // Step 2: sample graph for each segment separately
  int saved_graph_size = 0;
  descartes_msgs::LadderGraphList graph_list_msg;
  std::vector<descartes_planner::LadderGraph> graphs;
  std::vector<std::size_t> graph_indices;

  if(!use_saved_graph)
  {
    std::vector<descartes_planner::LadderGraph> empty_graph_list;

    constructLadderGraphSegments(model, moveit_model, segs, planning_scenes,
                                 move_group_name, joint_names,
                                 empty_graph_list, graphs, graph_indices);
  }
  else
  {
    std::vector<descartes_planner::LadderGraph> saved_graphs;

    // parse saved graph
    if(framefab_param_helpers::fromFile(saved_graph_file_name, graph_list_msg))
    {
      const auto parse_graph_start = ros::Time::now();
      saved_graphs = descartes_parser::convertToLadderGraphList(graph_list_msg);
      saved_graph_size = saved_graphs.size();

      const auto parse_graph_end = ros::Time::now();
      ROS_INFO_STREAM("[Process Planning] ladder graph parsing took " << (parse_graph_end-parse_graph_start).toSec()
                                                                      << " seconds.");
    }
    else
    {
      ROS_WARN("no saved graph found, recompute ladder graphs.");
    }

    // generate graphs based on saved graph
    constructLadderGraphSegments(model, moveit_model, segs, planning_scenes,
                                 move_group_name, joint_names,
                                 saved_graphs, graphs, graph_indices);

    // release saved_graphs memory
    std::vector<descartes_planner::LadderGraph>().swap(saved_graphs);
  }

  // Step 2': save newly generated graphs to msgs
  if(!use_saved_graph || graphs.size() > saved_graph_size)
  {
    // not using saved graph or graph_list size grow

    const auto save_graph_start = ros::Time::now();

    graph_list_msg = descartes_parser::convertToLadderGraphMsg(graphs);

    saveLadderGraph(saved_graph_file_name, graph_list_msg);

    const auto save_graph_end = ros::Time::now();
    ROS_INFO_STREAM("[Process Planning] ladder graph saving took " << (save_graph_end - save_graph_start).toSec()
                                                                   << " seconds.");
  }
  else
  {
    ROS_INFO_STREAM("[Process Planning] a subset of the saved graph is used, no need to save it.");
  }

  // Step 3: append individual graph together to an unified one
  descartes_planner::LadderGraph final_graph(model->getDOF());
  appendLadderGraphSegments(graphs, final_graph);

  // release graphs list memory
  std::vector<descartes_planner::LadderGraph>().swap(graphs);

  ROS_INFO_STREAM("[Proceess Planning] unified ladder graph has " << final_graph.numVertices() << " vertices, "
                                                                  << final_graph.size() << " rungs");

  // Next, we build a search for the whole problem
  searchLadderGraphforProcessROSTraj(model, final_graph, graph_indices, seg_type_tags, plans);

  // retract planning
  // TODO: move this into param
  double retraction_dist = 0.010; // meters
  double ret_TCP_speed = 0.0005; // m/s
  retractionPlanning(plans, model, planning_scenes, retraction_dist, ret_TCP_speed);

  // Step 5 : Plan for transition between each pair of sequential path
  transitionPlanning(plans, moveit_model, planning_scene_diff_client, move_group_name,
                     start_state, planning_scenes);

  // Step 7 : fill in trajectory's time headers and pack into sub_process_plans
  // for each unit_process (process id is added here too)
  adjustTrajectoryTiming(plans, joint_names);

  // Step 8: fill in TCP pose according to trajectories
  appendTCPPosetoPlans(model, planning_scenes, plans);

  ROS_INFO("[Process Planning] trajectories solved and packing finished");
  return true;
}