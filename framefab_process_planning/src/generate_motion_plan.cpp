//
// Created by yijiangh on 7/8/17.
//
#include <ros/console.h>

#include <Eigen/Geometry>

#include "generate_motion_plan.h"
#include "trajectory_utils.h"
#include "common_utils.h"
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

#include <descartes_planner/ladder_graph_dag_search.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/graph_builder.h>
#include <moveit/planning_scene/planning_scene.h>


#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <swri_profiler/profiler.h>

// msg
#include <geometry_msgs/Pose.h>

#include <eigen_conversions/eigen_msg.h>

const static bool validateTrajectory(const trajectory_msgs::JointTrajectory& pts,
                                     const descartes_core::RobotModel& model,
                                     const double min_segment_size)
{
  for (std::size_t i = 1; i < pts.points.size(); ++i)
  {
    const auto& pt_a = pts.points[i - 1].positions;
    const auto& pt_b = pts.points[i].positions;

    auto interpolate = framefab_process_planning::interpolateJoint(pt_a, pt_b, min_segment_size);

    // The thought here is that the graph building process already checks the waypoints in the
    // trajectory for collisions. What we want to do is check between these waypoints when they
    // move a lot. 'interpolateJoint()' returns a list of positions where the maximum joint motion
    // between them is no more 'than min_segment_size' and its inclusive. So if there are only two
    // solutions, then we just have the start & end which are already checked.
    if (interpolate.size() > 2)
    {
      for (std::size_t j = 1; j < interpolate.size() - 1; ++j)
      {
        if (!model.isValid(interpolate[j]))
        {
          return false;
        }
      }
    }
  }
  return true;
}

static bool generateUnitProcessMotionPlan(
    const int& id,
    const descartes_core::RobotModelPtr model,
    descartes_planner::ConstrainedSegment& seg,
    std::vector<double>& last_pose,
    moveit::core::RobotModelConstPtr moveit_model,
    const std::string& move_group_name,
    const std::vector<std::string>& joint_names,
    framefab_msgs::UnitProcessPlan& plan)
{
  using framefab_process_planning::DescartesTraj;
  using framefab_process_planning::toROSTrajectory;
  using framefab_process_planning::planFreeMove;
  using framefab_process_planning::fillTrajectoryHeaders;
  using framefab_process_planning::extractJoints;

  std::string profile_id = "unit-" + std::to_string(id);
  SWRI_PROFILE(profile_id);

  // build graph
  auto graph = descartes_planner::sampleConstrainedPaths(*model, seg);

  // Create a planning graph (it has a solve method - you could use the DagSearch class yourself if you wanted)
  descartes_planner::PlanningGraph plan_graph (model);
  plan_graph.setGraph(graph); // set the graph we built earlier (instead of calling insertGraph)

  const auto dof = graph.dof();

  // Now we perform the search using the starting costs from our estimation above
  descartes_planner::DAGSearch search(graph);
  double cost = search.run();
  if (cost == std::numeric_limits<double>::max())
  {
    ROS_ERROR("%s: Failed to search graph. All points have IK, but process constraints (e.g velocity) "
                  "prevent a solution", __FUNCTION__);
    return false;
  }

  // Here we search the graph for the shortest path and build a descartes trajectory of it!
  auto path_idxs = search.shortestPath();
  ROS_WARN("%s: Descartes computed path with cost %lf", __FUNCTION__, cost);
  DescartesTraj sol;
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto* data = graph.vertex(j, idx);
    const auto& tm = graph.getRung(j).timing;
    auto pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(
        std::vector<double>(data, data + dof), tm));
    sol.push_back(pt);
  }

  trajectory_msgs::JointTrajectory ros_traj = toROSTrajectory(sol, *model);

  // and get free plan for connect path
  trajectory_msgs::JointTrajectory connection =
      planFreeMove(*model, move_group_name, moveit_model,
                   last_pose,
                   extractJoints(*model, *sol.front()));

  const static double SMALLEST_VALID_SEGMENT = 0.05;
  if (!validateTrajectory(ros_traj, *model, SMALLEST_VALID_SEGMENT))
  {
    ROS_ERROR_STREAM("%s: Computed path contains joint configuration changes that would result in a collision.");
    return false;
  }

  // record process path size for later process identification
  int approach_size = seg.retract_start_pt_num;
  int process_size = seg.process_pt_num;
  int depart_size = seg.retract_end_pt_num;

  // fill in connection traj
  ROS_INFO_STREAM("connection traj size: " << connection.points.size());
  plan.trajectory_connection = connection;

  // fill in process traj from descartes computed result
  for(std::size_t j = 0; j < ros_traj.points.size(); j++)
  {
    if(0 <= j && j <= approach_size - 1)
    {
      plan.trajectory_approach.points.push_back(ros_traj.points[j]);
    }
    if(approach_size <= j && j <= approach_size + process_size - 1)
    {
      plan.trajectory_process.points.push_back(ros_traj.points[j]);
    }
    if(approach_size + process_size <= j && j <= approach_size + process_size + depart_size - 1)
    {
      plan.trajectory_depart.points.push_back(ros_traj.points[j]);
    }
  }

  // Fill in result header information
  fillTrajectoryHeaders(joint_names, plan.trajectory_connection);
  fillTrajectoryHeaders(joint_names, plan.trajectory_approach);
  fillTrajectoryHeaders(joint_names, plan.trajectory_process);
  fillTrajectoryHeaders(joint_names, plan.trajectory_depart);

  // update last pose (joint)
  last_pose = extractJoints(*model, *sol.back());

  return true;
}

#define SANITY_CHECK(cond) do { if ( !(cond) ) { throw std::runtime_error(#cond); } } while (false);

bool framefab_process_planning::generateMotionPlan(
    descartes_core::RobotModelPtr model,
    std::vector<descartes_planner::ConstrainedSegment>& segs,
    const std::vector<moveit_msgs::CollisionObject>& collision_objs,
    moveit::core::RobotModelConstPtr moveit_model,
    ros::ServiceClient& planning_scene_diff_client,
    const std::string& move_group_name,
    const std::vector<double>& start_state,
    std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  // Step 0: Sanity checks
  if (segs.size() == 0)
  {
    return false;
  }

  const std::vector<std::string>& joint_names =
      moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

  // Step 1: Let's create all of our planning scenes to collision check against
  std::vector<planning_scene::PlanningScenePtr> planning_scenes;
  planning_scenes.reserve(segs.size());

  planning_scene::PlanningScenePtr root_scene (new planning_scene::PlanningScene(moveit_model));
  root_scene->getCurrentStateNonConst().setToDefaultValues();
  root_scene->getCurrentStateNonConst().update();
  planning_scenes.push_back(root_scene);

  for (std::size_t i = 0; i < collision_objs.size() - 1; ++i) // We use all but the last collision object
  {
    auto last_scene = planning_scenes.back();
    auto child = last_scene->diff();
    // update collision objects (built model elements)
    addCollisionObject(planning_scene_diff_client, collision_objs[i]);

    if (!child->processCollisionObjectMsg(collision_objs[i]))
    {
      ROS_WARN("Failed to process collision object");
    }
    planning_scenes.push_back(child);
  }

//  SANITY_CHECK(planning_scenes.size() == segs.size());


  // Next, we want to sample the graph for each segment
  auto graph_build_start = ros::Time::now();
  std::vector<descartes_planner::LadderGraph> graphs;
  graphs.reserve(segs.size());

  for (std::size_t i = 0; i < segs.size(); ++i)
  {
    model->setPlanningScene(planning_scenes[i]);
    if (true)
    {
      descartes_planner::ConstrainedSegment& seg = segs[i];
      ROS_INFO_STREAM("seg " << i << ": " << seg.orientations.size());
    }
    graphs.push_back(descartes_planner::sampleConstrainedPaths(*model, segs[i]));
  }
  auto graph_build_end = ros::Time::now();

  ROS_INFO_STREAM("Graph building took: " << (graph_build_end - graph_build_start).toSec() << " seconds");

  // Next let's construct a big graph to search through
  const auto append_start = ros::Time::now();
  descartes_planner::LadderGraph final_graph (model->getDOF());

  for (const auto graph : graphs)
  {
    descartes_planner::appendInTime(final_graph, graph);
  }
  const auto append_end = ros::Time::now();
  ROS_INFO_STREAM("Graph appending took: " << (append_end - append_start).toSec() << " seconds");

  // Next, we build a search for the whole problem
  const auto search_start = ros::Time::now();
  descartes_planner::DAGSearch search (final_graph);
  double cost = search.run();
  const auto search_end = ros::Time::now();
  ROS_INFO_STREAM("Search took " << (search_end-search_start).toSec() << " seconds and produced a result with dist = " << cost);

  // Now we have a rough solution for the entire process...


  // What?
  auto path_idxs = search.shortestPath();
  DescartesTraj sol;
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto* data = final_graph.vertex(j, idx);
    const auto& tm = final_graph.getRung(j).timing;
    auto pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(
        std::vector<double>(data, data + 6), tm));
    sol.push_back(pt);
  }

  trajectory_msgs::JointTrajectory ros_traj = toROSTrajectory(sol, *model);
  for (auto& pt : ros_traj.points) pt.time_from_start *= 3.0;
  fillTrajectoryHeaders(joint_names, ros_traj);

  ros::NodeHandle nh;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client (nh, "joint_trajectory_action");
  if (!client.waitForServer(ros::Duration(1.0)))
  {
    ROS_WARN("Exec timed out");
  }
  else
  {
    ROS_INFO("Found action");
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = ros_traj;

  client.sendGoal(goal);



//  SWRI_PROFILE("generate-Motion-Plan");

//  plans.resize(segs.size());
//  std::vector<double> last_pose = start_state;
////

//  for(std::size_t i = 0; i < segs.size(); i++)
//  {
//    model->updateInternals();

//    ROS_INFO_STREAM("Process Planning #" << i);

//    if(!generateUnitProcessMotionPlan(i, model, segs[i], last_pose,
//                                  moveit_model, move_group_name, joint_names,
//                                  plans[i]))
//    {
//      return false;
//    }

//  }

  return true;
}
