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
#include <framefab_msgs/WireFrameCollisionObject.h>
#include <descartes_msgs/LadderGraphList.h>

// msg
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/PlanningScene.h>

// srv
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

// serialization
#include <framefab_param_helpers/framefab_param_helpers.h>

const static std::string GET_PLANNING_SCENE_SERVICE = "get_planning_scene";

const static int TRANSITION_PLANNING_LOOP_COUNT = 5;

namespace{ //util function namespace

void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                             const std::vector<framefab_msgs::WireFrameCollisionObject>& wf_collision_objs,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_approach,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_depart,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_full)
{
  const auto build_scenes_start = ros::Time::now();

  planning_scenes_shrinked_approach.reserve(wf_collision_objs.size());
  planning_scenes_shrinked_depart.reserve(wf_collision_objs.size());
  planning_scenes_full.reserve(wf_collision_objs.size());

  ros::NodeHandle nh;
  auto planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);

  if(!planning_scene_client.waitForExistence())
  {
    ROS_ERROR_STREAM("[Process Planning] cannot connect with get planning scene server...");
  }

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      moveit_msgs::PlanningSceneComponents::ROBOT_STATE
          | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  if(!planning_scene_client.call(srv))
  {
    ROS_ERROR_STREAM("[Process Planning] Failed to fetch planning scene srv!");
  }

  planning_scene::PlanningScenePtr root_scene(new planning_scene::PlanningScene(moveit_model));
  root_scene->getCurrentStateNonConst().setToDefaultValues();
  root_scene->getCurrentStateNonConst().update();
  root_scene->setPlanningSceneDiffMsg(srv.response.scene);

  // start with no element constructed in the scene
  planning_scenes_shrinked_approach.push_back(root_scene);
  planning_scenes_shrinked_depart.push_back(root_scene);
  planning_scenes_full.push_back(root_scene);

  for (std::size_t i = 1; i < wf_collision_objs.size(); ++i) // We use all but the last collision object
  {
    auto last_scene_shrinked = planning_scenes_shrinked_approach.back();
    auto child_shrinked = last_scene_shrinked->diff();

    auto c_list = wf_collision_objs[i].recovered_last_neighbor_objs;
    c_list.push_back(wf_collision_objs[i].last_full_obj);

    const auto& shrinked_neighbor_objs = wf_collision_objs[i].shrinked_neighbor_objs;
    c_list.insert(c_list.begin(), shrinked_neighbor_objs.begin(), shrinked_neighbor_objs.end());

    for(const auto& c_obj : c_list)
    {
      if (!child_shrinked->processCollisionObjectMsg(c_obj))
      {
        ROS_WARN("[Process Planning] Failed to process shrinked collision object");
      }
    }

    auto child_shrinked_depart = child_shrinked->diff();
    if (!child_shrinked_depart->processCollisionObjectMsg(wf_collision_objs[i].both_side_shrinked_obj))
    {
      ROS_WARN("[Process Planning] Failed to process shrinked collision object");
    }

    // push in partial_collision_geometry_planning_scene
    planning_scenes_shrinked_approach.push_back(child_shrinked);
    planning_scenes_shrinked_depart.push_back(child_shrinked_depart);

    // get diff as child
    // restore changed element back to full geometry
    // push in full_collision_geometry_planning_scene
    auto last_scene_full = planning_scenes_full.back();
    auto child_full = last_scene_full->diff();

    // TODO: temp fix
    auto inflated_full_obj = wf_collision_objs[i-1].full_obj;
//    inflated_full_obj.primitives[0].dimensions[1] += 0.0;

    if (!child_full->processCollisionObjectMsg(inflated_full_obj))
    {
      ROS_WARN("[Process Planning] Failed to process full collision object");
    }

    // push in full_scene[i]
    planning_scenes_full.push_back(child_full);
  }

  const auto build_scenes_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] constructing planning scenes took " << (build_scenes_end-build_scenes_start).toSec()
                                                                          << " seconds.");
}

bool saveLadderGraph(const std::string& filename, const descartes_msgs::LadderGraphList& graph_list_msg)
{
  if (!framefab_param_helpers::toFile(filename, graph_list_msg))
  {
    ROS_WARN_STREAM("Unable to save ladder graph list to: " << filename);
  }
}

void CLTRRTforProcessROSTraj(descartes_core::RobotModelPtr model,
                             std::vector<descartes_planner::ConstrainedSegment>& segs,
                             const double clt_rrt_unit_process_timeout,
                             const double clt_rrt_timeout,
                             const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_approach,
                             const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_depart,
                             const std::vector<framefab_msgs::ElementCandidatePoses>& task_seq,
                             std::vector<framefab_msgs::UnitProcessPlan>& plans,
                             const std::string& saved_graph_file_name,
                             bool use_saved_graph)
{
  // sanity check
  assert(segs.size() == task_seq.size());

  const auto clt_start = ros::Time::now();

  framefab_process_planning::DescartesTraj sol;

  std::vector<descartes_planner::LadderGraph> graphs;
  std::vector<int> graph_indices;
  descartes_msgs::LadderGraphList graph_list_msg;
  double clt_cost = 0;

  if(use_saved_graph)
  {
    if(framefab_param_helpers::fromFile(saved_graph_file_name, graph_list_msg))
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

  if(!use_saved_graph || segs.size() > graphs.size())
  {
    // reconstruct and search, output sol, graphs, graph_indices
    clt_cost = CLT_RRT.solve(*model, clt_rrt_unit_process_timeout, clt_rrt_timeout);
    CLT_RRT.extractSolution(*model, sol,
                            graphs,
                            graph_indices, use_saved_graph);

    graph_list_msg = descartes_parser::convertToLadderGraphMsg(graphs);
    saveLadderGraph(saved_graph_file_name, graph_list_msg);
  }
  else
  {
    // default start from begin
    std::vector<descartes_planner::LadderGraph> chosen_graphs(graphs.begin(), graphs.begin() + segs.size());
    CLT_RRT.extractSolution(*model, sol,
                            chosen_graphs,
                            graph_indices, use_saved_graph);
  }

  const auto clt_end = ros::Time::now();
  ROS_INFO_STREAM("[CLT RRT*] CLT RRT* Search took " << (clt_end-clt_start).toSec()
                                                     << " seconds");

  trajectory_msgs::JointTrajectory ros_traj = framefab_process_planning::toROSTrajectory(sol, *model);

  auto it = ros_traj.points.begin();
  for(size_t i = 0; i < task_seq.size(); i++)
  {
    framefab_msgs::SubProcess sub_process;

    sub_process.unit_process_id = i;
    sub_process.process_type = framefab_msgs::SubProcess::PROCESS;
    sub_process.main_data_type = framefab_msgs::SubProcess::CART;

    switch(task_seq[i].type)
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

void retractionPlanning(descartes_core::RobotModelPtr model,
                        const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_approach,
                        const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_depart,
                        const std::vector<descartes_planner::ConstrainedSegment>& segs,
                        std::vector<framefab_msgs::UnitProcessPlan>& plans)
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
//      if(last_process_end_joint == start_process_joint)
//      {
//        // skip retraction planning
//        ROS_INFO_STREAM("[retraction Planning] process #" << i << "retraction planning skipped.");
//        continue;
//      }
    }

    model->setPlanningScene(planning_scenes_approach[i]);

    std::vector<std::vector<double>> approach_retract_traj;
    if(!framefab_process_planning::retractPath(start_process_joint, segs[i].retract_dist, segs[i].linear_vel,
                                               segs[i].orientations,
                                               model, approach_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible approach retract motion!");
    }
    else
    {

      std::reverse(approach_retract_traj.begin(), approach_retract_traj.end());

      trajectory_msgs::JointTrajectory approach_ros_traj =
          framefab_process_planning::toROSTrajectory(approach_retract_traj, *model);

      framefab_msgs::SubProcess sub_process_approach;

      sub_process_approach.process_type = framefab_msgs::SubProcess::RETRACTION;
      sub_process_approach.main_data_type = framefab_msgs::SubProcess::CART;
      sub_process_approach.element_process_type = framefab_msgs::SubProcess::APPROACH;
      sub_process_approach.joint_array = approach_ros_traj;

      plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process_approach);
    }

    model->setPlanningScene(planning_scenes_depart[i]);

    std::vector<std::vector<double>> depart_retract_traj;
    if(!framefab_process_planning::retractPath(end_process_joint, segs[i].retract_dist, segs[i].linear_vel,
                                               segs[i].orientations,
                                               model, depart_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible depart retract motion!");
    }
    else
    {
      trajectory_msgs::JointTrajectory depart_ros_traj =
          framefab_process_planning::toROSTrajectory(depart_retract_traj, *model);

      framefab_msgs::SubProcess sub_process_depart;

      sub_process_depart.process_type = framefab_msgs::SubProcess::RETRACTION;
      sub_process_depart.main_data_type = framefab_msgs::SubProcess::CART;
      sub_process_depart.element_process_type = framefab_msgs::SubProcess::DEPART;
      sub_process_depart.joint_array = depart_ros_traj;

      plans[i].sub_process_array.insert(plans[i].sub_process_array.end(), sub_process_depart);
    }
  } // loop for all unit plans

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

  // generate full eef collision object
  bool add_eef_full = true;
  auto full_eef_collision_obj = framefab_process_planning::addFullEndEffectorCollisionObject(add_eef_full);

  std::vector<int> planning_failure_ids;

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
    auto scene_with_attached_eef = planning_scenes[i]->diff();
    if(!scene_with_attached_eef->processAttachedCollisionObjectMsg(full_eef_collision_obj))
    {
      ROS_ERROR_STREAM("[Tr Planning] planning scene # " << i << "fails to add attached full eef collision geometry");
    }

    scene_with_attached_eef->getPlanningSceneMsg(srv.request.scene);

    if(!planning_scene_diff_client.call(srv))
    {
      ROS_ERROR_STREAM("[Tr Planning] Failed to publish planning scene diff srv!");
    }

    int repeat_planning_call = 0;
    trajectory_msgs::JointTrajectory ros_trans_traj;

    bool joint_target_meet = true;
    while(repeat_planning_call < TRANSITION_PLANNING_LOOP_COUNT)
    {
      // reset joint target meet flag
      joint_target_meet = true;

      ros_trans_traj = framefab_process_planning::getMoveitTransitionPlan(move_group_name,
                                                                          last_joint_pose,
                                                                          current_first_joint_pose,
                                                                          start_state,
                                                                          moveit_model);

      // TODO: recover from transition planning failure
      if(repeat_planning_call > 0)
      {
        ROS_WARN_STREAM("[Process Planning] transition planning retry - round "
                            << repeat_planning_call << "/" << TRANSITION_PLANNING_LOOP_COUNT);
      }

      if (0 == ros_trans_traj.points.size())
      {
//        ROS_ERROR_STREAM("<<<<<<<<<<<<<<< \n[Process Planning] Transition planning fails.");
        joint_target_meet = false;
        repeat_planning_call++;
        continue;
      }

      for(int s=0; s < current_first_joint_pose.size(); s++)
      {
        if(current_first_joint_pose[s] - ros_trans_traj.points.back().positions[s] > 0.0001)
        {
          joint_target_meet = false;
          break;
        }
      }

      if(joint_target_meet)
      {
        ROS_WARN_STREAM("[Process Planning] transition planning retry succeed!");
        break;
      }

      repeat_planning_call++;
    }

    if(!joint_target_meet)
    {
      planning_failure_ids.push_back(i);
      ROS_ERROR_STREAM("[Tr planning] transition planning fails at index #" << i);
      continue;
    }

    framefab_msgs::SubProcess sub_process;

    sub_process.process_type = framefab_msgs::SubProcess::TRANSITION;
    sub_process.main_data_type = framefab_msgs::SubProcess::JOINT;
    sub_process.joint_array = ros_trans_traj;

    plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process);
  }

  for(auto id : planning_failure_ids)
  {
    ROS_ERROR_STREAM("[Tr planning] transition planning fails at process #" << id);
  }

  const auto tr_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Transition Planning took " << (tr_planning_end-tr_planning_start).toSec()
                                                                 << " seconds.");
}

void adjustTrajectoryTiming(std::vector<framefab_msgs::UnitProcessPlan>& plans,
                            const std::vector<std::string> &joint_names,
                            const std::string world_frame)
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

  framefab_process_planning::fillTrajectoryHeaders(joint_names, plans[0].sub_process_array[0].joint_array, world_frame);
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
        sim_speed = 1.0;
      }

      adjustTrajectoryHeaders(last_filled_jts, plans[i].sub_process_array[j], sim_speed);
    }

//    ROS_INFO_STREAM("[Process Planning] process #" << i << " time stamp adjusted.");
  }
}

void appendTCPPosetoPlans(const descartes_core::RobotModelPtr model,
                          const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked,
                          const std::vector<planning_scene::PlanningScenePtr>& planning_scenes_full,
                          std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  int process_id_count = 0;
  for(auto& unit_plan : plans)
  {
    for(auto& sub_process : unit_plan.sub_process_array)
    {
      if(sub_process.process_type == framefab_msgs::SubProcess::TRANSITION)
      {
        model->setPlanningScene(planning_scenes_full[process_id_count]);
      }
      else
      {
        model->setPlanningScene(planning_scenes_shrinked[process_id_count]);
      }

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
//    ROS_INFO_STREAM("[Process Planning] process #" << process_id_count << "TCP added.");
    process_id_count++;
  }
}

} // end util namespace

bool framefab_process_planning::generateMotionPlan(
    descartes_core::RobotModelPtr model,
    std::vector<descartes_planner::ConstrainedSegment>& segs,
    const std::vector<framefab_msgs::ElementCandidatePoses>& task_seq,
    const std::vector<framefab_msgs::WireFrameCollisionObject>& wf_collision_objs,
    const std::string world_frame,
    const bool use_saved_graph,
    const std::string& saved_graph_file_name,
    const double clt_rrt_unit_process_timeout,
    const double clt_rrt_timeout,
    moveit::core::RobotModelConstPtr moveit_model,
    ros::ServiceClient& planning_scene_diff_client,
    const std::string& move_group_name,
    const std::vector<double>& start_state,
    std::vector<framefab_msgs::UnitProcessPlan>& plans)
{
  // Step 0: Sanity checks
  if (segs.size() == 0 || task_seq.size() != segs.size())
  {
    ROS_ERROR_STREAM("[Process Planning] input descartes Constrained Segment size" << segs.size());
    return false;
  }
  assert(task_seq.size() == wf_collision_objs.size());

  plans.resize(segs.size());
  const std::vector<std::string> &joint_names =
      moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

  // Step 1: Let's create all of our planning scenes to collision check against
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_shrinked_approach;
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_shrinked_depart;
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_full;
  constructPlanningScenes(moveit_model, wf_collision_objs,
                          planning_scenes_shrinked_approach,
                          planning_scenes_shrinked_depart,
                          planning_scenes_full);

  // Step 2: CLT RRT* to solve process trajectory
  CLTRRTforProcessROSTraj(model, segs, clt_rrt_unit_process_timeout, clt_rrt_timeout,
                          planning_scenes_shrinked_approach, planning_scenes_shrinked_depart,
                          task_seq, plans, saved_graph_file_name, use_saved_graph);

  // retract planning
  retractionPlanning(model, planning_scenes_shrinked_approach, planning_scenes_shrinked_depart, segs, plans);

  // Step 5 : Plan for transition between each pair of sequential path
  transitionPlanning(plans, moveit_model, planning_scene_diff_client, move_group_name,
                     start_state, planning_scenes_full);

  // Step 7 : fill in trajectory's time headers and pack into sub_process_plans
  // for each unit_process (process id is added here too)
  adjustTrajectoryTiming(plans, joint_names, world_frame);

  // Step 8: fill in TCP pose according to trajectories
  appendTCPPosetoPlans(model, planning_scenes_shrinked_approach, planning_scenes_full, plans);

  ROS_INFO("[Process Planning] trajectories solved and packing finished");
  return true;
}