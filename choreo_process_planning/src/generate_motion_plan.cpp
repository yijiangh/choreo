//
// Created by yijiangh on 7/8/17.
//

#include "generate_motion_plan.h"
#include "trajectory_utils.h"
#include "path_transitions.h"
#include "common_utils.h"

#include "construct_planning_scene.h"
#include "semi_constrained_cartesian_planning.h"

// pose conversion
#include <eigen_conversions/eigen_msg.h>

// msg
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <choreo_msgs/SubProcess.h>
#include <choreo_msgs/ElementCandidatePoses.h>
#include <choreo_msgs/WireFrameCollisionObject.h>
#include <choreo_msgs/AssemblySequencePickNPlace.h>
#include <choreo_msgs/SequencedElement.h>

#include <shape_msgs/Mesh.h>
#include <moveit_msgs/PlanningScene.h>

// srv
#include <moveit_msgs/ApplyPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/console.h>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>

const static int TRANSITION_PLANNING_LOOP_COUNT = 5;

namespace choreo_process_planning
{
void retractionPlanning(descartes_core::RobotModelPtr model,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_approach,
                        const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_depart,
                        const std::vector <descartes_planner::ConstrainedSegment> &segs,
                        std::vector<choreo_msgs::UnitProcessPlan> &plans)
{
  if (plans.size() == 0)
  {
    ROS_ERROR("[retraction Planning] plans size = 0!");
    assert(false);
  }

  const auto ret_planning_start = ros::Time::now();

  for (size_t i = 0; i < plans.size(); i++)
  {
    ROS_INFO_STREAM("[retraction Planning] process #" << i);

    const std::vector<double>
        start_process_joint = plans[i].sub_process_array.back().joint_array.points.front().positions;
    const std::vector<double> end_process_joint = plans[i].sub_process_array.back().joint_array.points.back().positions;

    if (0 != i)
    {
      const auto last_process_end_joint = plans[i - 1].sub_process_array.back().joint_array.points.back().positions;

      // TODO: user option if start process pt = end process pt, skip retraction planning
//      if(last_process_end_joint == start_process_joint)
//      {
//        // skip retraction planning
//        ROS_INFO_STREAM("[retraction Planning] process #" << i << "retraction planning skipped.");
//        continue;
//      }
    }

    model->setPlanningScene(planning_scenes_approach[i]);

    std::vector <std::vector<double>> approach_retract_traj;
    if (!choreo_process_planning::retractPath(
        start_process_joint, segs[i].retract_dist, segs[i].retract_vel,
        segs[i].retract_disc,
        segs[i].orientations,
        model, approach_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible approach retract motion!");
    }
    else
    {

      std::reverse(approach_retract_traj.begin(), approach_retract_traj.end());

      trajectory_msgs::JointTrajectory approach_ros_traj =
          choreo_process_planning::toROSTrajectory(approach_retract_traj, *model);

      choreo_msgs::SubProcess sub_process_approach;

      sub_process_approach.process_type = choreo_msgs::SubProcess::RETRACTION;
      sub_process_approach.main_data_type = choreo_msgs::SubProcess::CART;
      sub_process_approach.element_process_type = choreo_msgs::SubProcess::APPROACH;
      sub_process_approach.joint_array = approach_ros_traj;

      plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process_approach);
    }

    model->setPlanningScene(planning_scenes_depart[i]);

    std::vector <std::vector<double>> depart_retract_traj;
    if (!choreo_process_planning::retractPath(
        end_process_joint, segs[i].retract_dist, segs[i].retract_vel,
        segs[i].retract_disc,
        segs[i].orientations,
        model, depart_retract_traj))
    {
      ROS_ERROR_STREAM("[retraction planning] process #" << i << " failed to find feasible depart retract motion!");
    }
    else
    {
      trajectory_msgs::JointTrajectory depart_ros_traj =
          choreo_process_planning::toROSTrajectory(depart_retract_traj, *model);

      choreo_msgs::SubProcess sub_process_depart;

      sub_process_depart.process_type = choreo_msgs::SubProcess::RETRACTION;
      sub_process_depart.main_data_type = choreo_msgs::SubProcess::CART;
      sub_process_depart.element_process_type = choreo_msgs::SubProcess::DEPART;
      sub_process_depart.joint_array = depart_ros_traj;

      plans[i].sub_process_array.insert(plans[i].sub_process_array.end(), sub_process_depart);
    }
  } // loop for all unit plans

  const auto ret_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[retraction planning] Retraction Planning took " << (ret_planning_end - ret_planning_start).toSec()
                                                                    << " seconds.");
}

void transitionPlanning(std::vector <choreo_msgs::UnitProcessPlan> &plans,
                        moveit::core::RobotModelConstPtr moveit_model,
                        ros::ServiceClient &planning_scene_diff_client,
                        const std::string &move_group_name,
                        const std::vector<double> &start_state,
                        std::vector <planning_scene::PlanningScenePtr> &planning_scenes)
{
  if (plans.size() == 0)
  {
    ROS_ERROR("[transionPlanning] plans size = 0!");
    assert(false);
  }

  const auto tr_planning_start = ros::Time::now();

  // generate full eef collision object
  bool add_eef_full = true;
  auto full_eef_collision_obj = choreo_process_planning::addFullEndEffectorCollisionObject(add_eef_full);

  std::vector<int> planning_failure_ids;

  std::vector<double> last_joint_pose = start_state;
  std::vector<double> current_first_joint_pose;

  for (size_t i = 0; i < plans.size(); i++)
  {
    ROS_INFO_STREAM("[Transition Planning] process #" << i);

    if (0 != i)
    {
      last_joint_pose = plans[i - 1].sub_process_array.back().joint_array.points.back().positions;
    }

    current_first_joint_pose = plans[i].sub_process_array.front().joint_array.points.front().positions;

    if (last_joint_pose == current_first_joint_pose)
    {
      // skip transition planning
      continue;
    }

    // update the planning scene
    if (!planning_scene_diff_client.waitForExistence())
    {
      ROS_ERROR_STREAM("[Tr Planning] cannot connect with planning scene diff server...");
    }

    moveit_msgs::ApplyPlanningScene srv;
    auto scene_with_attached_eef = planning_scenes[i]->diff();
    if (!scene_with_attached_eef->processAttachedCollisionObjectMsg(full_eef_collision_obj))
    {
      ROS_ERROR_STREAM("[Tr Planning] planning scene # " << i << "fails to add attached full eef collision geometry");
    }

    scene_with_attached_eef->getPlanningSceneMsg(srv.request.scene);

    if (!planning_scene_diff_client.call(srv))
    {
      ROS_ERROR_STREAM("[Tr Planning] Failed to publish planning scene diff srv!");
    }

    int repeat_planning_call = 0;
    trajectory_msgs::JointTrajectory ros_trans_traj;

    bool joint_target_meet = true;
    while (repeat_planning_call < TRANSITION_PLANNING_LOOP_COUNT)
    {
      // reset joint target meet flag
      joint_target_meet = true;

      ros_trans_traj = choreo_process_planning::getMoveitTransitionPlan(move_group_name,
          last_joint_pose,
          current_first_joint_pose,
          start_state,
          moveit_model);

      // TODO: recover from transition planning failure
      if (repeat_planning_call > 0)
      {
        ROS_WARN_STREAM("[Process Planning] transition planning retry - round "
            << repeat_planning_call << "/" << TRANSITION_PLANNING_LOOP_COUNT);
      }

      if (0 == ros_trans_traj.points.size())
      {
//        ROS_ERROR_STREAM("[Process Planning] Transition planning fails.");
        joint_target_meet = false;
        repeat_planning_call++;
        continue;
      }

      for (int s = 0; s < current_first_joint_pose.size(); s++)
      {
        if (current_first_joint_pose[s] - ros_trans_traj.points.back().positions[s] > 0.0001)
        {
          joint_target_meet = false;
          break;
        }
      }

      if (joint_target_meet)
      {
        std::string retry_msg = repeat_planning_call>0 ? "retry":"";
        ROS_WARN_STREAM("[Process Planning] transition planning "
            << retry_msg << " succeed!");
        break;
      }

      repeat_planning_call++;
    }

    if (!joint_target_meet)
    {
      planning_failure_ids.push_back(i);
      ROS_ERROR_STREAM("[Tr planning] transition planning fails at index #" << i);
      continue;
    }

    choreo_msgs::SubProcess sub_process;

    sub_process.process_type = choreo_msgs::SubProcess::TRANSITION;
    sub_process.main_data_type = choreo_msgs::SubProcess::JOINT;
    sub_process.joint_array = ros_trans_traj;

    plans[i].sub_process_array.insert(plans[i].sub_process_array.begin(), sub_process);
  }

  for (auto id : planning_failure_ids)
  {
    ROS_ERROR_STREAM("[Tr planning] transition planning fails at process #" << id);
  }

  const auto tr_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Transition Planning took " << (tr_planning_end - tr_planning_start).toSec()
                                                                 << " seconds.");
}

void transitionPlanningPickNPlace(std::vector <choreo_msgs::UnitProcessPlan> &plans,
                                  moveit::core::RobotModelConstPtr moveit_model,
                                  ros::ServiceClient &planning_scene_diff_client,
                                  const std::string &move_group_name,
                                  const std::vector<double> &start_state,
                                  const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes)
{
  if (plans.size() == 0)
  {
    ROS_ERROR("[transionPlanning] plans size = 0!");
    assert(false);
  }

  const auto tr_planning_start = ros::Time::now();

//  // generate full eef collision object
//  bool add_eef_full = true;
//  auto full_eef_collision_obj = choreo_process_planning::addFullEndEffectorCollisionObject(add_eef_full);

  std::vector<int> planning_failure_ids;

  std::vector<double> last_joint_pose = start_state;
  std::vector<double> current_first_joint_pose;

  for (size_t i = 0; i < plans.size(); i++)
  {
    ROS_INFO_STREAM("[Transition Planning] process #" << i);

    // transition planning scenes' number
    // <transition> - <sub-process> - <transition> - <sub-process> -... - <sub-process>
    assert(planning_scenes[i].size() == plans[i].sub_process_array.size());
    std::vector<choreo_msgs::SubProcess> weaved_sub_process;

    for (size_t j = 0; j < plans[i].sub_process_array.size(); j++)
    {
      if(0 == j)
      {
        // last joint pose from last unit process's last subprocess
        if (0 != i)
        {
          last_joint_pose = plans[i - 1].sub_process_array.back().joint_array.points.back().positions;
        }
      }
      else
      {
        last_joint_pose = plans[i].sub_process_array[j-1].joint_array.points.back().positions;
      }

      current_first_joint_pose = plans[i].sub_process_array[j].joint_array.points.front().positions;

      if (last_joint_pose != current_first_joint_pose)
      {
        // update the planning scene
        if (!planning_scene_diff_client.waitForExistence())
        {
          ROS_ERROR_STREAM("[Tr Planning] cannot connect with planning scene diff server...");
        }

        moveit_msgs::ApplyPlanningScene srv;
        auto scene = planning_scenes[i][j]->diff();

//      if (!scene_with_attached_eef->processAttachedCollisionObjectMsg(full_eef_collision_obj))
//      {
//        ROS_ERROR_STREAM("[Tr Planning] planning scene # " << i << "fails to add attached full eef collision geometry");
//      }

        scene->getPlanningSceneMsg(srv.request.scene);

        if (!planning_scene_diff_client.call(srv))
        {
          ROS_ERROR_STREAM("[Tr Planning] Failed to publish planning scene diff srv!");
        }

        int repeat_planning_call = 0;
        trajectory_msgs::JointTrajectory ros_trans_traj;

        bool joint_target_meet = true;
        while (repeat_planning_call < TRANSITION_PLANNING_LOOP_COUNT)
        {
          // reset joint target meet flag
          joint_target_meet = true;

          ros_trans_traj = choreo_process_planning::getMoveitTransitionPlan(move_group_name,
              last_joint_pose,
              current_first_joint_pose,
              start_state,
              moveit_model);

          // TODO: recover from transition planning failure
          if (repeat_planning_call > 0)
          {
            ROS_WARN_STREAM("[Process Planning] transition planning retry - round "
                << repeat_planning_call << "/" << TRANSITION_PLANNING_LOOP_COUNT);
          }

          if (0 == ros_trans_traj.points.size())
          {
//        ROS_ERROR_STREAM("[Process Planning] Transition planning fails.");
            joint_target_meet = false;
            repeat_planning_call++;
            continue;
          }

          for (int s = 0; s < current_first_joint_pose.size(); s++)
          {
            if (current_first_joint_pose[s] - ros_trans_traj.points.back().positions[s] > 0.0001)
            {
              joint_target_meet = false;
              break;
            }
          }

          if (joint_target_meet)
          {
            std::string retry_msg = repeat_planning_call > 0 ? "retry" : "";
            ROS_WARN_STREAM("[Process Planning] transition planning "
                << retry_msg << " succeed!");
            break;
          }

          repeat_planning_call++;
        } // end while

        if (!joint_target_meet)
        {
          planning_failure_ids.push_back(i);
          ROS_ERROR_STREAM("[Tr planning] transition planning fails at index #" << i << ", subprocess #" << j);
          continue;
        }

        choreo_msgs::SubProcess sub_process;

        sub_process.process_type = choreo_msgs::SubProcess::TRANSITION;
        sub_process.main_data_type = choreo_msgs::SubProcess::JOINT;
        sub_process.joint_array = ros_trans_traj;

        weaved_sub_process.push_back(sub_process);
      }

      weaved_sub_process.push_back(plans[i].sub_process_array[j]);

    } // end subprocess

    plans[i].sub_process_array = weaved_sub_process;
  }// end process

  for (auto id : planning_failure_ids)
  {
    ROS_ERROR_STREAM("[Tr planning] transition planning fails at process #" << id);
  }

  const auto tr_planning_end = ros::Time::now();
  ROS_INFO_STREAM("[Process Planning] Transition Planning took " << (tr_planning_end - tr_planning_start).toSec()
                                                                 << " seconds.");
}

void adjustTrajectoryTiming(std::vector <choreo_msgs::UnitProcessPlan> &plans,
                            const std::vector <std::string> &joint_names,
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

  choreo_process_planning::fillTrajectoryHeaders(joint_names, plans[0].sub_process_array[0].joint_array, world_frame);
  auto last_filled_jts = plans[0].sub_process_array[0].joint_array;

  // inline function for append trajectory headers (adjust time frame)
  auto adjustTrajectoryHeaders =
      [](trajectory_msgs::JointTrajectory &last_filled_jts, choreo_msgs::SubProcess &sp, double sim_speed)
      {
        choreo_process_planning::appendTrajectoryHeaders(last_filled_jts, sp.joint_array, sim_speed);
        last_filled_jts = sp.joint_array;
      };

  for (size_t i = 0; i < plans.size(); i++)
  {
    for (size_t j = 0; j < plans[i].sub_process_array.size(); j++)
    {
      plans[i].sub_process_array[j].unit_process_id = i;
      plans[i].sub_process_array[j].sub_process_id = j;

      double sim_speed = 1.0;
      if (choreo_msgs::SubProcess::PROCESS == plans[i].sub_process_array[j].process_type)
      {
        sim_speed = 1.0;
      }
      if (choreo_msgs::SubProcess::RETRACTION == plans[i].sub_process_array[j].process_type)
      {
        sim_speed = 4;
      }
      if (choreo_msgs::SubProcess::TRANSITION == plans[i].sub_process_array[j].process_type)
      {
        sim_speed = 0.9;
      }

      adjustTrajectoryHeaders(last_filled_jts, plans[i].sub_process_array[j], sim_speed);
    }

//    ROS_INFO_STREAM("[Process Planning] process #" << i << " time stamp adjusted.");
  }
}

void appendTCPPoseToPlans(const descartes_core::RobotModelPtr model,
                          const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked,
                          const std::vector <planning_scene::PlanningScenePtr> &planning_scenes_full,
                          std::vector <choreo_msgs::UnitProcessPlan> &plans)
{
  int process_id_count = 0;
  for (auto &unit_plan : plans)
  {
    for (auto &sub_process : unit_plan.sub_process_array)
    {
      if (sub_process.process_type == choreo_msgs::SubProcess::TRANSITION)
      {
        model->setPlanningScene(planning_scenes_full[process_id_count]);
      }
      else
      {
        model->setPlanningScene(planning_scenes_shrinked[process_id_count]);
      }

      for (const auto &jt_pt : sub_process.joint_array.points)
      {
        Eigen::Affine3d TCP_pose = Eigen::Affine3d::Identity();
        geometry_msgs::Pose geo_pose_msg;

        // convert it to TCP pose
        if (!model->getFK(jt_pt.positions, TCP_pose))
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

void appendTCPPoseToPlansPickNPlace(const descartes_core::RobotModelPtr model,
                                    const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_transition,
                                    const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_subprocess,
                                    std::vector<choreo_msgs::UnitProcessPlan>& plans)
{
  int process_id_count = 0;
  for (auto &unit_plan : plans)
  {
    int sp_count = 0;
    int tr_count = 0;

    for (auto &sub_process : unit_plan.sub_process_array)
    {
      if (sub_process.process_type == choreo_msgs::SubProcess::TRANSITION)
      {
        model->setPlanningScene(planning_scenes_transition[process_id_count][tr_count]);
        tr_count++;
      }
      else
      {
        model->setPlanningScene(planning_scenes_subprocess[process_id_count][sp_count]);
        sp_count++;
      }

      for (const auto &jt_pt : sub_process.joint_array.points)
      {
        Eigen::Affine3d TCP_pose = Eigen::Affine3d::Identity();
        geometry_msgs::Pose geo_pose_msg;

        // convert it to TCP pose
        if (!model->getFK(jt_pt.positions, TCP_pose))
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

// TODO: spatial extrusion overload
bool generateMotionPlan(
    const std::string world_frame,
    const bool use_saved_graph,
    const std::string &saved_graph_file_name,
    const double clt_rrt_unit_process_timeout,
    const double clt_rrt_timeout,
    const std::string &move_group_name,
    const std::vector<double> &start_state,
    const std::vector <choreo_msgs::ElementCandidatePoses> &task_seq,
    const std::vector <choreo_msgs::WireFrameCollisionObject> &wf_collision_objs,
    std::vector <descartes_planner::ConstrainedSegment> &segs,
    descartes_core::RobotModelPtr model,
    moveit::core::RobotModelConstPtr moveit_model,
    ros::ServiceClient &planning_scene_diff_client,
    std::vector <choreo_msgs::UnitProcessPlan> &plans)
{
  // Step 0: Sanity checks
  if (segs.size() == 0 || task_seq.size() != segs.size())
  {
    ROS_ERROR_STREAM("[Process Planning] input descartes Constrained Segment size" << segs.size());
    return false;
  }
  assert(task_seq.size() == wf_collision_objs.size());

  plans.resize(segs.size());
  const std::vector <std::string> &joint_names =
      moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

  // Step 1: Let's create all of our planning scenes to collision check against
  std::vector <planning_scene::PlanningScenePtr> planning_scenes_shrinked_approach;
  std::vector <planning_scene::PlanningScenePtr> planning_scenes_shrinked_depart;
  std::vector <planning_scene::PlanningScenePtr> planning_scenes_full;
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
  appendTCPPoseToPlans(model, planning_scenes_shrinked_approach, planning_scenes_full, plans);

  ROS_INFO("[Process Planning] trajectories solved and packing finished");
  return true;
}

namespace {

void testApplyPlanningScene(ros::ServiceClient& planning_scene_diff_client,
                            planning_scene::PlanningScenePtr& planning_scene)
{
  // update the planning scene
  if (!planning_scene_diff_client.waitForExistence())
  {
    ROS_ERROR_STREAM("[Tr Planning] cannot connect with planning scene diff server...");
  }

  moveit_msgs::ApplyPlanningScene srv;

  planning_scene->getPlanningSceneMsg(srv.request.scene);

  if (!planning_scene_diff_client.call(srv))
  {
    ROS_ERROR_STREAM("[Tr Planning] Failed to publish TEST planning scene diff srv!");
  }
}

}

// TODO: picknplace overload
bool generateMotionPlan(
    const std::string world_frame,
    const bool use_saved_graph,
    const std::string &saved_graph_file_name,
    const double clt_rrt_unit_process_timeout,
    const double clt_rrt_timeout,
    const double& linear_vel,
    const double& linear_disc,
    const std::string &move_group_name,
    const std::vector<double> &start_state,
    const choreo_msgs::AssemblySequencePickNPlace& as_pnp,
    descartes_core::RobotModelPtr model,
    moveit::core::RobotModelConstPtr moveit_model,
    ros::ServiceClient &planning_scene_diff_client,
    std::vector <choreo_msgs::UnitProcessPlan> &plans)
{
  plans.resize(as_pnp.sequenced_elements.size());
  const std::vector <std::string> &joint_names =
      moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

  // Step 1: Let's create all of our planning scenes to collision check against
  std::vector<std::vector<planning_scene::PlanningScenePtr>> planning_scenes_transition;
  std::vector<std::vector<planning_scene::PlanningScenePtr>> planning_scenes_subprocess;

  constructPlanningScenes(moveit_model,
      world_frame,
      as_pnp,
      planning_scenes_transition,
      planning_scenes_subprocess);

  // Step 2: CLT RRT* to solve process trajectory
  CLTRRTforProcessROSTraj(model,
      as_pnp,
      clt_rrt_unit_process_timeout,
      clt_rrt_timeout,
      linear_vel,
      linear_disc,
      planning_scenes_subprocess,
      plans,
      saved_graph_file_name,
      use_saved_graph);

  // skip retract planning for picknplace

//  // Step 5 : Plan for transition between each pair of sequential path
  transitionPlanningPickNPlace(plans, moveit_model, planning_scene_diff_client, move_group_name,
      start_state, planning_scenes_subprocess);

//  // Step 7 : fill in trajectory's time headers and pack into sub_process_plans
//  // for each unit_process (process id is added here too)
  adjustTrajectoryTiming(plans, joint_names, world_frame);

//  // Step 8: fill in TCP pose according to trajectories
  appendTCPPoseToPlansPickNPlace(model, planning_scenes_transition, planning_scenes_subprocess, plans);

  ROS_INFO("[Process Planning] trajectories solved and packing finished");
  return true;
}

} // end choreo process planning namespace