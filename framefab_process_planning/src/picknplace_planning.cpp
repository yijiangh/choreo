//
// Created by yijiangh on 4/3/18.
//

#include <framefab_process_planning/framefab_process_planning.h>
#include <ros/console.h>

// service
#include <framefab_msgs/PickNPlacePlanning.h>

// msg
#include <trajectory_msgs/JointTrajectory.h>

#include "path_transitions.h"
#include "common_utils.h"

// for immediate execution
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace {


} // end anon util namespace

namespace framefab_process_planning
{

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

bool ProcessPlanningManager::handlePickNPlacePlanning(
    framefab_msgs::PickNPlacePlanning::Request& req,
    framefab_msgs::PickNPlacePlanning::Response& res)
{
//    trajectory_msgs::JointTrajectory ros_traj = getMoveitPlan(hotend_group_name_,
//                                                              current_joints,
//                                                              target_pose,
//                                                              moveit_model_);

//    ros::Duration base_time = ros_traj.points[0].time_from_start;
//
//    double sim_time_scale = 0.6;
//    for (int i = 0; i < ros_traj.points.size(); i++)
//    {
//      ros_traj.points[i].time_from_start -= base_time;
//
//      //sim speed tuning
//      ros_traj.points[i].time_from_start *= sim_time_scale;
//    }
//
//    fillTrajectoryHeaders(joint_names, ros_traj, world_frame_);
//
//    // step 5: immediate execution (a quick solution for debugging)
//    ros::NodeHandle nh;
//    actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> client(nh, "joint_trajectory_action");
//    if (!client.waitForServer(ros::Duration(1.0)))
//    {
//      ROS_WARN("[Reset Exe] Exec timed out");
//    }
//    else
//    {
//      ROS_INFO("[Reset Exe] Found action server");
//    }
//
//    control_msgs::FollowJointTrajectoryGoal goal;
//    goal.trajectory = ros_traj;
//
//    client.sendGoalAndWait(goal);
//    return true;
//  }

  ROS_INFO_STREAM("pick and place process planner connected.");

  return true;
}
}// end ff_process_planning ns