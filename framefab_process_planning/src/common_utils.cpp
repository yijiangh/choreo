//
// Created by yijiangh on 7/5/17.
//

#include "common_utils.h"
#include <ros/topic.h>

// services
#include <moveit_msgs/ApplyPlanningScene.h>

// Constants
const static double DEFAULT_JOINT_WAIT_TIME = 5.0; // Maximum time allowed to capture a new joint
// state message

std::vector<double> framefab_process_planning::getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(
      topic, ros::Duration(DEFAULT_JOINT_WAIT_TIME));
  if (!state)
    throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

bool framefab_process_planning::addCollisionObject(
    ros::ServiceClient& planning_scene_diff_client, const moveit_msgs::CollisionObject& c_obj)
{
  if(planning_scene_diff_client.waitForExistence())
  {
//    ROS_INFO_STREAM("planning scene diff srv connected!");
  }
  else
  {
    ROS_ERROR_STREAM("cannot connect with planning scene diff server...");
  }

  moveit_msgs::ApplyPlanningScene srv;

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(c_obj);
  planning_scene_msg.is_diff = true;
  srv.request.scene = planning_scene_msg;

  if(planning_scene_diff_client.call(srv))
  {
//    ROS_INFO_STREAM("adding new collision object to planning scene published!");
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to publish planning scene diff srv!");
    return false;
  }
}
