//
// Created by yijiangh on 7/5/17.
//

#include "common_utils.h"

#include <ros/topic.h>
#include <eigen_conversions/eigen_msg.h>

// services
#include <moveit_msgs/ApplyPlanningScene.h>

// Constants
const static double DEFAULT_JOINT_WAIT_TIME = 5.0; // Maximum time allowed to capture a new joint
// state message

Eigen::Affine3d framefab_process_planning::createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                                               const geometry_msgs::Point& pt)
{
  Eigen::Affine3d eigen_pose;
  Eigen::Vector3d eigen_pt;

  tf::poseMsgToEigen(ref_pose, eigen_pose);
  tf::pointMsgToEigen(pt, eigen_pt);

  // Translation transform
  Eigen::Affine3d to_point;
  to_point = Eigen::Translation3d(eigen_pt);

  // Reverse the Z axis
  Eigen::Affine3d flip_z;
  flip_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  // "snap" to the pt and flip z axis
  return eigen_pose * to_point * flip_z;
}

Eigen::Affine3d framefab_process_planning::createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                                               const double z_adjust)
{
  Eigen::Affine3d eigen_pose;

  tf::poseMsgToEigen(ref_pose, eigen_pose);

  return createNominalTransform(eigen_pose, z_adjust);
}

Eigen::Affine3d framefab_process_planning::createNominalTransform(const Eigen::Affine3d &ref_pose,
                                                               const double z_adjust)
{
  // Reverse the Z axis
  Eigen::Affine3d flip_z;
  flip_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  return ref_pose * Eigen::Translation3d(0, 0, z_adjust) * flip_z;
}

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

double framefab_process_planning::freeSpaceCostFunction(const std::vector<double> &source,
                                                     const std::vector<double> &target)
{
  const double FREE_SPACE_MAX_ANGLE_DELTA =
      M_PI; // The maximum angle a joint during a freespace motion
  // from the start to end position without that motion
  // being penalized. Avoids flips.
  const double FREE_SPACE_ANGLE_PENALTY =
      2.0; // The factor by which a joint motion is multiplied if said
  // motion is greater than the max.

  // The cost function here penalizes large single joint motions in an effort to
  // keep the robot from flipping a joint, even if some other joints have to move
  // a bit more.
  double cost = 0.0;
  for (std::size_t i = 0; i < source.size(); ++i)
  {
    double diff = std::abs(source[i] - target[i]);
    if (diff > FREE_SPACE_MAX_ANGLE_DELTA)
      cost += FREE_SPACE_ANGLE_PENALTY * diff;
    else
      cost += diff;
  }
  return cost;
}
