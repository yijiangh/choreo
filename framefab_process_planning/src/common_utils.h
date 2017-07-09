//
// Created by yijiangh on 7/5/17.
//

#ifndef FRAMEFAB_PROCESS_PLANNING_COMMON_UTILS_H
#define FRAMEFAB_PROCESS_PLANNING_COMMON_UTILS_H

#include <ros/ros.h>

#include <Eigen/Geometry>

// descartes
#include <descartes_core/trajectory_pt.h>
#include <descartes_core/robot_model.h>

// msgs
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

namespace framefab_process_planning
{
typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTraj;

Eigen::Affine3d createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                       const geometry_msgs::Point& pt);

Eigen::Affine3d createNominalTransform(const geometry_msgs::Pose& ref_pose, const double z_adjust = 0.0);


Eigen::Affine3d createNominalTransform(const Eigen::Affine3d& ref_pose, const double z_adjust = 0.0);

std::vector<double> getCurrentJointState(const std::string& topic);

bool addCollisionObject(
    ros::ServiceClient& planning_scene, const moveit_msgs::CollisionObject& c_obj);

}

#endif //FRAMEFAB_PROCESS_PLANNING_COMMON_UTILS_H
