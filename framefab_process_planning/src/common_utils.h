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

trajectory_msgs::JointTrajectory toROSTrajectory(const DescartesTraj& solution,
                                                 const descartes_core::RobotModel& model);

trajectory_msgs::JointTrajectory toROSTrajectory(const std::vector<std::vector<double>>& solution,
                                                 const descartes_core::RobotModel& model);

void fillTrajectoryHeaders(const std::vector<std::string>& joints,
                           trajectory_msgs::JointTrajectory& traj);

void appendTrajectoryHeaders(const trajectory_msgs::JointTrajectory& orig_traj,
                             trajectory_msgs::JointTrajectory& traj,
                             const double sim_time_scale = 0.0);

std::vector<double> getCurrentJointState(const std::string& topic);

bool addCollisionObject(
    ros::ServiceClient& planning_scene, const moveit_msgs::CollisionObject& c_obj);

bool clearAllCollisionObjects(ros::ServiceClient& planning_scene);

static inline std::vector<double> extractJoints(const descartes_core::RobotModel& model,
                                                const descartes_core::TrajectoryPt& pt)
{
  std::vector<double> dummy, result;
  pt.getNominalJointPose(dummy, model, result);
  return result;
}

DescartesTraj createJointPath(const std::vector<double>& start, const std::vector<double>& stop,
                              double dtheta = M_PI / 180.0);

trajectory_msgs::JointTrajectory getMoveitPlan(const std::string& group_name,
                                               const std::vector<double>& joints_start,
                                               const std::vector<double>& joints_stop,
                                               moveit::core::RobotModelConstPtr model);

trajectory_msgs::JointTrajectory getMoveitTransitionPlan(const std::string& group_name,
                                                         const std::vector<double>& joints_start,
                                                         const std::vector<double>& joints_stop,
                                                         const std::vector<double>& initial_pose,
                                                         moveit::core::RobotModelConstPtr model,
                                                         const bool force_insert_reset = false);

trajectory_msgs::JointTrajectory planFreeMove(descartes_core::RobotModel& model,
                                              const std::string& group_name,
                                              moveit::core::RobotModelConstPtr moveit_model,
                                              const std::vector<double>& start,
                                              const std::vector<double>& stop);


double freeSpaceCostFunction(const std::vector<double>& source,
                             const std::vector<double>& target);
}

#endif //FRAMEFAB_PROCESS_PLANNING_COMMON_UTILS_H
