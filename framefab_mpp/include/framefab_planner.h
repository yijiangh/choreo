/*
 * FrameFab.h
 * 
 * Created on:  April 7, 2017
 * Author:      Yijiang Huang, Thomas Cook
 * Institute:   MIT, Digital Structure Group, Building Tech
*/

#ifndef FRAMEFAB_PLANNER_H
#define FRAMEFAB_PLANNER_H

// boost::shared_ptr declaration
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// msgs & actions
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// Moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// descartes
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>

// framefab
#include <wire_frame/wire_frame_collision_objects.h>

// framefab msgs
#include <framefab_msgs/AdvanceRobot.h>
#include <framefab_msgs/TestDescartes.h>

namespace framefab
{

/*! @class FrameFabPlanner
 *  @brief The framefab main container class. ros service client.
 *
 * service client, listening service from FrameFabRenderWidget Rviz
 * service node (from user's clicking buttons etc)
*/
class FrameFabPlanner
{
  typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
  typedef TrajectoryVec::const_iterator TrajectoryIter;

  /* public functions */
 public:

  /*!
   *	@brief Constructor.
   * 	@param[in] ROS nodeHandle for parameters request
   */
  FrameFabPlanner(ros::NodeHandle &node_handle);

  /*!
   *	@brief Destructor.
   */
  ~FrameFabPlanner();

  /*
   * function for development, experiment goes here.
   */
  bool testCartPlanning(framefab_msgs::AdvanceRobot::Request &req,
                        framefab_msgs::AdvanceRobot::Response &res);

  bool testDescartesPlanning(framefab_msgs::TestDescartes::Request &req,
                             framefab_msgs::TestDescartes::Response &res);

  /* private functions */
 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful
   */
  void readParameters();

  /*!
  * Performs the initialization procedure.
  * @return true if successful.
  */
  bool initialize() {}

  // descartes
  /*!
  * Generates an completely defined (zero-tolerance) cartesian point from a pose
  */
  descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);

  /*!
  * Generates a cartesian point with free rotation about the Z axis of the EFF frame
  */
  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

  /*!
  * Translates a descartes trajectory to a ROS joint trajectory
  */
  trajectory_msgs::JointTrajectory
  toROSJointTrajectory(const TrajectoryVec& trajectory,
                       const descartes_core::RobotModel& model,
                       const std::vector<std::string>& joint_names,
                       double time_delay);

  /*!
  * Sends a ROS trajectory to the robot controller
  */
  bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

  /* private data */
 private:

  //! ROS nodehandle.
  ros::NodeHandle &node_handle_;

  moveit::planning_interface::PlanningSceneInterfacePtr ptr_planning_scene_interface_;
  move_group_interface::MoveGroupPtr              ptr_move_group_;
  wire_frame::WireFrameCollisionObjectsPtr        ptr_wire_frame_collision_objects_;

  moveit_visual_tools::MoveItVisualToolsPtr ptr_moveit_visual_tools;
};

typedef boost::shared_ptr<FrameFabPlanner>        FrameFabPlannerPtr;
typedef boost::shared_ptr<const FrameFabPlanner>  FrameFabPlannerConstaPtr;

} /* namespace */

#endif // FRAMEFAB_PLANNER_H