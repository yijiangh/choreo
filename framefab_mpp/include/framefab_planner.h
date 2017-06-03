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

// Moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// framefab
#include <wire_frame/wire_frame_collision_objects.h>

// framefab msgs
#include <framefab_msgs/AdvanceRobot.h>

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
  bool testDescartesPlanning();
  bool setRobotHomePose(framefab_msgs::AdvanceRobot::Request& req,
                        framefab_msgs::AdvanceRobot::Response& res);

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

  void makePlan(const std::vector<geometry_msgs::Pose> &way_points){}

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