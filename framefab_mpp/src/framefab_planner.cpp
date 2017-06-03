
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <framefab_msgs/AdvanceRobot.h>

#include <framefab_planner.h>

namespace framefab
{

FrameFabPlanner::FrameFabPlanner(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  ROS_INFO_NAMED("framefab_mpp", "[ff_planner] FrameFabPlanner node started.");

  readParameters();

  ptr_planning_scene_interface_ = boost::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  ptr_move_group_ = boost::make_shared<move_group_interface::MoveGroup>("manipulator");

  ptr_moveit_visual_tools = boost::make_shared<moveit_visual_tools::MoveItVisualTools>(
      ptr_move_group_->getPlanningFrame());
  ptr_moveit_visual_tools->deleteAllMarkers();
}

FrameFabPlanner::~FrameFabPlanner()
{

}

void FrameFabPlanner::readParameters()
{
//  node_handle_.param()
}

bool FrameFabPlanner::testCartPlanning(
    framefab_msgs::AdvanceRobot::Request &req, framefab_msgs::AdvanceRobot::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM("moveRobot request:" << std::endl << req);

  // Get current robot pose
  tf::TransformListener listener;
  listener.waitForTransform("/arm_base_link", "/tool_tip", ros::Time::now(), ros::Duration(1.0));
  tf::StampedTransform transform_stamp;
  Eigen::Affine3d current_pose;

  try
  {
    listener.lookupTransform("/arm_base_link", "/tool_tip", ros::Time(0), transform_stamp);
    transformTFToEigen(transform_stamp, current_pose);
  }
  catch (tf::TransformException &ex)
  {
    res.success = false;
    ROS_ERROR("[testCartPlanning] tf error.");
    return true;
  }

  // Apply the offset on the given axis of the current pose
   current_pose.translate(Eigen::Vector3d(0.05 , 0, 0));

  // Try to move to new pose
  std::vector<geometry_msgs::Pose> way_points_msg(1);
  tf::poseEigenToMsg(current_pose, way_points_msg[0]);

  ROS_WARN_STREAM("robot current pose" << way_points_msg[0]);

  moveit_msgs::ExecuteKnownTrajectory srv;

  ros::ServiceClient executeKnownTrajectoryServiceClient =
  node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");

  if (ptr_move_group_->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) < 0.99)
  {
    // No trajectory can be found, aborting and sending error message:
    res.success = false;
    ROS_ERROR("[testCartPlannig] cart plan computation fails.");
    return false;
  }

  executeKnownTrajectoryServiceClient.call(srv);
  res.success = true;
  return true;
}

bool FrameFabPlanner::testDescartesPlanning()
{

}

bool FrameFabPlanner::setRobotHomePose(
    framefab_msgs::AdvanceRobot::Request& req,
    framefab_msgs::AdvanceRobot::Response& res)
{
  if(req.is_advance)
  {
    moveit::planning_interface::MoveGroup::Plan my_plan;
    const std::map<std::string, double> home_state = ptr_move_group_->getNamedTargetValues("home_pose");

    std::vector<double> group_variable_values;
    ptr_move_group_->getCurrentState()->copyJointGroupPositions(
        ptr_move_group_->getCurrentState()->getRobotModel()->getJointModelGroup(ptr_move_group_->getName()),
        group_variable_values);

//  group_variable_values[0] = -1.0;
//  ptr_move_group_->setJointValueTarget(group_variable_values);
//  bool success = ptr_move_group_->plan(my_plan);

    for (std::map<std::string, double>::const_iterator it = home_state.begin(); it != home_state.end(); ++it)
    {
      ROS_INFO("[ff_planner] home_pose: %s, %f", it->first.c_str(), it->second);
    }

    ptr_move_group_->setJointValueTarget(home_state);
    bool success = ptr_move_group_->plan(my_plan);

    ROS_INFO_NAMED("framefab_mpp", "[ff_planner] Returning Home Pose %s", success ? "" : "FAILED");

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    res.success = success;

    return success;
  }
  else
  {
    res.success = false;
    return false;
  }
}

}// namespace frammefab