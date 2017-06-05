// boost smart ptr
#include <boost/shared_ptr.hpp>

// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// moveit
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

// descartes


// framefab
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

  // TODO: make the x, y, z into service
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

}// namespace frammefab