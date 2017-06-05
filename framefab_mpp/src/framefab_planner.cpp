// boost smart ptr
#include <boost/shared_ptr.hpp>

// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// actions & msgs
#include <framefab_msgs/AdvanceRobot.h>

// moveit
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

// framefab
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
  // 1. Define sequence of points
  TrajectoryVec points;
  for (unsigned int i = 0; i < 10; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.0, 0.0, 1.0 + 0.05 * i);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }

  for (unsigned int i = 0; i < 5; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.0, 0.04 * i, 1.3);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }

  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "manipulator";

  // TODO: change here!

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "base_link";

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "tool0";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

  TrajectoryVec result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }


  // TODO: check do we have this parameter?
  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::vector<std::string> names;
  node_handle_.getParam("controller_joint_names", names);
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -4;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
}

descartes_core::TrajectoryPtPtr FrameFabPlanner::makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr FrameFabPlanner::makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory FrameFabPlanner::toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // TODO: check this frame_id!
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

bool FrameFabPlanner::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  }
  else
  {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}

}// namespace frammefab