// boost smart ptr
#include <boost/shared_ptr.hpp>

// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// actions & msgs
#include <framefab_msgs/AdvanceRobot.h>
#include <framefab_msgs/TestDescartes.h>

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

//  ptr_moveit_visual_tools = boost::make_shared<moveit_visual_tools::MoveItVisualTools>(
//      ptr_move_group_->getPlanningFrame());
//  ptr_moveit_visual_tools->deleteAllMarkers();
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
  ROS_WARN_STREAM("advanceRobot request:" << std::endl << req);

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
    return false;
  }

  // TODO: make the x, y, z into service
  // Apply the offset on the given axis of the current pose
   current_pose.translate(Eigen::Vector3d(req.offset_x , req.offset_y, req.offset_z));

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

bool FrameFabPlanner::testDescartesPlanning(
     framefab_msgs::TestDescartes::Request &req, framefab_msgs::TestDescartes::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM("testDescartes request:" << std::endl << req);

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
    ROS_ERROR("[testDescartesPlanning] tf error.");
    return false;
  }

  ros::Rate loop_rate(10);

  // 1. Define sequence of points
  double x, y, z, rx, ry, rz;
  x = 0.67734;
  y = 0;
  z = 0.5;
  rx = 0.0;
  ry = M_PI;
  rz = 0.0;
  TrajectoryVec points;
  int N_points = 10;

  framefab_process_path::ProcessPathGenerator  path_generator;
  framefab_process_path::ProcessPathVisualizer path_visualizer;

  std::vector<Eigen::Affine3d> poses;
  Eigen::Affine3d startPose;
  Eigen::Affine3d endPose;
  startPose = descartes_core::utils::toFrame(x, y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ);
  endPose = descartes_core::utils::toFrame(x, y + 0.5, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ);
  poses = path_generator.addLinePathPts(startPose, endPose, N_points);


  for (unsigned int i = 0; i < N_points; ++i)
  {
//    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(poses[i], 0.0, 0.4, M_PI);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(poses[i]);
    points.push_back(pt);
  }

  // Visualize the trajectory points in RViz
  // Transform the generated poses into a markerArray message that can be visualized by RViz
  visualization_msgs::MarkerArray ma;
  ma = path_visualizer.createMarkerArray(poses);

  ros::Publisher vis_pub = node_handle_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
  if(waitForSubscribers(vis_pub, ros::Duration(0.5)))
  {
    vis_pub.publish(ma);
    loop_rate.sleep();
  }


//  // 1. Define sequence of points
//  TrajectoryVec points;
//  for (unsigned int i = 0; i < 4; ++i)
//  {
//    Eigen::Affine3d pose = current_pose.translate(Eigen::Vector3d(0, 0, 0.005*i));
//    //pose = Eigen::Translation3d(0.05*i, 0.0, 0.0);
//    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
//    points.push_back(pt);
//  }

//  for (unsigned int i = 0; i < 4; ++i)
//  {
//    Eigen::Affine3d pose;
//    pose = Eigen::Translation3d(0.0, 0.04 * i, 0.05);
//    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
//    points.push_back(pt);
//  }

  // TODO: move model construction in constructor
  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "manipulator";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = ptr_move_group_->getPlanningFrame();

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "tool_tip";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    res.success = false;
    return false;
  }

  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    res.success = false;
    return false;
  }

  TrajectoryVec result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    res.success = false;
    return false;
  }

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
    res.success = false;
    return false;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("test Descartes Done!");
  res.success = true;
  return true;
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

descartes_core::TrajectoryPtPtr FrameFabPlanner::makeTolerancedCartesianPoint(Eigen::Affine3d pose,
                                                             double rxTolerance, double ryTolerance, double rzTolerance)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  double rotStepSize = 0.1; //M_PI/180;

  Eigen::Vector3d translations;
  translations = pose.translation();
  Eigen::Vector3d eulerXYZ;
  eulerXYZ = pose.rotation().eulerAngles(0, 1, 2);

  PositionTolerance p;
  p = ToleranceBase::zeroTolerance<PositionTolerance>(translations(0), translations(1), translations(2));
  OrientationTolerance o;
  o = ToleranceBase::createSymmetric<OrientationTolerance>(eulerXYZ(0), eulerXYZ(1), eulerXYZ(2), rxTolerance, ryTolerance, rzTolerance);
  return TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose, p, o), 0.0, rotStepSize));
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
  result.header.frame_id = ptr_move_group_->getPlanningFrame();
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

bool FrameFabPlanner::waitForSubscribers(ros::Publisher &pub, ros::Duration timeout)
{
  if (pub.getNumSubscribers() > 0)
    return true;
  ros::Time start = ros::Time::now();
  ros::Rate waitTime(0.5);
  while (ros::Time::now() - start < timeout)
  {
    waitTime.sleep();
    if (pub.getNumSubscribers() > 0)
      break;
  }
  return pub.getNumSubscribers() > 0;
}

std::vector<double> getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state =
      ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));

  if (!state)
  {
    throw std::runtime_error("Joint state message capture failed");
  }

  return state->position;
}

}// namespace frammefab