
#include <boost/shared_ptr.hpp>

#include <moveit_msgs/DisplayTrajectory.h>

#include <framefab_planner.h>

namespace framefab
{

FrameFabPlanner::FrameFabPlanner(
    ros::NodeHandle &node_handle,
    moveit::planning_interface::PlanningSceneInterfacePtr       ptr_planning_scene_interface,
    const move_group_interface::MoveGroupPtr                    ptr_move_group,
    const wire_frame::WireFrameCollisionObjectsPtr              ptr_wire_frame_collision_objects)
    : node_handle_(node_handle),
      ptr_planning_scene_interface_(ptr_planning_scene_interface),
      ptr_move_group_(ptr_move_group),
      ptr_wire_frame_collision_objects_(ptr_wire_frame_collision_objects)
{
  ROS_INFO_NAMED("framefab_mpp", "[ff_planner] FrameFabPlanner node started.");

  readParameters();

  ptr_moveit_visual_tools = boost::make_shared<moveit_visual_tools::MoveItVisualTools>(
      ptr_move_group->getPlanningFrame());
  ptr_moveit_visual_tools->deleteAllMarkers();
}

FrameFabPlanner::~FrameFabPlanner()
{

}

bool FrameFabPlanner::readParameters()
{
  return true;
}

void FrameFabPlanner::testCartPlanning()
{
  std::string ff_tag = "framefab_planner";

  ROS_INFO_NAMED(ff_tag, "[ff_planner] ROS test function called.");

  // Tom's experiment
//  moveit::planning_interface::MoveGroup::Plan my_plan;
//  std::vector<geometry_msgs::Pose> waypoints;
//
//  geometry_msgs::Point start = ptr_move_group_->getCurrentPose().pose.position;
//  geometry_msgs::Pose start_pose, next_pose;
//
//  next_pose = ptr_wire_frame_collision_objects_->getCollisionObject(0)->start_vertex_collision.primitive_poses[0];
//
//  waypoints.push_back(next_pose);
//  moveit_msgs::RobotTrajectory trajectory;
//
//  double fraction = ptr_move_group_->computeCartesianPath(waypoints, 0.001, 0.00, trajectory);
//
//  if (fraction > 0.1)
//  {
//    my_plan.trajectory_ = trajectory;
//    ROS_INFO_NAMED("framefab_planner", "Fraction %.2f", fraction);
//    ptr_move_group_->asyncExecute(my_plan);
//  }
//  else
//  {
//    ROS_INFO_NAMED("framefab_planner", "Fraction to small");
//    return;
//  }

  // from move_group_interface_tutorial
  // https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/planning/src/move_group_interface_tutorial.cpp#L287

  namespace rviz_vt = rviz_visual_tools;

  // (optional)
  ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1 ,true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED(ff_tag, "[ff_planner] Reference frame: %s", ptr_move_group_->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED(ff_tag, "[ff_planner] Reference frame: %s", ptr_move_group_->getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1 = ptr_wire_frame_collision_objects_->getCollisionObject(0)->start_vertex_collision.primitive_poses[0];
//  target_pose1.orientation.w = 0;
//  target_pose1.position.x = 0.8;
//  target_pose1.position.y = 0;
//  target_pose1.position.z = 0.7;

  ptr_move_group_->setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = ptr_move_group_->plan(my_plan);

  ROS_INFO_NAMED(ff_tag, "[ff_planner] Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.
  if (1)
  {
    ROS_INFO("[ff_planner] Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
}

void FrameFabPlanner::testDescartesPlanning()
{

}

void FrameFabPlanner::setRobotHomePose()
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

  for(std::map<std::string, double>::const_iterator it = home_state.begin(); it!=home_state.end(); ++it)
  {
    ROS_INFO("[ff_planner] home_pose: %s, %f", it->first.c_str(), it->second);
  }

  ptr_move_group_->setJointValueTarget(home_state);
  bool success = ptr_move_group_->plan(my_plan);

  ROS_INFO_NAMED("framefab_mpp", "[ff_planner] Returning Home Pose %s", success ? "" : "FAILED");

  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
}

}// namespace frammefab