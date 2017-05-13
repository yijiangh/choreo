#include <framefab_planner.h>

namespace framefab
{

FrameFabPlanner::FrameFabPlanner(
    const ros::NodeHandle &node_handle,
    moveit::planning_interface::PlanningSceneInterfacePtr       ptr_planning_scene_interface,
    const move_group_interface::MoveGroupPtr                    ptr_move_group,
    const wire_frame::WireFrameCollisionObjectsPtr              ptr_wire_frame_collision_objects)
    : node_handle_(node_handle),
      ptr_planning_scene_interface_(ptr_planning_scene_interface),
      ptr_move_group_(ptr_move_group),
      ptr_wire_frame_collision_objects_(ptr_wire_frame_collision_objects)
{
  ROS_INFO_NAMED("framefab_mpp", "FrameFabPlanner node started.");

  readParameters();
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
  ROS_INFO_NAMED("framefab_planner", "ROS test function called.");

  moveit::planning_interface::MoveGroup::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Point start = ptr_move_group_->getCurrentPose().pose.position;
  geometry_msgs::Pose start_pose, next_pose;

  next_pose = ptr_wire_frame_collision_objects_->getCollisionObject(0)->start_vertex_collision.primitive_poses[0];

  waypoints.push_back(next_pose);
  moveit_msgs::RobotTrajectory trajectory;

  double fraction = ptr_move_group_->computeCartesianPath(waypoints, 0.001, 0.00, trajectory);

  if (fraction > 0.1)
  {
    my_plan.trajectory_ = trajectory;
    ROS_INFO_NAMED("framefab_planner", "Fraction %.2f", fraction);
    ptr_move_group_->asyncExecute(my_plan);
  }
  else
  {
    ROS_INFO_NAMED("framefab_planner", "Fraction to small");
    return;
  }
}

void FrameFabPlanner::testDescartesPlanning()
{

}

}// namespace frammefab