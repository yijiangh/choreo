//
// Created by yijiangh on 4/3/18.
//

#include <framefab_process_planning/framefab_process_planning.h>
#include <ros/console.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

// service
#include <framefab_msgs/PickNPlacePlanning.h>

// msg
#include <trajectory_msgs/JointTrajectory.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/Grasp.h>

#include "path_transitions.h"
#include "common_utils.h"

// for immediate execution
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

static const std::string ROBOT_DESCRIPTION="robot_description";

namespace {

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = 0.34;
  p.pose.position.y = -0.7;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

//  openGripper(g.pre_grasp_posture);
//  closedGripper(g.grasp_posture);

  grasps.push_back(g);
  group.setSupportSurfaceName("table");
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;

  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "base_footprint";

  g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

//  openGripper(g.post_place_posture);

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  group.setPathConstraints(constr);
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}

} // end anon util namespace

namespace framefab_process_planning
{


bool ProcessPlanningManager::handlePickNPlacePlanning(
    framefab_msgs::PickNPlacePlanning::Request& req,
    framefab_msgs::PickNPlacePlanning::Response& res)
{
//    trajectory_msgs::JointTrajectory ros_traj = getMoveitPlan(hotend_group_name_,
//                                                              current_joints,
//                                                              target_pose,
//                                                              moveit_model_);

//    ros::Duration base_time = ros_traj.points[0].time_from_start;
//
//    double sim_time_scale = 0.6;
//    for (int i = 0; i < ros_traj.points.size(); i++)
//    {
//      ros_traj.points[i].time_from_start -= base_time;
//
//      //sim speed tuning
//      ros_traj.points[i].time_from_start *= sim_time_scale;
//    }
//
//    fillTrajectoryHeaders(joint_names, ros_traj, world_frame_);
//
//    // step 5: immediate execution (a quick solution for debugging)
//    ros::NodeHandle nh;
//    actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> client(nh, "joint_trajectory_action");
//    if (!client.waitForServer(ros::Duration(1.0)))
//    {
//      ROS_WARN("[Reset Exe] Exec timed out");
//    }
//    else
//    {
//      ROS_INFO("[Reset Exe] Found action server");
//    }
//
//    control_msgs::FollowJointTrajectoryGoal goal;
//    goal.trajectory = ros_traj;
//
//    client.sendGoalAndWait(goal);
//    return true;
//  }

  ROS_INFO_STREAM("pick and place process planner connected.");

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_link";

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.175;
  pub_co.publish(co);

  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  pub_aco.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.z = 0.5;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

//  pick(group);

  ros::WallDuration(1.0).sleep();

//  place(group);

  return true;
}
}// end ff_process_planning ns