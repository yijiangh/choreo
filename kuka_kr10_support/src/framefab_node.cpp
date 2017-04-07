/*
 * framefab_node.cpp
 * 
 * Created on: April 7, 2017
 * Author:		 Yijiang Huang, Thomas Cook
 * Institute:  MIT, Digital Structure Group, Building Tech 
*/

#include <ros/ros.h>
#include <framefab/FrameFab.h>

int main(int argc, char **argv)
{
	//todo: 
	// should keep main function clean
	// should move these marker related 
	// setting to visualization class

  ros::init(argc, argv, "framefab_node");
  ros::NodeHandle node_handle("~");

	// init framefab class
	kuka_node k_node;
	k_node.setNodeHandle(&node_handle);
	
	// spin
  ros::AsyncSpinner spinner(1);
  spinner.start();
	
  while(ros::ok()) {
    ros::spinOnce();
  } 
  ros::shutdown();
  return 0;

	// visualize framelinks message subsriber
  ros::Subscriber frame_sub = node_handle.subscribe("framelinks", 		0, &frameCallback);  
	ros::Subscriber mplan_sub = node_handle.subscribe("activate_mplan", 0, &k_node.mplanCallback);

	// global marker(link in Rviz) property init
	link_list.header.frame_id = "world";
  link_list.action = visualization_msgs::Marker::ADD;
  link_list.pose.orientation.w = 1.0;
  link_list.id = 0; 
  link_list.type = visualization_msgs::Marker::LINE_LIST;

  marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
 
	/* This sleep is ONLY to allow Rviz to come up */
  r = new ros::Rate(20.0);
  r->sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // Publisher for visualizing plans in Rviz.
  display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}
