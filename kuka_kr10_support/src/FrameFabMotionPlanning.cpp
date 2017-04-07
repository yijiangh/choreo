#include <cmath>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// ros message
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

// visualizer-related
#include <visualization_msgs/Marker.h>

// framefab
#include <kuka_node.h>

// todo:
// static variable for temporal experiment
// should get them into class later
// static visualization_msgs::Marker link_list;
static ros::Rate * r;
static float f;

void frameCallback(geometry_msgs::PoseArray msg){
  ROS_INFO("frame subscriber callback");
  std::cout << msg << std::endl;
  link_list.color.b = 1.0;
  link_list.color.a = 1.0;
  link_list.scale.x = 1.0;
  link_list.scale.y = 1.0;
  link_list.scale.z = 1.0; 
  link_list.header.stamp = ros::Time::now();
  
  geometry_msgs::Point p;	
  p.x = 1.0;
  p.y = 1.0;
  p.z = 0.0;
  link_list.points.push_back(p);
  link_list.points.push_back(p);  
  marker_pub.publish(link_list); 
  r->sleep();
}    

void kuka_node::mplanCallback(std_msgs::Bool mp_msg){
	// todo: end effector "printer" added - need to remake moveit config files
	
	/* --- planner setup --- */

	// consturct RobotModel for planner
	// RobotModelLoader object will look up the robot description on the ROS
	// parameter server and construct a :moveit_core:'RobotModel' for us
	ROS_INFO("motion planning callback");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	
	// construct Planning scene for planner
	// Using the :moveit_core:`RobotModel`, construct a :planning_scene:`PlanningScene` 
	// that maintains the state of the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));	
	
  // construct a loader to load a planner
	// use ROS pluginlib interface to load a planner
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
	// make sure target planner name has been implemented in ROSParam/
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
	{
    ROS_FATAL_STREAM("Could not find planner plugin name");
	}

	// try resetting plugin_loader (creating a new planner_mananger using class PlannerManager in package moveit_core)
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
	
	// try resetting (scoped_ptr delete old and init a new) planner_plugin_loader (planner_manager)
	// plugin::ClassLoader<T>::createUnmanagedInstance (loopup_name)
	try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
		{
      ROS_FATAL_STREAM("Could not initialize planner instance");
		}

    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }	
}
